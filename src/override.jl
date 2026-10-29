const OVERRIDE_SYNTAX = """
    @override ros_field[ as julia_field][::JuliaType][ = default]
    @override (ros_field, next_ros_field, …) as julia_field::JuliaType[ = default]
    @override ROS_CONSTANT[ as [TypeName.]JULIA_CONSTANT][::JuliaType]"""

# Indent each line by four spaces, e.g. to show the syntax as code in a docstring.
indent(s::AbstractString) = replace(s, r"^"m => "    ")

"""
$(indent(OVERRIDE_SYNTAX))

Override a field or constant of a ROS message in the `begin` `end` block of [`@importrosmsg`](@ref).

A constant is a property of the imported type. `as TypeName.JULIA_CONSTANT` renames that property, while `as JULIA_CONSTANT` additionally defines a module constant. `@importrosmsg` reads these lines itself, so `@override` throws an `ArgumentError` anywhere else.

```jldoctest
julia> @importrosmsg builtin_interfaces/Time as Stamp begin
           @override sec as seconds
       end

julia> fieldnames(Stamp)
(:seconds, :nanosec)

julia> @override sec
ERROR: LoadError: ArgumentError: `@override` is only valid in the override block of `@importrosmsg`.
[...]
```
"""
macro override(xs...)
    throw(ArgumentError("`@override` is only valid in the override block of `@importrosmsg`."))
end

"""
    Override(rosnames, juliafield, target, type, default, source)

Hold one parsed `@override` line: the ROS fields or constant it covers, the Julia name, the name after `as` (`nothing` without `as`, a `Symbol`, or `TypeName.NAME` for a constant), and the type and default, each `nothing` if not overridden. A given `default` is wrapped in `Some`, as the default expression `nothing` is valid.

```jldoctest
julia> ROSMessages.Override(:(@override (latitude, longitude) as position::Position = Position(0, 0)))
ROSMessages.Override([:latitude, :longitude], :position, :position, :Position, Some(:(Position(0, 0))), :(#= none:1 =#))

julia> ROSMessages.Override(:(@override STATUS_FIX as NavSatStatus.FIX::Flag))
ROSMessages.Override([:STATUS_FIX], :FIX, :(NavSatStatus.FIX), :Flag, nothing, :(#= none:1 =#))

julia> ROSMessages.Override(:(@override frame_id))
ROSMessages.Override([:frame_id], :frame_id, nothing, nothing, nothing, :(#= none:1 =#))

julia> ROSMessages.Override(:(@override a b))
ERROR: ArgumentError: none:1: Each `@override` line needs one of these forms:
[...]
```
"""
struct Override
    rosnames::Vector{Symbol}
    juliafield::Symbol
    target::Union{Symbol, Expr, Nothing}
    type::Any
    default::Union{Some, Nothing}
    source::LineNumberNode
end

override_error(source::LineNumberNode, message::AbstractString) = ArgumentError("$(source.file):$(source.line): $message")
syntax_error(source::LineNumberNode) = override_error(source, "Each `@override` line needs one of these forms:\n$OVERRIDE_SYNTAX")

function Override(ex)
    isexpr(ex, :macrocall) && length(ex.args) >= 3 && ex.args[1] === Symbol("@override") ||
        throw(ArgumentError("Each line of the override block of `@importrosmsg` needs to start with `@override`:\n$OVERRIDE_SYNTAX"))
    source = ex.args[2]::LineNumberNode
    args = ex.args[3:end]
    if length(args) == 1
        rosnames, type, default = split_declaration(args[1])
        rosnames = rosfieldnames(rosnames, source)
        target = nothing
        juliafield = first(rosnames)
    elseif length(args) == 3 && args[2] === :as
        rosnames = rosfieldnames(args[1], source)
        target, type, default = split_declaration(args[3])
        juliafield = target isa Symbol ? target : qualifiedname(target, source)
    else
        throw(syntax_error(source))
    end
    return Override(rosnames, juliafield, target, type, default, source)
end

# Return `NAME` of `TypeName.NAME`.
function qualifiedname(x, source::LineNumberNode)
    isexpr(x, :., 2) && x.args[1] isa Symbol && x.args[2] isa QuoteNode && x.args[2].value isa Symbol || throw(syntax_error(source))
    return x.args[2].value
end

# Split `name[::Type][ = default]`.
function split_declaration(ex)
    default = nothing
    if isexpr(ex, :(=), 2)
        ex, default = ex.args[1], Some(ex.args[2])
    end
    type = nothing
    if isexpr(ex, :(::), 2)
        ex, type = ex.args
    end
    return ex, type, default
end

rosfieldnames(x::Symbol, ::LineNumberNode) = [x]
function rosfieldnames(x, source::LineNumberNode)
    isexpr(x, :tuple) && length(x.args) >= 2 && all(a -> a isa Symbol, x.args) || throw(syntax_error(source))
    return Symbol[x.args...]
end

"""
    JuliaField(name, type, default, rosfields)

Describe a field of a generated Julia type: its name, type and default expression, and the range of ROS fields it covers.
"""
struct JuliaField
    name::Symbol
    type::Any
    default::Any
    rosfields::UnitRange{Int}
end

# The expression of the type `T` in a module that imports ROS types.
typeexpr(T) = parentmodule(T) === ROSMessages ? Expr(:., :ROSMessages, QuoteNode(nameof(T))) : nameof(T)

"""
    juliatype(T::Type, reference)

Return the expression of the Julia type of a field of type `T` (see [`parse_type`](@ref)), calling `reference(name::MsgName)` for the Julia type of a nested message.

It's important to use an `SVector` and not an `MVector` for fixed-size arrays even in mutable messages, as an `MVector` is a mutable struct, which Julia stores as a pointer instead of inline as C does.

```jldoctest
julia> ROSMessages.juliatype(ROSMessages.parse_type("float64[9]", ""), identity)
:(ROSMessages.SVector{9, Float64})

julia> ROSMessages.juliatype(ROSMessages.parse_type("Header[]", "std_msgs"), name -> Symbol(name.name))
:(ROSMessages.Sequence{Header})
```
"""
juliatype(T::Type, _) = typeexpr(T)
juliatype(T::Type{<:Nested}, reference) = reference(msgname(T))
juliatype(::Type{SVector{N, T}}, reference) where {N, T} = :(ROSMessages.SVector{$N, $(juliatype(T, reference))})
juliatype(::Type{Sequence{T}}, reference) where T = :(ROSMessages.Sequence{$(juliatype(T, reference))})

"""
    juliadefault(T::Type, default, reference)

Return the default expression of a field of type `T` with the parsed ROS default `default` (`nothing` if none), calling `reference` like [`juliatype`](@ref).

Strings and sequences start empty, because their memory may only be allocated by C, which the C `init` function of the message ([`init!`](@ref)) does when it applies their ROS defaults. Every value needs a default, as the C library reallocates a string or sequence based on its size and capacity.

```jldoctest
julia> ROSMessages.juliadefault(ROSMessages.parse_type("int16[2]", ""), Int16[3, -1], identity)
:(ROSMessages.SVector{2, Int16}(3, -1))

julia> ROSMessages.juliadefault(ROSMessages.parse_type("string", ""), "ignored", identity)
:(ROSMessages.ROSString())

julia> ROSMessages.juliadefault(ROSMessages.parse_type("Point[2]", "geometry_msgs"), nothing, name -> Symbol(name.name))
:(fill(Point(), ROSMessages.SVector{2, Point}))
```
"""
juliadefault(T::Type, _, reference) = :($(juliatype(T, reference))())
juliadefault(::Type{T}, default, _) where T <: Real = something(default, zero(T))
function juliadefault(::Type{SVector{N, T}}, default, reference) where {N, T}
    type = juliatype(SVector{N, T}, reference)
    T <: Real || return :(fill($(juliadefault(T, nothing, reference)), $type))
    return isnothing(default) ? :(zero($type)) : :($type($(default...)))
end

"""
    apply_overrides(def::Definition, overrides, reference)

Return the [`JuliaField`](@ref)s of `def` with `overrides` applied, and the pairs of overridden constants and their overrides.

`reference(name::MsgName)` returns the expression of the Julia type of a nested message, defining it if needed. It is only called for fields whose type is not overridden.

```jldoctest
julia> def = ROSMessages.parse_definition("int32 X = 1\\nfloat64 latitude\\nfloat64 longitude\\nuint8 status", ROSMessages.MsgName("p", "msg", "Fix"));

julia> overrides = ROSMessages.Override.([:(@override X as Y), :(@override (latitude, longitude) as position::Position)]);

julia> fields, constants = ROSMessages.apply_overrides(def, overrides, identity);

julia> [(f.name, f.type, f.default, f.rosfields) for f in fields]
2-element Vector{Tuple{Symbol, Symbol, Any, UnitRange{Int64}}}:
 (:position, :Position, :(Position()), 1:2)
 (:status, :UInt8, 0x00, 3:3)

julia> ROSMessages.apply_overrides(def, reverse(overrides), identity)
ERROR: ArgumentError: none:1: Overrides need to follow the order of p/msg/Fix, but `X` comes before the previous override.
[...]
```
"""
function apply_overrides(def::Definition, overrides::Vector{Override}, reference)
    fieldoverrides = Dict{Symbol, Override}()
    constants = Tuple{Constant, Override}[]
    previous = -1
    for o in overrides
        name = first(o.rosnames)
        i = findfirst(f -> f.name === name, def.fields)
        c = findfirst(c -> c.name === name, def.constants)
        line = if isnothing(i)
            isnothing(c) && throw(override_error(o.source, "`$name` is neither a field nor a constant of $(def.name)."))
            def.constants[c].line
        else
            def.fields[i].line
        end
        line > previous || throw(override_error(o.source, "Overrides need to follow the order of $(def.file), but `$name` comes before the previous override."))
        if isnothing(c)
            o.target isa Expr && throw(override_error(o.source, "Only constants can be renamed to a qualified name, but `$name` is a field."))
            stop = i + length(o.rosnames) - 1
            stop <= length(def.fields) && [f.name for f in def.fields[i:stop]] == o.rosnames ||
                throw(override_error(o.source, "Fields overridden together need to be consecutive in $(def.file)."))
            length(o.rosnames) == 1 || !isnothing(o.type) ||
                throw(override_error(o.source, "Fields overridden together need a type."))
            fieldoverrides[name] = o
            line = def.fields[stop].line
        else
            length(o.rosnames) == 1 || throw(override_error(o.source, "Only fields can be overridden together, but `$name` is a constant."))
            isnothing(o.default) || throw(override_error(o.source, "The value of constant `$name` cannot be overridden."))
            push!(constants, (def.constants[c], o))
        end
        previous = line
    end

    fields = JuliaField[]
    i = 1
    while i <= length(def.fields)
        f = def.fields[i]
        o = get(fieldoverrides, f.name, Override([f.name], f.name, nothing, nothing, nothing, LineNumberNode(f.line)))
        if isnothing(o.type)
            type, default = juliatype(f.type, reference), juliadefault(f.type, f.default, reference)
        else
            type, default = o.type, :($(o.type)())
        end
        push!(fields, JuliaField(o.juliafield, type, something(o.default, default), i:i + length(o.rosnames) - 1))
        i += length(o.rosnames)
    end
    return fields, constants
end
