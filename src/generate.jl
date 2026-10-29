"""
    rosname(T::Type)

Return the ROS name of the message type `T`.

`@importrosmsg` defines a method for each type it generates, so the ROS name identifies a Julia type independently of its Julia name.

```jldoctest
julia> @importrosmsg std_msgs/Header as Stamped

julia> ROSMessages.rosname(Stamped)
"std_msgs/msg/Header"

julia> ROSMessages.rosname(Empty)
"std_msgs/msg/Empty"
```
"""
function rosname end
rosname(::Type{Empty}) = "std_msgs/msg/Empty"

"""
    typesupport(T::Type)

Return the type support handle of the C library for the message type `T`, which `rcl` needs to create publishers and subscriptions.

```jldoctest
julia> @importrosmsg std_msgs/Header

julia> ROSMessages.typesupport(Header) != C_NULL
true
```
"""
function typesupport end

"""
    service_typesupport(Request::Type)

Return the type support handle of the C library for the ROS service whose request type is `Request`.

```jldoctest
julia> @importrosmsg example_interfaces/srv/AddTwoInts

julia> ROSMessages.service_typesupport(AddTwoInts_Request) != C_NULL
true
```
"""
function service_typesupport end

"""
    init!(x)

Initialize the mutable message `x` with the C `init` function of its ROS type, which allocates its strings and sequences and applies the ROS defaults.

```jldoctest
julia> @importrosmsg mutable std_msgs/String as Str

julia> x = Str(); ROSMessages.init!(x); (x.data.size, x.data.capacity)
(0x0000000000000000, 0x0000000000000001)
```
"""
function init! end

"""
    fini!(x)

Free the memory of the mutable message `x` that C allocated, with the C `fini` function of its ROS type.

```jldoctest
julia> @importrosmsg mutable std_msgs/String as Str

julia> x = Str(); ROSMessages.init!(x); ROSMessages.fini!(x); x.data.capacity
0x0000000000000000
```
"""
function fini! end

const IMPORT_SYNTAX = """
    @importrosmsg [mutable|immutable] ros_package/[msg/|srv/]MsgName [as TypeName[{T, …}][ <: Supertype]] [in "file.jl"] [begin
    $(indent(OVERRIDE_SYNTAX))
        …
    end]"""

"""
    TypeSpec(name, typename, mutable, params, supertype, overrides)
    TypeSpec(name::MsgName)

Describe the Julia type to define for the ROS type `name`: its name with type parameters and supertype, its mutability and the overrides.

The second form describes a type nested in another message: immutable, named like the message and without overrides.
"""
struct TypeSpec
    name::MsgName
    typename::Symbol
    mutable::Bool
    params::Vector{Any}
    supertype::Any
    overrides::Vector{Override}
end
TypeSpec(name::MsgName) = TypeSpec(name, Symbol(name.name), false, Any[], nothing, Override[])

# The request and response of the service described by `t`.
parts(t::TypeSpec) = map(("_Request", "_Response")) do suffix
    return TypeSpec(MsgName(t.name.package, "srv", t.name.name * suffix), Symbol(t.typename, suffix), t.mutable, Any[], nothing, Override[])
end

"""
    ImportSpec(type::TypeSpec, file, source)

Hold what an `@importrosmsg` call requests: the type (the service itself for a `srv`), the absolute path of the file to generate (`nothing` to define the types directly), and the file of the call.
"""
struct ImportSpec
    type::TypeSpec
    file::Union{String, Nothing}
    source::String
end

"""
    ImportSpec(args::Tuple, source::LineNumberNode)

Parse the arguments `args` of an `@importrosmsg` call at `source`.

```jldoctest
julia> spec = ROSMessages.ImportSpec((:immutable, :(sensor_msgs/PointCloud2), :as, :(PointCloud2{T} <: AbstractVector{T})), LineNumberNode(1, :none));

julia> spec.type.name, spec.type.mutable, spec.type.typename, spec.type.params, spec.type.supertype
(ROSMessages.MsgName("sensor_msgs", "msg", "PointCloud2"), false, :PointCloud2, Any[:T], :(AbstractVector{T}))

julia> ROSMessages.ImportSpec((:(std_msgs/Header), :as, :(Stamped in "gen/Stamped.jl")), LineNumberNode(1, Symbol("/src/messages.jl"))).file
"/src/gen/Stamped.jl"

julia> ROSMessages.ImportSpec((:(std_msgs/Header), :in, "gen/Header.jl"), LineNumberNode(1, :none))
ERROR: ArgumentError: none:1: `@importrosmsg` needs the form
[...]
```
"""
function ImportSpec(args::Tuple, source::LineNumberNode)
    syntax() = ArgumentError("$(source.file):$(source.line): `@importrosmsg` needs the form\n$IMPORT_SYNTAX")
    args = Any[args...]
    isempty(args) && throw(syntax())
    # Explicit imports are mutable by default, as they typically type a message, which C writes in place.
    mutable = true
    if args[1] isa Symbol
        args[1] in (:mutable, :immutable) || throw(syntax())
        mutable = popfirst!(args) === :mutable
    end
    isempty(args) && throw(syntax())
    ros, file = split_file(popfirst!(args), source)
    name = parse_rosname(ros, syntax)
    typename, params, supertype = Symbol(name.name), Any[], nothing
    if !isempty(args) && args[1] === :as
        popfirst!(args)
        !isempty(args) && isnothing(file) || throw(syntax())
        target, file = split_file(popfirst!(args), source)
        typename, params, supertype = parse_typename(target, syntax)
    end
    overrides = Override[]
    if !isempty(args)
        block = popfirst!(args)
        isexpr(block, :block) && isempty(args) || throw(syntax())
        overrides = [Override(x) for x in block.args if !(x isa LineNumberNode)]
    end
    caller = string(source.file)
    path = isnothing(file) ? nothing : normpath(joinpath(dirname(caller), file))
    return ImportSpec(TypeSpec(name, typename, mutable, params, supertype, overrides), path, caller)
end

# Split `x in "file.jl"`, which also parses as a comparison chain after `T <: S`.
function split_file(x, source::LineNumberNode)
    isexpr(x, :call, 3) && x.args[1] === :in && return x.args[2], generatedpath(x.args[3], source)
    isexpr(x, :comparison, 5) && x.args[2] === :<: && x.args[4] === :in && return Expr(:<:, x.args[1], x.args[3]), generatedpath(x.args[5], source)
    return x, nothing
end

function generatedpath(x, source::LineNumberNode)
    x isa String || throw(ArgumentError("$(source.file):$(source.line): `in` needs to be followed by a literal file name."))
    return x
end

function parse_rosname(x, syntax)
    parts = Any[]
    while isexpr(x, :call, 3) && x.args[1] === :/
        pushfirst!(parts, x.args[3])
        x = x.args[2]
    end
    pushfirst!(parts, x)
    length(parts) in (2, 3) && all(p -> p isa Symbol, parts) || throw(syntax())
    interface = length(parts) == 3 ? string(parts[2]) : "msg"
    interface in ("msg", "srv") || throw(ArgumentError("The interface kind needs to be `msg` or `srv`, not `$interface`."))
    checkpackage(string(parts[1]))
    checkmessage(string(parts[end]))
    return MsgName(string(parts[1]), interface, string(parts[end]))
end

function parse_typename(x, syntax)
    supertype = nothing
    if isexpr(x, :<:, 2)
        x, supertype = x.args
    end
    x isa Symbol && return x, Any[], supertype
    isexpr(x, :curly) && x.args[1] isa Symbol || throw(syntax())
    return x.args[1], x.args[2:end], supertype
end

"""
    ImportContext(caller::Module, source = nothing)

Collect the code generated for one import into `caller`, retaining its source location for nested macro calls.
"""
struct ImportContext
    caller::Module
    source::Union{LineNumberNode, Nothing}
    exprs::Vector{Any}
    definitions::Dict{MsgName, Definition}
    defined::Vector{MsgName}
    bindings::Set{Symbol}
end
ImportContext(caller::Module, source::Union{LineNumberNode, Nothing} = nothing) =
    ImportContext(caller, source, Any[], Dict{MsgName, Definition}(), MsgName[], Set{Symbol}())

definition!(s::ImportContext, name::MsgName) = get!(() -> read_definition(name), s.definitions, name)

isrostype(T, name::MsgName) = T isa Type && applicable(rosname, T) && rosname(T) == string(name)

"""
    imported(caller::Module, name::MsgName, mutable::Bool)

Return the expression of the type with this mutability that an earlier import defined in `caller` for the ROS type `name`, under any Julia name, or `nothing`.

The `rosname` methods record the imports, so precompiled modules keep them. A type that ROSMessages predefines, like `Empty`, counts as imported. Imports are top-level statements, so every earlier import is defined when the next one is expanded.

```jldoctest
julia> @importrosmsg immutable std_msgs/Header as Stamped

julia> ROSMessages.imported(@__MODULE__, ROSMessages.MsgName("std_msgs", "msg", "Header"), false)
:Stamped

julia> isnothing(ROSMessages.imported(@__MODULE__, ROSMessages.MsgName("std_msgs", "msg", "Header"), true))
true

julia> ROSMessages.imported(@__MODULE__, ROSMessages.MsgName("std_msgs", "msg", "Empty"), false)
:(ROSMessages.Empty)
```
"""
function imported(caller::Module, name::MsgName, mutable::Bool)
    for m in methods(rosname)
        sig = Base.unwrap_unionall(m.sig)
        length(sig.parameters) == 2 || continue
        P = Base.unwrap_unionall(sig.parameters[2])
        Base.isType(P) || continue
        T = P.parameters[1]
        T isa TypeVar && (T = T.ub)
        T isa Union{DataType, UnionAll} && parentmodule(T) in (caller, ROSMessages) || continue
        ismutabletype(Base.unwrap_unionall(T)) == mutable && rosname(T) == string(name) || continue
        parentmodule(T) === ROSMessages || isdefined(caller, nameof(T)) && getglobal(caller, nameof(T)) === T || continue
        return typeexpr(T)
    end
    return nothing
end

"""
    reference!(s::ImportContext, name::MsgName)

Return the expression of the Julia type of the nested message `name`, defining it as an immutable type named like the message if the caller has no immutable type for it yet.

A nested type needs to be immutable, as C stores it inline, which Julia only does for immutable types.
"""
function reference!(s::ImportContext, name::MsgName)
    # Only nested types are referenced, which are named like the message, as ROS messages cannot contain themselves.
    name in s.defined && return Symbol(name.name)
    existing = imported(s.caller, name, false)
    isnothing(existing) || return existing
    typename = Symbol(name.name)
    if isdefined(s.caller, typename)
        T = getglobal(s.caller, typename)
        isrostype(T, name) && throw(ArgumentError("$name is nested in another message, so it needs an immutable type, but `$typename` is its mutable import. Import it a second time with `@importrosmsg immutable $(name.package)/$(name.interface)/$(name.name) as OtherName` before."))
        throw(ArgumentError("$name would be defined as `$typename`, but `$typename` already refers to $T in $(s.caller). Import $name first under another name with `@importrosmsg immutable $(name.package)/$(name.interface)/$(name.name) as OtherName`."))
    end
    return define!(s, TypeSpec(name))
end

"""
    import_type!(s::ImportContext, t::TypeSpec)

Define the Julia type described by `t` unless the caller has it already, and return its name.
"""
function import_type!(s::ImportContext, t::TypeSpec)
    (; name, typename, mutable) = t
    isdefined(s.caller, typename) || return define!(s, t)
    T = getglobal(s.caller, typename)
    isrostype(T, name) || throw(ArgumentError("`$typename` already refers to $T in $(s.caller), so $name cannot be imported under this name. Import it under another name with `as`."))
    ismutabletype(Base.unwrap_unionall(T)) == mutable || throw(ArgumentError(string("`$typename` already is the ", mutable ? "immutable" : "mutable", " import of $name. Import $name ",
        mutable ? "mutable" : "immutable", " a second time under another name with `as`.")))
    return typename
end

function import!(s::ImportContext, t::TypeSpec)
    t.name.interface == "msg" && return import_type!(s, t)
    isempty(t.overrides) && isempty(t.params) && isnothing(t.supertype) ||
        throw(ArgumentError("Overrides, type parameters and supertypes are not supported for services, yet."))
    return map(p -> import_type!(s, p), parts(t))
end

function define!(s::ImportContext, t::TypeSpec)
    # `typecode` defines the nested types first, so their code precedes this type's code.
    t.typename in s.bindings && throw(ArgumentError("`$(t.typename)` is already defined by this import."))
    push!(s.bindings, t.typename)
    append!(s.exprs, typecode(s, t))
    push!(s.defined, t.name)
    return t.typename
end

function ccall_expr(library::String, symbol::String, args::Vector, rettype)
    call = Expr(:call, Expr(:., library, QuoteNode(Symbol(symbol))), args...)
    return Expr(:macrocall, Expr(:., :Base, QuoteNode(Symbol("@ccall"))), nothing, Expr(:(::), call, rettype))
end

# Keep the type of a constant, which a plain number literal would lose.
literal(value::String, ::Type) = value
literal(value, T::Type) = :($(typeexpr(T))($value))

function type_binding(caller::Module, x)
    if x isa Symbol
        isdefined(caller, x) && return getglobal(caller, x)
        isdefined(Base, x) && return getfield(Base, x)
        isdefined(Core, x) && return getfield(Core, x)
    elseif x isa QuoteNode
        return x.value
    elseif x isa GlobalRef
        return getfield(x.mod, x.name)
    elseif isexpr(x, :., 2) && x.args[2] isa QuoteNode
        parent = type_binding(caller, x.args[1])
        parent isa Module && isdefined(parent, x.args[2].value) && return getfield(parent, x.args[2].value)
    end
    return nothing
end

const ENUMX_INTEGER_TYPES = (Int8, UInt8, Int16, UInt16, Int32, UInt32, Int64, UInt64, Int128, UInt128)
is_enumx_integer_type(T) = T isa DataType && T in ENUMX_INTEGER_TYPES

function enum_type(caller::Module, override_type, original_type::Type)
    T = isnothing(override_type) ? original_type : type_binding(caller, override_type)
    return is_enumx_integer_type(T) ? T : nothing
end

function constant_enum_groups(properties, types, values)
    candidates = Dict{String, Vector{Int}}()
    for (i, property) in enumerate(properties)
        parts = split(string(first(property)), '_')
        for n in 1:length(parts)-1
            prefix = join(parts[1:n], "_")
            push!(get!(candidates, prefix, Int[]), i)
        end
    end

    prefixes = sort!(collect(keys(candidates)); by = p -> (-count(==('_'), p), -length(p), p))
    assigned = Set{Int}()
    groups = NamedTuple[]
    for prefix in prefixes
        members = filter(i -> i ∉ assigned, candidates[prefix])
        length(members) >= 2 || continue
        T = types[first(members)]
        is_enumx_integer_type(T) || continue
        all(i -> types[i] === T, members) || continue

        prefix_length = length(split(prefix, '_'))
        variants = Pair{Symbol, Any}[]
        for i in members
            suffix = Symbol(join(split(string(first(properties[i])), '_')[prefix_length+1:end], '_'))
            value = values[i]
            value isa Integer && typemin(T) <= value <= typemax(T) || break
            push!(variants, suffix => convert(T, value))
        end
        length(variants) == length(members) || continue
        allunique(first.(variants)) || continue

        push!(groups, (prefix = Symbol(prefix), type = T, members = members, variants = variants))
        union!(assigned, members)
    end
    return groups
end

function enum_message end

enumx_macro() = Expr(:., Expr(:., :ROSMessages, QuoteNode(:EnumX)), QuoteNode(Symbol("@enumx")))
is_enum_code(ex) = isexpr(ex, :macrocall, 5) && ex.args[1] == enumx_macro()

function enum_code(s::ImportContext, typename::Symbol, def::Definition, group)
    prefix = group.prefix
    prefix in s.bindings &&
        throw(ArgumentError("EnumX enum `$prefix` for $(def.name) collides with another binding in $(s.caller)."))
    type_name = :T
    while type_name in first.(group.variants)
        type_name = Symbol(type_name, "_")
    end
    if isdefined(s.caller, prefix)
        existing = getglobal(s.caller, prefix)
        if existing isa Module && nameof(existing) === prefix && parentmodule(existing) === s.caller &&
                isdefined(existing, type_name)
            existing_type = getfield(existing, type_name)
            # Unlike the value-to-name map, symbol_map retains aliases and their order.
            if existing_type isa DataType && existing_type <: Base.Enum && parentmodule(existing_type) === existing &&
                    applicable(enum_message, existing_type) && applicable(EnumX.symbol_map, existing_type) &&
                    isrostype(enum_message(existing_type), def.name) &&
                    Base.Enums.basetype(existing_type) === group.type &&
                    EnumX.symbol_map(existing_type) == group.variants
                push!(s.bindings, prefix)
                return Any[]
            end
        end
        throw(ArgumentError("EnumX enum `$prefix` for $(def.name) collides with an existing binding in $(s.caller)."))
    end
    push!(s.bindings, prefix)
    entries = Expr(:block, (Expr(:(=), name, value) for (name, value) in group.variants)...)
    source = if isnothing(s.source)
        constant = def.constants[first(group.members)]
        LineNumberNode(constant.line, Symbol(def.file))
    else
        s.source
    end
    definition = Expr(:macrocall, enumx_macro(), source,
        Expr(:(=), :T, type_name),
        Expr(:(::), prefix, typeexpr(group.type)),
        entries)
    enum_type = Expr(:., prefix, QuoteNode(type_name))
    message_definition = Expr(:(=),
        Expr(:call, Expr(:., :ROSMessages, QuoteNode(:enum_message)),
            Expr(:(::), Expr(:curly, :Type, enum_type))),
        typename)
    return Any[definition, message_definition]
end

"""
    constantcode(typename::Symbol, def::Definition, constants)

Return the top-level expressions defining enum groups and the remaining constants of `def` as properties of the type `typename`, with the overridden `constants` (pairs of [`Constant`](@ref) and [`Override`](@ref)) applied.

Without `as` or with `as TypeName.NAME`, an override changes the property. With `as NAME`, it defines the module constant `NAME` and keeps the property.

```jldoctest
julia> def = ROSMessages.parse_definition("uint8 A = 1\\nuint8 B = 2\\nuint8 C = 3\\nuint8 x", ROSMessages.MsgName("p", "msg", "M"));

julia> constants = [(def.constants[i], ROSMessages.Override(o)) for (i, o) in ((1, :(@override A as M.Alpha::Flag)), (3, :(@override C as GAMMA)))];

julia> foreach(ex -> (ROSMessages.printcode(stdout, ex); println()), ROSMessages.constantcode(:M, def, constants))
function Base.getproperty(T::Type{<:M}, s::Symbol)
    s === :Alpha && return Flag(UInt8(0x01))
    s === :B && return UInt8(0x02)
    s === :C && return UInt8(0x03)
    return getfield(T, s)
end
const global GAMMA = UInt8(0x03)

julia> ROSMessages.constantcode(:M, def, [(def.constants[1], ROSMessages.Override(:(@override A as Other.Alpha)))])
ERROR: ArgumentError: none:1: The qualifier of `Other.Alpha` needs to be the imported type `M`.
[...]
```
"""
function constantcode(s::ImportContext, typename::Symbol, def::Definition, constants)
    properties = Pair{Symbol, Any}[k.name => literal(k.value, k.type) for k in def.constants]
    property_types = Any[k.type for k in def.constants]
    property_values = Any[k.value for k in def.constants]
    code = Any[]
    for (k, o) in constants
        value = isnothing(o.type) ? literal(k.value, k.type) : :($(o.type)($(literal(k.value, k.type))))
        if o.target isa Symbol
            push!(s.bindings, o.target)
            push!(code, :(global const $(o.target) = $value))
        else
            isnothing(o.target) || o.target.args[1] === typename ||
                throw(override_error(o.source, "The qualifier of `$(o.target)` needs to be the imported type `$typename`."))
            i = findfirst(p -> first(p) === k.name, properties)
            properties[i] = o.juliafield => value
            property_types[i] = enum_type(s.caller, o.type, k.type)
        end
    end
    names = first.(properties)
    allunique(names) || throw(ArgumentError("The constants of $(def.name) would have the duplicate names $(unique(filter(n -> count(==(n), names) > 1, names)))."))
    groups = constant_enum_groups(properties, property_types, property_values)
    grouped = Set{Int}()
    for group in groups
        append!(code, enum_code(s, typename, def, group))
        union!(grouped, group.members)
    end
    properties = [property for (i, property) in enumerate(properties) if !(i in grouped)]
    isempty(properties) && return code
    checks = [:(s === $(QuoteNode(n)) && return $v) for (n, v) in properties]
    pushfirst!(code, Expr(:function, :(Base.getproperty(T::Type{<:$typename}, s::Symbol)), Expr(:block, checks..., :(return getfield(T, s)))))
    return code
end

function constantcode(typename::Symbol, def::Definition, constants)
    s = ImportContext(ROSMessages)
    push!(s.bindings, typename)
    return constantcode(s, typename, def, constants)
end

"""
    ccode(name::MsgName, typename, mutable::Bool)

Return the top-level expressions defining [`rosname`](@ref), [`typesupport`](@ref), [`service_typesupport`](@ref) for a service request, and [`init!`](@ref) and [`fini!`](@ref) for a mutable type, for the Julia type `typename` of the ROS type `name`.

```jldoctest
julia> code = ROSMessages.ccode(ROSMessages.MsgName("std_msgs", "msg", "Header"), :Header, false);

julia> ROSMessages.printcode(stdout, code[1])
ROSMessages.rosname(::Type{<:Header}) = "std_msgs/msg/Header"

julia> length(code), length(ROSMessages.ccode(ROSMessages.MsgName("std_msgs", "msg", "Header"), :Header, true))
(2, 4)
```
"""
function ccode(name::MsgName, typename, mutable::Bool)
    (; package, interface) = name
    code = Any[:(ROSMessages.rosname(::Type{<:$typename}) = $(string(name)))]
    c = "$(package)__$(interface)__$(name.name)"
    typesupport_lib = "lib$(package)__rosidl_typesupport_c"
    push!(code, :(ROSMessages.typesupport(::Type{<:$typename}) = $(ccall_expr(typesupport_lib, "rosidl_typesupport_c__get_message_type_support_handle__$c", [], :(Ptr{Cvoid})))))
    if interface == "srv" && endswith(name.name, "_Request")
        service = chop(name.name; tail = length("_Request"))
        push!(code, :(ROSMessages.service_typesupport(::Type{<:$typename}) = $(ccall_expr(typesupport_lib, "rosidl_typesupport_c__get_service_type_support_handle__$(package)__srv__$service", [], :(Ptr{Cvoid})))))
    end
    # Only a mutable message has a fixed address, which the C functions need to modify it in place.
    if mutable
        generator_lib = "lib$(package)__rosidl_generator_c"
        init = ccall_expr(generator_lib, "$(c)__init", [:(x::Ref{T})], :Bool)
        push!(code, :(ROSMessages.init!(x::T) where T <: $typename = $init || error($("$(c)__init failed"))))
        fini = ccall_expr(generator_lib, "$(c)__fini", [:(x::Ref{T})], :Cvoid)
        push!(code, :(ROSMessages.fini!(x::T) where T <: $typename = $fini))
    end
    return code
end

"""
    bindtype(caller::Module, T::Type, package, interface, name)

Define the C functions of the ROS type `package/interface/name` for the manually defined type `T`, unless `T` has them already.

`T` needs the memory layout of the C struct of the ROS type, which is not checked.

```jldoctest
julia> mutable struct Color; r::Float32; g::Float32; b::Float32; a::Float32; end

julia> ROSMessages.bindtype(@__MODULE__, Color, "std_msgs", "msg", "ColorRGBA"); ROSMessages.rosname(Color)
"std_msgs/msg/ColorRGBA"

julia> ROSMessages.bindtype(@__MODULE__, Color, "std_msgs", "msg", "Header")
ERROR: ArgumentError: `Color` already is the Julia type of std_msgs/msg/ColorRGBA, not of std_msgs/msg/Header.
[...]
```
"""
function bindtype(caller::Module, T::Type, package::AbstractString, interface::AbstractString, name::AbstractString)
    ros = MsgName(package, interface, name)
    if applicable(rosname, T)
        rosname(T) == string(ros) || throw(ArgumentError("`$T` already is the Julia type of $(rosname(T)), not of $ros."))
        return nothing
    end
    Core.eval(caller, Expr(:toplevel, resolved(ccode(ros, T, ismutabletype(T)))...))
    return nothing
end

"""
    typecode(s::ImportContext, t::TypeSpec)

Return the top-level expressions defining the Julia type described by `t`: the struct, equality, hashing, the ROS name, the C functions, the constants and the layout check.

Defining the struct references its nested types, which `s` defines first if needed.
"""
function typecode(s::ImportContext, t::TypeSpec)
    (; name, typename, mutable, params, supertype) = t
    def = definition!(s, name)
    fields, constants = apply_overrides(def, t.overrides, n -> reference!(s, n))
    layout = clayout(def, n -> definition!(s, n))
    T = isempty(params) ? typename : Expr(:curly, typename, params...)
    head = isnothing(supertype) ? T : Expr(:<:, T, supertype)
    body = Expr(:block, (:($(f.name)::$(f.type) = $(f.default)) for f in fields)...)
    kwdef = mutable ? :(ROSMessages.var"@kwdef!") : :(Base.var"@kwdef")
    code = Any[Expr(:macrocall, kwdef, nothing, Expr(:struct, mutable, head, body))]
    push!(code, :(Base.:(==)(x::$typename, y::$typename) = ROSMessages.fieldvalues(x) == ROSMessages.fieldvalues(y)))
    push!(code, :(Base.isequal(x::$typename, y::$typename) = isequal(ROSMessages.fieldvalues(x), ROSMessages.fieldvalues(y))))
    push!(code, :(Base.hash(x::$typename, h::UInt) = hash(ROSMessages.fieldvalues(x), hash($(string(name)), h))))
    mutable && push!(code, :(function Base.copy!(dst::$typename, src::$typename)
        foreach(i -> setfield!(dst, i, getfield(src, i)), 1:fieldcount(typeof(dst)))
        return dst
    end))
    append!(code, ccode(name, typename, mutable))
    append!(code, constantcode(s, typename, def, constants))
    concrete = isempty(params) ? typename : Expr(:curly, typename, (isexpr(p, :<:, 2) ? p.args[2] : :Any for p in params)...)
    expected = map(fields) do f
        start, stop = first(f.rosfields), last(f.rosfields)
        rosfields = join((field.name for field in def.fields[f.rosfields]), ", ")
        start_layout = layout.fields[start]
        stop_layout = layout.fields[stop]
        item = Expr(:tuple, start_layout.offset, stop_layout.offset + stop_layout.size - start_layout.offset)
        rosfields == string(f.name) || push!(item.args, rosfields)
        return item
    end
    push!(code, :(ROSMessages.check_layout($concrete, $(Expr(:tuple, expected...)), $(layout.alignment))))
    return code
end

"""
    resolve(ex)

Replace each `ROSMessages.name` in `ex` with a `GlobalRef`, so that generated code works in modules without a binding `ROSMessages`.

```jldoctest
julia> ROSMessages.resolve(:(x::ROSMessages.ROSString))
:(x::ROSMessages.ROSString)

julia> ROSMessages.resolve(:(ROSMessages.ROSString)) isa GlobalRef
true
```
"""
function resolve(ex::Expr)
    isexpr(ex, :., 2) && ex.args[1] === :ROSMessages && ex.args[2] isa QuoteNode && return GlobalRef(ROSMessages, ex.args[2].value)
    return Expr(ex.head, Any[resolve(a) for a in ex.args]...)
end
resolve(x) = x

resolved(exprs) = Any[resolve(Base.remove_linenums!(ex)) for ex in exprs]

"""
    importrosmsg(caller::Module, package, interface, name; mutable = true, params = [], supertype = nothing)

Define the Julia type of a ROS message, or the request and response types of a ROS service, in `caller` unless they exist already, and return their names.

This is the function form of [`@importrosmsg`](@ref) for macros of other packages, which import the types while they are expanded. It returns the types that earlier imports defined for the ROS type with this mutability under any name, and names new types like the ROS type.

```jldoctest
julia> @importrosmsg std_msgs/ColorRGBA as Color

julia> ROSMessages.importrosmsg(@__MODULE__, "std_msgs", "msg", "ColorRGBA")
:Color

julia> ROSMessages.importrosmsg(@__MODULE__, "example_interfaces", "srv", "Trigger")
(:Trigger_Request, :Trigger_Response)
```
"""
function importrosmsg(caller::Module, package::AbstractString, interface::AbstractString, name::AbstractString;
                      mutable::Bool = true, params::Vector = Any[], supertype = nothing)
    syntax() = ArgumentError("`$package/$interface/$name` is no valid ROS type.")
    ros = parse_rosname(:($(Symbol(package))/$(Symbol(interface))/$(Symbol(name))), syntax)
    t = TypeSpec(ros, Symbol(name), mutable, params, supertype, Override[])
    existing = map(p -> imported(caller, p.name, mutable), interface == "msg" ? (t,) : parts(t))
    all(!isnothing, existing) && return interface == "msg" ? only(existing) : existing
    s = ImportContext(caller)
    names = import!(s, t)
    isempty(s.exprs) || Core.eval(caller, Expr(:toplevel, resolved(s.exprs)...))
    return names
end

"""
    @importrosmsg ros_package/MsgName
$(indent(IMPORT_SYNTAX))

Define a Julia struct with the memory layout of the C struct of a ROS message, read from its `.msg` file.

The type is mutable unless `immutable` is given, because a message needs a fixed address for C to read and write it. A type contained in another message needs to be immutable, as C stores it inline. A service (`srv/`) defines the two types `TypeName_Request` and `TypeName_Response`. Nested message types are defined as immutable types named like the message, unless the module has an immutable type for them already. `@kwdef` provides keyword and zero-argument constructors with the ROS defaults; a mutable type additionally gets a reconstructor (see [`@kwdef!`](@ref)) and `copy!`.

With `in "file.jl"`, the struct and enum declarations are written to that file, relative to the calling file, whenever they change. The macro defines the types and methods itself; the generated file is not included.

`@importrosmsg` needs to be a top-level statement of its own, not part of a function, a `let` block or another expression: types, parameters, supertypes and overrides can only use global names, and nested types are found among the types defined by earlier statements.

```jldoctest
julia> @importrosmsg geometry_msgs/Vector3 as Vec begin
           @override (x, y, z) as xyz::NTuple{3, Float64} = (0.0, 0.0, 0.0)
       end

julia> ismutabletype(Vec), Vec()
(true, Vec((0.0, 0.0, 0.0)))

julia> @importrosmsg immutable geometry_msgs/Vector3 as Vec3 begin
           @override x::Float32
       end
ERROR: ArgumentError: field `x` of `Vec3` (ROS geometry_msgs/msg/Vector3: x) has type Float32 of 4 bytes instead of 8 bytes.
[...]
```
"""
macro importrosmsg(args...)
    spec = ImportSpec(args, __source__)
    s = ImportContext(__module__, __source__)
    import!(s, spec.type)
    isempty(s.exprs) && return nothing
    isnothing(spec.file) || writefile(s, spec)
    return esc(Expr(:block, resolved(s.exprs)..., nothing))
end

# Write the generated code to the file of `spec` unless the file has this content already, which keeps its mtime for tools watching it.
function writefile(s::ImportContext, spec::ImportSpec)
    text = render(s, spec)
    isfile(spec.file) && read(spec.file, String) == text && return nothing
    mkpath(dirname(spec.file))
    write(spec.file, text)
    return nothing
end

# Drop the `begin` `end` around single-expression bodies, which `print` would write for `f(x) = y` and `x -> y`.
function tidy(ex::Expr)
    args = Any[tidy(a) for a in ex.args]
    ex.head in (:(=), :->) && isexpr(args[2], :block, 1) && (args[2] = only(args[2].args))
    return Expr(ex.head, args...)
end
tidy(x) = x

# Print structs and enum blocks inside macro calls with the usual indentation, which `print` would double.
function printcode(io::IO, ex)
    ex = tidy(Base.remove_linenums!(ex))
    if ex isa Expr && ex.head === :macrocall && length(ex.args) >= 2 && ex.args[2] isa LineNumberNode
        # The parser assigns the generated file's source location when it is included.
        ex.args[2] = nothing
    end
    if is_enum_code(ex)
        print(io, Expr(:macrocall, ex.args[1:4]...), " begin")
        foreach(entry -> print(io, "\n    ", entry), ex.args[5].args)
        return print(io, "\nend")
    end
    isexpr(ex, :macrocall, 3) && isexpr(ex.args[3], :struct) || return print(io, ex)
    (mutable, head, body) = ex.args[3].args
    name = ex.args[1]
    print(io, isexpr(name, :., 2) ? "$(name.args[1]).$(name.args[2].value)" : name, mutable ? " mutable struct " : " struct ", head)
    foreach(field -> print(io, "\n    ", field), body.args)
    print(io, "\nend")
end

function render(s::ImportContext, spec::ImportSpec)
    io = IOBuffer()
    sources = unique!(["$(n.package)/$(n.interface)/$(basename(s.definitions[n].file))" for n in s.defined])
    origin = isabspath(spec.source) ? " in $(relpath(spec.source, dirname(spec.file)))" : ""
    println(io, "# This file is generated by `@importrosmsg`", origin, " from ", join(sources, ", "), ".")
    println(io, "# Do not edit it: `@importrosmsg` overwrites it whenever the generated content changes.")
    context = IOContext(io, :module => s.caller)
    for ex in s.exprs
        declaration = is_enum_code(ex) || isexpr(ex, :macrocall, 3) && isexpr(ex.args[3], :struct)
        declaration || continue
        println(io)
        printcode(context, ex)
        println(io)
    end
    return String(take!(io))
end
