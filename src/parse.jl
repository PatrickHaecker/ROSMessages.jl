# ROS 2 normally uses IDL files (https://design.ros2.org/articles/idl_interface_definition.html), but only the legacy msg and srv files (https://design.ros2.org/articles/legacy_interface_definition.html) are supported here.
# File format specification: https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html
# Reference implementation: `rosidl_adapter/parser.py`, which this parser follows.

const SPECIFICATION = "https://design.ros2.org/articles/legacy_interface_definition.html"

"""
    MsgName(package, interface, name)

Identify a ROS message, or the request or response of a ROS service, by its package, interface kind (`"msg"` or `"srv"`) and name.

`print` writes the ROS form used in error messages and by [`rosname`](@ref).

```jldoctest
julia> print(ROSMessages.MsgName("std_msgs", "msg", "Header"))
std_msgs/msg/Header

julia> string(ROSMessages.MsgName("example_interfaces", "srv", "AddTwoInts_Request"))
"example_interfaces/srv/AddTwoInts_Request"
```
"""
struct MsgName
    package::String
    interface::String
    name::String
end
Base.print(io::IO, n::MsgName) = print(io, n.package, '/', n.interface, '/', n.name)

"""
    checkname(pattern::Regex, name, kind, anchor)

Throw an `ArgumentError` if `name` does not match `pattern`, citing the rule for names of `kind` at `anchor` of the specification.

The patterns of the callers are the name rules as `rosidl_adapter` implements them.

```jldoctest
julia> ROSMessages.checkname(r"^[a-z][a-z0-9_]*\$", "frame_id", "field", "#naming-of-fields")

julia> ROSMessages.checkname(r"^(?!.*__)[a-z][a-z0-9_]*\$", "frame__id", "field", "#naming-of-fields")
ERROR: ArgumentError: `frame__id` is no valid field name, see https://design.ros2.org/articles/legacy_interface_definition.html#naming-of-fields
[...]
```
"""
function checkname(pattern::Regex, name::AbstractString, kind::AbstractString, anchor::AbstractString)
    occursin(pattern, name) || throw(ArgumentError("`$name` is no valid $kind name, see $SPECIFICATION$anchor"))
    return nothing
end
# No ROS 2 specification for package names was found, so the ROS 1 one is assumed to still hold.
checkpackage(name::AbstractString) = checkname(r"^(?!.*__)(?!.*_$)[a-z][a-z0-9_]*$", name, "package", "")
checkmessage(name::AbstractString) = checkname(r"^[A-Z][A-Za-z0-9]*$", name, "message", "#naming-of-messages-and-services")

"""
    Nested{package, name}

Stand for the message `package/msg/name` as the type of a field, until the field gets the Julia type of that message.

```jldoctest
julia> ROSMessages.parse_type("Header[3]", "std_msgs") == ROSMessages.SVector{3, ROSMessages.Nested{:std_msgs, :Header}}
true

julia> ROSMessages.msgname(ROSMessages.Nested{:std_msgs, :Header})
ROSMessages.MsgName("std_msgs", "msg", "Header")
```
"""
struct Nested{package, name} end
msgname(::Type{Nested{P, N}}) where {P, N} = MsgName(String(P), "msg", String(N))

"""
    parse_type(s::AbstractString, package::AbstractString)

Return the type of a field or constant written as `s` in a definition belonging to `package`: the primitive Julia type, an `SVector` for a fixed-size array, a [`Sequence`](@ref) for a bounded or unbounded sequence and [`Nested`](@ref) for a message.

The C layout does not depend on the upper bounds of strings and sequences, so they are only checked for their syntax. `rosidl` checks the values against them when it builds the package.

```jldoctest
julia> ROSMessages.parse_type("float64[9]", "sensor_msgs") == ROSMessages.SVector{9, Float64}
true

julia> ROSMessages.parse_type("Point[<=3]", "geometry_msgs") == Sequence{ROSMessages.Nested{:geometry_msgs, :Point}}
true

julia> ROSMessages.parse_type("int32<=4", "std_msgs")
ERROR: ArgumentError: only `string` and `wstring` can have an upper bound, not `int32`
[...]
```
"""
function parse_type(s::AbstractString, package::AbstractString)
    m = match(r"^(?<base>[^\[<]+)(?<bound><=[1-9]\d*)?(?<array>\[(?:(?<bounded><=)?(?<size>[1-9]\d*))?\])?$", s)
    isnothing(m) && throw(ArgumentError("`$s` is no valid type, see $SPECIFICATION#field-types"))
    base = m["base"]
    isnothing(m["bound"]) || base in ("string", "wstring") || throw(ArgumentError("only `string` and `wstring` can have an upper bound, not `$base`"))
    primitives = (bool = Bool, byte = UInt8, char = UInt8, float32 = Float32, float64 = Float64, int8 = Int8, uint8 = UInt8, int16 = Int16, uint16 = UInt16,
                  int32 = Int32, uint32 = UInt32, int64 = Int64, uint64 = UInt64, string = ROSString, wstring = ROSU16String)
    T = get(primitives, Symbol(base), nothing)
    if isnothing(T)
        # ROS 2 keeps the ROS 1 built-in types `time` and `duration` for compatibility, mapped to `builtin_interfaces`.
        parts = base in ("time", "duration") ? ["builtin_interfaces", uppercasefirst(base)] : split(base, '/')
        length(parts) <= 2 || throw(ArgumentError("`$base` is no valid type, see $SPECIFICATION#field-types"))
        length(parts) == 2 && checkpackage(parts[1])
        checkmessage(parts[end])
        T = Nested{Symbol(length(parts) == 2 ? parts[1] : package), Symbol(parts[end])}
    end
    isnothing(m["array"]) && return T
    return isnothing(m["size"]) || !isnothing(m["bounded"]) ? Sequence{T} : SVector{parse(Int, m["size"]), T}
end

"""
    parse_value(T::Type, s::AbstractString)

Parse the constant value or default value `s` of a field of type `T`, as `rosidl` does.

Booleans accept `true`, `false`, `1` and `0` in any case, integers a `0x`, `0o` or `0b` prefix. Strings may be quoted with `"` or `'`, which then need to be escaped inside. Arrays are written as `[a, b, …]`.

```jldoctest
julia> ROSMessages.parse_value(ROSMessages.parse_type("int8", ""), "-0x80")
-128

julia> ROSMessages.parse_value(ROSMessages.parse_type("string[]", ""), "[\\"a, b\\", 'it\\\\'s']")
2-element Vector{String}:
 "a, b"
 "it's"

julia> ROSMessages.parse_value(ROSMessages.parse_type("uint8[2]", ""), "[1, 256]")
ERROR: ArgumentError: `256` is no valid UInt8 value
[...]
```
"""
function parse_value(::Type{Bool}, s::AbstractString)
    lowercase(s) in ("true", "1") && return true
    lowercase(s) in ("false", "0") && return false
    throw(ArgumentError("`$s` is no valid Bool value"))
end
function parse_value(::Type{T}, s::AbstractString) where T <: Real
    value = tryparse(T, s)
    isnothing(value) && throw(ArgumentError("`$s` is no valid $T value"))
    return value
end
function parse_value(::Type{<:Union{ROSString, ROSU16String}}, s::AbstractString)
    for q in ('"', '\'')
        length(s) >= 2 && startswith(s, q) && endswith(s, q) || continue
        inner = chop(s; head = 1, tail = 1)
        occursin(Regex("(?<!\\\\)" * q), inner) && throw(ArgumentError("string value `$s` contains an unescaped `$q`"))
        return replace(inner, "\\$q" => q)
    end
    return String(s)
end
function parse_value(::Type{A}, s::AbstractString) where A <: Union{SVector, Sequence}
    startswith(s, '[') && endswith(s, ']') || throw(ArgumentError("array value `$s` needs to start with `[` and end with `]`"))
    inner = strip(chop(s; head = 1, tail = 1))
    # An element of a string array is a quoted string, which may contain commas, or unquoted text.
    elements = isempty(inner) ? SubString{String}[] :
               eltype(A) <: Union{ROSString, ROSU16String} ? [m.match for m in eachmatch(r"(?:\"(?:[^\"\\]|\\.)*\"|'(?:[^'\\]|\\.)*'|[^,])+", inner)] :
               split(inner, ',')
    A <: SVector && length(elements) != length(A) && throw(ArgumentError("array value `$s` needs exactly $(length(A)) elements"))
    return [parse_value(eltype(A), strip(e)) for e in elements]
end
parse_value(::Type{<:Nested}, ::AbstractString) = throw(ArgumentError("only fields of primitive types can have a value, see $SPECIFICATION#field-default-value"))

"A field of a ROS message with its parsed default value, which is `nothing` if the definition has none."
struct Field
    type::Type
    name::Symbol
    default::Any
    line::Int
end

"A constant of a ROS message with its parsed value."
struct Constant
    type::Type
    name::Symbol
    value::Any
    line::Int
end

"""
    Definition(name, fields, constants, file)

Hold the fields and constants of the ROS message `name`, read from `file`.
"""
struct Definition
    name::MsgName
    fields::Vector{Field}
    constants::Vector{Constant}
    file::String
end

"""
    parse_definition(text, name::MsgName, file = string(name), firstline = 1)

Parse the definition `text` of the message `name` into a [`Definition`](@ref), counting its lines from `firstline` for error messages.

```jldoctest
julia> d = ROSMessages.parse_definition("uint8 OK = 0  # all good\\nuint8 level\\nstring name 'x'", ROSMessages.MsgName("p", "msg", "Status"));

julia> [(f.name, f.default) for f in d.fields]
2-element Vector{Tuple{Symbol, Any}}:
 (:level, nothing)
 (:name, "x")

julia> [(c.name, c.value) for c in d.constants]
1-element Vector{Tuple{Symbol, UInt8}}:
 (:OK, 0x00)

julia> only(ROSMessages.parse_definition("", ROSMessages.MsgName("p", "msg", "Nothing")).fields).name
:structure_needs_at_least_one_member

julia> ROSMessages.parse_definition("int32 x\\nint32 Y_", ROSMessages.MsgName("p", "msg", "Bad"))
ERROR: ArgumentError: p/msg/Bad:2: `Y_` is no valid field name, see https://design.ros2.org/articles/legacy_interface_definition.html#naming-of-fields
[...]
```
"""
function parse_definition(text::AbstractString, name::MsgName, file::AbstractString = string(name), firstline::Int = 1)
    members = map(enumerate(eachline(IOBuffer(text)))) do (i, line)
        n = firstline + i - 1
        try
            return parse_line(line, n, name.package)
        catch e
            e isa ArgumentError || rethrow()
            throw(ArgumentError("$file:$n: $(e.msg)"))
        end
    end
    fields = Field[m for m in members if m isa Field]
    # ROS adds this member to a message without fields, as a C struct needs at least one member.
    isempty(fields) && push!(fields, Field(UInt8, :structure_needs_at_least_one_member, nothing, 0))
    return Definition(name, fields, Constant[m for m in members if m isa Constant], file)
end

# Return the field or constant defined by `line`, or `nothing` for an empty line or a comment.
function parse_line(line::AbstractString, n::Int, package::AbstractString)
    # The reference parser cuts each line at its first `#`, so a string value cannot contain `#` either.
    content = strip(first(split(replace(line, '\t' => ' '), '#'; limit = 2)))
    isempty(content) && return nothing
    m = match(r"^(\S+) +(.+)$", content)
    isnothing(m) && throw(ArgumentError("`$content` needs a type and a name, see $SPECIFICATION#syntax"))
    type, rest = m
    T = parse_type(type, package)
    if contains(rest, '=')
        # ROS does not stick to its own specification. Although https://design.ros2.org/articles/legacy_interface_definition.html#syntax implies that there is no space allowed around the assignment operator (equal sign: =), it is being used by official ROS msg files like sensor_msgs/PointField, so accept it here, too.
        name, value = strip.(split(rest, '='; limit = 2))
        checkname(r"^[A-Z]([A-Z0-9_]?[A-Z0-9]+)*$", name, "constant", "#naming-of-constants")
        T <: Union{Real, ROSString, ROSU16String} || throw(ArgumentError("constant `$name` needs a primitive type, see $SPECIFICATION#constants"))
        return Constant(T, Symbol(name), parse_value(T, value), n)
    end
    name, default = match(r"^(\S+) *(.*)$", rest)
    checkname(r"^(?!.*__)(?!.*_$)[a-z][a-z0-9_]*$", name, "field", "#naming-of-fields")
    return Field(T, Symbol(name), isempty(default) ? nothing : parse_value(T, default), n)
end

"""
    msgfile(name::MsgName)

Return the path of the `.msg` or `.srv` file defining `name`, searching `share/<package>/<interface>` in each prefix of `AMENT_PREFIX_PATH`.

```jldoctest
julia> ROSMessages.msgfile(ROSMessages.MsgName("std_msgs", "msg", "Header")) |> basename
"Header.msg"

julia> ROSMessages.msgfile(ROSMessages.MsgName("example_interfaces", "srv", "AddTwoInts_Response")) |> basename
"AddTwoInts.srv"
```
"""
function msgfile(name::MsgName)
    base = name.interface == "srv" ? replace(name.name, r"_(Request|Response)$" => "") : name.name
    prefixes = split(get(ENV, "AMENT_PREFIX_PATH", ""), ':'; keepempty = false)
    for prefix in prefixes
        path = joinpath(prefix, "share", name.package, name.interface, "$base.$(name.interface)")
        isfile(path) && return path
    end
    throw(ArgumentError("$name: no prefix of AMENT_PREFIX_PATH=\"$(join(prefixes, ':'))\" contains share/$(name.package)/$(name.interface)/$base.$(name.interface). Source the setup script of the ROS installation or workspace providing $(name.package)."))
end

"""
    read_definition(name::MsgName)

Read and parse the definition of `name`, and declare its file a precompilation dependency of the module being compiled.

```jldoctest
julia> [f.name for f in ROSMessages.read_definition(ROSMessages.MsgName("std_msgs", "msg", "Header")).fields]
2-element Vector{Symbol}:
 :stamp
 :frame_id

julia> [f.name for f in ROSMessages.read_definition(ROSMessages.MsgName("example_interfaces", "srv", "AddTwoInts_Response")).fields]
1-element Vector{Symbol}:
 :sum
```
"""
function read_definition(name::MsgName)
    file = msgfile(name)
    include_dependency(file)
    text = read(file, String)
    name.interface == "msg" && return parse_definition(text, name, file)
    parts = split(text, r"^---[ \t\r]*(?:\n|$)"m)
    length(parts) == 2 || throw(ArgumentError("$file needs exactly one line `---` separating the request from the response, see $SPECIFICATION#service-description-specification"))
    request, response = parts
    endswith(name.name, "_Request") && return parse_definition(request, name, file)
    return parse_definition(response, name, file, count('\n', request) + 2)
end
