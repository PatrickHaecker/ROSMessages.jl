"""
    CLayout(fields, size, alignment)

Describe the memory layout of the C struct of a ROS message: the offset and size of each field, the size of the struct and its alignment.
"""
struct FieldLayout
    offset::Int
    size::Int
end

struct CLayout
    fields::Vector{FieldLayout}
    size::Int
    alignment::Int
end

"""
    clayout(def::Definition, definition)

Compute the layout of the C struct for `def`, calling `definition(name::MsgName)` for the definitions of nested messages.

C places each field at the next multiple of its alignment and pads the struct to a multiple of its largest alignment. Julia lays out primitive types like C, so their `sizeof` and alignment apply.

```jldoctest
julia> header = ROSMessages.read_definition(ROSMessages.MsgName("std_msgs", "msg", "Header"));

julia> layout = ROSMessages.clayout(header, ROSMessages.read_definition);

julia> ([(field.offset, field.size) for field in layout.fields], layout.size, layout.alignment)
([(0, 8), (8, 24)], 32, 8)

julia> point = ROSMessages.parse_definition("uint8 kind\\nfloat64[3] xyz\\nuint8 flag", ROSMessages.MsgName("p", "msg", "P"));

julia> layout = ROSMessages.clayout(point, ROSMessages.read_definition);

julia> ([(field.offset, field.size) for field in layout.fields], layout.size, layout.alignment)
([(0, 1), (8, 24), (32, 1)], 40, 8)
```
"""
function clayout(def::Definition, definition)
    fields = FieldLayout[]
    offset = 0
    alignment = 1
    for field in def.fields
        size, align = csize(field.type, definition)
        offset = cld(offset, align) * align
        push!(fields, FieldLayout(offset, size))
        offset += size
        alignment = max(alignment, align)
    end
    return CLayout(fields, cld(offset, alignment) * alignment, alignment)
end

# Return the size and alignment of a field of type `T` in C. Julia lays out the other types like C, including sequences, which are a data pointer, a size and a capacity regardless of their bound.
csize(T::Type, definition) = sizeof(T), Base.datatype_alignment(T)
function csize(T::Type{<:Nested}, definition)
    layout = clayout(definition(msgname(T)), definition)
    return layout.size, layout.alignment
end
function csize(::Type{SVector{N, T}}, definition) where {N, T}
    size, alignment = csize(T, definition)
    return N * size, alignment
end

"""
    check_layout(T::DataType, expected::Tuple, alignment::Int)

Throw an `ArgumentError` unless `T` has the memory layout of the C struct of its ROS message.

`expected` holds `(offset, size)` for each field of `T`, followed by the ROS field names if they differ from the Julia field name. `@importrosmsg` calls this after defining a type, so that a field override cannot change the layout unnoticed.

```jldoctest
julia> struct Stamp; sec::Int32; nanosec::UInt32; end

julia> ROSMessages.rosname(::Type{Stamp}) = "builtin_interfaces/msg/Time";

julia> ROSMessages.check_layout(Stamp, ((0, 4), (4, 4)), 4)

julia> struct WideStamp; sec::Int64; nanosec::UInt32; end

julia> ROSMessages.rosname(::Type{WideStamp}) = "builtin_interfaces/msg/Time";

julia> ROSMessages.check_layout(WideStamp, ((0, 4), (4, 4)), 4)
ERROR: ArgumentError: field `sec` of `WideStamp` (ROS builtin_interfaces/msg/Time: sec) has type Int64 of 8 bytes instead of 4 bytes.
[...]
```
"""
function check_layout(T::DataType, expected::Tuple, alignment::Int)
    ros = rosname(T)
    for (i, item) in enumerate(expected)
        offset, fieldsize = item[1], item[2]
        rosfields = length(item) == 2 ? fieldname(T, i) : item[3]
        F = fieldtype(T, i)
        field = "field `$(fieldname(T, i))` of `$T` (ROS $ros: $rosfields)"
        isbitstype(F) || throw(ArgumentError(string(field, " has type ", F, ", which is not an isbits type.",
            ismutabletype(F) ? " Import the ROS type of `$F` a second time as an immutable type with a different name." : "")))
        fieldoffset(T, i) == offset || throw(ArgumentError("$field has offset $(Int(fieldoffset(T, i))) instead of $offset bytes."))
        sizeof(F) == fieldsize || throw(ArgumentError("$field has type $F of $(sizeof(F)) bytes instead of $fieldsize bytes."))
    end
    lastfield = last(expected)
    size = cld(lastfield[1] + lastfield[2], alignment) * alignment
    sizeof(T) == size || throw(ArgumentError("`$T` has $(sizeof(T)) bytes instead of the $size bytes of ROS $ros."))
    Base.datatype_alignment(T) == alignment ||
        throw(ArgumentError("`$T` has alignment $(Base.datatype_alignment(T)) instead of $alignment bytes for ROS $ros."))
    return nothing
end
