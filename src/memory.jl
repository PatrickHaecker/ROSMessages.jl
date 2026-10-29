"""ROSString represents a string in ROS 2. It is a wrapper around a Cstring with a size and a capacity field. The size field is the number of bytes used in the string, the capacity field is the number of bytes allocated for the string. The capacity field is always at least one byte larger than the size field to account for the mandatory null terminator.

# Lifetime
A `ROSString` does not own its memory, so it is only valid as long as that memory lives: for a received message only within that message's callback, for `ROSString(s)` only while `s` is preserved. Everything reading the content (`==`, `hash`, `show`, `unsafe_string`) requires a valid `ROSString`, so keep a copy made with `unsafe_string` instead.
"""
struct ROSString
    data::Cstring
    size::Csize_t
    capacity::Csize_t
end

"""
    ROSString()

Create an empty `ROSString`.

This is useful if you need a variable of type `ROSString`, but do not care about its content and do not want to think about preserving memory correctness.
"""
ROSString() = ROSString(Ptr{Int8}() |> Cstring, 0, 0)

"""
    ROSString(s::AbstractString)

Create a `ROSString` from a Julia string.

As every non-empty ROSString uses `unsafe_convert` for creation, the warnings listed in `unsafe_convert`'s documentation apply here, too, so read them carefully.
"""
ROSString(s::AbstractString) = ROSString(Base.unsafe_convert(Cstring, s), ncodeunits(s), ncodeunits(s)+1)
isnull(r::ROSString) = r.data |> pointer === Ptr{Int8}()
Base.unsafe_string(r::ROSString) = unsafe_string(r.data)
Base.Symbol(r::ROSString) = r |> unsafe_string |> Symbol
function Base.show(io::IO, ::MIME"text/plain", r::ROSString)
    print(io, "ROSString with size ", r.size, " and capacity ", r.capacity, ":\n")
    isnull(r) ? print(io, "NULL") : show(io, unsafe_string(r))
end

"""
    ROSU16String

Represent a ROS 2 `wstring` as UTF-16 code units, laid out like `rosidl_runtime_c__U16String`.

Like a [`ROSString`](@ref), a `ROSU16String` does not own its memory, so it is only valid as
long as that memory lives.

```jldoctest
julia> ROSU16String()
ROSU16String()

julia> s = transcode(UInt16, "Grüße"); GC.@preserve s unsafe_string(ROSU16String(pointer(s), length(s), length(s)))
"Grüße"
```
"""
struct ROSU16String
    data::Ptr{UInt16}
    size::Csize_t
    capacity::Csize_t
end
ROSU16String() = ROSU16String(Ptr{UInt16}(), 0, 0)
isnull(r::ROSU16String) = r.data === Ptr{UInt16}()
Base.unsafe_string(r::ROSU16String) = isnull(r) ? "" : transcode(String, unsafe_wrap(Array, r.data, r.size))

# Print a valid Julia expression for creating the string as [suggested by the documentation](https://docs.julialang.org/en/v1/manual/types/#man-custom-pretty-printing).
function Base.show(io::IO, r::Union{ROSString, ROSU16String})
    print(io, nameof(typeof(r)), "(")
    isnull(r) || show(io, unsafe_string(r))
    print(io, ")")
end

# Each type defines a TypeNameSequence data struct with the following content
# The sequences for the primitive data types are defined here: https://docs.ros.org/en/rolling/p/rosidl_runtime_c/generated/program_listing_file_include_rosidl_runtime_c_primitives_sequence.h.html
"A sequence in ROS 2, which, like a [`ROSString`](@ref), does not own its memory and is only valid as long as that memory lives."
struct Sequence{T} <: AbstractVector{T}
    data::Ptr{T}
    size::Csize_t
    capacity::Csize_t
end
Sequence{T}() where T = Sequence{T}(Ptr{T}(), 0, 0)
Base.size(s::Sequence{T}) where T = (s.size,)
Base.IndexStyle(::Type{<:Sequence{T}}) where T = IndexLinear()
@inline Base.getindex(s::Sequence{T}, i::Int) where T = (@boundscheck checkbounds(s, i); unsafe_load(s.data, i))
@inline Base.setindex!(s::Sequence{T}, v::T, i::Int) where T = (@boundscheck checkbounds(s, i); unsafe_store!(s.data, v, i))

# The code units of a string, so that strings compare and hash by content.
units(r::ROSString) = Sequence{UInt8}(r.data |> pointer |> Ptr{UInt8}, r.size, r.capacity)
units(r::ROSU16String) = Sequence{UInt16}(r.data, r.size, r.capacity)
Base.:(==)(a::T, b::T) where T <: Union{ROSString, ROSU16String} = units(a) == units(b)
Base.hash(r::Union{ROSString, ROSU16String}, h::UInt) = hash(units(r), h)

"""
    fieldvalues(x)

Return the values of all fields of `x` as a tuple.

Generated message types compare and hash these tuples, because the fallback `===` of an
immutable message would compare a [`ROSString`](@ref) by its pointer.

```jldoctest
julia> ROSMessages.fieldvalues(1 => "a")
(1, "a")

julia> ROSMessages.fieldvalues(Empty())
(0x00,)
```
"""
fieldvalues(x) = ntuple(i -> getfield(x, i), x |> typeof |> fieldcount |> Val)

# ROS adds this member to every message without fields, as a C struct needs at least one member (`std_msgs/msg/Empty.idl` shows it). `Empty` is predefined, because `@importrosmsg` maps `std_msgs/Empty` to it.
struct Empty
    structure_needs_at_least_one_member::UInt8
end
Empty() = Empty(UInt8(0))
Base.show(io::IO, ::Empty) = print(io, "Empty()")
Base.show(io::IO, ::MIME"text/plain", ::Empty) = print(io, "_")
