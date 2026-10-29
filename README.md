# ROSMessages.jl

*Julia types for ROS 2 messages with the memory layout of their C structs*

**ROSMessages.jl** defines Julia structs from ROS 2 `.msg` and `.srv` files. The structs have the memory layout of the C structs of `rosidl_runtime_c`, so C libraries like `rcl` read and write them in place, without copying. [RCLC.jl](../RCLC/README.md) uses these types and reexports the macros and types documented here.

## Prerequisites
The ROS 2 installation or workspace providing a message needs to be sourced, so that its `.msg` or `.srv` file is found in `share/<package>/msg` or `share/<package>/srv` of a prefix in `AMENT_PREFIX_PATH`. Calling the C functions of a type (see [C Functions](#c-functions)) additionally needs its C libraries.

## Usage
```julia
using ROSMessages
@importrosmsg sensor_msgs/NavSatFix
```

This defines the struct `NavSatFix` and the structs of the messages it contains, here `Header`, `Time` and `NavSatStatus`.

### Automatically Defined Types
All imported types have a `@kwdef` constructor defined, whose defaults are the ROS default values. Mutable types additionally have an analogue reconstructor (which is realized through `@kwdef!`) and `copy!` defined. Equality and `hash` compare the contents of the fields, so the lifetime of their `ROSString` and `Sequence` fields applies to them, too.

A message containing other messages needs their types. `@importrosmsg` defines each of them as an immutable type named like the message, unless the module already has an immutable type for that ROS message, under any name (see [Mutability of Message Types](#mutability-of-message-types)). Types are identified by their ROS name, e.g. `ROSMessages.rosname(NavSatFix) == "sensor_msgs/msg/NavSatFix"`, so the same message name in two packages does not lead to a mix-up. If the name is already taken by something else, e.g. `Char` by Julia's `Char` for `std_msgs/Char`, `@importrosmsg` throws an error and asks for importing the message under another name.

The ROS types are represented as follows:

| ROS | Julia |
|---|---|
| `bool`, `byte`, `char`, `float32`, …, `uint64` | `Bool`, `UInt8`, `UInt8`, `Float32`, …, `UInt64` |
| `string`, `string<=N` | `ROSString` |
| `wstring`, `wstring<=N` | `ROSU16String` |
| `T[N]` | `SVector{N, T}` |
| `T[]`, `T[<=N]` | `Sequence{T}` |
| `time`, `duration` | the types of `builtin_interfaces/Time` and `builtin_interfaces/Duration` |
| message without fields | a struct with the field `structure_needs_at_least_one_member::UInt8`, which ROS adds |

Strings and sequences start empty even if the definition has a default value, because only C may allocate their memory. The C `init` function of the message applies their ROS defaults (see [C Functions](#c-functions)).

### Renaming Type
The `@importrosmsg` macro supports the same name space management as `import` by renaming the imported type:

```julia
@importrosmsg package_name/MsgName as TypeName
```

### Services
A service defines the two types `TypeName_Request` and `TypeName_Response`:

```julia
@importrosmsg example_interfaces/srv/AddTwoInts
```

Overrides, type parameters and supertypes are not supported for services, yet.

### Constants
Integer constants sharing the longest underscore-delimited prefix and the same type are grouped into an EnumX module named by that prefix, with the suffixes as enum members. For example, importing `sensor_msgs/NavSatStatus` defines `STATUS.FIX` and `SERVICE.GPS` in the importing module. Constants that do not form a group remain type properties. Duplicate values are represented as EnumX aliases. An existing binding with the group name causes an import error.

### Mutability of Message Types
Types used as a message currently need to be `mutable`. However, to match the ROS memory layout, types used within a message need to be `immutable`. To use the same ROS type both as a message and as part of another message, you need to import it twice with different mutability and rename one of both during import as described above.

The following mutability defaults are used:

- Explicitly imported types via `@importrosmsg` are imported as mutable structs per default (if the `immutable` keyword is not provided), as an explicitly imported type is typically used as a message. Use `immutable` for a type that is used as part of another message, e.g. to override its fields.
- Implicitly imported types loaded via RCLC's `@addpublisher!`/`@addsubscription!`/`@addclient!` are imported as mutable structs, as it's known that they are used as a message. These macros reuse a mutable type imported before, under any name.
- Indirectly needed ROS types (directly or indirectly referred to in used message files) are also automatically imported. They are always imported as immutable structs, as it's known that they will be used as part of a message. An immutable type imported before is reused, under any name. If the only type for such a ROS type is mutable, `@importrosmsg` throws an error asking for an additional immutable import under another name.

### Field Overrides
Instead of defining the whole type corresponding to a `.msg` file manually, `@importrosmsg` is able to override fields when importing a ROS msg file. This avoids the problems related to code duplication. The following constraints shall be fulfilled:

- Field overrides shall be inside a single `begin` `end` block as in a struct definition.
- The `@override` macro shall be used for each field override.
- There shall be one field override per line.
- Field overrides shall refer to actually existing fields.
- Field overrides shall not change the fields' size.
- Field overrides shall not change the fields' memory layout.
- Field overrides shall be provided in the same order as the ROS msg file's fields.

`@importrosmsg` throws an error if one of these constraints is violated. The memory layout is checked after defining the type: every field needs to be an isbits type, at the offset and with the size of the C struct, and the whole struct needs the size and alignment of the C struct. This also rejects inline fields containing Julia-managed references, which C cannot safely overwrite.

#### Override Field Names
Field names of imported types can be renamed similar to renaming a type:

```julia
@importrosmsg package_name/MsgName begin
    @override field_with_crazy_name_a as name_a
end
```

Supporting the `as` syntax is the reason why `@override` is used although we already are inside the macro `@importrosmsg`, but it would not parse otherwise.

#### Field Type Override
For a field type override you need to list the type override after the msg name using the typical type annotation syntax:

```julia
@importrosmsg package_name/MsgName begin
    @override field_a::NewTypeX
    @override field_b::NewTypeY
end
```

Without a default value override, the default of a field with an overridden type is `NewTypeX()`.

#### Default Value Override
A default value override is done as in the [`@kwdef`](https://docs.julialang.org/en/v1/base/base/#Base.@kwdef) macro:

```julia
@importrosmsg package_name/MsgName begin
    @override field_a = TypeWithConstructor(42, true)
end
```

#### Override Fields with Struct
Consecutive fields can be overridden with an immutable struct of the same size minding the padding:

```julia
    @override (start, stop) as range::UnitRange{Int64}
```

This basically moves the two values into an inlined substructure.

#### Override Constants
A constant override follows the field override syntax: without `as`, it only checks that the constant exists, and a type converts the value by calling the type. A qualified name renames the type property, while an unqualified name additionally defines a module constant. Enum grouping is based on the resulting property names and types:

```julia
@importrosmsg sensor_msgs/NavSatStatus begin
    @override STATUS_NO_FIX                              # only checks that it exists
    @override STATUS_FIX as NavSatStatus.FIX             # renames NavSatStatus.STATUS_FIX
    @override SERVICE_GPS::ServiceFlag                   # NavSatStatus.SERVICE_GPS is a ServiceFlag
    @override SERVICE_GLONASS as GLONASS::ServiceFlag    # defines the module constant GLONASS
end
```

The value of a constant cannot be overridden. Enum groups are inferred after overrides, so a renamed or converted constant may no longer belong to its original group.

#### Combined Overrides
All the override functionalities can be combined, so the general syntax of `@importrosmsg` is the following:

```julia
@importrosmsg [mutable|immutable] ros_package_name/[msg/|srv/]MsgName [as TypeName[{T, …}][ <: Supertype]] [in "file.jl"] [begin
    @override ros_field[ as julia_field][::JuliaType][ = Expr_for_default_value]
    @override (ros_field, next_ros_field, after_next_ros_field, …) as julia_field::JuliaType[ = Expr_for_default_value]
    @override ROS_CONSTANT[ as [TypeName.]JULIA_CONSTANT][::JuliaType]
    …
end]
```

`@importrosmsg` needs to be a top-level statement of its own, not part of a function, a `let` block or another expression. Types, parameters, supertypes and overrides can only use global names, and nested types are found among the types defined by earlier statements.

#### Change Management
`ROSMessages.jl` will throw an error at compile time if the field override constraints are violated. This means the following changes on the msg file can be monitored by `@importrosmsg` by using (possibly empty) overrides:

- removed fields
- reordered fields
- renamed fields
- changed types of fields with overridden types, if the change modifies the memory layout

Added fields and changed types of fields without overridden types change the imported type accordingly. Added fields are often uncritical, as the implemented logic typically is not influenced by new fields. If you want to monitor them, you can use `fieldcount`. Changes of (possibly non-primitive) field types can be critical or not. If necessary, you need to monitor them manually. You can use `fieldtypes`.

Precompiled modules depend on the `.msg` and `.srv` files they import, so changing such a file, e.g. with `colcon build`, recompiles them.

### Tests
Run the tests with `julia --project -e 'using Pkg; Pkg.test()'`. The tests use their own message definitions and do not require a ROS installation. If ROS message headers are available through `AMENT_PREFIX_PATH` and a C compiler is installed, an additional test compares the computed layouts against those headers.

### Type Parameters and Supertype
The imported type can have type parameters, which its fields do not need to use, and a supertype:

```julia
@importrosmsg sensor_msgs/PointCloud2 as PointCloud2{T} <: AbstractVector{T}
```

RCLC uses this to type point clouds by their point type.

### Generated Files
`@importrosmsg` defines the types and their methods. With `in "file.jl"`, it also writes the struct and enum declarations to `file.jl`, relative to the file containing the call:

```julia
@importrosmsg immutable ros_package_name/ROSMsgName as MyType in "gen/MyType.jl" begin
    @override m_distance as distance::Distance = Distance(0)
end
```

The macro does not execute the generated file: it returns the type definitions, methods, and layout checks to Julia directly. The generated file does not contain the methods or layout checks. The suggested file name is the Julia type name in a `gen` directory. Missing directories are created. The file is only written if its content changes, e.g. after changing an override or the msg file, and starts with a comment stating that it is generated and must not be edited.

### C Functions
`ROSMessages.jl` defines methods calling the C libraries of each imported type:

- `ROSMessages.typesupport(T)` returns the type support handle that `rcl` needs to create publishers and subscriptions.
- `ROSMessages.service_typesupport(T_Request)` returns the type support handle of the service.
- `ROSMessages.init!(x)` and `ROSMessages.fini!(x)` initialize and free a mutable message with the C `init` and `fini` functions of its type.

The library and symbol names are literals, so these calls need no `eval`.

## Limitations
- Only the legacy `.msg` and `.srv` files are supported, not `.idl` files and not actions.
- As in the reference parser `rosidl_adapter`, a `#` starts a comment and a `=` makes a line
a constant even inside a string value.
- Overrides, type parameters and supertypes are not supported for services, yet.

## Related Packages
No other Julia package generates structs from ROS 2 interface definitions with the C memory layout of `rosidl_runtime_c`, and none supports field overrides. The following packages generate or define message types differently (checked 2026-10):

| Package | ROS | Source of the definitions | Generated types | Memory |
|---|---|---|---|---|
| [RobotOS.jl](https://github.com/jdlangs/RobotOS.jl) | 1 | rospy message classes at run time | `mutable struct`s with `String` and `Vector` fields, one module per ROS package | Copied through PyCall |
| [RobotMessages.jl](https://github.com/kylejbrown17/RobotMessages.jl) | 1 | RobotOS.jl for three types | As RobotOS.jl | As RobotOS.jl |
| [ROS.jl](https://github.com/gstavrinos/ROS.jl) | 1 | `.msg` and `.srv` files | Cxx.jl property wrappers around roscpp C++ objects, written to `.jl` files | C++ objects, Julia ≤ 1.3 only |
| [ROS2.jl](https://github.com/Michi-Tsubaki/ROS2.jl) | 2 | None | None, messages are rclpy Python objects | Python objects |
| [RobotOSData.jl](https://github.com/damiendr/RobotOSData.jl) | 1 | `.msg` files | Immutable structs with `String`, `Vector` and `SVector` fields, written to one `.jl` file per ROS package, without constants and defaults | Copied from the bag's byte stream |
| [Caesar.jl](https://github.com/JuliaRobotics/Caesar.jl) | 1 | None | Hand-written PCL structs, ROS through RobotOS.jl | Copied |
