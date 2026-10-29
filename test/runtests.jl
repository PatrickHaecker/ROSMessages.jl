using ROSMessages
using ROSMessages: MsgName, Override, parse_definition, parse_type, parse_value, apply_overrides, read_definition, clayout, check_layout, importrosmsg
using Aqua
using StaticArrays
using Test
using Unitful

@testset "Aqua.jl" begin
    Aqua.test_all(ROSMessages)
end

const INSTALLED_PREFIXES = split(get(ENV, "AMENT_PREFIX_PATH", ""), ':'; keepempty = false)

# Test definitions take precedence over any installed ROS packages.
const PREFIX = mktempdir()
function definefile(path, text)
    mkpath(dirname(path))
    write(path, text)
end
definefile(joinpath(PREFIX, "share", "builtin_interfaces", "msg", "Time.msg"), "int32 sec\nuint32 nanosec\n")
definefile(joinpath(PREFIX, "share", "std_msgs", "msg", "Header.msg"), "builtin_interfaces/Time stamp\nstring frame_id\n")
definefile(joinpath(PREFIX, "share", "std_msgs", "msg", "Char.msg"), "uint8 data\n")
definefile(joinpath(PREFIX, "share", "geometry_msgs", "msg", "Vector3.msg"), "float64 x\nfloat64 y\nfloat64 z\n")
definefile(joinpath(PREFIX, "share", "sensor_msgs", "msg", "NavSatStatus.msg"), """
    int8 STATUS_UNKNOWN = -2
    int8 STATUS_NO_FIX = -1
    int8 STATUS_FIX = 0
    int8 STATUS_SBAS_FIX = 1
    int8 STATUS_GBAS_FIX = 2
    int8 status -2
    uint16 SERVICE_UNKNOWN = 0
    uint16 SERVICE_GPS = 1
    uint16 SERVICE_GLONASS = 2
    uint16 SERVICE_COMPASS = 4
    uint16 SERVICE_GALILEO = 8
    uint16 service
    """)
definefile(joinpath(PREFIX, "share", "sensor_msgs", "msg", "NavSatFix.msg"), """
    std_msgs/Header header
    NavSatStatus status
    float64 latitude
    float64 longitude
    float64 altitude
    float64[9] position_covariance
    uint8 COVARIANCE_TYPE_UNKNOWN = 0
    uint8 COVARIANCE_TYPE_APPROXIMATED = 1
    uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
    uint8 COVARIANCE_TYPE_KNOWN = 3
    uint8 position_covariance_type
    """)
definefile(joinpath(PREFIX, "share", "rosmessages_test", "msg", "Inner.msg"), "int16 value 5\n")
definefile(joinpath(PREFIX, "share", "rosmessages_test", "msg", "Blank.msg"), "# Only a comment\n")
definefile(joinpath(PREFIX, "share", "rosmessages_test", "msg", "Wide.msg"), "uint64 data\n")
definefile(joinpath(PREFIX, "share", "rosmessages_test", "msg", "Bytes.msg"), "uint8[8] data\n")
definefile(joinpath(PREFIX, "share", "rosmessages_test", "msg", "Tail.msg"), "uint32 x\nuint8 y\nuint8 z\n")
definefile(joinpath(PREFIX, "share", "rosmessages_test", "msg", "Mixed.msg"), """
    # A comment line
    uint8 KIND_A = 1
    string GREETING='hello'
    uint8 kind
    uint64 big 7
    float64[3] xyz [1.0, 2.0, 3.0]
    string<=5 name "abc"
    string[] names
    bool[2] flags [true, false]
    Inner inner
    Inner[2] inners
    Inner[<=3] more
    """)
definefile(joinpath(PREFIX, "share", "rosmessages_test", "srv", "Echo.srv"), "string text\n---\nstring text\nbool ok\n")
ENV["AMENT_PREFIX_PATH"] = join([PREFIX; INSTALLED_PREFIXES], ':')

newmodule() = (m = Module(); Core.eval(m, :(using ROSMessages, StaticArrays, Unitful)); m)
evalimport(m::Module, ex) = Core.eval(m, macroexpand(m, ex))
expansionerror(m::Module, ex) = try
    macroexpand(m, ex)
    nothing
catch e
    e isa ArgumentError || rethrow()
    e
end
# Since Julia 1.12, code sees the definitions evaluated after it started only in a new world.
macro latest()
    return isdefined(Core, Symbol("@latestworld")) ? esc(:(Core.@latestworld)) : nothing
end

@importrosmsg immutable std_msgs/Header
@importrosmsg sensor_msgs/NavSatFix

@testset "ROSString" begin
    let abc = "abc"
        @test abc |> ROSString |> string == "ROSString(\"abc\")"
    end
    # Two-arg `show` of a null `ROSString` must be round-trippable.
    @test string(ROSString()) == "ROSString()"

    # Round-trippability: parsing the 2-arg `show` must yield an object whose 2-arg `show` is identical.
    for s in ("", "abc", "hello \"world\"", "back\\slash", "newline\nhere", "tab\there")
        @test (r = s |> ROSString |> repr) |> Meta.parse |> eval |> repr == r
    end
    @test (r = ROSString() |> repr) |> Meta.parse |> eval |> repr == r

    units = transcode(UInt16, "Grüße")
    GC.@preserve units begin
        wide = ROSU16String(pointer(units), length(units), length(units))
        @test unsafe_string(wide) == "Grüße" && repr(wide) == "ROSU16String(\"Grüße\")"
        @test wide == ROSU16String(pointer(units), length(units), length(units)) && wide != ROSU16String()
    end
    @test unsafe_string(ROSU16String()) == "" && repr(ROSU16String()) == "ROSU16String()"
end

@testset "Equality and hash compare contents" begin
    # Equal strings at different addresses.
    a, b = "frame", String(collect("frame"))
    GC.@preserve a b begin
        @test ROSString(a) == ROSString(b)
        @test hash(ROSString(a)) == hash(ROSString(b))
        @test ROSString(a) != ROSString("other")
        # `Header` is immutable, so its fallback `===` would compare the pointers.
        x, y = Header(frame_id = ROSString(a)), Header(frame_id = ROSString(b))
        @test x == y
        @test isequal(x, y)
        @test hash(x) == hash(y)
    end
    @test ROSString() == ROSString("")
    @test NavSatFix(latitude = -0.0) == NavSatFix(latitude = 0.0)
    @test !isequal(NavSatFix(latitude = -0.0), NavSatFix(latitude = 0.0))
end

@testset "Override syntax" begin
    o(ex) = (parsed = Override(ex); (parsed.rosnames, parsed.juliafield, parsed.type, parsed.default))
    @test :(@override ros_field) |> o == ([:ros_field], :ros_field, nothing, nothing)
    @test :(@override ros_field = 1+0) |> o == ([:ros_field], :ros_field, nothing, Some(:(1+0)))
    @test :(@override ros_field = nothing) |> o == ([:ros_field], :ros_field, nothing, Some(:nothing))
    @test :(@override ros_field::JuliaType) |> o == ([:ros_field], :ros_field, :JuliaType, nothing)
    @test :(@override ros_field::JuliaType = a) |> o == ([:ros_field], :ros_field, :JuliaType, Some(:a))
    @test :(@override ros_field as julia_field) |> o == ([:ros_field], :julia_field, nothing, nothing)
    @test :(@override ros_field as julia_field = a) |> o == ([:ros_field], :julia_field, nothing, Some(:a))
    @test :(@override ros_field as julia_field::JuliaType) |> o == ([:ros_field], :julia_field, :JuliaType, nothing)
    @test :(@override ros_field as julia_field::JuliaType = a) |> o == ([:ros_field], :julia_field, :JuliaType, Some(:a))
    @test :(@override (a, b)::JuliaType) |> o == ([:a, :b], :a, :JuliaType, nothing)
    @test :(@override (a, b) as c::JuliaType = d) |> o == ([:a, :b], :c, :JuliaType, Some(:d))
    @test :(@override CONSTANT as ALIAS::Flag) |> o == ([:CONSTANT], :ALIAS, :Flag, nothing)
    @test :(@override CONSTANT as Msg.ALIAS::Flag) |> o == ([:CONSTANT], :ALIAS, :Flag, nothing)
    # The target tells a module constant (a name) from a renamed property (a qualified name) and from no rename (`nothing`).
    @test [Override(ex).target for ex in (:(@override C), :(@override C as D), :(@override C as Msg.D::T))] == [nothing, :D, :(Msg.D)]
    for invalid in (:(@override a b), :(@override (a,) as b), :(@override a as b.c.d), :(@override a as (b, c)), :(@override f(a)), :(@other a), :(a = 1))
        @test_throws ArgumentError Override(invalid)
    end
    @test_throws ArgumentError macroexpand(@__MODULE__, :(@override a))
end

@testset "Definition parsing" begin
    name = MsgName("p", "msg", "M")
    d = parse_definition("int8 A=-1\nint8 B = 2 # spaces around `=`\n\tuint8 tabbed\nint8 x\nint8 y 3", name)
    @test [(c.name, c.value) for c in d.constants] == [(:A, Int8(-1)), (:B, Int8(2))]
    @test typeof(d.constants[1].value) == Int8
    @test [(f.name, f.default) for f in d.fields] == [(:tabbed, nothing), (:x, nothing), (:y, 3)]
    # Like the reference parser, a `=` makes a line a constant even in a string default, and a `#` starts a comment even in a string.
    @test_throws ArgumentError parse_definition("string s \"a = b\"", name)
    @test parse_definition("string t 'with # hash'", name).fields[1].default == "'with"

    @test parse_value(parse_type("bool", ""), "FALSE") === false
    @test parse_value(parse_type("uint16", ""), "0x10") === 0x0010
    @test parse_value(parse_type("float32", ""), "1e3") === 1f3
    @test parse_value(parse_type("string<=3", ""), "\"abc\"") == "abc"
    @test parse_value(parse_type("wstring", ""), "'\\'q\\''") == "'q'"
    @test parse_value(parse_type("string[2]", ""), "[\"a,b\", c]") == ["a,b", "c"]
    @test parse_value(parse_type("int8[]", ""), "[]") == Int8[]
    @test parse_value(parse_type("int8[<=2]", ""), "[1,2]") == Int8[1, 2]
    for (type, value) in (("bool", "yes"), ("int8", "128"), ("uint8", "-1"), ("float64", "pi"),
                          ("int8[2]", "[1]"), ("int8[2]", "1, 2"), ("string", "\"a\"b\""), ("Header", "x"))
        @test_throws ArgumentError parse_value(parse_type(type, ""), value)
    end

    @test parse_type("time", "p") == ROSMessages.Nested{:builtin_interfaces, :Time}
    @test parse_type("duration", "p") == ROSMessages.Nested{:builtin_interfaces, :Duration}
    @test parse_type("other_pkg/Thing[4]", "p") == SVector{4, ROSMessages.Nested{:other_pkg, :Thing}}
    @test parse_type("string<=3[<=2]", "p") == ROSMessages.Sequence{ROSString}
    for type in ("int8[0]", "int8[x]", "int8[<=]", "string<=0", "int8<=2", "a/b/C", "bad__pkg/Thing", "p/lowercase", "Under_score")
        @test_throws ArgumentError parse_type(type, "p")
    end

    for line in ("int8", "int8 Bad", "int8 bad_", "int8 bad__name", "int8 _x", "int8 lower = 1", "int8 X_ = 1", "Inner X = 1", "int8[2] X = [1, 2]", "Inner x 1")
        err = try
            parse_definition("int8 ok\n$line", name)
            nothing
        catch e
            e isa ArgumentError || rethrow()
            e
        end
        @test err isa ArgumentError
        @test startswith(err.msg, "p/msg/M:2: ")
    end

    @test only(parse_definition("# no fields\nint8 C = 1", name).fields).name === :structure_needs_at_least_one_member
    response = read_definition(MsgName("rosmessages_test", "srv", "Echo_Response"))
    @test [f.name for f in response.fields] == [:text, :ok]
    @test response.fields[2].line == 4
    @test_throws ArgumentError read_definition(MsgName("rosmessages_test", "msg", "Missing"))
end

@testset "Override application" begin
    def = parse_definition("int32 X = 1\nfloat64 latitude\nfloat64 longitude\nuint8 status\nint32 Y = 2", MsgName("p", "msg", "Fix"))
    apply(lines...) = apply_overrides(def, Override[Override(l) for l in lines], identity)
    fields, constants = apply(:(@override X as Z::Flag), :(@override (latitude, longitude) as position::Position = Position(0, 0)), :(@override status = 3))
    @test [(f.name, f.type, f.default, f.rosfields) for f in fields] == [(:position, :Position, :(Position(0, 0)), 1:2), (:status, :UInt8, 3, 3:3)]
    @test [(c.name, o.juliafield, o.type) for (c, o) in constants] == [(:X, :Z, :Flag)]
    @test apply(:(@override status::Status))[1][3].default == :(Status())
    for invalid in ((:(@override latitude), :(@override X)),                      # order
                    (:(@override longitude), :(@override longitude)),             # duplicate
                    (:(@override missing),),                                      # unknown
                    (:(@override (latitude, status)::T),),                        # not consecutive
                    (:(@override (latitude, longitude)),),                        # no type
                    (:(@override (status, Y)::T),),                               # beyond the fields
                    (:(@override X = 3),),                                        # constant value
                    (:(@override (X, latitude)::T),),                             # constant in a group
                    (:(@override status as Fix.state),))                          # qualified field
        @test_throws ArgumentError apply(invalid...)
    end
end

@testset "Parametric reconstructors" begin
    m = newmodule()
    Core.eval(m, :(@kwdef! mutable struct Parametric{T <: Integer, U <: AbstractFloat}
        value::T = zero(T)
        scale::U = one(U)
        required::Int
    end))
    Core.eval(m, :(@kwdef! struct Immutable{T}
        value::T = zero(T)
    end))
    @latest
    x = m.Parametric{Int32, Float64}(value = 7, scale = 2, required = 3)
    @test m.Parametric!(x; required = 4) === x
    @test (x.value, x.scale, x.required) === (Int32(0), 1.0, 4)
    @test m.Parametric!(x; value = 5, scale = 3, required = 6) === x
    @test (x.value, x.scale, x.required) === (Int32(5), 3.0, 6)
    @test_throws UndefKeywordError m.Parametric!(x)
    @test !isdefined(m, :Immutable!)
    evalimport(m, :(@importrosmsg rosmessages_test/Inner as GenericInner{T}))
    @latest
    inner = m.GenericInner{Int32}(value = 7)
    @test m.GenericInner!(inner) === inner && inner.value == 5
    @test m.GenericInner!(inner; value = 9) === inner && inner.value == 9
end

@testset "Imports" begin
    m = newmodule()
    evalimport(m, :(@importrosmsg immutable rosmessages_test/Mixed))
    @latest
    x = m.Mixed()
    @test (x.kind, x.big, x.xyz, x.flags) == (0x00, UInt64(7), SVector(1.0, 2.0, 3.0), SVector(true, false))
    @test x.inner == m.Inner(5) && x.inners == fill(m.Inner(5), SVector{2, m.Inner}) && isempty(x.more)
    # Strings and sequences start empty, as only C may allocate their memory.
    @test x.name == ROSString() && x.names isa Sequence{ROSString} && isempty(x.names)
    @test m.Mixed.KIND_A === 0x01 && m.Mixed.GREETING == "hello"
    @test !ismutabletype(m.Mixed) && ROSMessages.rosname(m.Mixed) == "rosmessages_test/msg/Mixed"
    @test x == m.Mixed() && isequal(x, m.Mixed()) && hash(x) == hash(m.Mixed())
    @test x != m.Mixed(kind = 1)
    # A second import of the same type in the same mutability is a no-op, another mutability is an error.
    @test evalimport(m, :(@importrosmsg immutable rosmessages_test/Mixed)) === nothing
    @test expansionerror(m, :(@importrosmsg rosmessages_test/Mixed)) isa ArgumentError
    @test importrosmsg(m, "rosmessages_test", "msg", "Mixed"; mutable = false) === :Mixed

    # Explicit imports are mutable by default, nested types are always immutable.
    evalimport(m, :(@importrosmsg rosmessages_test/Mixed as MutableMixed))
    @latest
    y = m.MutableMixed()
    @test ismutabletype(m.MutableMixed) && fieldtype(m.MutableMixed, :inner) === m.Inner && !ismutabletype(m.Inner)
    @test m.MutableMixed!(y; kind = 2) === y && y.kind == 2
    @test copy!(m.MutableMixed(), y) == y
    # Without a Julia name, `importrosmsg` finds the type by its ROS name and mutability.
    @test importrosmsg(m, "rosmessages_test", "msg", "Mixed") === :MutableMixed

    evalimport(m, :(@importrosmsg rosmessages_test/Blank))
    @test importrosmsg(m, "rosmessages_test", "srv", "Echo") == (:Echo_Request, :Echo_Response)
    @latest
    @test fieldnames(m.Blank) == (:structure_needs_at_least_one_member,) && ismutabletype(m.Blank)
    @test fieldnames(m.Echo_Response) == (:text, :ok) && ismutabletype(m.Echo_Request)
    @test expansionerror(m, :(@importrosmsg rosmessages_test/srv/Echo as E begin
        @override text
    end)) isa ArgumentError

    # Nested types are found by their ROS name, also under another Julia name.
    m = newmodule()
    evalimport(m, :(@importrosmsg immutable rosmessages_test/Inner as Renamed))
    evalimport(m, :(@importrosmsg rosmessages_test/Mixed))
    @latest
    @test fieldtype(m.Mixed, :inner) === m.Renamed && !isdefined(m, :Inner)

    # A mutable import cannot be nested, as Julia would store it as a pointer.
    m = newmodule()
    evalimport(m, :(@importrosmsg rosmessages_test/Inner))
    @latest
    err = expansionerror(m, :(@importrosmsg rosmessages_test/Mixed))
    @test err isa ArgumentError && contains(err.msg, "@importrosmsg immutable rosmessages_test/msg/Inner as OtherName")
    evalimport(m, :(@importrosmsg immutable rosmessages_test/Inner as InlineInner))
    evalimport(m, :(@importrosmsg rosmessages_test/Mixed))
    @latest
    @test fieldtype(m.Mixed, :inner) === m.InlineInner

    # A name that is taken by something else is no reason to silently reuse it.
    m = newmodule()
    Core.eval(m, :(struct Inner; value::Int16; end))
    @latest
    @test expansionerror(m, :(@importrosmsg rosmessages_test/Mixed)) isa ArgumentError
    @test expansionerror(m, :(@importrosmsg rosmessages_test/Inner)) isa ArgumentError
    @test expansionerror(newmodule(), :(@importrosmsg std_msgs/Char)) isa ArgumentError
    evalimport(m, :(@importrosmsg std_msgs/Char as ROSChar))
    @latest
    @test sizeof(m.ROSChar) == 1

    for invalid in (:(@importrosmsg), :(@importrosmsg Header), :(@importrosmsg lazy std_msgs/Header), :(@importrosmsg std_msgs/action/Header),
                    :(@importrosmsg std_msgs/Header as), :(@importrosmsg std_msgs/Header as Stamped extra), :(@importrosmsg std_msgs/Header in file),
                    :(@importrosmsg std_msgs/Header in include("Header.jl")), :(@importrosmsg std_msgs/Header in "a.jl" as B in "b.jl"),
                    :(@importrosmsg Std_msgs/Header), :(@importrosmsg std_msgs/header))
        @test expansionerror(newmodule(), invalid) isa ArgumentError
    end
end

@testset "Overrides" begin
    m = newmodule()
    evalimport(m, :(@importrosmsg immutable builtin_interfaces/Time as ROSTime begin
        @override sec::typeof(Int32(0)u"s") = 0u"s"
        @override nanosec::typeof(UInt32(0)u"ns") = 0u"ns"
    end))
    @latest
    @test m.ROSTime().sec === Int32(0)u"s"
    evalimport(m, :(@importrosmsg immutable std_msgs/Header begin
        @override stamp::ROSTime
        @override frame_id as frame
    end))
    @latest
    @test fieldnames(m.Header) == (:stamp, :frame) && m.Header().stamp == m.ROSTime()
    evalimport(m, :(@importrosmsg sensor_msgs/NavSatFix begin
        @override (latitude, longitude) as coordinates::NTuple{2, typeof(1.0u"°")} = (NaN * u"°", NaN * u"°")
        @override altitude::typeof(1.0u"m") = NaN * u"m"
        @override position_covariance::SMatrix{3, 3, typeof(1.0u"m^2"), 9} = zeros(SMatrix{3, 3, typeof(1.0u"m^2")})
        @override COVARIANCE_TYPE_KNOWN as KNOWN
    end))
    status_module = newmodule()
    evalimport(status_module, :(@importrosmsg immutable sensor_msgs/NavSatStatus as Status begin
        @override STATUS_NO_FIX
        @override STATUS_FIX as FIX::Int16
        @override STATUS_SBAS_FIX as Status.SBAS_FIX
        @override STATUS_GBAS_FIX::Int16
        @override SERVICE_GPS as Status.GPS::UInt32
    end))
    @latest
    @test fieldtype(m.NavSatFix, :header) === m.Header && sizeof(m.NavSatFix) == 144
    @test isnan(m.NavSatFix().altitude) && m.KNOWN === 0x03
    @test UInt8(m.COVARIANCE_TYPE.KNOWN) === 0x03
    @test_throws (VERSION >= v"1.12" ? FieldError : ErrorException) m.NavSatFix.COVARIANCE_TYPE_KNOWN
    # An unqualified name defines a module constant and keeps the property, a qualified one renames the property.
    @test status_module.FIX === Int16(0) && status_module.Status.STATUS_FIX === Int8(0)
    @test status_module.Status.SBAS_FIX === Int8(1) && status_module.Status.GPS === UInt32(1) && status_module.Status.STATUS_GBAS_FIX === Int16(2)
    @test !isdefined(status_module, :SBAS_FIX) && !isdefined(status_module, :STATUS_NO_FIX) && status_module.Status.STATUS_NO_FIX === Int8(-1)
    @test UInt16(status_module.SERVICE.GLONASS) == 0x0002
    @test !isdefined(status_module, :STATUS)
    @test_throws (VERSION >= v"1.12" ? FieldError : ErrorException) status_module.Status.STATUS_SBAS_FIX
    for invalid in (:(@override STATUS_FIX as NavSatStatus.FIX),        # not the Julia name of the imported type
                    :(@override STATUS_FIX as Status2.STATUS_SBAS_FIX))  # duplicate property
        @test_throws ArgumentError evalimport(status_module, :(@importrosmsg immutable sensor_msgs/NavSatStatus as Status2 begin
            $invalid
        end))
    end

    # Overrides that change the memory layout.
    m = newmodule()
    Core.eval(m, :(mutable struct Boxed; value::Int16; end))
    @latest
    for (i, override) in enumerate((:(@override big::NTuple{8, UInt8}),   # same size, but no 8-byte alignment
                                    :(@override big::UInt32),
                                    :(@override inner::Boxed),            # mutable, so stored as a pointer
                                    :(@override inner::Integer),          # abstract, so stored as a pointer
                                    :(@override (kind, big)::Tuple{UInt8, UInt32})))
        @test_throws ArgumentError evalimport(m, :(@importrosmsg rosmessages_test/Mixed as $(Symbol(:Changed, i)) begin
            $override
        end))
    end
    for (name, override) in ((:Wide, :(@override data::NTuple{8, UInt8})),
                             (:Bytes, :(@override data::UInt64)))
        for mutability in (:mutable, :immutable)
            err = try
                evalimport(newmodule(), :(@importrosmsg $mutability $(Expr(:call, :/, :rosmessages_test, name)) begin
                    $override
                end))
                nothing
            catch e
                e isa ArgumentError || rethrow()
                e
            end
            @test err isa ArgumentError && contains(err.msg, "alignment")
        end
    end
    for nested in (false, true)
        m = newmodule()
        Core.eval(m, :(struct Reference; value::String; end))
        Core.eval(m, nested ?
            :(struct Managed; value::Reference; size::UInt; capacity::UInt; end) :
            :(struct Managed; value::String; size::UInt; capacity::UInt; end))
        @latest
        @test Base.allocatedinline(m.Managed) && !isbitstype(m.Managed)
        err = try
            evalimport(m, :(@importrosmsg immutable std_msgs/Header begin
                @override frame_id as renamed::Managed
            end))
            nothing
        catch e
            e isa ArgumentError || rethrow()
            e
        end
        @test err isa ArgumentError && contains(err.msg, "isbits") && contains(err.msg, "frame_id")
    end
    for fieldtype in (:Integer, :(Union{Int8, UInt8}))
        err = try
            evalimport(newmodule(), :(@importrosmsg immutable rosmessages_test/Wide begin
                @override data::$fieldtype
            end))
            nothing
        catch e
            e isa ArgumentError || rethrow()
            e
        end
        @test err isa ArgumentError && contains(err.msg, "isbits") && contains(err.msg, "data")
    end
    m = newmodule()
    evalimport(m, :(@importrosmsg immutable rosmessages_test/Tail))
    evalimport(m, :(@importrosmsg immutable rosmessages_test/Tail as GroupedTail begin
        @override (y, z) as pair::NTuple{2, UInt8}
    end))
    @latest
    @test sizeof(m.Tail) == sizeof(m.GroupedTail) == 8

end

@testset "Constant enums" begin
    m = newmodule()
    evalimport(m, :(@importrosmsg immutable sensor_msgs/NavSatStatus as Status))
    @latest
    @test Int8(m.STATUS.UNKNOWN) == -2
    @test Int8(m.STATUS.NO_FIX) == -1
    @test UInt16(m.SERVICE.GPS) == 1
    @test_throws (VERSION >= v"1.12" ? FieldError : ErrorException) m.Status.STATUS_FIX

    evalimport(m, :(@importrosmsg sensor_msgs/NavSatStatus as MutableStatus))
    @latest
    @test Int8(m.STATUS.GBAS_FIX) == 2 && Int8(m.STATUS.NO_FIX) == -1

    collision = newmodule()
    Core.eval(collision, :(const STATUS = :already_defined))
    @test_throws ArgumentError evalimport(collision, :(@importrosmsg sensor_msgs/NavSatStatus))
    @test_throws ArgumentError evalimport(newmodule(), :(@importrosmsg sensor_msgs/NavSatStatus as STATUS))

    aliases = newmodule()
    def = parse_definition("uint8 STATE_FIRST=1\nuint8 STATE_ALIAS=1", MsgName("rosmessages_test", "msg", "Aliases"), "aliases.msg", 10)
    Core.eval(aliases, :(struct Aliases end))
    Core.eval(aliases, :(ROSMessages.rosname(::Type{Aliases}) = "rosmessages_test/msg/Aliases"))
    context = ROSMessages.ImportContext(aliases)
    push!(context.bindings, :Aliases)
    code = ROSMessages.constantcode(context, :Aliases, def, [])
    @test first(code).args[2] == LineNumberNode(10, Symbol("aliases.msg"))
    Core.eval(aliases, Expr(:toplevel, code...))
    @latest
    @test aliases.STATE.FIRST === aliases.STATE.ALIAS
    @test ROSMessages.EnumX.symbol_map(aliases.STATE.T) == [:FIRST => UInt8(1), :ALIAS => UInt8(1)]
    @test ROSMessages.enum_message(aliases.STATE.T) === aliases.Aliases
    @test isempty(ROSMessages.constantcode(ROSMessages.ImportContext(aliases), :Aliases, def, []))
    for (name, text) in (
        (MsgName("other", "msg", "Aliases"), "uint8 STATE_FIRST=1\nuint8 STATE_ALIAS=1"),
        (MsgName("rosmessages_test", "srv", "Aliases"), "uint8 STATE_FIRST=1\nuint8 STATE_ALIAS=1"),
        (MsgName("rosmessages_test", "msg", "Other"), "uint8 STATE_FIRST=1\nuint8 STATE_ALIAS=1"),
        (def.name, "uint16 STATE_FIRST=1\nuint16 STATE_ALIAS=1"),
        (def.name, "uint8 STATE_FIRST=2\nuint8 STATE_ALIAS=1"),
        (def.name, "uint8 STATE_FIRST=1\nuint8 STATE_OTHER=1"),
        (def.name, "uint8 STATE_ALIAS=1\nuint8 STATE_FIRST=1"))
        changed = parse_definition(text, name)
        @test_throws ArgumentError ROSMessages.constantcode(ROSMessages.ImportContext(aliases), :Aliases, changed, [])
    end

    foreign = newmodule()
    Core.eval(foreign, :(const STATE = $(QuoteNode(aliases.STATE))))
    @test_throws ArgumentError ROSMessages.constantcode(ROSMessages.ImportContext(foreign), :Aliases, def, [])

    type_name_module = newmodule()
    type_name_def = parse_definition("uint8 STATE_T=1\nuint8 STATE_OTHER=2",
        MsgName("rosmessages_test", "msg", "TypeName"), "type-name.msg")
    Core.eval(type_name_module, :(struct TypeName end))
    Core.eval(type_name_module, :(ROSMessages.rosname(::Type{TypeName}) = "rosmessages_test/msg/TypeName"))
    type_name_code = ROSMessages.constantcode(
        ROSMessages.ImportContext(type_name_module), :TypeName, type_name_def, [])
    @test sprint(ROSMessages.printcode, first(type_name_code)) == """
        ROSMessages.EnumX.@enumx T = T_ STATE::UInt8 begin
            T = 0x01
            OTHER = 0x02
        end"""
    Core.eval(type_name_module, Expr(:toplevel, type_name_code...))
    @latest
    @test type_name_module.STATE.T == type_name_module.STATE.T_(1)
    @test isempty(ROSMessages.constantcode(
        ROSMessages.ImportContext(type_name_module), :TypeName, type_name_def, []))

    parametric = newmodule()
    evalimport(parametric, :(@importrosmsg immutable sensor_msgs/NavSatStatus as ParamStatus{T}))
    @latest
    @test ROSMessages.enum_message(parametric.STATUS.T) === parametric.ParamStatus
    evalimport(parametric, :(@importrosmsg sensor_msgs/NavSatStatus as OrdinaryStatus))
    @latest
    @test Int8(parametric.STATUS.NO_FIX) == -1

    callsite = LineNumberNode(42, :source)
    source_context = ROSMessages.ImportContext(newmodule(), callsite)
    push!(source_context.bindings, :Aliases)
    source_code = ROSMessages.constantcode(source_context, :Aliases, def, [])
    @test first(source_code).args[2] == callsite
    @test !contains(sprint(ROSMessages.printcode, first(source_code)), "#=")
end

@testset "Enum groups in nested imports" begin
    share = joinpath(PREFIX, "share", "rosmessages_test")
    definefile(joinpath(share, "msg", "SharedEnum.msg"),
        "uint8 STATE_FIRST=1\nuint8 STATE_ALIAS=1\nuint8 value\n")
    definefile(joinpath(share, "msg", "Left.msg"), "SharedEnum shared\n")
    definefile(joinpath(share, "msg", "Right.msg"), "SharedEnum shared\n")
    definefile(joinpath(share, "msg", "Diamond.msg"), "Left left\nRight right\n")
    definefile(joinpath(share, "srv", "Shared.srv"), "SharedEnum input\n---\nSharedEnum output\n")
    for name in ("First", "Second")
        definefile(joinpath(share, "msg", "$name.msg"), "uint8 MODE_A=1\nuint8 MODE_B=2\nuint8 value\n")
    end
    definefile(joinpath(share, "msg", "Collision.msg"), "First first\nSecond second\n")
    definefile(joinpath(share, "msg", "CollisionReversed.msg"), "Second second\nFirst first\n")

    diamond = newmodule()
    evalimport(diamond, :(@importrosmsg rosmessages_test/Diamond))
    @latest
    @test fieldtype(diamond.Left, :shared) === fieldtype(diamond.Right, :shared) === diamond.SharedEnum
    @test diamond.STATE.FIRST === diamond.STATE.ALIAS
    @test ROSMessages.enum_message(diamond.STATE.T) === diamond.SharedEnum

    service = newmodule()
    evalimport(service, :(@importrosmsg rosmessages_test/srv/Shared))
    @latest
    @test fieldtype(service.Shared_Request, :input) === fieldtype(service.Shared_Response, :output) === service.SharedEnum
    @test service.STATE.FIRST === service.STATE.ALIAS

    for name in (:Collision, :CollisionReversed)
        m = newmodule()
        err = expansionerror(m, :(@importrosmsg $(Expr(:call, :/, :rosmessages_test, name))))
        @test err isa ArgumentError && contains(err.msg, "EnumX enum `MODE`")
        @test !isdefined(m, :MODE)
    end
end

@testset "Generated file" begin
    dir = mktempdir()
    source = joinpath(dir, "source.jl")
    file = joinpath(dir, "gen", "Vec.jl")
    location = LineNumberNode(1, Symbol(source))
    spec = ROSMessages.ImportSpec((:(std_msgs/Header in "gen/Header.jl"),), location)
    @test spec.file == joinpath(dir, "gen", "Header.jl")
    spec = ROSMessages.ImportSpec((:(std_msgs/Header), :as, :(Stamped{T} <: AbstractVector{T} in "gen/Stamped.jl")), location)
    @test spec.file == joinpath(dir, "gen", "Stamped.jl") && spec.type.params == [:T] &&
        spec.type.supertype == :(AbstractVector{T})
    absolute_file = joinpath(dir, "Absolute.jl")
    spec = ROSMessages.ImportSpec((Expr(:call, :in, :(std_msgs/Header), absolute_file),), location)
    @test spec.file == absolute_file
    importcode(value) = """
        const L = 2
        @importrosmsg geometry_msgs/Vector3 as Vec in "gen/Vec.jl" begin
            @override (x, y) as xy::NTuple{L, Float64} = ntuple(_ -> $value, L)
        end
        Base.length(::Type{Vec}) = L
        """
    write(source, importcode(1.0))
    m = newmodule()
    Base.include(m, source)
    @latest
    @test m.Vec().xy == (1.0, 1.0) && length(m.Vec) == 2
    text = read(file, String)
    @test text == """
        # This file is generated by `@importrosmsg` in ../source.jl from geometry_msgs/msg/Vector3.msg.
        # Do not edit it: `@importrosmsg` overwrites it whenever the generated content changes.

        ROSMessages.@kwdef! mutable struct Vec
            xy::NTuple{L, Float64} = ntuple((_->1.0), L)
            z::Float64 = 0.0
        end
        """
    @test !contains(text, "check_layout") && !contains(text, "Base.hash") && !contains(text, "ROSMessages.rosname")
    @test ROSMessages.rosname(m.Vec) == "geometry_msgs/msg/Vector3"
    @test m.Vec() == m.Vec() && isequal(m.Vec(), m.Vec()) && hash(m.Vec()) == hash(m.Vec())
    @test copy!(m.Vec(), m.Vec(z = 3.0)).z == 3.0
    @test getfield(m, :Vec!)(m.Vec(); z = 4.0).z == 4.0
    # Without changes, the file is not written, which would fail for a read-only file.
    chmod(file, 0o444)
    Base.include(newmodule(), source)
    @test read(file, String) == text
    chmod(file, 0o644)
    write(source, importcode(2.0))
    m = newmodule()
    Base.include(m, source)
    @latest
    @test m.Vec().xy == (2.0, 2.0)
    @test contains(read(file, String), "ntuple((_->2.0), L)")
    # Evaluating the import a second time defines nothing, as the types exist.
    Base.include(m, source)

    managed_source = joinpath(dir, "managed_source.jl")
    managed_file = joinpath(dir, "gen", "ManagedHeader.jl")
    write(managed_source, """
        @importrosmsg immutable std_msgs/Header as ManagedHeader in "gen/ManagedHeader.jl" begin
            @override frame_id as renamed::Managed
        end
        """)
    managed_module = newmodule()
    Core.eval(managed_module, :(struct Managed; value::String; size::UInt; capacity::UInt; end))
    err = try
        Base.include(managed_module, managed_source)
        nothing
    catch e
        e isa LoadError || rethrow()
        e
    end
    @test err isa LoadError && contains(sprint(showerror, err), "isbits") && contains(sprint(showerror, err), "frame_id")
    @test !contains(read(managed_file, String), "check_layout")

    deferred_module = newmodule()
    deferred_file = joinpath(dir, "gen", "Deferred.jl")
    ex = Meta.parse("@importrosmsg immutable rosmessages_test/Bytes as Deferred in \"gen/Deferred.jl\"")
    ex.args[2] = LineNumberNode(1, Symbol(source))
    expanded = macroexpand(deferred_module, ex)
    write(deferred_file, "error(\"the generated file was executed\")\n")
    Core.eval(deferred_module, expanded)
    @latest
    @test ROSMessages.rosname(deferred_module.Deferred) == "rosmessages_test/msg/Bytes"

    param_source = joinpath(dir, "param_source.jl")
    param_file = joinpath(dir, "gen", "ByteVector.jl")
    write(param_source, """
        @importrosmsg immutable rosmessages_test/Bytes as ByteVector{T} <: AbstractVector{T} in "gen/ByteVector.jl"
        """)
    param_module = newmodule()
    Base.include(param_module, param_source)
    @latest
    @test param_module.ByteVector{UInt8} <: AbstractVector{UInt8}
    @test ROSMessages.rosname(param_module.ByteVector{UInt8}) == "rosmessages_test/msg/Bytes"
    @test contains(read(param_file, String), "struct ByteVector{T} <: AbstractVector{T}")

    service_source = joinpath(dir, "service_source.jl")
    service_file = joinpath(dir, "gen", "Echo.jl")
    write(service_source, "@importrosmsg rosmessages_test/srv/Echo in \"gen/Echo.jl\"\n")
    service_module = newmodule()
    Base.include(service_module, service_source)
    @latest
    @test ROSMessages.rosname(service_module.Echo_Request) == "rosmessages_test/srv/Echo_Request"
    @test ROSMessages.rosname(service_module.Echo_Response) == "rosmessages_test/srv/Echo_Response"
    service_text = read(service_file, String)
    @test contains(service_text, "struct Echo_Request") && contains(service_text, "struct Echo_Response") &&
        !contains(service_text, "service_typesupport")

    share = joinpath(PREFIX, "share", "rosmessages_test", "msg")
    definefile(joinpath(share, "ChildWithConstant.msg"), "uint8 VALUE=1\nuint8 data\n")
    definefile(joinpath(share, "ParentWithConstant.msg"), "ChildWithConstant child\nuint8 data\n")
    nested_source = joinpath(dir, "nested_source.jl")
    nested_file = joinpath(dir, "gen", "Parent.jl")
    write(nested_source, """
        @importrosmsg immutable rosmessages_test/ParentWithConstant as Parent in "gen/Parent.jl" begin
            @override data::typeof(ChildWithConstant.VALUE)
        end
        """)
    nested_module = newmodule()
    Base.include(nested_module, nested_source)
    @latest
    @test fieldtype(nested_module.Parent, :data) === UInt8
    @test nested_module.ChildWithConstant.VALUE === UInt8(1)
    @test !contains(read(nested_file, String), "Base.getproperty")

    constant_source = joinpath(dir, "constant_source.jl")
    constant_file = joinpath(dir, "gen", "Mixed.jl")
    write(constant_source, """
        @importrosmsg rosmessages_test/Mixed in "gen/Mixed.jl" begin
            @override KIND_A as KIND
        end
        """)
    constant_module = newmodule()
    Base.include(constant_module, constant_source)
    @latest
    @test constant_module.KIND === UInt8(1)
    @test !contains(read(constant_file, String), "KIND =")

    enum_source = joinpath(dir, "enum_source.jl")
    enum_file = joinpath(dir, "gen", "Status.jl")
    write(enum_source, "@importrosmsg sensor_msgs/NavSatStatus as Status in \"gen/Status.jl\"\n")
    enum_module = newmodule()
    Base.include(enum_module, enum_source)
    @latest
    @test UInt16(enum_module.SERVICE.GPS) == 1
    enum_text = read(enum_file, String)
    @test !contains(enum_text, "#=")
    @test contains(enum_text, """
        ROSMessages.EnumX.@enumx T = T SERVICE::UInt16 begin
            UNKNOWN = 0x0000
            GPS = 0x0001
            GLONASS = 0x0002
            COMPASS = 0x0004
            GALILEO = 0x0008
        end

        ROSMessages.EnumX.@enumx T = T STATUS::Int8 begin
            UNKNOWN = -2
            NO_FIX = -1
            FIX = 0
            SBAS_FIX = 1
            GBAS_FIX = 2
        end
        """)
    @test !contains(enum_text, "enum_message") && !contains(enum_text, "check_layout")
    @test ROSMessages.enum_message(enum_module.SERVICE.T) === enum_module.Status
    evalimport(enum_module, :(@importrosmsg immutable sensor_msgs/NavSatStatus as OtherStatus))
    @latest
    @test UInt16(enum_module.SERVICE.GPS) == 1
end

# Compare the layouts computed from the definitions with the C headers of all installed messages and service parts.
@testset "C layout of installed messages" begin
    prefixes = INSTALLED_PREFIXES
    names = MsgName[]
    for prefix in prefixes, (dir, _, files) in walkdir(joinpath(prefix, "share")), f in files
        interface = basename(dir)
        package = basename(dirname(dir))
        isdir(joinpath(prefix, "include", package)) || continue
        if interface == "msg" && endswith(f, ".msg")
            push!(names, MsgName(package, "msg", chop(f; tail = 4)))
        elseif interface == "srv" && endswith(f, ".srv")
            append!(names, MsgName(package, "srv", chop(f; tail = 4) * suffix) for suffix in ("_Request", "_Response"))
        end
    end
    unique!(names)
    cc = Sys.which("cc")
    if isempty(names)
        @test_skip "no installed ROS message headers"
    elseif isnothing(cc)
        @test_skip "no C compiler"
    else
        definitions = Dict{MsgName, ROSMessages.Definition}()
        definition(n) = get!(() -> read_definition(n), definitions, n)
        asserts = String[]
        withenv("AMENT_PREFIX_PATH" => join(prefixes, ':')) do
            for name in names
                def = definition(name)
                layout = clayout(def, definition)
                c = "$(name.package)__$(name.interface)__$(name.name)"
                push!(asserts, "_Static_assert(sizeof($c) == $(layout.size), \"$c\");")
                push!(asserts, "_Static_assert(_Alignof($c) == $(layout.alignment), \"$c alignment\");")
                append!(asserts, ("_Static_assert(offsetof($c, $(f.name)) == $(fieldlayout.offset), \"$c.$(f.name)\");" for (f, fieldlayout) in zip(def.fields, layout.fields)))
            end
        end
        flags = String[]
        includes = String[]
        for prefix in prefixes
            include = joinpath(prefix, "include")
            isdir(include) || continue
            append!(flags, ("-I$d" for d in readdir(include; join = true) if isdir(d)))
            for package in unique(n.package for n in names), interface in ("msg", "srv")
                detail = joinpath(include, package, package, interface, "detail")
                isdir(detail) && append!(includes, ("#include <$package/$interface/detail/$f>" for f in readdir(detail) if endswith(f, "__struct.h")))
            end
        end
        output = IOBuffer()
        source = IOBuffer(join(["#include <stddef.h>"; includes; asserts], '\n'))
        ok = success(pipeline(`$cc -fsyntax-only -x c $flags -`; stdin = source, stdout = output, stderr = output))
        ok || println(String(take!(output)))
        @test ok
    end
end
