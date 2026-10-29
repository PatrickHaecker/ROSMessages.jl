module ROSMessages

export @importrosmsg, @override, @kwdef!
export ROSString, ROSU16String, Sequence, Empty

using Base.Meta: isexpr
using EnumX
# ROS arrays are represented as `SVector`s, which have the same bit representation, which is crucially important, and provide the optimizations of `StaticArrays.jl` for arrays of primitive types.
using StaticArrays: SVector

include("reconstructor.jl")
include("memory.jl")
include("parse.jl")
include("layout.jl")
include("override.jl")
include("generate.jl")

end
