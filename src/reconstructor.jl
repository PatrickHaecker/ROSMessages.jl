# This is in a separate file, as ideally, this can be upstreamed and removed from ROSMessages. It extends the code from https://github.com/JuliaLang/julia/blob/master/base/util.jl. License is MIT: https://julialang.org/license

using Base: isexpr, extract_names_and_defvals_from_kwdef_fieldblock!

"""
    @kwdef! typedef

This is a helper macro that automatically defines a keyword-based constructor for the type
declared in the expression `typedef`, which must be a `struct` or `mutable struct`
expression. The default argument is supplied by declaring fields of the form `field::T =
default` or `field = default`. If no default is provided then the keyword argument becomes
a required keyword argument in the resulting type constructor.

Inner constructors can still be defined, but at least one should accept arguments in the
same form as the default inner constructor (i.e. one positional argument per field) in
order to function correctly with the keyword outer constructor.

Additionally, for a `mutable struct`, a keyword-based "reconstructor" is defined with a
trailing bang (`!`). This enables in-place modification of a mutable object, equivalent to
how an immutable object is handled, using the same argument syntax as the keyword-based
constructor . The "reconstructor" combines the convenience of the keyword-based constructor
with the efficiency or necessity of reusing an object.

# Examples
```jldoctest
julia> @kwdef struct Foo
           a::Int = 1         # specified default
           b::String          # required keyword
       end
Foo

julia> Foo(b="hi")
Foo(1, "hi")

julia> Foo()
ERROR: UndefKeywordError: keyword argument `b` not assigned
Stacktrace:
[...]
```

```jldoctest
julia> @kwdef! mutable struct Bar
           a::Int = 0
           b::Tuple{Int, Int} = (0, 0)
        end
Bar

julia> immutable = Some(Bar(42, (1, 2)))
Some(Bar(42, (1, 2)))

julia> Bar!(immutable.value)
Bar(0, (0, 0))

julia> Bar!(immutable.value, a=43)
Bar(43, (0, 0))

julia> immutable
Some(Bar(43, (0, 0)))
```
"""
macro kwdef!(expr)
    expr = macroexpand(__module__, expr) # to expand @static
    isexpr(expr, :struct) || error("Invalid usage of @kwdef!")
    mutable, T, fieldsblock = expr.args
    if T isa Expr && T.head === :<:
        T = T.args[1]
    end

    fieldnames = Any[]
    defvals = Any[]
    extract_names_and_defvals_from_kwdef_fieldblock!(fieldsblock, fieldnames, defvals)
    parameters = map(fieldnames, defvals) do fieldname, defval
        if isnothing(defval)
            return fieldname
        else
            return Expr(:kw, fieldname, esc(defval))
        end
    end

    # Only define a constructor if the type has fields, otherwise we'll get a stack
    # overflow on construction
    if !isempty(parameters)
        T_no_esc = Meta.unescape(T)
        if T_no_esc isa Symbol
            sig1 = Expr(:call, esc(T), Expr(:parameters, parameters...))
            body1 = Expr(:block, __source__, Expr(:call, esc(T), fieldnames...))
            def1 = Expr(:function, sig1, body1)

            if mutable
                # if struct is mutable define a constructor and a "reconstructor"
                #   T(...) = ...
                #   T!(obj; a, b, ...) = (obj.a = a; obj.b = b; ...; obj)
                T! = Symbol(T, "!")
                sig2 = Expr(:call, esc(T!), Expr(:parameters, parameters...), esc(:obj))
                assign = x -> Expr(:(=), Expr(:., esc(:obj), QuoteNode(x)), esc(x))
                assigns = (assign(x) for x in fieldnames)
                body2 = Expr(:block, __source__, assigns..., esc(:obj))
                def2 = Expr(:function, sig2, body2)
                # Reverse the definitions to return the type
                kwdefs = Expr(:block, def2, def1)
            else
                kwdefs = def1
            end
        elseif isexpr(T_no_esc, :curly)
            # if T == S{A<:AA,B<:BB}, define two methods
            #   S(...) = ...
            #   S{A,B}(...) where {A<:AA,B<:BB} = ...
            S = T.args[1]
            P = T.args[2:end]
            Q = Any[isexpr(U, :<:) ? U.args[1] : U for U in P]
            SQ = :($S{$(Q...)})
            body1 = Expr(:block, __source__, Expr(:call, esc(S), fieldnames...))
            sig1 = Expr(:call, esc(S), Expr(:parameters, parameters...))
            def1 = Expr(:function, sig1, body1)
            body2 = Expr(:block, __source__, Expr(:call, esc(SQ), fieldnames...))
            sig2 = :($(Expr(:call, esc(SQ), Expr(:parameters, parameters...))) where {$(esc.(P)...)})
            def2 = Expr(:function, sig2, body2)
            kwdefs = Expr(:block, def1, def2)
            if mutable
                S! = Symbol(S, "!")
                arg = Expr(:(::), esc(:obj), esc(SQ))
                sig3 = :($(Expr(:call, esc(S!), Expr(:parameters, parameters...), arg)) where {$(esc.(P)...)})
                assign = x -> Expr(:(=), Expr(:., esc(:obj), QuoteNode(x)), esc(x))
                body3 = Expr(:block, __source__, (assign(x) for x in fieldnames)..., esc(:obj))
                pushfirst!(kwdefs.args, Expr(:function, sig3, body3))
            end
        else
            error("Invalid usage of @kwdef!")
        end
    else
        kwdefs = nothing
    end
    return quote
        $(esc(:($Base.@__doc__ $expr)))
        $kwdefs
    end
end
