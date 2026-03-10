using LoopVectorization
using StaticArrays
using BenchmarkTools
using IfElse


V_0 = SVector(33.23942244, 33.18908023, 33.03475197)


@inline function seq_gemv_3!(A::Matrix{Float64}, x::Vector{Float64}, y::Vector{Float64}, z::Vector{Float64}, r::Vector{Float64})
    @tturbo for i ∈ eachindex(z)
        x[i] = A[1, 1] * x[i] + A[1, 2] * y[i] + A[1, 3] * z[i] + r[1]
        y[i] = A[1, 1] * x[i] + A[1, 2] * y[i] + A[1, 3] * z[i] + r[2]
        z[i] = A[1, 1] * x[i] + A[1, 2] * y[i] + A[1, 3] * z[i] + r[3]
    end
end


@inline function vadd!(x::Vector{Float64}, y::Vector{Float64})
    @tturbo for i ∈ eachindex(x)
        x[i] += y[i]
    end
end


@inline function vhadamard!(x::Vector{Float64}, y::Vector{Float64})
    @tturbo for i ∈ eachindex(x)
        x[i] *= y[i]
    end
end

#=
A = randn(3, 3)
r = randn(3)
x = randn(100)
y = randn(100)
z = randn(100)
v = [x'; y'; z']
seqgemv3!(A, x, y, z, r)
vfilter!(foo, x)
=#

#only tturbo on each time series
#exit loop once all solutions have acceptable error


A = randn(4)
B = randn(4)
C = zeros(4)

map(A, B, C) do x, y, z
    local a::Float64
    if x > y
        a = y
    elseif x > 0.0
        a = 0.0
    else
        a = x
    end
    z = a
end


