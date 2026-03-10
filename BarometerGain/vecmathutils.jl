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
@inline function vhadamard!(x::Vector{Float64}, y::BitVector)
    @tturbo for i ∈ eachindex(x)
        x[i] *= y[i]
    end
end


@inline function seq_gemv_3!(A::Matrix{Float64}, x::Vector{Float64}, y::Vector{Float64}, z::Vector{Float64}, r::Vector{Float64})
    @tturbo for i ∈ eachindex(z)
        xi = A[1, 1] * x[i] + A[1, 2] * y[i] + A[1, 3] * z[i] + r[1]
        yi = A[2, 1] * x[i] + A[2, 2] * y[i] + A[2, 3] * z[i] + r[2]
        zi = A[3, 1] * x[i] + A[3, 2] * y[i] + A[3, 3] * z[i] + r[3]
        x[i] = xi
        y[i] = yi
        z[i] = zi
    end
end


