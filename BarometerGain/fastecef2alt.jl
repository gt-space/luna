# WGS84 reference ellipsoid properties
a_Earth::Float64 = 6378137.0                            # semi-major axis
f_Earth::Float64 = (298.257223563)^-1                   # flattening
b_Earth::Float64 = a_Earth * (1.0 - f_Earth)            # semi-minor axis
e_first::Float64 = sqrt(1.0 - (b_Earth * a_Earth^-1)^2) # first eccentricity
c_Earth::Float64 = a_Earth * e_first^2
e′::Float64      = √(1.0 - e_first^2)


function lla2ecef(φ::Float64, λ::Float64, h::Float64)
    local N::Float64 = a_Earth * √(1.0 - (e_first^2 * sin(φ)^2))^-1
    local r_ECEF::Vector{Float64} = [
    (N + h) * cos(φ) * cos(λ);
    (N + h) * cos(φ) * sin(λ);
    ((b_Earth^2 * a_Earth^-2) * N + h) * sin(φ)
    ]
    return r_ECEF
end


@inline function ecef2alt_solver_init!(t::Vector{Float64}, tₘ::Vector{Float64}, fₘ::Vector{Float64}, t₁::Vector{Float64}, t₀::Vector{Float64})
    local t_buffer::Float64
    for i ∈ eachindex(t)
        if tₘ[i] ≤ 0.0
            t_buffer = t₁[i]
        elseif tₘ[i] ≥ 1.0
            t_buffer = t₀[i]
        elseif fₘ[i] ≥ 0.0
            t_buffer = t₀[i]
        else
            t_buffer = t₁[i]
        end
        t[i] = t_buffer
    end
end


@inline function seq_ecef2alt_init!(t::Vector{Float64}, p::Vector{Float64}, u::Vector{Float64}, v::Vector{Float64}, x::Vector{Float64}, y::Vector{Float64}, z::Vector{Float64})
    tₘ::Vector{Float64} = zeros(Float64, length(t))
    fₘ::Vector{Float64} = zeros(Float64, length(t))
    t₁::Vector{Float64} = zeros(Float64, length(t))
    t₀::Vector{Float64} = zeros(Float64, length(t))
    # Calculate quartic polynomial parameters:
    @tturbo for n ∈ eachindex(z)
        pn = √(x[n]^2 + y[n]^2)
        z′n = e′ * abs(z[n])
        un = 2.0(z′n - c_Earth)
        vn = 2.0(z′n + c_Earth)
        tₘn = (c_Earth - z′n) * pn^-1
        p[n] = pn
        u[n] = un
        v[n] = vn
        tₘ[n] = tₘn
        fₘ[n] = pn * tₘn^4 + un * tₘn^3 + vn * tₘn - pn
        t₁[n] = (pn - c_Earth + z′n) * (pn - c_Earth + 2.0z′n)^-1
        t₀[n] = pn * (z′n + c_Earth)^-1
    end
    # Initialize Newton solver with appropriate initial guess:
    ecef2alt_solver_init!(t, tₘ, fₘ, t₁, t₀)
end


@inline function seq_newton_step!(Δt::Vector{Float64}, t::Vector{Float64}, p::Vector{Float64}, u::Vector{Float64}, v::Vector{Float64})
    @tturbo for i ∈ eachindex(Δt)
        Δt[i] = (p[i] - (p[i] * t[i]^4 + u[i] * t[i]^3 + v[i] * t[i])) * (4.0 * p[i] * t[i]^3 + 3.0 * u[i] * t[i]^2 + v[i])^-1
    end
end


@inline function seq_newton_error_check!(should_update::BitVector, Δt::Vector{Float64}, ε::Float64)
    @tturbo for i ∈ eachindex(Δt)
        should_update[i] = ε ≤ abs(Δt[i])
    end
end


@inline function check_all_false(v::BitVector)
    s::Integer = 0
    @tturbo for i ∈ eachindex(v)
        s += v[i]
    end
    return s
end


@inline function seq_t2h!(t::Vector{Float64}, p::Vector{Float64}, z::Vector{Float64})
    @tturbo for i ∈ eachindex(t)
        t[i] = ((2.0 * p[i] * e′ * t[i]) + (abs(z[i]) * (1.0 - t[i]^2)) - (a_Earth * e′ * (1.0 + t[i]^2))) *
            √((1.0 + t[i]^2)^2 - (4.0 * e_first^2 * t[i]^2))^-1
    end
end


"""
    seq_ecef2alt!(sol::Vector{Float64}, x::Vector{Float64}, y::Vector{Float64}, z::Vector{Float64}; ε::Float64 = 10.0^-13, n_max::Integer = 128)

Convert a sequence of ECEF position vectors into LLA coordinates, overwriting the buffer `sol`. The function takes as input three separate vectors, with the `i`-th entry in each vector representing the 1st, 2nd, and 3rd component (respectively) of the `i`-th vector in the sequence.

For each individual position vector, the function refactors the ECEF to LLA conversion problem into a quartic root finding problem, then uses Newton-Raphson method to find the solution.

...
# Arguments
- `x::Vector{Float64}`: The entries in the 1st component of each position vector
- `y::Vector{Float64}`: The entries in the 2nd component of each position vector
- `z::Vector{Float64}`: The entries in the 3rd component of each position vector
- `ε::Float64 = 10.0^-13`: Maximum allowable Newton-Raphson solver error
- `n_max::Integer = 128`: Maximum allowable Newton-Raphson solver steps

...
"""
@inline function seq_ecef2alt!(sol::Vector{Float64}, x::Vector{Float64}, y::Vector{Float64}, z::Vector{Float64}; ε::Float64 = 10.0^-13, n_max::Integer = 64)
    # initialization:
    p = zeros(Float64, length(sol))
    u = zeros(Float64, length(sol))
    v = zeros(Float64, length(sol))
    seq_ecef2alt_init!(sol, p, u, v, x, y, z)
    Δt = zeros(Float64, length(sol))
    should_update = trues(length(sol))
    for i = 1:n_max
        # multithreaded vectorized calculation of Newton step:
        seq_newton_step!(Δt, sol, p, u, v)
        # only applies steps where needed:
        vhadamard!(Δt, should_update)
        # step the solution:
        vadd!(sol, Δt)
        # if step taken was small enough, stop updating that entry:
        seq_newton_error_check!(should_update, Δt, ε)
        # break from loop if all steps taken were small enough:
        if check_all_false(should_update) == false
            break
        end
    end
    # turn t values to altitudes
    seq_t2h!(sol, p, z)
end


# For comparison, here is my non-vectorized implementation I based my code off of
#=
function ecef2lla(r::Vector{Float64}; n_max::Integer = 64, ε::Float64 = 10.0^-13)
    local p::Float64 = √(r[1]^2 + r[2]^2)
    local z_norm::Float64 = abs(r[3])
    local φ::Float64
    local λ::Float64
    local h::Float64
    if p == 0.0
        if r[3] == 0.0
            φ = 0.0
            λ = 0.0
            h = -b_Earth
        else
            if r[3] > 0
                φ = 0.5π
            else
                φ = -0.5π
            end
            λ = 0.0
            h = z_norm - b_Earth
        end
    elseif r[3] == 0.0
        φ = 0.0
        λ = atan(r[2], r[1])
        h = p - a_Earth
    else
        # in the non-trivial case, we are essentially solving Bowring's formula, but parameterized into a more efficient quartic equation
        # this saves computation time by avoiding unnecessary expensive calls to transcendental functions
        # to avoid the issue of the Newton-Raphson solver converging on the incorrect root, we must first initialize the iteration variable, t, for a number of different circumstances: 
        local c::Float64 = a_Earth * e_first^2
        local e′::Float64 = √(1.0 - e_first^2)
        local z′::Float64 = e′ * z_norm
        local u::Float64 = 2.0 * (z′ - c)
        local v::Float64 = 2.0 * (z′ + c)
        local t_M::Float64 = (c - z′) * p^-1
        local f_M::Float64 = p * t_M^4 + u * t_M^3 + v * t_M - p
        local t_1::Float64 = (p - c + z′) * (p - c + 2.0 * z′)^-1
        local t_0::Float64 = p * (z′ + c)^-1
        local t::Float64
        local Δt::Float64
        if t_M <= 0
            t = t_1
        elseif t_M >= 1 
            t = t_0
        elseif f_M >= 0
            t = t_0
        else
            t = t_1
        end
        for i = 1:n_max
            Δt = (p - (p * t^4 + u * t^3 + v * t)) * (4.0 * p * t^3 + 3.0 * u * t^2 + v)^-1
            t += Δt
            if abs(Δt) <= ε
                break
            end
        end
        if r[3] > 0
        φ = atan(1.0 - t^2, 2.0 * e′ * t)
        else
            φ = -atan(1.0 - t^2, 2.0 * e′ * t)
        end
        h = ((2.0 * p * e′ * t) + (z_norm * (1.0 - t^2)) - (a_Earth * e′ * (1.0 + t^2))) * √((1.0 + t^2)^2 - (4.0 * e_first^2 * t^2))^-1
        λ = atan(r[2], r[1])
    end
    if λ < 0
        λ = 2.0π + λ
    end
    return φ, λ, h
end
=#


