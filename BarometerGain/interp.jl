using JLD2


dat = load(normpath(joinpath((@__FILE__), raw"..\simalts.jld2")))
simalts = dat["h"]
simtimes = dat["time"]


β = 0.5
G::Float64 = (1 - β^2)
H::Float64 = (1 - β)^2


# sensor polling rate:
poll_rate::Integer = 200
T_s::Float64 = 1 / poll_rate
# sims data sampling rate, DO NOT CHANGE:
sims_rate::Integer = 100
# subsampling ratio:
r_ss = round(Integer, poll_rate / sims_rate)
t_ss::Vector{Float64} = collect(range(0, (r_ss - 1) // r_ss, r_ss))
w1::Vector{Float64} = [0.5 * ( -x^3 + 2x^2 - 1x    ) for x ∈ t_ss]
w2::Vector{Float64} = [0.5 * ( 3x^3 - 5x^2 +      2) for x ∈ t_ss]
w3::Vector{Float64} = [0.5 * (-3x^3 + 4x^2 + 1x    ) for x ∈ t_ss]
w4::Vector{Float64} = [0.5 * (  x^3  - x^2         ) for x ∈ t_ss]


t_vec = zeros(r_ss * (length(h) - 3))
p_vec = zeros(r_ss * (length(h) - 3))
p1::Float64 = h[1]
p2::Float64 = h[2]
p3::Float64 = h[3]
p4::Float64 = h[4]
for i ∈ 1:(length(h) - 3)
    p4 = h[i + 3] 
    for j ∈ eachindex(t_ss)
        alt = w1[j] * p1 + w2[j] * p2 + w3[j] * p3 + w4[j] * p4
        
    end
    p1 = p2
    p2 = p3
    p3 = p4
end

using GLMakie
fig = Figure()
ax = Axis(fig[1, 1])
lines!(ax, t_vec, p_vec, color = :black)
scatter!(ax, simtimes[1:(length(h) - 2)], h[2:end - 1])
fig



