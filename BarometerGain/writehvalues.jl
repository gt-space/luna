using LoopVectorization
include("fastecef2alt.jl")
include("vecmathutils.jl")


using JLD2


φ_0::Float64 = deg2rad(35.34788)
λ_0::Float64 = deg2rad(-117.80683)
# launch site ECEF position
r_0::Vector{Float64} = lla2ecef(φ_0, λ_0, 625.0)
NED2ECEF_DCM_0::Matrix{Float64} = [
    -sin(φ_0)cos(λ_0) -sin(λ_0) -cos(φ_0)cos(λ_0)
    -sin(φ_0)sin(λ_0)  cos(λ_0) -cos(φ_0)sin(λ_0)
    cos(φ_0)           0.0      -sin(φ_0)
]


simdat = load(normpath(joinpath((@__FILE__), raw"..\rawsimdat.jld2")))


x_list = simdat["x"]
y_list = simdat["y"]
z_list = simdat["z"]
h_list = simdat["h"]


for i ∈ eachindex(z_list)
    h = h_list[i][2:end - 1]
    x = x_list[i]
    y = y_list[i]
    z = z_list[i]
    seq_gemv_3!(NED2ECEF_DCM_0, x, y, z, r_0)
    seq_ecef2alt!(h, x, y, z)
    h_list[i] = [2.0h[1] - h[2]; h; 2.0h[end] - h[end - 1]]
    println("run $i processed")
end


#=
save(normpath(joinpath((@__FILE__), raw"..\simalts.jld2")), Dict(
    "time" => simdat["time"],
    "h" => h_list
))
println("jld saved")
=#

