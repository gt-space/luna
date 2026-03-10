using LoopVectorization


using JLD2
using CSV
using DataFrames


@inline function my_push_function()
    x_temp = zeros(52168)
    y_temp = zeros(52168)
    z_temp = zeros(52168)
    is_valid = falses(52168)
    for i = 0:(324 - 1)
        x_temp = dat[:, 3i + 2]
        y_temp = dat[:, 3i + 3]
        z_temp = dat[:, 3i + 4]
        is_valid = .!((x_temp .== 0.0) .&& (y_temp .== 0.0) .&& (z_temp .== 0.0))
        push!(x_list, x_temp[is_valid])
        push!(y_list, y_temp[is_valid])
        push!(z_list, z_temp[is_valid])
    end
end


dat = CSV.read(normpath(joinpath((@__FILE__), raw"..\simdat.csv")), DataFrame)
times = dat[:, 1]
x_list = [] 
y_list = []
z_list = []
h_list = []
my_push_function()
for i ∈ eachindex(z_list)
    push!(h_list, zeros(length(z_list[i]) + 2))
end
println(length.(x_list) == length.(y_list))
println(length.(y_list) == length.(z_list))


save(normpath(joinpath((@__FILE__), raw"..\rawsimdat.jld2")), Dict(
    "time" => times,
    "x" => x_list,
    "y" => y_list,
    "z" => z_list,
    "h" => h_list
))

