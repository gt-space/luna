"""
    step_fmfs_o1!(x̂::Float64, x_meas::Float64, G::Float64)

Steps the state estimate `x̂` of a first-order discrete-time fading memory filter

`G` must be initialized to a constant at compile time using `G =  (1 - β)`

...
# Arguments
- `x̂::Float64`: State estimate stored by the filter
- `x_meas::Float64`: Measured value of the state
- `G::Float64`: First order fading memory filter state estimate gain, `G = (1 - β)``

...
"""
function step_fmfs_o1!(x̂::Float64, x_meas::Float64, G::Float64)
    # a-priori step: do nothing because we assume the solution is constant in time in the absence of perturbations; x^-_k+1 = x^+_k
    # a-posteriori step: assume the system is influenced by first order perturbations, so compute the residual y_k = x_meas - x̂_k
    # multiply the residual by G and add to x^-_k+1 to get x^+_k+1
    x̂ += G * (x_meas - x̂)
end


"""
    step_fmfs_o2!(x̂::Float64, ẋ̂::Float64, x_m::Float64, G::Float64, H::Float64)

Steps the state estimate `x̂` and state derivative estimate `ẋ̂` of a second-order discrete-time fading memory filter

`G` and `H` must be initialized to constants at compile time using `G = 1 - β^2` and `H = (1 - β)^2`

...
# Arguments
- `x̂::Float64`: State estimate stored by the filter
- `ẋ̂::Float64`: State derivative estimate stored by the filter
- `x_meas::Float64`: Measured value of the state
- `T_s::Float64` Fading memory filter sampling rate
- `G::Float64`: Second order fading memory filter state estimate gain, `G = 1 - β^2`
- `H::Float64`: Second order fading memory filter state derivative estimate gain, `H = (1 - β)^2`

...
"""
function step_fmfs_o2!(x̂::Float64, ẋ̂::Float64, x_meas::Float64, T_s::Float64, G::Float64, H::Float64)
    # a-priori step: we assume the solution is linear in time so we propagate the state estimate to x^-_k+1 = x^+_k + T_s * ẋ̂
    # this also implies the state derivative is constant, so we don't change it
    # a-posteriori step: assume the system is influenced by second order perturbations, so compute the residual y_k = x_meas - x^-_k+1
    # it follows then that the state derivative residual is ẏ_k = y_k / T_s
    # multiply both residuals by their respective gains and add to get x^+_k+1
    y = x_meas - (x̂ + ẋ̂ * T_s)
    x̂ += ẋ̂ * T_s + G * y
    ẋ̂ += H * (y / T_s)
end




