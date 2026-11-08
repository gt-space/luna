function [filter, flags] = srekf_step(filter, imu_meas, gps_meas, baro_meas, mag_meas)
% executes one complete iteration of the SR-EKF update cycle (Core/EKF/Src/update_EKF.c)
%
%   1. Propagation: Predicts nominal state x and square-root covariance Sigma
%      using IMU accelerometer and gyroscope data.
%   2. GPS Update (if available): Corrects position and velocity error states.
%   3. Magnetometer Update (if available): Corrects attitude tilt error states.
%   4. Barometer Update (if available): Corrects altitude error state.
%   5. Positive Definiteness Assurance:
%      **`nearestPSD` is omitted entirely** because the SR-EKF formulation
%      guarantees positive semi-definiteness by construction (P = Sigma' * Sigma >= 0).
%   6. Numerical Sanity Check: Detects overflow / NaN / Inf divergence.
%
% Inputs:
%   filter    - Current filter struct (from srekf_init)
%   imu_meas  - struct with fields:
%                 .accel - [ax, ay, az]' (m/s^2)
%                 .gyro  - [wx, wy, wz]' (rad/s)
%                 .dt    - timestep (s, optional, defaults to filter.dt)
%   gps_meas  - (optional) struct with fields:
%                 .valid - boolean flag
%                 .lla   - [lat, lon, alt]' (deg, deg, m)
%   baro_meas - (optional) struct with fields:
%                 .valid    - boolean flag
%                 .pressure - ambient pressure (Pa)
%   mag_meas  - (optional) struct with fields:
%                 .valid - boolean flag
%                 .mag   - [mx, my, mz]' (normalized body field)
%
% Outputs:
%   filter - Updated filter struct
%   flags  - struct with status flags (.diverged, .gps_applied, .baro_applied, .mag_applied)

flags.diverged = false;
flags.gps_applied = false;
flags.baro_applied = false;
flags.mag_applied = false;

dt = filter.dt;
if isfield(imu_meas, 'dt') && ~isempty(imu_meas.dt)
    dt = imu_meas.dt;
end

%% 1. time update (propagation)
[x, Sigma] = srekf_propagate( ...
    filter.x, ...
    filter.Sigma, ...
    imu_meas.accel, ...
    imu_meas.gyro, ...
    filter.W_Q, ...
    dt, ...
    filter.we, ...
    filter.a, ...
    filter.e2 ...
    );

%% 2. measurement updates
% GPS Update
if nargin >= 3 && ~isempty(gps_meas) && isfield(gps_meas, 'valid') && gps_meas.valid
    [x, Sigma, ~, ~] = srekf_update_gps(x, Sigma, gps_meas.lla, filter.R_gps, filter.V_gps);
    flags.gps_applied = true;
end

% Magnetometer Update
if nargin >= 5 && ~isempty(mag_meas) && isfield(mag_meas, 'valid') && mag_meas.valid
    [x, Sigma, ~, ~] = srekf_update_mag(x, Sigma, mag_meas.mag, filter.magI, filter.Rq, filter.V_mag);
    flags.mag_applied = true;
end

% Barometer Update
if nargin >= 4 && ~isempty(baro_meas) && isfield(baro_meas, 'valid') && baro_meas.valid
    [x, Sigma, ~, ~] = srekf_update_baro(x, Sigma, baro_meas.pressure, filter.Rb);
    flags.baro_applied = true;
end

%% 3. Numerical Integrity & Divergence Fallback Check
% matches Core/EKF/Src/update_EKF.c:
% loops through diagonal elements. In SR-EKF, diagonal elements are standard deviations.
% if any element exceeds 1e6 or is NaN/Inf, trigger fallback flag
sigma_diag = diag(Sigma);
for i = 1:length(sigma_diag)
    val = sigma_diag(i);
    if isnan(val) || isinf(val) || val > 1e6
        flags.diverged = true;
        break;
    end
end

filter.x = x;
filter.Sigma = Sigma;

end

