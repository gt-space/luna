function [xMinus, SigmaMinus] = srekf_propagate(x, Sigma, a_meas, w_meas, W_Q, dt, we, a, e2)
% complete SR-EKF prediction step for a single timestep dt (Core/EKF/Src/propogate.c (propogate function))
%
% Inputs:
%   x       - nominal state vector (22x1)
%   Sigma   - current square-root covariance (21x21)
%   a_meas  - accelerometer measurement (3x1, m/s^2)
%   w_meas  - gyroscope measurement (3x1, rad/s)
%   W_Q     - square-root process noise matrix (12x12)
%   dt      - timestep duration (seconds)
%   we      - earth rotation rate (rad/s)
%   a       - earth semi-major axis (m)
%   e2      - earth eccentricity squared
%
% Outputs:
%   xMinus     - predicted nominal state vector (22x1)
%   SigmaMinus - predicted square-root covariance (21x21)

if nargin < 7, we = 7.29211e-5; end
if nargin < 8, a = 6378137.0; end
if nargin < 9, e2 = 1.0 - (6356752.31425 / a)^2; end

%% extract state vectors
q   = x(1:4);
phi = x(5);     % latitude (deg)
h   = x(7);     % altitude (m)
vn  = x(8);     % north velocity (m/s)
ve  = x(9);     % east velocity (m/s)
vd  = x(10);    % down velocity (m/s)
bg  = x(11:13); % gyro bias
ba  = x(14:16); % accel bias
sg  = x(17:19); % gyro scale factor
sa  = x(20:22); % accel scale factor

%% compensate sensor readings
what   = srekf_utils.compute_what(q, bg, sg, phi, h, vn, ve, we, w_meas, a, e2);
ahat_n = srekf_utils.compute_ahat(q, sa, ba, a_meas);

%% compute continuous derivatives
% 1. quaternion derivative: qdot = 0.5 * q (x) [0; what]
qdot = 0.5 * srekf_utils.quat_mult(q, [0; what]);

% 2. geodetic LLA derivative
radii = srekf_utils.compute_radii(phi, a, e2);
R_phi  = radii(1);
R_lamb = radii(2);
phidot   = rad2deg(vn / (R_phi + h));
lambadot = rad2deg(ve / ((R_lamb + h) * cosd(phi)));
hdot     = -vd;
pdot     = [phidot; lambadot; hdot];

% 3. NED velocity derivative
sin_phi = sind(phi);
cos_phi = cosd(phi);
[ghat, ~, ~] = srekf_utils.compute_gravity(phi, h);
R_phi_h  = R_phi  + h;
R_lamb_h = R_lamb + h;

vndot = -(ve / (R_lamb_h * cos_phi) + 2.0 * we) * ve * sin_phi + (vn * vd) / R_phi_h + ahat_n(1);
vedot =  (ve / (R_lamb_h * cos_phi) + 2.0 * we) * vn * sin_phi + (ve * vd) / R_lamb_h + 2.0 * we * vd * cos_phi + ahat_n(2);
vddot = -ve^2 / R_lamb_h - vn^2 / R_phi_h - 2.0 * we * ve * cos_phi + ghat + ahat_n(3);
vdot  = [vndot; vedot; vddot];

%% State Prediction (Nominal Integration)
xMinus = srekf_integrate(x, qdot, pdot, vdot, dt);

%% Square-Root Covariance Prediction (Block QR Factorization)
F = compute_F(x, a_meas, w_meas, we, a, e2);
G = compute_G(x);
SigmaMinus = compute_Sigma_hat_minus(Sigma, F, G, W_Q, dt);

end

