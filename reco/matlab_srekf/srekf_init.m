function filter = srekf_init(dt)
% initializes the Square-Root Extended Kalman Filter (SR-EKF) (Core/EKF/Src/compute_initial_consts.c & Core/EKF/Inc/ekf.h)
%
% Follows Hanvit Lee's QR-MEKF Specification ("State Estimation for Actively Controlled Landers"):
% - Covariance is tracked as its upper-triangular Cholesky square root Sigma (P = Sigma' * Sigma).
% - Coupled 6x6 process noise matrices Q_kg and Q_ka (with T_s^2 off-diagonal and T_s^3 terms)
%   are initialized in Float64 (double), decomposed via upper-triangular Cholesky (chol),
%   and stored as W_kg and W_ka (Section B.A, p. 12).
% - Measurement noise square roots V_gps, V_mag, V_baro are computed via upper-triangular Cholesky.
%
% Output:
%   filter - struct containing state vector, square-root covariance, and noise matrices

if nargin < 1
    dt = 0.005; % Default 200 Hz timestep (5 ms, matching main.c)
end

%% state dimension definitions
% state vector x: 22 x 1
% [1:4]   q   - attitude quaternion [q0, q1, q2, q3]' (scalar first)
% [5:7]   lla - geodetic position [latitude (deg), longitude (deg), altitude (m)]'
% [8:10]  vel - NED velocity [vn, ve, vd]' (m/s)
% [11:13] bg  - gyroscope biases [bgx, bgy, bgz]' (rad/s)
% [14:16] ba  - accelerometer biases [bax, bay, baz]' (m/s^2)
% [17:19] sg  - gyroscope scale factors [sgx, sgy, sgz]' (unitless)
% [20:22] sa  - accelerometer scale factors [sax, say, saz]' (unitless)
%
% error state vector delta_x: 21 x 1
% [1:3]   delta_theta - attitude tilt error (rad)
% [4:6]   delta_p     - position error [lat, lon, alt]
% [7:9]   delta_v     - velocity error [vn, ve, vd]
% [10:12] delta_bg    - gyro bias error
% [13:15] delta_ba    - accel bias error
% [16:18] delta_sg    - gyro scale factor error
% [19:21] delta_sa    - accel scale factor error

%% earth, atmospheric & Nyquist constants
filter.we = 7.29211e-5;            % earth sidereal rotation rate (rad/s)
filter.a  = 6378137.0;             % WGS-84 earth semi-major axis (m)
filter.b  = 6356752.31425;         % WGS-84 earth semi-minor axis (m)
filter.e2 = 1.0 - (filter.b / filter.a)^2; % first eccentricity squared
filter.dt = dt;
filter.fs = 1.0 / dt;              % sampling frequency (Hz)
filter.f_nyquist = 0.5 / dt;       % Nyquist limit frequency (Hz)

%% initial nominal state (x0)
% matches xInitData from compute_initial_consts.c
filter.x = [ ...
    0.9659258;             % q0
    0.0;                   % q1
    0.0;                   % q2
    0.258819;              % q3 (pitch / yaw initial attitude)
    33.876409;             % Lat (deg)
    -84.301860;            % Lon (deg)
    306.0;                 % Alt (m)
    0.0; 0.0; 0.0;         % Vel NED (m/s)
    0.004170685449620679;  % bgx (rad/s)
    0.001874003189266571;  % bgy (rad/s)
    0.003769141999622996;  % bgz (rad/s)
    0.12668820347955903;   % bax (m/s^2)
    0.0;                   % bay (m/s^2)
    0.08570472381241734;   % baz (m/s^2)
    0.0; 0.0; 0.0;         % Gyro scale factors
    0.0; 0.0; 0.0          % Accel scale factors
    ];

%% Initial Standard Deviations (SR-EKF P0 Initialization, Eq. 13)
att_unc0   = [4e-3, 4e-3, 4e-3];
pos_unc0   = [1e-4, 1e-4, 1.0];
vel_unc0   = [1e-3, 1e-3, 1e-3];
gbias_unc0 = [2e-6, 2e-6, 2e-6];
abias_unc0 = [0.2, 0.2, 0.2];
gsf_unc0   = [1e-4, 1e-4, 1e-4];
asf_unc0   = [1e-4, 1e-4, 1e-4];

P0 = diag(double([att_unc0, pos_unc0, vel_unc0, gbias_unc0, abias_unc0, gsf_unc0, asf_unc0]));
% Sigma_0 = chol(P_0) in upper-triangular convention (Eq. 13)
filter.Sigma = chol(P0, 'upper');

%% Coupled 6x6 Process Noise Matrices Q_kg, Q_ka & Upper-Triangular Cholesky W_kg, W_ka (Section B.A, p. 12)
% Continuous noise variances (sigma_vg^2, sigma_ug^2, sigma_va^2, sigma_ua^2)
sigma_vg2 = double([2e-4, 2e-4, 2e-4]);
sigma_ug2 = double([1e-6, 1e-6, 1e-6]);
sigma_va2 = double([0.1,  0.1,  0.1]);
sigma_ua2 = double([0.0004, 0.0004, 0.0004]);

Ts = double(dt);

% Q_kg (6x6): Gyro white noise & gyro bias random walk coupled covariance
Q_kg = [ ...
    diag(sigma_vg2 * Ts + (1.0 / 3.0) * sigma_ug2 * (Ts^3)),  diag(-0.5 * sigma_ug2 * (Ts^2)); ...
    diag(-0.5 * sigma_ug2 * (Ts^2)),                          diag(sigma_ug2 * Ts) ...
];

% Q_ka (6x6): Accel white noise & accel bias random walk coupled covariance
Q_ka = [ ...
    diag(sigma_va2 * Ts + (1.0 / 3.0) * sigma_ua2 * (Ts^3)),  diag(0.5 * sigma_ua2 * (Ts^2)); ...
    diag(0.5 * sigma_ua2 * (Ts^2)),                           diag(sigma_ua2 * Ts) ...
];

% Upper-triangular Cholesky decompositions in double precision (Eq. 14 & p. 12)
filter.W_kg = chol(Q_kg, 'upper');
filter.W_ka = chol(Q_ka, 'upper');
filter.W_Q  = blkdiag(filter.W_kg, filter.W_ka); % 12 x 12 upper-triangular square-root process noise

%% Measurement Noise Square Roots via Upper-Triangular Cholesky (Eq. 15 & p. 12)
% GPS Measurement Noise Matrix R (3 x 3)
filter.R_gps = diag(double([1e-9, 1e-9, 400.0]));
filter.V_gps = chol(filter.R_gps, 'upper');

% Magnetometer Noise Matrix (3 x 3)
filter.Rq    = diag(double([0.005^2, 0.005^2, 0.005^2]));
filter.V_mag = chol(filter.Rq, 'upper');

% Reference Magnetic Field vector at launch site (normalized)
filter.magI = [0.4891; 0.1040; 0.8660];

% Barometer Measurement Noise
filter.Rb     = 2500.0;             % Variance (Pa^2)
filter.V_baro = sqrt(filter.Rb);    % Standard deviation (50 Pa)

%% Measurement Jacobians
% GPS Jacobian H (3 x 21): measures position states [lat, lon, alt] (columns 4:6)
filter.H_gps = zeros(3, 21);
filter.H_gps(1:3, 4:6) = eye(3);

end
