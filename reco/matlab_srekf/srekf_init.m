function filter = srekf_init(dt)
% initializes the Square-Root Extended Kalman Filter (SR-EKF) (Core/EKF/Src/compute_initial_consts.c & Core/EKF/Inc/ekf.h)
%
% In an SR-EKF:
% - Covariance is tracked as its upper-triangular square root Sigma (P = Sigma' * Sigma).
% - Initial uncertainties and process/measurement noises are initialized as standard deviations,
%   eliminating variance squaring and preserving positive semi-definiteness.
%
% Output:
%   filter - struct containing state vector, square-root covariance, and noise matrices

if nargin < 1
    dt = 0.01; % Default 100 Hz timestep
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

%% earth & atmospheric constants
filter.we = 7.29211e-5;      % earth sidereal rotation rate (rad/s)
filter.a  = 6378137.0;       % WGS-84 earth semi-major axis (m)
filter.b  = 6356752.31425;   % WGS-84 earth semi-minor axis (m)
filter.e2 = 1.0 - (filter.b / filter.a)^2; % first eccentricity squared
filter.dt = dt;

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

%% Initial Standard Deviations (SR-EKF P0 Initialization)
% In standard EKF, PInitData was populated with variances.
% In SR-EKF, diagonal elements are standard deviations (sqrt of variance).
att_unc0  = [4e-3, 4e-3, 4e-3];
pos_unc0  = [1e-4, 1e-4, 1.0];
vel_unc0  = [1e-3, 1e-3, 1e-3];
gbias_unc0 = [2e-6, 2e-6, 2e-6];
abias_unc0 = [0.2, 0.2, 0.2];
gsf_unc0  = [1e-4, 1e-4, 1e-4];
asf_unc0  = [1e-4, 1e-4, 1e-4];

sigma_diag = [ ...
    sqrt(att_unc0), ...
    sqrt(pos_unc0), ...
    sqrt(vel_unc0), ...
    sqrt(gbias_unc0), ...
    sqrt(abias_unc0), ...
    sqrt(gsf_unc0), ...
    sqrt(asf_unc0) ...
    ];

% SQRT covariance matrix (21 x 21 upper triangular)
filter.Sigma = diag(sigma_diag);

%% Continuous Process Noise Standard Errors (W_Q factor)
% process noise sources: gyro noise, gyro bias random walk, accel noise, accel bias random walk
nu_gv = [2e-4, 2e-4, 2e-4];
nu_gu = [1e-6, 1e-6, 1e-6];
nu_av = [0.1,  0.1,  0.1];
nu_au = [0.0004, 0.0004, 0.0004];

q_diag = [nu_gv, nu_gu, nu_av, nu_au];
% scaled by (10 * dt) matching compute_Q0 in compute_initial_consts.c
filter.W_Q = diag(sqrt(10.0 * dt * q_diag)); % 12 x 12 diagonal square-root process noise

%% Measurement Noise Standard Errors
% GPS Measurement Noise Matrix R (3 x 3)
filter.R_gps = diag([1e-9, 1e-9, 400.0]);
filter.V_gps = diag(sqrt([1e-9, 1e-9, 400.0])); % Square root measurement noise

% Magnetometer Noise Matrix (3 x 3)
% RqInitData in SR-EKF uses standard deviations: 0.005 directly
filter.Rq = diag([0.005^2, 0.005^2, 0.005^2]);
filter.V_mag = diag([0.005, 0.005, 0.005]);

% Reference Magnetic Field vector at launch site (normalized)
filter.magI = [0.4891; 0.1040; 0.8660];

% Barometer Measurement Noise
filter.Rb = 2500.0;             % Variance (Pa^2)
filter.V_baro = sqrt(filter.Rb); % Standard deviation (50 Pa)

%% Measurement Jacobians
% GPS Jacobian H (3 x 21): measures position states [lat, lon, alt] (columns 4:6)
filter.H_gps = zeros(3, 21);
filter.H_gps(1:3, 4:6) = eye(3);

end

