function [xPlus, SigmaPlus, K, innov] = srekf_update_gps(xMinus, SigmaMinus, lla_meas, R_gps, V_gps)
% performs SR-EKF measurement update from GPS geodetic position (Core/EKF/Src/update_sensors.c (update_GPS function))
%
% mathematical formulation:
%   1. Adaptive underweighting factor beta based on position uncertainty trace.
%   2. Innovation covariance: S = (1 + beta) * H * P^- * H' + R
%   3. Kalman gain right-solve: K * S = P^- * H'  =>  K = (P^- * H') / S
%   4. Posterior square-root covariance via Joseph form block QR:
%        P^+ = (I - KH) P^- (I - KH)' + K R K'
%            = [Sigma^- (I - KH)']' [Sigma^- (I - KH)'] + [V * K']' [V * K']
%            = M_post' * M_post
%        where M_post = [ Sigma^- * (I - K*H)' ; V_gps * K' ]  (24 x 21)
%        QR decomposition: M_post = Q_post * R_post
%        SigmaPlus = R_post(1:21, 1:21) (upper triangular)
%
% Inputs:
%   xMinus     - a-priori state vector (22x1)
%   SigmaMinus - a-priori square-root covariance (21x21)
%   lla_meas   - GPS measurement [lat (deg), lon (deg), alt (m)]' (3x1)
%   R_gps      - GPS measurement noise covariance (3x3)
%   V_gps      - Square-root measurement noise (3x3), V_gps' * V_gps = R_gps
%
% Outputs:
%   xPlus      - A-posteriori updated state (22x1)
%   SigmaPlus  - A-posteriori square-root covariance (21x21)
%   K          - Kalman gain (21x3)
%   innov      - Measurement innovation (3x1)

if nargin < 4, R_gps = diag([1e-9, 1e-9, 400.0]); end
if nargin < 5, V_gps = diag(sqrt([1e-9, 1e-9, 400.0])); end

%% Measurement Matrix H (3 x 21)
% Observes position states (indices 4:6 in error state)
H = zeros(3, 21);
H(1:3, 4:6) = eye(3);

%% Reconstruct Full Covariance for Innovation and Cross-Covariance
% P^- = SigmaMinus' * SigmaMinus
% PHT = P^- * H' (21 x 3)
PHT = (SigmaMinus') * (SigmaMinus * (H'));

% Position variance trace for adaptive underweighting
trace_pos = sum(diag(PHT(4:6, 1:3)));
if trace_pos > 1000.0
    beta = 0.25;
else
    beta = 0.0;
end

% Innovation Covariance S (3 x 3)
HPHT = H * PHT;
S = (1.0 + beta) * HPHT + R_gps;

%% Robust Right Linear Solve for Kalman Gain (K * S = PHT)
% Matches arm_mat_linsolve_right_f64 in matrix_extensions.c
% Transposing gives S' * K' = PHT', so K = (S' \ PHT')'
K = (S \ (PHT'))';

%% Measurement Residual and State Update
innov = lla_meas(:) - xMinus(5:7);
delta_x = K * innov; % 21 x 1 error state correction

xPlus = xMinus;
% attitude quaternion is unchanged in GPS update (tilt not observed directly)
% position update
xPlus(5:7) = xMinus(5:7) + delta_x(4:6);
% velocity update
xPlus(8:10) = xMinus(8:10) + delta_x(7:9);
% gyro bias update
xPlus(11:13) = xMinus(11:13) + delta_x(10:12);
% accel bias update
xPlus(14:16) = xMinus(14:16) + delta_x(13:15);
% gyro scale factor update
xPlus(17:19) = xMinus(17:19) + delta_x(16:18);
% accel scale factor update
xPlus(20:22) = xMinus(20:22) + delta_x(19:21);

%% Posterior Square-Root Covariance via Stacked QR
% M_post = [ SigmaMinus * (I - K*H)' ; V_gps * K' ] (24 x 21)
I_KH_T = (eye(21) - K * H)';
M_post = [SigmaMinus * I_KH_T; ...
    V_gps * (K')];

[~, R_post] = qr(M_post, 0);
SigmaPlus = triu(R_post(1:21, 1:21));

% Canonical positive diagonal check
for i = 1:21
    if SigmaPlus(i, i) < 0
        SigmaPlus(i, :) = -SigmaPlus(i, :);
    end
end

end

