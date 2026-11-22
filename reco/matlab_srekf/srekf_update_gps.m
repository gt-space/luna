function [xPlus, SigmaPlus, K, innov, PiHat] = srekf_update_gps(xMinus, SigmaMinus, lla_meas, R_gps, V_gps)
% performs SR-EKF measurement update from GPS geodetic position (Core/EKF/Src/update_sensors.c (update_GPS function))
%
% Follows Hanvit Lee's QR-MEKF Derivation (Equations 33, 39, 40, 42):
%   1. Innovation Square-Root Factor (Eq. 33):
%        PiHat = qr_R([ sqrt(1+beta) * SigmaMinus * H' ; V_gps ]) (3x3 upper triangular)
%   2. QR-Accelerated Kalman Gain (Eqs. 39-40):
%        L_k = PiHat \ ( ((PiHat') \ H) * (SigmaMinus' * SigmaMinus) )
%        K_k = L_k'
%      - Inner Call (Lower Triangular): Z = (PiHat') \ H
%      - Multiply by [SigmaMinus]^T * SigmaMinus: Y = (Z * SigmaMinus') * SigmaMinus
%      - Outer Call (Upper Triangular): L_k = PiHat \ Y
%   3. Square-Root A-Posteriori Covariance Update (Eq. 42):
%        SigmaPlus = qr_R([ SigmaMinus * (I - K_k * H)' ; V_gps * K_k' ])
%
% Inputs:
%   xMinus     - a-priori state vector (22x1)
%   SigmaMinus - a-priori upper-triangular square-root covariance (21x21)
%   lla_meas   - GPS measurement [lat (deg), lon (deg), alt (m)]' (3x1)
%   R_gps      - GPS measurement noise covariance (3x3)
%   V_gps      - Square-root measurement noise (3x3), V_gps' * V_gps = R_gps
%
% Outputs:
%   xPlus      - A-posteriori updated state (22x1)
%   SigmaPlus  - A-posteriori square-root covariance (21x21)
%   K          - Kalman gain (21x3)
%   innov      - Measurement innovation (3x1)
%   PiHat      - Upper-triangular innovation square-root factor (3x3)

if nargin < 4, R_gps = diag([1e-9, 1e-9, 400.0]); end
if nargin < 5, V_gps = chol(double(R_gps), 'upper'); end

%% 1. Exploit Sparsity of H_gps (3x21, Identity at cols 4:6)
% Sigma_HT = SigmaMinus * H' is simply columns 4:6 of SigmaMinus (21 x 3)
Sigma_HT = SigmaMinus(:, 4:6);

% Position variance trace = sum of squared elements in columns 4:6 (rows 1:6)
trace_pos = sum(sum(Sigma_HT(1:6, :).^2));
if trace_pos > 1000.0
    beta = 0.25;
else
    beta = 0.0;
end
sqrt_beta = sqrt(1.0 + beta);

%% 2. Square-Root A-Priori Innovation Covariance (Eq. 33)
% PiHat = qr_R([ sqrt(1+beta) * SigmaMinus * H' ; V_gps ])
M_innov = [sqrt_beta * Sigma_HT; ...
           V_gps];
[~, R_innov] = qr(M_innov, 0);
PiHat = triu(R_innov(1:3, 1:3));

%% 3. QR-Accelerated Kalman Gain Calculation (Eqs. 39-40)
% Eq. 39: L_k = PiHat \ [ [ (PiHat') \ H ] * (SigmaMinus' * SigmaMinus) ]
% Since H = [0_{3x3}, I_3, 0_{3x15}], (PiHat') \ H is only non-zero at cols 4:6:
%   Z_pos = (PiHat') \ I_3  (Inner Lower-Triangular Solve)
opts_LT.LT = true;
Z_pos = linsolve(PiHat', eye(3), opts_LT); % 3 x 3

% Multiply [ (PiHat') \ H ] * [SigmaMinus]^T * SigmaMinus:
% Since Z is non-zero only at cols 4:6, Z * SigmaMinus' = Z_pos * Sigma_HT(1:6, :)'
Y = (Z_pos * (Sigma_HT(1:6, :)')) * SigmaMinus(1:6, :); % 3 x 21

% Outer Upper-Triangular Solve: L_k = PiHat \ Y
opts_UT.UT = true;
L_k = linsolve(PiHat, Y, opts_UT); % 3 x 21

% Eq. 40: K_k = L_k^T
K = L_k'; % 21 x 3

%% 4. Measurement Residual and State Update
innov = lla_meas(:) - xMinus(5:7);
delta_x = K * innov; % 21 x 1 error state correction

xPlus = xMinus;
% Attitude quaternion is unchanged in GPS update
xPlus(5:22) = xMinus(5:22) + delta_x(4:21);

%% 5. Square-Root A-Posteriori Covariance Update (Eq. 42)
% SigmaPlus = qr_R([ SigmaMinus * (I - K_k * H)' ; V_gps * K_k' ])
M_post_top = SigmaMinus;
M_post_top(1:6, :) = M_post_top(1:6, :) - Sigma_HT(1:6, :) * L_k;
M_post_bot = V_gps * L_k;

M_post = [M_post_top; M_post_bot]; % 24 x 21

[~, R_post] = qr(M_post, 0);
SigmaPlus = triu(R_post(1:21, 1:21));

% Canonical positive diagonal check
for i = 1:21
    if SigmaPlus(i, i) < 0
        SigmaPlus(i, :) = -SigmaPlus(i, :);
    end
end

end
