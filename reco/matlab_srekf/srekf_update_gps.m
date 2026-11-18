function [xPlus, SigmaPlus, K, innov, PiHat] = srekf_update_gps(xMinus, SigmaMinus, lla_meas, R_gps, V_gps)
% performs SR-EKF measurement update from GPS geodetic position (Core/EKF/Src/update_sensors.c (update_GPS function))
%
% Mathematical & Sparsity Formulation:
%   1. Sparse H_gps (3x21, I_3 at cols 4:6):
%        Sigma_HT = SigmaMinus(:, 4:6) (only rows 1:6 are non-zero since SigmaMinus is upper triangular).
%   2. NASA Adaptive Underweighting:
%        trace_pos = ||Sigma_HT(1:6, :)||_F^2, beta = 0.25 if trace_pos > 1000 else 0.
%   3. Innovation Square-Root Factor (PiHat) via 24x3 QR Decomposition:
%        M_innov = [ sqrt(1 + beta) * Sigma_HT ; V_gps ] = Q_qr * R_qr
%        PiHat = R_qr(1:3, 1:3) (upper triangular Cholesky factor of S = PiHat' * PiHat).
%   4. Two-Step Triangular Cholesky Solve for Kalman Gain (No general linsolve/inversion):
%        We solve K * (PiHat' * PiHat) = P^- * H'  <=>  PiHat' * (PiHat * K') = B
%        where B = (P^- * H')' = Sigma_HT(1:6, :)' * SigmaMinus(1:6, :) (3 x 21).
%        - Inner call (Lower Triangular): solve (PiHat') * Y = B
%        - Outer call (Upper Triangular): solve PiHat * K' = Y
%   5. Posterior Square-Root Covariance via Sparse Rank-3 Update + Joseph Block QR:
%        SigmaMinus * (I - K*H)' = SigmaMinus - Sigma_HT * K'
%        M_post = [ SigmaMinus - Sigma_HT * K' ; V_gps * K' ] (24 x 21)
%        [~, R_post] = qr(M_post, 0)  =>  SigmaPlus = triu(R_post(1:21, 1:21))
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
if nargin < 5, V_gps = diag(sqrt(diag(R_gps))); end

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

%% 2. Innovation QR Sequence to Obtain Upper-Triangular Cholesky Factor PiHat
% M_innov = [ sqrt(1+beta) * SigmaMinus * H' ; V_gps ] (24 x 3)
M_innov = [sqrt_beta * Sigma_HT; ...
           V_gps];
[~, R_innov] = qr(M_innov, 0);
PiHat = triu(R_innov(1:3, 1:3));

%% 3. Two-Step Triangular Solve for Kalman Gain (Inner = Lower, Outer = Upper)
% Cross-covariance transposed: B = (P^- * H')' = (SigmaMinus * H')' * SigmaMinus (3 x 21)
% Since rows 7:21 of Sigma_HT are zero, only rows 1:6 participate:
B = (Sigma_HT(1:6, :)') * SigmaMinus(1:6, :); % 3 x 21

% Inner Call (Lower Triangular Solve): L * Y = B, where L = PiHat' (lower triangular)
% Matches arm_mat_solve_lower_triangular_f64(&PiHatT, &B, &Y)
opts_LT.LT = true;
Y = linsolve(PiHat', B, opts_LT);

% Outer Call (Upper Triangular Solve): U * K' = Y, where U = PiHat (upper triangular)
% Matches arm_mat_solve_upper_triangular_f64(&PiHat, &Y, &KT)
opts_UT.UT = true;
KT = linsolve(PiHat, Y, opts_UT);

K = KT'; % 21 x 3 Kalman gain

%% 4. Measurement Residual and State Update
innov = lla_meas(:) - xMinus(5:7);
delta_x = K * innov; % 21 x 1 error state correction

xPlus = xMinus;
% Attitude quaternion is unchanged in GPS update (tilt not observed directly)
xPlus(5:22) = xMinus(5:22) + delta_x(4:21);

%% 5. Posterior Square-Root Covariance via Sparse Rank-3 Update & Stacked QR
% Top block: SigmaMinus * (I - K*H)' = SigmaMinus - Sigma_HT * K'
% Since rows 7:21 of Sigma_HT are zero, only rows 1:6 of SigmaMinus are modified!
M_post_top = SigmaMinus;
M_post_top(1:6, :) = M_post_top(1:6, :) - Sigma_HT(1:6, :) * KT;

% Bottom block: V_gps * K' (V_gps is 3x3 diagonal)
v_gps_diag = diag(V_gps);
M_post_bot = v_gps_diag .* KT;

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
