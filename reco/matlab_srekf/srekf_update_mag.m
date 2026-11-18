function [xPlus, SigmaPlus, K, innov, PiHat] = srekf_update_mag(xMinus, SigmaMinus, mag_meas, magI, Rq, V_mag)
% performs SR-EKF measurement update from 3D Magnetometer readings (Core/EKF/Src/update_sensors.c (update_mag function))
%
% Mathematical & Sparsity Formulation:
%   1. Sparse H_mag (3x21, skew(magB) at cols 1:3, zeros elsewhere):
%        Since SigmaMinus is upper triangular, only its top-left 3x3 block multiplies H_mag'.
%        Thus Sigma_HT = SigmaMinus * H' has non-zero entries ONLY in rows 1:3!
%   2. Innovation Square-Root Factor (PiHat) via QR Decomposition:
%        M_innov = [ Sigma_HT ; V_mag ] = Q_qr * R_qr  =>  PiHat = R_qr(1:3, 1:3)
%   3. Two-Step Triangular Cholesky Solve for Kalman Gain:
%        B = (P^- * H')' = Sigma_HT(1:3, :)' * SigmaMinus(1:3, :) (3 x 21)
%        - Inner call (Lower Triangular): solve (PiHat') * Y = B
%        - Outer call (Upper Triangular): solve PiHat * K' = Y
%   4. Posterior Square-Root Covariance via Sparse Rank-3 Update + Joseph Block QR:
%        M_post_top = SigmaMinus; M_post_top(1:3, :) -= Sigma_HT(1:3, :) * K'
%        M_post = [ M_post_top ; V_mag * K' ] (24 x 21)
%
% Inputs:
%   xMinus     - A-priori state vector (22x1)
%   SigmaMinus - A-priori upper-triangular square-root covariance (21x21)
%   mag_meas   - Normalized 3D body magnetic field measurement (3x1)
%   magI       - Normalized reference inertial/NED magnetic field vector (3x1)
%   Rq         - Magnetometer measurement noise covariance (3x3)
%   V_mag      - Square-root measurement noise (3x3), default diag([0.005, 0.005, 0.005])
%
% Outputs:
%   xPlus      - A-posteriori updated state (22x1)
%   SigmaPlus  - A-posteriori square-root covariance (21x21)
%   K          - Kalman gain (21x3)
%   innov      - Magnetic field innovation (3x1)
%   PiHat      - Upper-triangular innovation square-root factor (3x3)

if nargin < 4, magI = [0.4891; 0.1040; 0.8660]; end
if nargin < 5, Rq = diag([0.005^2, 0.005^2, 0.005^2]); end
if nargin < 6, V_mag = diag(sqrt(diag(Rq))); end

q = xMinus(1:4);
D_bn = srekf_utils.quaternion2dcm(q);
D_nb = D_bn'; % NED to body

% Expected magnetic field in body frame
magB = D_nb * magI(:);
H_mag3 = srekf_utils.skew(magB); % 3x3 block at H(1:3, 1:3)

%% 1. Exploit Upper-Triangular & Block Sparsity for SigmaMinus * H'
% Since H is [H_mag3, zeros(3, 18)] and SigmaMinus is upper triangular,
% rows 4:21 of columns 1:3 in SigmaMinus are all zero!
Sigma_HT_top3 = SigmaMinus(1:3, 1:3) * (H_mag3'); % 3 x 3
Sigma_HT = [Sigma_HT_top3; zeros(18, 3)];         % 21 x 3

%% 2. Innovation QR Sequence to Obtain Upper-Triangular Cholesky Factor PiHat
M_innov = [Sigma_HT; ...
           V_mag]; % 24 x 3
[~, R_innov] = qr(M_innov, 0);
PiHat = triu(R_innov(1:3, 1:3));

%% 3. Two-Step Triangular Solve for Kalman Gain (Inner = Lower, Outer = Upper)
% Cross-covariance transposed: B = (P^- * H')' = Sigma_HT(1:3, :)' * SigmaMinus(1:3, :) (3 x 21)
B = (Sigma_HT_top3') * SigmaMinus(1:3, :); % 3 x 21

% Inner Call (Lower Triangular Solve): (PiHat') * Y = B
opts_LT.LT = true;
Y = linsolve(PiHat', B, opts_LT);

% Outer Call (Upper Triangular Solve): PiHat * K' = Y
opts_UT.UT = true;
KT = linsolve(PiHat, Y, opts_UT);

K = KT'; % 21 x 3

%% 4. Innovation and Multiplicative Quaternion Update
innov = mag_meas(:) - magB;
delta_x = K * innov;

% MEKF quaternion multiplicative update using Xi matrix (Xi(q) maps 3x1 tilt error to 4x1 quaternion perturbation)
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
Xi = [ ...
    -q1, -q2, -q3; ...
     q0, -q3,  q2; ...
     q3,  q0, -q1; ...
    -q2,  q1,  q0  ...
    ];

delta_q = 0.5 * (Xi * delta_x(1:3));
q_plus = q + delta_q;
q_plus = q_plus / norm(q_plus);

xPlus = xMinus;
xPlus(1:4) = q_plus;
% Update remaining states (pos, vel, biases, scale factors)
xPlus(5:22) = xMinus(5:22) + delta_x(4:21);

%% 5. Posterior Square-Root Covariance via Sparse Rank-3 Update & Block QR
% Top block: SigmaMinus * (I - K*H)' = SigmaMinus - Sigma_HT * K'
% Since only rows 1:3 of Sigma_HT are non-zero, only rows 1:3 of SigmaMinus change!
M_post_top = SigmaMinus;
M_post_top(1:3, :) = M_post_top(1:3, :) - Sigma_HT_top3 * KT;

% Bottom block: V_mag * K' (V_mag is 3x3 diagonal)
v_mag_diag = diag(V_mag);
M_post_bot = v_mag_diag .* KT;

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
