function [xPlus, SigmaPlus, K, innov, PiHat] = srekf_update_mag(xMinus, SigmaMinus, mag_meas, magI, Rq, V_mag)
% performs SR-EKF measurement update from 3D Magnetometer readings (Core/EKF/Src/update_sensors.c (update_mag function))
%
% Follows Hanvit Lee's QR-MEKF Derivation (Equations 33, 39, 40, 42):
%   1. Innovation Square-Root Factor (Eq. 33):
%        PiHat = qr_R([ SigmaMinus * H' ; V_mag ]) (3x3 upper triangular)
%   2. QR-Accelerated Kalman Gain (Eqs. 39-40):
%        L_k = PiHat \ ( ((PiHat') \ H) * (SigmaMinus' * SigmaMinus) )
%        K_k = L_k'
%   3. Square-Root A-Posteriori Covariance Update (Eq. 42):
%        SigmaPlus = qr_R([ SigmaMinus * (I - K_k * H)' ; V_mag * K_k' ])
%
% Inputs:
%   xMinus     - A-priori state vector (22x1)
%   SigmaMinus - A-priori upper-triangular square-root covariance (21x21)
%   mag_meas   - Normalized 3D body magnetic field measurement (3x1)
%   magI       - Normalized reference inertial/NED magnetic field vector (3x1)
%   Rq         - Magnetometer measurement noise covariance (3x3)
%   V_mag      - Square-root measurement noise (3x3), default chol(Rq, 'upper')
%
% Outputs:
%   xPlus      - A-posteriori updated state (22x1)
%   SigmaPlus  - A-posteriori square-root covariance (21x21)
%   K          - Kalman gain (21x3)
%   innov      - Magnetic field innovation (3x1)
%   PiHat      - Upper-triangular innovation square-root factor (3x3)

if nargin < 4, magI = [0.4891; 0.1040; 0.8660]; end
if nargin < 5, Rq = diag([0.005^2, 0.005^2, 0.005^2]); end
if nargin < 6, V_mag = chol(double(Rq), 'upper'); end

q = xMinus(1:4);
D_bn = srekf_utils.quaternion2dcm(q);
D_nb = D_bn'; % NED to body

% Expected magnetic field in body frame
magB = D_nb * magI(:);
H_mag3 = srekf_utils.skew(magB); % 3x3 block at H(1:3, 1:3)

%% 1. Exploit Upper-Triangular & Block Sparsity for SigmaMinus * H'
Sigma_HT_top3 = SigmaMinus(1:3, 1:3) * (H_mag3'); % 3 x 3
Sigma_HT = [Sigma_HT_top3; zeros(18, 3)];         % 21 x 3

%% 2. Square-Root A-Priori Innovation Covariance (Eq. 33)
M_innov = [Sigma_HT; ...
           V_mag]; % 24 x 3
[~, R_innov] = qr(M_innov, 0);
PiHat = triu(R_innov(1:3, 1:3));

%% 3. QR-Accelerated Kalman Gain Calculation (Eqs. 39-40)
% Eq. 39: L_k = PiHat \ [ [ (PiHat') \ H ] * (SigmaMinus' * SigmaMinus) ]
% Inner Lower-Triangular Solve on active 3x3 block H_mag3:
opts_LT.LT = true;
Z_att = linsolve(PiHat', H_mag3, opts_LT); % 3 x 3

% Multiply [ (PiHat') \ H ] * [SigmaMinus]^T * SigmaMinus:
Y = (Z_att * (SigmaMinus(1:3, 1:3)')) * SigmaMinus(1:3, :); % 3 x 21

% Outer Upper-Triangular Solve: L_k = PiHat \ Y
opts_UT.UT = true;
L_k = linsolve(PiHat, Y, opts_UT); % 3 x 21

% Eq. 40: K_k = L_k^T
K = L_k'; % 21 x 3

%% 4. Innovation and Multiplicative Quaternion Update
innov = mag_meas(:) - magB;
delta_x = K * innov;

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
xPlus(5:22) = xMinus(5:22) + delta_x(4:21);

%% 5. Square-Root A-Posteriori Covariance Update (Eq. 42)
M_post_top = SigmaMinus;
M_post_top(1:3, :) = M_post_top(1:3, :) - Sigma_HT_top3 * L_k;
M_post_bot = V_mag * L_k;

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
