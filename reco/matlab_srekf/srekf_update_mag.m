function [xPlus, SigmaPlus, K, innov] = srekf_update_mag(xMinus, SigmaMinus, mag_meas, magI, Rq, V_mag)
% performs SR-EKF measurement update from 3D Magnetometer readings (Core/EKF/Src/update_sensors.c (update_mag function))
%
% Inputs:
%   xMinus     - A-priori state vector (22x1)
%   SigmaMinus - A-priori square-root covariance (21x21)
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

if nargin < 4, magI = [0.4891; 0.1040; 0.8660]; end
if nargin < 5, Rq = diag([0.005^2, 0.005^2, 0.005^2]); end
if nargin < 6, V_mag = diag([0.005, 0.005, 0.005]); end

q = xMinus(1:4);
D_bn = srekf_utils.quaternion2dcm(q);
D_nb = D_bn'; % NED to body

% Expected magnetic field in body frame
magB = D_nb * magI(:);

% Measurement matrix H: H = [ [magB x], zeros(3, 18) ] (3 x 21)
H = zeros(3, 21);
H(1:3, 1:3) = srekf_utils.skew(magB);

% A-priori full covariance and cross-covariance
P_minus = SigmaMinus' * SigmaMinus;
PHT = P_minus * (H');

% Innovation covariance S (3 x 3)
S = H * PHT + Rq;

% Right solve for Kalman gain: K * S = PHT  =>  K = PHT / S
K = (S \ (PHT'))';

% innovation and delta state
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

% Posterior Square-Root Covariance via Block QR
I_KH_T = (eye(21) - K * H)';
M_post = [SigmaMinus * I_KH_T; ...
    V_mag * (K')];

[~, R_post] = qr(M_post, 0);
SigmaPlus = triu(R_post(1:21, 1:21));

% Canonical positive diagonal check
for i = 1:21
    if SigmaPlus(i, i) < 0
        SigmaPlus(i, :) = -SigmaPlus(i, :);
    end
end

end

