function [xPlus, SigmaPlus, K, innov] = srekf_update_baro(xMinus, SigmaMinus, press_meas, Rb, h_offset)
% performs SR-EKF measurement update from Barometer pressure (Core/EKF/Src/update_sensors.c (update_baro_new function))
%
% Optimized 1D scalar update:
%   - Avoids matrix inversions: S is scalar, so invS = 1 / S.
%   - H_b is 1x21 with only element 6 (altitude error) nonzero: h5 = dP/dh.
%   - Computes K = P^- * H_b' / S efficiently using row/column slices.
%   - Updates   square-root covariance via (22 x 21) block QR factorization:
%       M_post = [ SigmaMinus * (I - K*Hb)' ; sqrt(Rb) * K' ]
%       [~, R_post] = qr(M_post, 0)
%       SigmaPlus = R_post(1:21, 1:21)
%
% Inputs:
%   xMinus     - A-priori state vector (22x1)
%   SigmaMinus - A-priori square-root covariance (21x21)
%   press_meas - Measured barometric pressure (Pa)
%   Rb         - Barometer measurement noise variance (Pa^2, default 2500)
%   h_offset   - Altimeter ground elevation offset (m, default 35.0)
%
% Outputs:
%   xPlus      - A-posteriori updated state (22x1)
%   SigmaPlus  - A-posteriori square-root covariance (21x21)
%   K          - Kalman gain (21x1)
%   innov      - Pressure innovation (Pa)

if nargin < 4, Rb = 2500.0; end
if nargin < 5, h_offset = 35.0; end

alt = xMinus(7); % geodetic altitude (m)

% atmospheric model evaluations
[P_expected, h5] = srekf_utils.filter_pressure(alt, h_offset);

% measurement matrix Hb (1 x 21): only altitude error state (index 6) is nonzero
Hb = zeros(1, 21);
Hb(6) = h5;

% full a-priori covariance P^- = SigmaMinus' * SigmaMinus
P_minus = SigmaMinus' * SigmaMinus;

% efficient scalar innovation covariance:
% Hb * P^- * Hb' = h5^2 * P_minus(6,6)
temp1 = h5 * P_minus(6, :); % 1 x 21 row vector (P^- column 6 scaled by h5)
S = temp1(6) * h5 + Rb;     % scalar innovation variance

invS = 1.0 / S;
K = (temp1 * invS)';        % 21 x 1 Kalman gain column vector

% innovation and state correction
innov = press_meas - P_expected;
delta_x = K * innov;

xPlus = xMinus;
% attitude quaternion is unchanged in baro update
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

% Posterior Square-Root Covariance Update
% M_post is 22 x 21: [ SigmaMinus * (I - K*Hb)' ; sqrt(Rb) * K' ]
I_KHb_T = (eye(21) - K * Hb)';
V = sqrt(Rb);
M_post = [SigmaMinus * I_KHb_T; ...
    V * (K')];

[~, R_post] = qr(M_post, 0);
SigmaPlus = triu(R_post(1:21, 1:21));

% Canonical positive diagonal check
for i = 1:21
    if SigmaPlus(i, i) < 0
        SigmaPlus(i, :) = -SigmaPlus(i, :);
    end
end

end

