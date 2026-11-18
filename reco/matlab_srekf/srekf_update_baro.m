function [xPlus, SigmaPlus, K, innov] = srekf_update_baro(xMinus, SigmaMinus, press_meas, Rb, h_offset)
% performs SR-EKF measurement update from Barometer pressure (Core/EKF/Src/update_sensors.c (update_baro_new function))
%
% Optimized 1D Sparse & Upper-Triangular Formulation:
%   - Avoids forming full 21x21 P^- = SigmaMinus' * SigmaMinus.
%   - H_b is 1x21 with only element 6 (altitude error) nonzero: h5 = dP/dh.
%   - Since SigmaMinus is upper triangular, column 6 (c6 = SigmaMinus(1:6, 6)) has only 6 non-zero entries!
%   - Cross-covariance: PHT = P^- * H_b' = h5 * (SigmaMinus(1:6, :)' * c6) (21 x 1).
%   - Scalar innovation variance: S = (h5^2) * (c6' * c6) + Rb.
%   - Kalman gain: K = PHT / S (scalar division, no matrix solver needed).
%   - Sparse rank-1 update for Posterior QR pre-array M_post (22 x 21):
%       Top block: SigmaMinus - (h5 * [c6; zeros(15,1)]) * K' (only rows 1:6 modified!)
%       Bottom row: sqrt(Rb) * K'
%
% Inputs:
%   xMinus     - A-priori state vector (22x1)
%   SigmaMinus - A-priori upper-triangular square-root covariance (21x21)
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

%% 1. Sparse Cross-Covariance & Scalar Innovation Variance from Upper-Triangular SigmaMinus
% Column 6 of SigmaMinus is non-zero only in rows 1:6
c6_top = SigmaMinus(1:6, 6); % 6 x 1

% P^- * Hb' = SigmaMinus' * (SigmaMinus * Hb') = h5 * SigmaMinus(1:6, :)' * c6_top (21 x 1)
PHT = h5 * (SigmaMinus(1:6, :)' * c6_top);

% Scalar innovation variance S = Hb * P^- * Hb' + Rb = h5 * PHT(6) + Rb
S = h5 * PHT(6) + Rb;
invS = 1.0 / S;
K = PHT * invS; % 21 x 1 Kalman gain column vector

%% 2. Innovation and State Correction
innov = press_meas - P_expected;
delta_x = K * innov;

xPlus = xMinus;
% Attitude quaternion is unchanged in baro update
xPlus(5:22) = xMinus(5:22) + delta_x(4:21);

%% 3. Posterior Square-Root Covariance via Sparse Rank-1 Update & Block QR
% M_post is 22 x 21: [ SigmaMinus * (I - K*Hb)' ; sqrt(Rb) * K' ]
% Since SigmaMinus * Hb' = [h5 * c6_top; zeros(15, 1)], only rows 1:6 of SigmaMinus are modified!
M_post_top = SigmaMinus;
M_post_top(1:6, :) = M_post_top(1:6, :) - (h5 * c6_top) * (K');

V = sqrt(Rb);
M_post = [M_post_top; ...
          V * (K')]; % 22 x 21

[~, R_post] = qr(M_post, 0);
SigmaPlus = triu(R_post(1:21, 1:21));

% Canonical positive diagonal check
for i = 1:21
    if SigmaPlus(i, i) < 0
        SigmaPlus(i, :) = -SigmaPlus(i, :);
    end
end

end
