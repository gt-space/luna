function [xPlus, SigmaPlus, K, innov, PiHat] = srekf_update_baro(xMinus, SigmaMinus, press_meas, Rb, h_offset)
% performs SR-EKF measurement update from Barometer pressure (Core/EKF/Src/update_sensors.c (update_baro_new function))
%
% Follows Hanvit Lee's QR-MEKF Derivation (Equations 33, 39, 40, 42):
%   1. Innovation Square-Root Factor (Eq. 33):
%        PiHat = qr_R([ SigmaMinus * Hb' ; V_baro ]) (1x1 upper triangular)
%   2. QR-Accelerated Kalman Gain (Eqs. 39-40):
%        L_k = PiHat \ ( ((PiHat') \ Hb) * (SigmaMinus' * SigmaMinus) )
%        K_k = L_k'
%   3. Square-Root A-Posteriori Covariance Update (Eq. 42):
%        SigmaPlus = qr_R([ SigmaMinus * (I - K_k * Hb)' ; V_baro * K_k' ])
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
%   PiHat      - 1x1 innovation square-root factor

if nargin < 4, Rb = 2500.0; end
if nargin < 5, h_offset = 35.0; end

alt = xMinus(7); % geodetic altitude (m)
V_baro = sqrt(Rb);

% atmospheric model evaluations
[P_expected, h5] = srekf_utils.filter_pressure(alt, h_offset);

%% 1. Square-Root A-Priori Innovation Covariance (Eq. 33)
% Column 6 of SigmaMinus is non-zero only in rows 1:6
c6_top = SigmaMinus(1:6, 6); % 6 x 1
Sigma_HbT_top = h5 * c6_top; % 6 x 1 (rows 7:21 are zero)

M_innov = [Sigma_HbT_top; zeros(15, 1); V_baro]; % 22 x 1
[~, R_innov] = qr(M_innov, 0);
PiHat = R_innov(1, 1); % 1 x 1 Cholesky factor of S

%% 2. QR-Accelerated Kalman Gain Calculation (Eqs. 39-40)
% Eq. 39: L_k = PiHat \ [ [ (PiHat') \ Hb ] * (SigmaMinus' * SigmaMinus) ]
z_alt = (PiHat') \ h5; % Inner solve on non-zero element 6 of Hb
Y = (z_alt * (c6_top')) * SigmaMinus(1:6, :); % 1 x 21
L_k = PiHat \ Y; % Outer solve (1 x 21)
K = L_k';        % Eq. 40: K_k = L_k^T (21 x 1)

%% 3. Innovation and State Correction
innov = press_meas - P_expected;
delta_x = K * innov;

xPlus = xMinus;
xPlus(5:22) = xMinus(5:22) + delta_x(4:21);

%% 4. Square-Root A-Posteriori Covariance Update (Eq. 42)
M_post_top = SigmaMinus;
M_post_top(1:6, :) = M_post_top(1:6, :) - Sigma_HbT_top * L_k;

M_post = [M_post_top; ...
          V_baro * L_k]; % 22 x 21

[~, R_post] = qr(M_post, 0);
SigmaPlus = triu(R_post(1:21, 1:21));

% Canonical positive diagonal check
for i = 1:21
    if SigmaPlus(i, i) < 0
        SigmaPlus(i, :) = -SigmaPlus(i, :);
    end
end

end
