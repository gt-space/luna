function F = compute_F(x, a_meas, w_meas, we, a, e2)
% computes 21x21 continuous-time system dynamics Jacobian matrix (Core/EKF/Src/compute_F.c)
% inputs:
%   x      - nominal state vector (22x1)
%   a_meas - raw accelerometer measurement [ax, ay, az]' (m/s^2)
%   w_meas - raw gyroscope measurement [wx, wy, wz]' (rad/s)
%   we     - earth sidereal rotation rate (rad/s)
%   a      - earth semi-major axis (m)
%   e2     - earth eccentricity squared
%
% output:
%   F      - 21x21 continuous error-state Jacobian matrix

if nargin < 4, we = 7.29211e-5; end
if nargin < 5, a = 6378137.0; end
if nargin < 6, e2 = 1.0 - (6356752.31425 / a)^2; end

%% state elements
q    = x(1:4);
phi  = x(5);     % latitude (deg)
h    = x(7);     % altitude (m)
vn   = x(8);     % north velocity (m/s)
ve   = x(9);     % east velocity (m/s)
vd   = x(10);    % down velocity (m/s)
bg   = x(11:13); % gyro biases
ba   = x(14:16); % accel biases
sg   = x(17:19); % gyro scale factors
sa   = x(20:22); % accel scale factors

%% direction cosine matrices with nav-to-body transpose
D_bn = srekf_utils.quaternion2dcm(q);
D_nb = D_bn';

%% WGS-84 radii & gravity
radii = srekf_utils.compute_radii(phi, a, e2);
R_phi       = radii(1);
R_lamb      = radii(2);
dR_phi_dphi = radii(3);
dR_lamb_dphi= radii(4);

[~, dg_dphi, dg_dh] = srekf_utils.compute_gravity(phi, h);

%% trig values (phi in degrees)
phi_rad  = deg2rad(phi);
sin_phi  = sind(phi);
cos_phi  = cosd(phi);
tan_phi  = tand(phi);
sec_phi  = 1.0 / cos_phi;
sec_phi2 = sec_phi^2;

RLh  = R_lamb + h;
RPh  = R_phi  + h;
RLh2 = RLh^2;
RPh2 = RPh^2;

%% Row 1 Sub-blocks: Attitude Error Dynamics (Rows 1:3)
% F11: skew of gyro correction terms
term_F11 = -1.0 ./ (1.0 + sg) .* (w_meas(:) - bg(:));
F11 = srekf_utils.skew(term_F11);

% dwdp: partial derivative of transport rate w.r.t position
m11_w = -we * sin_phi - ve / RLh2 * dR_lamb_dphi;
m13_w = -ve / RLh2;
m21_w =  vn / RPh2 * dR_phi_dphi;
m23_w =  vn / RPh2;
m31_w = -we * cos_phi - (ve * sec_phi2) / RLh + (ve * tan_phi / RLh2) * dR_lamb_dphi;
m33_w = (ve * tan_phi) / RLh2;
dwdp  = [m11_w, 0, m13_w;
      m21_w, 0, m23_w;
      m31_w, 0, m33_w];
F12 = -D_nb * dwdp;

% dwdv: partial derivative of transport rate w.r.t velocity
dwdv = [    0,             1 / RLh, 0;
      -1 / RPh,               0,  0;
      0,   -tan_phi / RLh,    0];
F13 = -D_nb * dwdv;

% F14: sensitivity to gyro biases
F14 = diag(-1.0 ./ (1.0 + sg));

% F16: sensitivity to gyro scale factors
F16 = diag(-(w_meas(:) - bg(:)));

%% Row 2 Sub-blocks: Position Error Dynamics (Rows 4:6)
% dpdot_dp: partial derivative of LLA rate w.r.t position
m11_p = -vn / RPh2 * dR_phi_dphi;
m13_p = rad2deg(-vn / RPh2);
m21_p = -(ve * sec_phi) / RLh2 * dR_lamb_dphi + (ve * sec_phi * tan_phi) / RLh;
m23_p = rad2deg(-ve * sec_phi / RLh2);
F22   = [m11_p, 0, m13_p;
      m21_p, 0, m23_p;
      0, 0,     0];

% dpdot_dv: partial derivative of LLA rate w.r.t velocity
F23 = [rad2deg(1.0 / RPh),                    0,  0;
      0, rad2deg(sec_phi / RLh), 0;
      0,                    0, -1];

%% Row 3 Sub-blocks: Velocity Error Dynamics (Rows 7:9)
% F31: cross product with specific force
ahat_n = srekf_utils.compute_ahat(q, sa, ba, a_meas);
ahat_b = D_nb * ahat_n;
F31    = -D_bn * srekf_utils.skew(ahat_b);

% dvdot_dp: partial derivative of acceleration w.r.t position
Y11 = -(ve^2 * sec_phi2) / RLh + (ve^2 * tan_phi / RLh2) * dR_lamb_dphi ...
      - 2.0 * we * ve * cos_phi - (vn * vd / RPh2) * dR_phi_dphi;
Y13 = (ve^2 * tan_phi) / RLh2 - (vn * vd) / RPh2;
Y21 = (ve * vn * sec_phi2) / RLh - (ve * vn * tan_phi / RLh2) * dR_lamb_dphi ...
      + 2.0 * we * vn * cos_phi - (ve * vd / RLh2) * dR_lamb_dphi - 2.0 * we * vd * sin_phi;
Y23 = -ve * ((vn * tan_phi + vd) / RLh2);
Y31 = (ve^2 / RLh2) * dR_lamb_dphi + (vn^2 / RPh2) * dR_phi_dphi ...
      + 2.0 * we * ve * sin_phi + dg_dphi;
Y33 = (ve^2 / RLh2) + (vn^2 / RPh2) + dg_dh;

F32 = [Y11, 0, Y13;
      Y21, 0, Y23;
      Y31, 0, Y33];

% dvdot_dv: partial derivative of acceleration w.r.t velocity
Z11 = vd / RPh;
Z12 = (-2.0 * ve * tan_phi) / RLh + 2.0 * we * sin_phi;
Z13 = vn / RPh;
Z21 = (ve * tan_phi) / RLh + 2.0 * we * sin_phi;
Z22 = (vd + vn * tan_phi) / RLh;
Z23 = ve / RLh + 2.0 * we * cos_phi;
Z31 = (-2.0 * vn) / RPh;
Z32 = (-2.0 * ve) / RLh - 2.0 * we * cos_phi;

F33 = [Z11, Z12, Z13;
      Z21, Z22, Z23;
      Z31, Z32,   0];

% F35: sensitivity to accel biases
F35 = D_bn * diag(1.0 ./ (1.0 + sa));

% F37: sensitivity to accel scale factors
F37 = D_bn * diag(a_meas(:) - ba(:));

%% assemble full 21x21 Jacobian
F = zeros(21, 21);

% row block 1 (attitude error, indices 1:3)
F(1:3, 1:3)   = F11;
F(1:3, 4:6)   = F12;
F(1:3, 7:9)   = F13;
F(1:3, 10:12) = F14;
F(1:3, 16:18) = F16;

% row block 2 (position error, indices 4:6)
F(4:6, 4:6)   = F22;
F(4:6, 7:9)   = F23;

% row block 3 (velocity error, indices 7:9)
F(7:9, 1:3)   = F31;
F(7:9, 4:6)   = F32;
F(7:9, 7:9)   = F33;
F(7:9, 13:15) = F35;
F(7:9, 19:21) = F37;

% rows 10:21 (sensor biases and scale factors) are modeled as random walks:
% F(10:21, :) = 0; (already zeros)

end

