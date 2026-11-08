function G = compute_G(x)
% computes the 21x12 continuous process noise influence matrix (Core/EKF/Src/compute_F.c (compute_G function))
%
% input:
%   x - nominal state vector (22x1)
%
% output:
%   G - 21x12 process noise mapping matrix

q  = x(1:4);
sg = x(17:19); % gyro scale factors
sa = x(20:22); % accel scale factors

D_bn = srekf_utils.quaternion2dcm(q);

% sub-block G11: gyro white noise sensitivity (3x3)
G11 = diag(-1.0 ./ (1.0 + sg));

% sub-block G33: accelerometer white noise sensitivity (3x3)
G33 = -D_bn * diag(1.0 ./ (1.0 + sa));

% sub-block eye3: bias random walks
eye3 = eye(3);

% assemble full 21x12 matrix
G = zeros(21, 12);

% rows 1:3 (attitude tilt noise <- Gyro white noise)
G(1:3, 1:3) = G11;

% rows 7:9 (velocity noise <- Accel white noise)
G(7:9, 7:9) = G33;

% rows 10:12 (gyro bias random walk)
G(10:12, 4:6) = eye3;

% rows 16:18 (accel bias random walk)
G(16:18, 10:12) = eye3;

end

