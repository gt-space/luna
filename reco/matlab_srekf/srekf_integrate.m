function xMinus = srekf_integrate(x, qdot, pdot, vdot, dt)
% propagates the nominal state vector using explicit Euler integration (Core/EKF/Src/propogate.c (integrate function) & Core/EKF/Inc/ekf.h.)
%
% In an SR-EKF, covariance integration (P + Pdot*dt) is eliminated from this
% because square-root propagation is handled separately by QR decomposition.
%
% Inputs:
%   x     - current 22x1 nominal state
%   qdot  - 4x1 quaternion time derivative
%   pdot  - 3x1 geodetic LLA time derivative [deg/s, deg/s, m/s]'
%   vdot  - 3x1 NED velocity time derivative [m/s^2]'
%   dt    - timestep duration (seconds)
%
% Output:
%   xMinus - propagated 22x1 a-priori state

xMinus = x;

% quaternion propagation & normalization
q = x(1:4) + qdot(:) * dt;
xMinus(1:4) = q / norm(q);

% position propagation (Lat, Lon, Alt)
xMinus(5:7) = x(5:7) + pdot(:) * dt;

% velocity propagation (Vn, Ve, Vd)
xMinus(8:10) = x(8:10) + vdot(:) * dt;

% biases and scale factors (indices 11:22) are constant during prediction
% xMinus(11:22) = x(11:22);

end

