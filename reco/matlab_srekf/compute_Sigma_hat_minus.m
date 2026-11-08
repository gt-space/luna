function SigmaMinus = compute_Sigma_hat_minus(Sigma, F, G, W_Q, dt)
% propagates the SQR covariance matrix via block QR decomposition
% (core/EKF/Src/propogate.c (compute_Sigma_hat_minus function) & Core/EKF/Inc/ekf.h)
%
% inputs:
%   Sigma - current 21x21 square-root covariance
%   F     - 21x21 continuous system dynamics Jacobian
%   G     - 21x12 continuous process noise influence matrix
%   W_Q   - 12x12 discrete square-root process noise (sqrt(10*dt*Q))
%   dt    - timestep duration (seconds)
%
% output:
%   SigmaMinus - propagated 21x21 upper-triangular square-root covariance

% discrete state transition matrix
Phi = eye(21) + F * dt;

% construct the 33 x 21 pre-array matrix M
% block 1: Sigma * Phi^T (21 x 21)
% block 2: W_Q * G^T    (12 x 21)
M = [Sigma * (Phi'); ...
    W_Q * (G')];

% economy Householder QR factorization
[~, R_qr] = qr(M, 0);

% extract top 21x21 upper triangular block
SigmaMinus = triu(R_qr(1:21, 1:21));

% ensure positive diagonal convention for canonical square-root
for i = 1:21
    if SigmaMinus(i, i) < 0
        SigmaMinus(i, :) = -SigmaMinus(i, :);
    end
end

end

