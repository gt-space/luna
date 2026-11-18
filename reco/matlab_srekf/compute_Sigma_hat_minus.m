function [SigmaMinus, Phi] = compute_Sigma_hat_minus(Sigma, F, G, W_Q, dt)
% propagates the SQRT covariance matrix via block QR decomposition (STM sandwich in square-root form)
% (Core/EKF/Src/propogate.c (compute_Sigma_hat_minus function) & Core/EKF/Inc/ekf.h)
%
% Key optimizations implemented:
%   1. Higher-Order State Transition Matrix (Phi) via 2nd-order Peano-Baker / Taylor expansion
%      factored as a single matrix product: Phi = I + (F*dt)*(I + 0.5*F*dt).
%   2. Block-sparsity of F: rows 10:21 of F (biases and scale factors) are identically zero,
%      so F^2 only requires multiplying the top 9x9 block by the top 9x21 block.
%   3. Block-sparsity of Sigma * Phi^T: columns 10:21 of Phi^T are [0; I_12], so columns
%      10:21 of Sigma * Phi^T equal Sigma(:, 10:21) with zero multiplications.
%   4. Diagonal/block-sparse W_Q * G^T: W_Q is 12x12 diagonal and G has only four 3x3
%      non-zero blocks (G11 diagonal, G33 3x3, and two I_3 blocks).
%
% inputs:
%   Sigma - current 21x21 upper-triangular square-root covariance
%   F     - 21x21 continuous system dynamics Jacobian (rows 10:21 are zero)
%   G     - 21x12 continuous process noise influence matrix
%   W_Q   - 12x12 diagonal discrete square-root process noise (or 12x1 diagonal vector)
%   dt    - timestep duration (seconds)
%
% outputs:
%   SigmaMinus - propagated 21x21 upper-triangular square-root covariance
%   Phi        - 21x21 2nd-order discrete state transition matrix

%% 1. Higher-Order State Transition Matrix (2nd-Order Peano-Baker / Taylor Series)
% F has non-zero dynamics only in rows 1:9 (attitude, position, velocity).
% Let F_top = F(1:9, 1:21) * dt. Since F(10:21, :) = 0:
%   (F*dt)^2 = [ F_top(1:9, 1:9) * F_top ; zeros(12, 21) ]
F_top = F(1:9, 1:21) * dt; % 9 x 21
Phi_top = [eye(9), zeros(9, 12)] + F_top + 0.5 * (F_top(1:9, 1:9) * F_top); % 9 x 21

if nargout > 1
    Phi = [Phi_top; zeros(12, 9), eye(12)];
end

%% 2. Top Block of Pre-Array M: Sigma * Phi^T (21 x 21)
% Since Phi^T = [ Phi_top' , [zeros(9,12); eye(12)] ]:
% - Columns 1:9 of (Sigma * Phi^T) are Sigma * Phi_top'
% - Columns 10:21 of (Sigma * Phi^T) are simply Sigma(:, 10:21)
M_top = zeros(21, 21);
M_top(:, 1:9)   = Sigma * (Phi_top');
M_top(:, 10:21) = Sigma(:, 10:21);

%% 3. Bottom Block of Pre-Array M: W_Q * G^T (12 x 21) via Diagonal/Block Sparsity
if size(W_Q, 2) == 1
    wq = W_Q;
else
    wq = diag(W_Q);
end

M_bot = zeros(12, 21);
% Block (1:3, 1:3): diag(wq(1:3)) * G(1:3, 1:3)^T (both are 3x3 diagonal)
M_bot(1:3, 1:3) = diag(wq(1:3) .* diag(G(1:3, 1:3)));
% Block (4:6, 10:12): diag(wq(4:6)) * I_3 (gyro bias random walk)
M_bot(4:6, 10:12) = diag(wq(4:6));
% Block (7:9, 7:9): diag(wq(7:9)) * G(7:9, 7:9)^T (accel white noise -> velocity)
M_bot(7:9, 7:9) = wq(7:9) .* (G(7:9, 7:9)');
% Block (10:12, 16:18): diag(wq(10:12)) * I_3 (accel bias random walk)
M_bot(10:12, 16:18) = diag(wq(10:12));

%% 4. Construct 33 x 21 Pre-Array & Economy Householder QR Factorization
M = [M_top; M_bot];
[~, R_qr] = qr(M, 0);

% Extract top 21x21 upper triangular block
SigmaMinus = triu(R_qr(1:21, 1:21));

% Ensure positive diagonal convention for canonical square-root
for i = 1:21
    if SigmaMinus(i, i) < 0
        SigmaMinus(i, :) = -SigmaMinus(i, :);
    end
end

end
