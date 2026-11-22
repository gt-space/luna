function [SigmaMinus, Phi] = compute_Sigma_hat_minus(Sigma, F, G, W_Q, dt)
% propagates the SQRT covariance matrix via block QR decomposition (Eq. 31)
% (Core/EKF/Src/propogate.c (compute_Sigma_hat_minus function) & Core/EKF/Inc/ekf.h)
%
% Key features:
%   1. Higher-Order State Transition Matrix (Phi) via 2nd-order Peano-Baker / Taylor series (Eqs. 26-27)
%      factored as a single matrix product: Phi = I + (F*dt)*(I + 0.5*F*dt).
%   2. Block-sparsity of F: rows 10:21 of F (biases and scale factors) are identically zero,
%      so F^2 only requires multiplying the top 9x9 block by the top 9x21 block.
%   3. Block-sparsity of Sigma * Phi^T: columns 10:21 of Phi^T are [0; I_12], so columns
%      10:21 of Sigma * Phi^T equal Sigma(:, 10:21) with zero multiplications.
%   4. Coupled Upper-Triangular W_Q * G^T (Section B.A, p. 12):
%      W_Q = blkdiag(W_kg, W_ka) has upper-triangular 6x6 blocks with main diagonals
%      (u11, u22) and super-diagonal coupling blocks (u12), while G has only four 3x3
%      non-zero blocks (G11 diagonal, G33 3x3, and two I_3 blocks).
%
% inputs:
%   Sigma - current 21x21 upper-triangular square-root covariance
%   F     - 21x21 continuous system dynamics Jacobian (rows 10:21 are zero)
%   G     - 21x12 continuous process noise influence matrix
%   W_Q   - 12x12 upper-triangular square-root process noise blkdiag(W_kg, W_ka)
%   dt    - timestep duration (seconds)
%
% outputs:
%   SigmaMinus - propagated 21x21 upper-triangular square-root covariance
%   Phi        - 21x21 2nd-order discrete state transition matrix

%% 1. Higher-Order State Transition Matrix (2nd-Order Peano-Baker / Taylor Series, Eqs. 26-27)
F_top = F(1:9, 1:21) * dt; % 9 x 21
Phi_top = [eye(9), zeros(9, 12)] + F_top + 0.5 * (F_top(1:9, 1:9) * F_top); % 9 x 21

if nargout > 1
    Phi = [Phi_top; zeros(12, 9), eye(12)];
end

%% 2. Top Block of Pre-Array M: Sigma * Phi^T (21 x 21)
M_top = zeros(21, 21);
M_top(:, 1:9)   = Sigma * (Phi_top');
M_top(:, 10:21) = Sigma(:, 10:21);

%% 3. Bottom Block of Pre-Array M: W_Q * G^T (12 x 21) via Coupled Upper-Triangular Block Sparsity
if size(W_Q, 2) == 1
    W_Q = diag(W_Q);
end

u11_g = diag(W_Q(1:3, 1:3));
u12_g = diag(W_Q(1:3, 4:6));
u22_g = diag(W_Q(4:6, 4:6));

u11_a = diag(W_Q(7:9, 7:9));
u12_a = diag(W_Q(7:9, 10:12));
u22_a = diag(W_Q(10:12, 10:12));

M_bot = zeros(12, 21);
% Rows 1:3 of W_Q * G^T: [u11_g * G11^T] at cols 1:3, and [u12_g * I_3] at cols 10:12
M_bot(1:3, 1:3)   = diag(u11_g .* diag(G(1:3, 1:3)));
M_bot(1:3, 10:12) = diag(u12_g);
% Rows 4:6 of W_Q * G^T: [u22_g * I_3] at cols 10:12 (gyro bias random walk)
M_bot(4:6, 10:12) = diag(u22_g);
% Rows 7:9 of W_Q * G^T: [u11_a * G33^T] at cols 7:9, and [u12_a * I_3] at cols 16:18
M_bot(7:9, 7:9)   = u11_a .* (G(7:9, 7:9)');
M_bot(7:9, 16:18) = diag(u12_a);
% Rows 10:12 of W_Q * G^T: [u22_a * I_3] at cols 16:18 (accel bias random walk)
M_bot(10:12, 16:18) = diag(u22_a);

%% 4. Construct 33 x 21 Pre-Array & Economy Householder QR Factorization (Eq. 31)
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
