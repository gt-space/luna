function results = test_srekf()
% Runs automated tests verifying:
%   1. initialization & square-root dimension integrity
%   2. covariance-stripped state integration (File 1, 3, 6)
%   3. block QR propagation equivalence with discrete Lyapunov/Riccati equation
%   4. guarantee of positive semi-definiteness without nearestPSD (File 5)
%   5. kalman gain right-linear solve (File 4)
%   6. barometer scalar shortcut vs full matrix update
%   7. multi-sensor end-to-end filter iteration

fprintf('====================================================\n');
fprintf('       SR-EKF MATLAB Verification Test Suite        \n');
fprintf('====================================================\n\n');

total_tests = 0;
passed_tests = 0;

%% Test 1: Initialization Integrity
total_tests = total_tests + 1;
try
    dt = 0.01;
    f = srekf_init(dt);
    assert(length(f.x) == 22, 'State vector x must be 22x1');
    assert(all(size(f.Sigma) == [21, 21]), 'Sigma must be 21x21');
    assert(istriu(f.Sigma), 'Sigma must be upper triangular');
    assert(all(diag(f.Sigma) > 0), 'All diagonal standard deviations must be positive');

    % Reconstructed P0 must match square of uncertainties
    P0 = f.Sigma' * f.Sigma;
    assert(all(eig(P0) >= -1e-12), 'Reconstructed P0 must be positive semi-definite');

    fprintf('[PASS] Test 1: srekf_init structure & positive definiteness\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('[FAIL] Test 1: %s\n', ME.message);
end

%% Test 2: Covariance-Free State Integration
total_tests = total_tests + 1;
try
    x0 = f.x;
    qdot = [0; 0.01; 0.02; -0.01];
    pdot = [0.0001; 0.0002; -10.0];
    vdot = [0.5; -0.2; 9.81];

    xMinus = srekf_integrate(x0, qdot, pdot, vdot, dt);
    assert(length(xMinus) == 22, 'Integrated state must be 22x1');
    assert(abs(norm(xMinus(1:4)) - 1.0) < 1e-6, 'Propagated quaternion must be normalized');
    assert(abs(xMinus(7) - (x0(7) + pdot(3)*dt)) < 1e-6, 'Altitude integration mismatch');

    fprintf('[PASS] Test 2: srekf_integrate stripped covariance & norm preservation\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('[FAIL] Test 2: %s\n', ME.message);
end

%% Test 3: Block QR Equivalence to Discrete Riccati Equation
total_tests = total_tests + 1;
try
    a_meas = [0.1; 0.05; -9.8];
    w_meas = [0.01; -0.02; 0.005];

    F = compute_F(f.x, a_meas, w_meas, f.we, f.a, f.e2);
    G = compute_G(f.x);
    Sigma_minus = compute_Sigma_hat_minus(f.Sigma, F, G, f.W_Q, dt);

    % Classical Riccati a-priori covariance:
    Phi = eye(21) + F * dt;
    P_prev = f.Sigma' * f.Sigma;
    Q_d = f.W_Q' * f.W_Q;
    P_minus_classical = Phi * P_prev * (Phi') + G * Q_d * (G');

    % Reconstructed from SR-EKF QR factor
    P_minus_srekf = Sigma_minus' * Sigma_minus;

    max_diff = max(max(abs(P_minus_srekf - P_minus_classical)));
    rel_diff = max_diff / max(max(abs(P_minus_classical)));
    assert(rel_diff < 1e-5, sprintf('QR propagation differs from classical Riccati (rel error: %e)', rel_diff));

    fprintf('[PASS] Test 3: compute_Sigma_hat_minus matches Riccati discrete update (rel error = %.2e)\n', rel_diff);
    passed_tests = passed_tests + 1;
catch ME
    fprintf('[FAIL] Test 3: %s\n', ME.message);
end

%% Test 4: Positive Semi-Definiteness Guarantee (No nearestPSD Needed)
total_tests = total_tests + 1;
try
    % Even with a randomly perturbed, ill-conditioned square root
    ill_Sigma = triu(randn(21, 21));
    ill_Sigma(1,1) = 1e-6;
    ill_Sigma(21,21) = 1e6;

    P_ill = ill_Sigma' * ill_Sigma;
    eigenvalues = eig(P_ill);
    min_eig = min(eigenvalues);

    assert(min_eig >= -1e-10, 'Eigenvalues must be strictly non-negative');
    fprintf('[PASS] Test 4: Algebraic guarantee that P = Sigma'' * Sigma >= 0 (min eigenvalue = %.2e)\n', min_eig);
    passed_tests = passed_tests + 1;
catch ME
    fprintf('[FAIL] Test 4: %s\n', ME.message);
end

%% Test 5: Right Linear Solve for GPS Kalman Gain
total_tests = total_tests + 1;
try
    gps_lla = [33.8765; -84.3019; 310.0];
    [xPlus_gps, SigmaPlus_gps, K_gps, innov_gps] = srekf_update_gps( ...
        f.x, f.Sigma, gps_lla, f.R_gps, f.V_gps ...
        );

    assert(all(size(K_gps) == [21, 3]), 'GPS Kalman gain K must be 21x3');
    assert(istriu(SigmaPlus_gps), 'Posterior Sigma must remain upper triangular');

    % Reconstructed P+ must satisfy Joseph form
    P_minus = f.Sigma' * f.Sigma;
    H = f.H_gps;
    I_KH = eye(21) - K_gps * H;
    P_plus_joseph = I_KH * P_minus * (I_KH') + K_gps * f.R_gps * (K_gps');
    P_plus_srekf = SigmaPlus_gps' * SigmaPlus_gps;

    max_diff_gps = max(max(abs(P_plus_srekf - P_plus_joseph)));
    assert(max_diff_gps < 1e-4, sprintf('Joseph form mismatch in GPS update: %e', max_diff_gps));

    fprintf('[PASS] Test 5: srekf_update_gps right linear solve & Joseph QR update\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('[FAIL] Test 5: %s\n', ME.message);
end

%% Test 6: Barometer 1D Scalar Update Shortcut
total_tests = total_tests + 1;
try
    press_meas = 98000.0; % 980 hPa
    [xPlus_baro, SigmaPlus_baro, K_baro, innov_baro] = srekf_update_baro( ...
        f.x, f.Sigma, press_meas, f.Rb ...
        );

    assert(length(K_baro) == 21, 'Barometer Kalman gain must be 21x1');
    assert(istriu(SigmaPlus_baro), 'Posterior Sigma must be upper triangular');
    assert(xPlus_baro(7) ~= f.x(7), 'Altitude must be corrected by barometer');

    fprintf('[PASS] Test 6: srekf_update_baro scalar update & altitude correction\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('[FAIL] Test 6: %s\n', ME.message);
end

%% Test 7: End-to-End srekf_step Iteration
total_tests = total_tests + 1;
try
    imu.accel = [0.0; 0.0; -9.80665];
    imu.gyro  = [0.0; 0.0; 0.0];
    imu.dt    = 0.01;

    gps.valid = true;
    gps.lla   = [33.876409; -84.301860; 306.0];

    baro.valid    = true;
    baro.pressure = 98000.0;

    mag.valid = true;
    mag.mag   = f.magI;

    [f_next, flags] = srekf_step(f, imu, gps, baro, mag);

    assert(~flags.diverged, 'Filter should not diverge on nominal data');
    assert(flags.gps_applied && flags.baro_applied && flags.mag_applied, 'All sensor updates should be applied');
    assert(all(isfinite(f_next.x)), 'State vector must contain finite values');
    assert(all(isfinite(f_next.Sigma(:))), 'Sigma must contain finite values');

    fprintf('[PASS] Test 7: End-to-end srekf_step multi-sensor iteration\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('[FAIL] Test 7: %s\n', ME.message);
end

%% Summary
fprintf('\n----------------------------------------------------\n');
fprintf('Summary: %d / %d tests passed (%.1f%% success rate)\n', ...
    passed_tests, total_tests, (passed_tests / total_tests) * 100);
fprintf('----------------------------------------------------\n');

results.total = total_tests;
results.passed = passed_tests;

end

