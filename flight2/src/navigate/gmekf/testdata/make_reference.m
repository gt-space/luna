function make_reference(srcDir, outDir)
% make_reference.m
%
% Writes the reference data the Rust GMEKF tests compare against, by
% running the MATLAB GMEKF functions themselves.
%
%   srcDir: hopper sims/hopper_6dof_sizer/subsystems/navigation/Src_filter
%           (branch rev_CTV_gmekf, commit ed077f9: gyro-driven predict)
%   outDir: where to write unit_cases.json and sequence.json
%
% Run with, for example:
%   matlab -batch "make_reference('<hopper>/.../Src_filter', '.')"
%
% Every number is written with 17 significant digits, so the Rust tests
% read back exactly the doubles MATLAB used.

    addpath(srcDir);
    rng(20261120, 'twister');

    unit = struct();
    unit.propagate_state = cases_propagate_state(8);
    unit.error_state_jacobian = cases_error_state_jacobian(6);
    unit.propagate_covariance = cases_propagate_covariance(6);
    unit.gmekf_predict = cases_gmekf_predict(8);
    unit.kalman_measurement_update = cases_kalman_update(8);
    unit.state_injection = cases_state_injection(6);
    unit.sensor_models = cases_sensor_models(6);
    unit.quaternion = cases_quaternion(6);
    write_json(fullfile(outDir, 'unit_cases.json'), unit);

    seq = run_sequence();
    write_json(fullfile(outDir, 'sequence.json'), seq);
end

%% ------------------------------------------------------------------------
%% Random inputs

function q = rand_quat()
    q = randn(4,1);
    q = q / norm(q);
end

function x = rand_state()
    x = [randn(3,1)*2; randn(3,1); randn(3,1)*0.5; rand_quat(); ...
         randn(3,1)*0.05; randn(3,1)*0.01; randn*0.5];
end

function P = rand_spd(n, scale)
    A = randn(n) * scale;
    P = A*A' + eye(n)*scale^2*0.1;
    P = 0.5*(P+P');
end

function J = ctv_inertia()
    % Roughly the CTV inertia (kg m^2), body x along the vehicle, with small
    % products of inertia so the full matrix path is exercised.
    J = [0.0123 0.0004 -0.0002;
         0.0004 0.1032  0.0011;
        -0.0002 0.0011  0.1038];
end

function Qc = test_qc()
    % Per-sample accelerometer and gyro covariance, then the accel bias,
    % gyro bias and barometer bias random-walk PSDs.
    Qc = diag([1e-4 1e-4 1.2e-4, 3e-6 3e-6 4e-6, 1.6e-10 1.6e-10 1.6e-10, ...
               1.5e-12 1.5e-12 1.5e-12, 1e-3]);
end

%% ------------------------------------------------------------------------
%% Unit cases

function c = cases_propagate_state(n)
    c = struct([]);
    for i = 1:n
        xn = rand_state();
        accel = randn(3,1)*3 + [9.8;0;0];
        gyro = randn(3,1)*0.3;
        validity = mod(i,4) ~= 0;
        J = ctv_inertia();
        g = 9.81;
        dt = 1/208 * (0.5 + rand);
        aux = struct('A_IB', quaternion_to_rotation(xn(10:13)));
        xnp1 = propagate_state(xn, aux, accel, struct('angvel', gyro), validity, J, g, dt);
        c(i).xn = xn; c(i).accel = accel; c(i).gyro = gyro;
        c(i).validity = double(validity); c(i).J = J; c(i).g = g; c(i).dt = dt;
        c(i).xnp1 = xnp1;
    end
end

function c = cases_error_state_jacobian(n)
    c = struct([]);
    for i = 1:n
        x = rand_state();
        A0 = quaternion_to_rotation(rand_quat());
        A05 = quaternion_to_rotation(rand_quat());
        a_I = randn(3,1)*3;
        w_I = randn(3,1)*0.5;
        J = ctv_inertia();
        dt = 1/208 * (0.5 + rand);
        validity = mod(i,3) ~= 0;
        aux = struct('A_IB', A0, 'A_IB05', A05, 'a_I', a_I, 'w_I', w_I);
        [F, G] = error_state_jacobian(x, aux, dt, validity, J);
        c(i).x = x; c(i).A_IB = A0; c(i).A_IB05 = A05; c(i).a_I = a_I; c(i).w_I = w_I;
        c(i).J = J; c(i).dt = dt; c(i).validity = double(validity);
        c(i).F = F; c(i).G = G;
    end
end

function c = cases_propagate_covariance(n)
    c = struct([]);
    for i = 1:n
        P = rand_spd(19, 0.1);
        F = randn(19)*0.3;
        G = randn(19,13)*0.2;
        Qc = test_qc();
        dt = 1/208 * (0.5 + rand);
        Pnp1 = propagate_covariance(P, F, G, Qc, dt);
        c(i).P = P; c(i).F = F; c(i).G = G; c(i).Qc = Qc; c(i).dt = dt;
        c(i).Pnp1 = Pnp1;
    end
end

function c = cases_gmekf_predict(n)
    c = struct([]);
    for i = 1:n
        xn = rand_state();
        Pn = rand_spd(19, 0.05);
        accel = randn(3,1)*3 + [9.8;0;0];
        gyro = randn(3,1)*0.3;
        validity = mod(i,4) ~= 0;
        J = ctv_inertia();
        g = 9.81;
        dt = 1/208;
        Qc = test_qc();
        [xnp1, Pnp1] = gmekf_predict(xn, Pn, accel, struct('angvel', gyro), validity, J, g, dt, Qc);
        c(i).xn = xn; c(i).Pn = Pn; c(i).accel = accel; c(i).gyro = gyro;
        c(i).validity = double(validity); c(i).J = J; c(i).g = g; c(i).dt = dt;
        c(i).Qc = Qc; c(i).xnp1 = xnp1; c(i).Pnp1 = Pnp1;
    end
end

function c = cases_kalman_update(n)
    c = struct([]);
    for i = 1:n
        m = 1 + mod(i-1, 6);          % measurement sizes 1..6
        P = rand_spd(19, 0.1);
        H = randn(m, 19) * 0.5;
        R = rand_spd(m, 0.05);
        innov = randn(m,1) * 0.1;
        if mod(i, 3) == 0
            innov = innov * 1e3;      % make some get gated out
        end
        gate = 6.635 + m;
        [dx, Pp, accepted, nis] = kalman_measurement_update(P, innov, H, R, gate);
        c(i).P = P; c(i).H = H; c(i).R = R; c(i).innovation = innov;
        c(i).m = m; c(i).gate = gate;
        c(i).dx = dx; c(i).P_post = Pp; c(i).accepted = double(accepted); c(i).nis = nis;
    end
end

function c = cases_state_injection(n)
    c = struct([]);
    for i = 1:n
        x = rand_state();
        dx = randn(19,1) * 0.05;
        xp = state_injection(x, dx);
        Jr = compute_error_reset_jacobian(dx);
        c(i).x = x; c(i).dx = dx; c(i).x_post = xp; c(i).reset_jacobian = Jr;
    end
end

function c = cases_sensor_models(n)
    c = struct([]);
    for i = 1:n
        x = rand_state();
        baro = struct('height', randn*5);
        [rb, Hb] = baro_update_model(x, baro, struct());
        gyro = struct('angvel', randn(3,1)*0.3);
        [rg, Hg] = gyro_update_model(x, gyro, struct());
        r_b = randn(3,1)*0.3;
        gps = struct('pos', randn(3,1)*2, 'vel', randn(3,1));
        [rp, Hp] = gps_update_model(x, gps, struct('r_b', r_b));
        mag = struct('bfield_E', randn(3,1)*0.3, 'bfield_meas', randn(3,1)*0.3);
        [rm, Hm] = mag_update_model(x, mag, struct());
        c(i).x = x;
        c(i).baro_height = baro.height; c(i).baro_innovation = rb; c(i).baro_H = Hb;
        c(i).gyro_angvel = gyro.angvel; c(i).gyro_innovation = rg; c(i).gyro_H = Hg;
        c(i).gps_r_b = r_b; c(i).gps_pos = gps.pos; c(i).gps_vel = gps.vel;
        c(i).gps_innovation = rp; c(i).gps_H = Hp;
        c(i).mag_field_ned = mag.bfield_E; c(i).mag_field_body = mag.bfield_meas;
        c(i).mag_innovation = rm; c(i).mag_H = Hm;
    end
end

function c = cases_quaternion(n)
    c = struct([]);
    for i = 1:n
        q1 = rand_quat(); q2 = rand_quat();
        c(i).q1 = q1; c(i).q2 = q2;
        c(i).product = qmult(q1, q2);
        c(i).rotation = quaternion_to_rotation(q1);
        c(i).psi = Psi_helper(q1);
    end
end

%% ------------------------------------------------------------------------
%% Closed-loop sequence: truth -> sensors -> gmekf_predict + gmekf_correct_ctv

function seq = run_sequence()
    dt = 1/208;            % ASM330LHBG1 output rate on the CTV
    N = 832;               % 4 s
    g = 9.81;
    J = ctv_inertia();
    r_b = [0.3; 0.05; -0.1];            % GPS antenna from the IMU, body (m)
    gps_frame = struct('r1', r_b, 'r2', -r_b);

    gyroStd = deg2rad(5e-3) * sqrt(1/(2*dt));
    accelStd = 70e-6 * 9.81 * sqrt(1/(2*dt));
    R_gyro = diag(gyroStd^2 * ones(3,1));
    Qc = blkdiag(accelStd^2*eye(3), R_gyro, 1.6e-10*eye(3), 1.5e-12*eye(3), 1e-3);
    R_baro = 0.1^2;
    pcov = diag([0.5 0.5 1.0].^2);
    vcov = diag([0.05 0.05 0.1].^2);
    % Magnetometer as in nav_sensor_params (Atlanta field, 3 mGauss noise,
    % variance inflated x10), in Gauss like the LIS2MDL driver.
    magField = [227.5; -22.4; 423.0] * 1e-3;
    magStd = 3.0e-3;
    R_mag = 9.0*10 * 1e-6;

    accelBias = [0.05; -0.03; 0.02];
    gyroBias = [0.002; -0.001; 0.003];
    baroBias = 0.4;

    % Truth: smooth translation, rotation about an upright attitude
    pos = @(t) [0.4*sin(0.6*t); 0.3*(1-cos(0.5*t)); -1 - 0.8*(1-cos(0.9*t))];
    vel = @(t) [0.24*cos(0.6*t); 0.15*sin(0.5*t); -0.72*sin(0.9*t)];
    acc = @(t) [-0.144*sin(0.6*t); 0.075*cos(0.5*t); -0.648*cos(0.9*t)];
    omega = @(t) [0.3*sin(1.1*t); 0.2*cos(0.7*t); 0.15*sin(0.5*t + 0.3)];

    q_true = [cos(pi/4); 0; sin(pi/4); 0];   % body x up
    substeps = 50;

    % Initial estimate: truth with errors
    dq0 = [1; 0.5*[0.01; -0.015; 0.02]];
    q0 = qmult(q_true, dq0); q0 = q0/norm(q0);
    x = [pos(0) + [0.2; -0.1; 0.3]; vel(0) + [0.05; -0.05; 0.02]; ...
         zeros(3,1); q0; zeros(3,1); zeros(3,1); 0];
    P = diag([0.1 0.1 0.1, 0.01 0.01 0.01, gyroStd^2*[1 1 1], ...
              1e-3 1e-3 1e-3, 1e-2 1e-2 1e-2, 1e-4 1e-4 1e-4, 1]);

    seq.config = struct('dt', dt, 'g', g, 'J', J, 'Qc', Qc, 'r_b', r_b, ...
        'x0', x, 'P0', P, 'R_gyro', R_gyro, 'R_baro', R_baro, ...
        'pcov', pcov, 'vcov', vcov, 'mag_field', magField, 'R_mag', R_mag, ...
        'full_P_every', 16);

    lidar_frames = struct('A1', eye(3), 'A2', eye(3), 'A3', eye(3), 'A4', eye(3), ...
        'r1', zeros(3,1), 'r2', zeros(3,1), 'r3', zeros(3,1), 'r4', zeros(3,1), ...
        'heightCOM', 1);
    meas_LID = struct('validity', false(4,1), 'timestamp', -1, ...
        'distance', zeros(4,1), 'covariance', ones(4,1));
    meas_mag = struct('validity', false, 'timestamp', -1, ...
        'bfield_meas', zeros(3,1), 'bfield_E', magField, 'covariance', R_mag*eye(3));

    meas_baro = struct('validity', false, 'timestamp', -1, 'height', 0, 'covariance', R_baro);
    meas_gps = struct('validity', false, 'timestamp', -1, 'pos', zeros(3,1), ...
        'vel', zeros(3,1), 'pcovariance', pcov, 'vcovariance', vcov);

    clear gmekf_correct_ctv;   % reset its persistent timestamps

    t_log = zeros(1,N); accel_log = zeros(3,N); gyro_log = zeros(3,N);
    baro_new = zeros(1,N); baro_h = zeros(1,N);
    gps_new = zeros(1,N); gps_pos = zeros(3,N); gps_vel = zeros(3,N);
    mag_new = zeros(1,N); mag_field = zeros(3,N);
    x_log = zeros(20,N); Pdiag_log = zeros(19,N); diag_log = zeros(16,N);
    truth_log = zeros(13,N);
    fullP_idx = []; fullP = [];

    t = 0;
    for k = 1:N
        % advance truth
        for s = 1:substeps
            ts = t + (s-0.5)*dt/substeps;
            th = omega(ts) * dt/substeps;
            a = norm(th);
            q_true = qmult(q_true, [cos(a/2); sin(a/2)*th/a]);
        end
        q_true = q_true/norm(q_true);
        t = k*dt;

        C = quaternion_to_rotation(q_true);
        f_b = C' * (acc(t) - [0;0;g]);
        accel = f_b + accelBias + accelStd*randn(3,1);
        gyro = omega(t) + gyroBias + gyroStd*randn(3,1);

        % Predict: the gyro drives the rotation
        [x, P] = gmekf_predict(x, P, accel, struct('angvel', gyro), true, J, g, dt, Qc);

        % Barometer, about 21 Hz, one outlier
        if mod(k, 10) == 3
            p_true = pos(t);
            h = -p_true(3) + baroBias + 0.1*randn;
            if k == 403
                h = h + 5;
            end
            meas_baro.validity = true;
            meas_baro.timestamp = t;
            meas_baro.height = h;
            baro_new(k) = 1; baro_h(k) = h;
        end

        % GPS, about 21 Hz
        if mod(k, 10) == 7
            w = omega(t);
            p = pos(t) + C*r_b + sqrt(diag(pcov)).*randn(3,1);
            v = vel(t) + C*cross(w, r_b) + sqrt(diag(vcov)).*randn(3,1);
            meas_gps.validity = true;
            meas_gps.timestamp = t;
            meas_gps.pos = p; meas_gps.vel = v;
            gps_new(k) = 1; gps_pos(:,k) = p; gps_vel(:,k) = v;
        end

        % Magnetometer, about 10 Hz (LIS2MDL ODR), one outlier
        if mod(k, 21) == 11
            m = C' * magField + magStd*randn(3,1);
            if k == 599
                m = m + [0.2; 0; 0];
            end
            meas_mag.validity = true;
            meas_mag.timestamp = t;
            meas_mag.bfield_meas = m;
            mag_new(k) = 1; mag_field(:,k) = m;
        end

        [x, P, d] = gmekf_correct_ctv(x, P, meas_gps, meas_baro, ...
            meas_LID, lidar_frames, meas_mag, gps_frame);

        t_log(k) = t; accel_log(:,k) = accel; gyro_log(:,k) = gyro;
        x_log(:,k) = x; Pdiag_log(:,k) = diag(P); diag_log(:,k) = d;
        truth_log(:,k) = [pos(t); vel(t); omega(t); q_true];
        if mod(k, 16) == 0
            fullP_idx(end+1) = k; %#ok<AGROW>
            fullP = [fullP, P(:)]; %#ok<AGROW>
        end
    end

    seq.steps = struct('t', t_log, 'accel', accel_log, 'gyro', gyro_log, ...
        'baro_new', baro_new, 'baro_height', baro_h, ...
        'gps_new', gps_new, 'gps_pos', gps_pos, 'gps_vel', gps_vel, ...
        'mag_new', mag_new, 'mag_field', mag_field, ...
        'x', x_log, 'P_diag', Pdiag_log, 'diagnostics', diag_log, ...
        'truth', truth_log, 'full_P_step', fullP_idx, 'full_P', fullP);
end

%% ------------------------------------------------------------------------
%% JSON writer: numbers as flat column-major arrays with 17 digits

function write_json(path, s)
    fid = fopen(path, 'w');
    fwrite(fid, enc(s));
    fclose(fid);
end

function out = enc(v)
    if isstruct(v) && numel(v) ~= 1
        parts = arrayfun(@(e) enc(e), v, 'UniformOutput', false);
        out = ['[' strjoin(parts, ',') ']'];
    elseif isstruct(v)
        f = fieldnames(v);
        parts = cell(1, numel(f));
        for i = 1:numel(f)
            parts{i} = ['"' f{i} '":' enc(v.(f{i}))];
        end
        out = ['{' strjoin(parts, ',') '}'];
    elseif ischar(v) || isstring(v)
        out = ['"' char(v) '"'];
    else
        d = double(v(:)');
        out = ['[' strjoin(compose('%.17g', d), ',') ']'];
    end
end
