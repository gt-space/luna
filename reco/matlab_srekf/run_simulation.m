% simulates a vertical launch trajectory with accelerometer, gyro, barometer,
% and GPS sensor data, running the SR-EKF filter and plotting the state trajectory
% along with 3-sigma confidence envelopes

clear; clc; close all;

fprintf('Initializing SR-EKF simulation...\n');

%% simulation parameters
dt = 0.01;            % 100 Hz IMU rate (10 ms)
t_final = 5.0;        % 5 seconds duration
time = 0:dt:t_final;
N = length(time);

% sensor update rates
gps_interval  = 10;   % GPS updates at 10 Hz (every 10 IMU steps)
baro_interval = 5;    % Baro updates at 20 Hz (every 5 IMU steps)

%% initialize filter
filter = srekf_init(dt);

%% preallocate trajectory storage
history.time = time;
history.alt_est = zeros(1, N);
history.alt_sigma3 = zeros(1, N);
history.vn_est = zeros(1, N);
history.vd_est = zeros(1, N);
history.pitch_est = zeros(1, N);

history.alt_true = zeros(1, N);
history.vel_true = zeros(1, N);

% true initial state
true_alt = filter.x(7);
true_vd  = 0.0; % down velocity
accel_z  = -15.0; % constant vertical thrust acceleration (-15 m/s^2 upwards)

fprintf('Running %d steps of SR-EKF propagation and multi-sensor update...\n', N);

for k = 1:N
    t = time(k);

    % simple 1D kinematic ground truth (vertical launch)
    if k > 1
        true_vd = true_vd + (accel_z + 9.80665) * dt;
        true_alt = true_alt - true_vd * dt;
    end
    history.alt_true(k) = true_alt;
    history.vel_true(k) = -true_vd;

    % synthetic IMU measurement with white noise
    imu.accel = [0.0; 0.0; accel_z] + 0.05 * randn(3, 1);
    imu.gyro  = [0.0; 0.0; 0.0]     + 0.001 * randn(3, 1);
    imu.dt    = dt;

    % synthetic GPS measurement (10 Hz)
    gps.valid = (mod(k, gps_interval) == 0);
    if gps.valid
        gps.lla = [filter.x(5); filter.x(6); true_alt + 1.5 * randn(1)];
    end

    % synthetic barometer measurement (20 Hz)
    baro.valid = (mod(k, baro_interval) == 0);
    if baro.valid
        [p_true, ~] = srekf_utils.filter_pressure(true_alt, 35.0);
        baro.pressure = p_true + 15.0 * randn(1);
    end

    % magnetometer (fixed NED reference)
    mag.valid = false; % off during main burn

    % step filter
    [filter, flags] = srekf_step(filter, imu, gps, baro, mag);

    if flags.diverged
        error('Filter diverged at step %d (t = %.2f s)!', k, t);
    end

    % extract 3-sigma bound on altitude
    % in error state, index 6 corresponds to altitude error
    % P = Sigma' * Sigma => P(6,6) is altitude variance
    P = filter.Sigma' * filter.Sigma;
    sigma_alt = sqrt(P(6, 6));

    % log history
    history.alt_est(k) = filter.x(7);
    history.alt_sigma3(k) = 3.0 * sigma_alt;
    history.vn_est(k) = filter.x(8);
    history.vd_est(k) = filter.x(10);

    % pitch angle from quaternion
    q = filter.x(1:4);
    history.pitch_est(k) = rad2deg(asin(2.0 * (q(1)*q(3) - q(4)*q(2))));
end

fprintf('Simulation complete! Generating analysis plots...\n');

%% visualizing
figure('Name', 'SR-EKF Trajectory Performance', 'Color', 'w', 'Position', [100, 100, 900, 700]);

% plot 1: altitude & 3-sigma bounds
subplot(2, 2, 1);
plot(time, history.alt_true, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
hold on;
plot(time, history.alt_est, 'b-', 'LineWidth', 1.2, 'DisplayName', 'SR-EKF Estimate');
fill([time, fliplr(time)], ...
    [history.alt_est + history.alt_sigma3, fliplr(history.alt_est - history.alt_sigma3)], ...
    'b', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'DisplayName', '3\sigma Bound');
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Tracking & 3\sigma Envelope');
legend('Location', 'northwest');

% plot 2: altitude error
subplot(2, 2, 2);
alt_error = history.alt_est - history.alt_true;
plot(time, alt_error, 'r-', 'LineWidth', 1.2, 'DisplayName', 'Estimate Error');
hold on;
plot(time,  history.alt_sigma3, 'b--', 'LineWidth', 1.0, 'DisplayName', '+3\sigma');
plot(time, -history.alt_sigma3, 'b--', 'LineWidth', 1.0, 'DisplayName', '-3\sigma');
grid on;
xlabel('Time (s)');
ylabel('Error (m)');
title('Altitude Error vs 3\sigma Bound');
legend('Location', 'northeast');

% plot 3: velocity
subplot(2, 2, 3);
plot(time, history.vel_true, 'k--', 'LineWidth', 1.5, 'DisplayName', 'True Vertical Vel');
hold on;
plot(time, -history.vd_est, 'g-', 'LineWidth', 1.2, 'DisplayName', 'Estimated Upward Vel');
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Vertical Velocity Tracking');
legend('Location', 'northwest');

% plot 4: pitch angle
subplot(2, 2, 4);
plot(time, history.pitch_est, 'm-', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Pitch (deg)');
title('Estimated Pitch Attitude');

fprintf('Done!\n');

