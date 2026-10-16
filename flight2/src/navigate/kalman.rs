//! A basic linear Kalman filter for the vertical channel.
//!
//! The state is the IMU's down position and down velocity in the navigation
//! frame. The accelerometer drives the prediction; barometer, LiDAR and GPS
//! correct it. Everything else is held from initialization (horizontal
//! position and velocity, attitude) or passed through (body rate from the
//! gyro) and reported with infinite uncertainty.
//!
//! The filter assumes the vehicle stays close to upright: the accelerometer
//! axis `up_axis` is taken as vertical and LiDAR ranges as vertical
//! distances. The error from a tilt of 5 degrees is under 0.4 %. It is a
//! starting point for testing the navigation interface, not the flight
//! filter.

use std::time::Duration;

use nalgebra::{DMatrix, Matrix2, RowVector2, Vector2, Vector3};

use super::{
    Estimator, ImuSample, Measurement, NavError, NavState, Rejection, Timestamped, UpdateOutcome,
};

/// Standard gravity, m/s^2.
pub const G0: f64 = 9.80665;
/// Specific gas constant of dry air, J/(kg K).
const R_AIR: f64 = 287.05;
/// Temperature lapse rate of the standard troposphere, K/m.
const LAPSE_RATE: f64 = 0.0065;

/// Tuning and sensor layout for [`VerticalKalmanFilter`].
#[derive(Debug, Clone, PartialEq)]
pub struct VerticalKalmanConfig {
    /// Body-frame direction that points up when the vehicle is upright.
    pub up_axis: Vector3<f64>,
    /// Gravity used to turn specific force into acceleration, m/s^2.
    pub gravity: f64,
    /// Accelerometer white noise density along `up_axis`, m/s^2/sqrt(Hz).
    /// Set it higher than the datasheet value to cover accelerometer bias
    /// and tilt, which the filter does not model.
    pub accel_noise_density: f64,
    /// Pressure at ground level at the pad, Pa.
    pub baro_reference_pressure: f64,
    /// Temperature at ground level at the pad, K.
    pub baro_reference_temperature: f64,
    /// Barometric altitude uncertainty, m. Includes the sensor's bias.
    pub baro_std: f64,
    /// Height of each LiDAR above the IMU, m (negative when below). Indexed
    /// by `Measurement::Lidar::index`.
    pub lidar_offsets: Vec<f64>,
    /// Shortest valid LiDAR range, m.
    pub lidar_min_range: f64,
    /// Longest valid LiDAR range, m.
    pub lidar_max_range: f64,
    /// LiDAR range uncertainty, m.
    pub lidar_std: f64,
    /// GPS vertical position uncertainty, m.
    pub gps_position_std: f64,
    /// GPS vertical velocity uncertainty, m/s.
    pub gps_velocity_std: f64,
    /// Largest allowed difference between a measurement's time and the
    /// current estimate time. Measurements inside the window are applied at
    /// the current time; the filter does not compensate for the delay.
    pub max_measurement_age: Duration,
    /// Innovation gate in standard deviations. Measurements further than
    /// this from the prediction are rejected as outliers.
    pub gate_sigma: f64,
}

impl Default for VerticalKalmanConfig {
    /// Starting values based on the sensor models in the hopper 6DOF
    /// simulation (placeholder datasheet values, see `sensor_params.m` in
    /// the hopper repository). Tune before flight.
    fn default() -> Self {
        VerticalKalmanConfig {
            up_axis: Vector3::x(),
            gravity: G0,
            accel_noise_density: 0.02,
            baro_reference_pressure: 101_325.0,
            baro_reference_temperature: 288.15,
            baro_std: 2.0,
            lidar_offsets: vec![-1.14; 4],
            lidar_min_range: 0.2,
            lidar_max_range: 8.0,
            lidar_std: 0.05,
            gps_position_std: 1.0,
            gps_velocity_std: 0.1,
            max_measurement_age: Duration::from_millis(200),
            gate_sigma: 5.0,
        }
    }
}

/// Altitude above the reference level for a pressure, using the standard
/// atmosphere lapse rate, m.
pub fn barometric_altitude(
    pressure: f64,
    reference_pressure: f64,
    reference_temperature: f64,
) -> f64 {
    let exponent = LAPSE_RATE * R_AIR / G0;
    reference_temperature / LAPSE_RATE * (1.0 - (pressure / reference_pressure).powf(exponent))
}

/// Linear Kalman filter on down position and down velocity.
///
/// See the module documentation for what it estimates and assumes.
#[derive(Debug, Clone)]
pub struct VerticalKalmanFilter {
    config: VerticalKalmanConfig,
    /// Down position (m) and down velocity (m/s).
    x: Vector2<f64>,
    p: Matrix2<f64>,
    /// Time of the estimate; `None` until initialized.
    time: Option<Duration>,
    /// Latest gyro reading, passed through as the body rate.
    gyro: Vector3<f64>,
    /// Initial state, for the components this filter does not estimate.
    held: NavState,
}

impl VerticalKalmanFilter {
    /// Creates an uninitialized filter. `config.up_axis` is normalized.
    pub fn new(mut config: VerticalKalmanConfig) -> Self {
        config.up_axis = config.up_axis.normalize();
        VerticalKalmanFilter {
            config,
            x: Vector2::zeros(),
            p: Matrix2::zeros(),
            time: None,
            gyro: Vector3::zeros(),
            held: NavState::unknown(Duration::ZERO),
        }
    }

    /// The filter's configuration.
    pub fn config(&self) -> &VerticalKalmanConfig {
        &self.config
    }

    /// Corrects the state with a scalar measurement `z = h x + noise`,
    /// using the Joseph form so the covariance stays symmetric and positive
    /// semi-definite.
    fn scalar_update(&mut self, h: RowVector2<f64>, z: f64, variance: f64) -> UpdateOutcome {
        let innovation = z - (h * self.x)[0];
        let s = (h * self.p * h.transpose())[0] + variance;
        let nis = innovation * innovation / s;
        if nis > self.config.gate_sigma * self.config.gate_sigma {
            return UpdateOutcome::Rejected(Rejection::Outlier { nis });
        }

        let k = self.p * h.transpose() / s;
        self.x += k * innovation;
        let i_kh = Matrix2::identity() - k * h;
        self.p = i_kh * self.p * i_kh.transpose() + k * variance * k.transpose();
        self.p = 0.5 * (self.p + self.p.transpose());
        UpdateOutcome::Applied { nis }
    }
}

impl Estimator for VerticalKalmanFilter {
    fn initialize(&mut self, initial: &NavState) -> Result<(), NavError> {
        let values = [
            initial.position.z,
            initial.velocity.z,
            initial.position_std.z,
            initial.velocity_std.z,
        ];
        if values.iter().any(|v| !v.is_finite()) {
            return Err(NavError::InvalidInitialState(
                "down position, down velocity and their std must be finite",
            ));
        }

        self.x = Vector2::new(initial.position.z, initial.velocity.z);
        self.p = Matrix2::from_diagonal(&Vector2::new(
            initial.position_std.z.powi(2),
            initial.velocity_std.z.powi(2),
        ));
        self.time = Some(initial.time);
        self.gyro = initial.body_rate;
        self.held = *initial;
        Ok(())
    }

    fn reset(&mut self) {
        self.time = None;
    }

    fn predict(&mut self, imu: &Timestamped<ImuSample>) -> Result<(), NavError> {
        let current = self.time.ok_or(NavError::NotInitialized)?;
        if imu.time <= current {
            return Err(NavError::TimeNotIncreasing {
                current,
                sample: imu.time,
            });
        }

        let dt = (imu.time - current).as_secs_f64();
        let accel_down = self.config.gravity - self.config.up_axis.dot(&imu.value.accel);

        let f = Matrix2::new(1.0, dt, 0.0, 1.0);
        let b = Vector2::new(0.5 * dt * dt, dt);
        let q = self.config.accel_noise_density.powi(2)
            * Matrix2::new(dt.powi(3) / 3.0, dt.powi(2) / 2.0, dt.powi(2) / 2.0, dt);

        self.x = f * self.x + b * accel_down;
        self.p = f * self.p * f.transpose() + q;
        self.time = Some(imu.time);
        self.gyro = imu.value.gyro;
        Ok(())
    }

    fn update(
        &mut self,
        measurement: &Timestamped<Measurement>,
    ) -> Result<UpdateOutcome, NavError> {
        let current = self.time.ok_or(NavError::NotInitialized)?;
        if measurement.time.abs_diff(current) > self.config.max_measurement_age {
            return Ok(UpdateOutcome::Rejected(Rejection::OutOfTimeWindow));
        }

        let position = RowVector2::new(1.0, 0.0);
        let velocity = RowVector2::new(0.0, 1.0);
        let invalid = UpdateOutcome::Rejected(Rejection::InvalidReading);

        let outcome = match measurement.value {
            Measurement::Barometer { pressure, .. } => {
                if !(pressure.is_finite() && pressure > 0.0) {
                    return Ok(invalid);
                }
                let altitude = barometric_altitude(
                    pressure,
                    self.config.baro_reference_pressure,
                    self.config.baro_reference_temperature,
                );
                self.scalar_update(position, -altitude, self.config.baro_std.powi(2))
            }
            Measurement::Lidar { index, range } => {
                let Some(&offset) = self.config.lidar_offsets.get(index) else {
                    return Ok(invalid);
                };
                if !(range >= self.config.lidar_min_range && range <= self.config.lidar_max_range) {
                    return Ok(invalid);
                }
                // The LiDAR sits `offset` above the IMU and measures its own
                // height above the ground.
                let imu_height = range - offset;
                self.scalar_update(position, -imu_height, self.config.lidar_std.powi(2))
            }
            Measurement::GpsPosition { position: p } => {
                if !p.z.is_finite() {
                    return Ok(invalid);
                }
                self.scalar_update(position, p.z, self.config.gps_position_std.powi(2))
            }
            Measurement::GpsVelocity { velocity: v } => {
                if !v.z.is_finite() {
                    return Ok(invalid);
                }
                self.scalar_update(velocity, v.z, self.config.gps_velocity_std.powi(2))
            }
            Measurement::Magnetometer { .. } => UpdateOutcome::NotUsed,
        };
        Ok(outcome)
    }

    fn nav_state(&self) -> Option<NavState> {
        let time = self.time?;
        let mut state = NavState::unknown(time);
        state.position = Vector3::new(self.held.position.x, self.held.position.y, self.x[0]);
        state.velocity = Vector3::new(self.held.velocity.x, self.held.velocity.y, self.x[1]);
        state.attitude = self.held.attitude;
        state.body_rate = self.gyro;
        state.position_std.z = self.p[(0, 0)].sqrt();
        state.velocity_std.z = self.p[(1, 1)].sqrt();
        Some(state)
    }

    fn covariance(&self) -> Option<DMatrix<f64>> {
        self.time?;
        Some(DMatrix::from_iterator(2, 2, self.p.iter().copied()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigate::testing::{self, Scenario, SensorNoise};

    fn at(seconds: f64) -> Duration {
        Duration::from_secs_f64(seconds)
    }

    /// A filter whose noise settings match `noise`, so it should be
    /// statistically consistent on scenarios generated with that noise.
    fn matched_filter(noise: &SensorNoise, scenario: &Scenario) -> VerticalKalmanFilter {
        VerticalKalmanFilter::new(VerticalKalmanConfig {
            accel_noise_density: noise.accel_std * scenario.imu_period.as_secs_f64().sqrt(),
            baro_std: noise.baro_altitude_std(),
            lidar_offsets: scenario.lidar_offsets.clone(),
            lidar_std: noise.lidar_std,
            gps_position_std: noise.gps_position_std,
            gps_velocity_std: noise.gps_velocity_std,
            ..VerticalKalmanConfig::default()
        })
    }

    fn initial(down: f64, down_velocity: f64, down_std: f64, velocity_std: f64) -> NavState {
        let mut state = NavState::unknown(Duration::ZERO);
        state.position.z = down;
        state.velocity.z = down_velocity;
        state.position_std.z = down_std;
        state.velocity_std.z = velocity_std;
        state
    }

    // ---- Shared estimator checks (navigate::testing) ----

    #[test]
    fn follows_time_rules() {
        let mut filter = VerticalKalmanFilter::new(VerticalKalmanConfig::default());
        testing::check_time_rules(&mut filter, &Scenario::stationary(2.0, 1.0));
    }

    #[test]
    fn starts_from_initial_state() {
        let mut filter = VerticalKalmanFilter::new(VerticalKalmanConfig::default());
        testing::check_initialization(&mut filter, &Scenario::stationary(2.0, 1.0));
    }

    #[test]
    fn stays_put_on_the_pad() {
        let scenario = Scenario::stationary(1.14, 20.0).with_noise(SensorNoise::typical());
        let mut filter = matched_filter(&scenario.noise, &scenario);
        testing::check_stationary(&mut filter, &scenario, 0.05, 0.05);
    }

    #[test]
    fn converges_from_a_wrong_start() {
        let scenario = Scenario::stationary(1.14, 10.0);
        let mut filter = matched_filter(&SensorNoise::typical(), &scenario);
        let error = testing::check_convergence(&mut filter, &scenario, 3.0, 1.0);
        assert!(error < 1e-3, "altitude error {error} m after 10 s");
    }

    #[test]
    fn tracks_a_hop() {
        let scenario = Scenario::hop(1.14, 50.0, 20.0).with_noise(SensorNoise::typical());
        let mut filter = matched_filter(&scenario.noise, &scenario);
        let rms = testing::check_tracking(&mut filter, &scenario, 4.0);
        assert!(rms.position < 0.05, "RMS altitude error {} m", rms.position);
        assert!(
            rms.velocity < 0.05,
            "RMS vertical velocity error {} m/s",
            rms.velocity
        );
    }

    #[test]
    fn is_statistically_consistent() {
        let scenario = Scenario::hop(1.14, 50.0, 20.0).with_noise(SensorNoise::typical());
        let mut filter = matched_filter(&scenario.noise, &scenario);
        let result = testing::check_consistency(&mut filter, &scenario, 20);
        // Every update is scalar, so the mean NIS should be about 1.
        assert!(
            (0.9..=1.1).contains(&result.mean_nis),
            "mean NIS {} ({result:?})",
            result.mean_nis
        );
    }

    // ---- Checks specific to this filter ----

    #[test]
    fn barometric_altitude_matches_standard_atmosphere() {
        assert_eq!(barometric_altitude(101_325.0, 101_325.0, 288.15), 0.0);
        // Standard atmosphere pressure at 100 m and 1000 m.
        let p100 = 101_325.0 * (1.0 - LAPSE_RATE * 100.0 / 288.15).powf(G0 / (LAPSE_RATE * R_AIR));
        assert!((barometric_altitude(p100, 101_325.0, 288.15) - 100.0).abs() < 1e-9);
        assert!((barometric_altitude(89_874.6, 101_325.0, 288.15) - 1000.0).abs() < 0.1);
    }

    #[test]
    fn prediction_matches_hand_calculation() {
        let mut filter = VerticalKalmanFilter::new(VerticalKalmanConfig {
            accel_noise_density: 0.1,
            ..VerticalKalmanConfig::default()
        });
        filter.initialize(&initial(-2.0, 0.5, 1.0, 0.2)).unwrap();

        // Specific force 1 m/s^2 above gravity along +x (up): 1 m/s^2 upward.
        let imu = ImuSample {
            accel: Vector3::new(G0 + 1.0, 0.0, 0.0),
            gyro: Vector3::new(0.01, 0.02, 0.03),
        };
        filter.predict(&Timestamped::new(at(0.5), imu)).unwrap();

        // x = F x + B a with dt = 0.5, a_down = -1.
        // d = -2 + 0.5*0.5 - 0.5*0.25 = -1.875, v = 0.5 - 0.5 = 0.
        let state = filter.nav_state().unwrap();
        assert!((state.position.z + 1.875).abs() < 1e-12);
        assert!(state.velocity.z.abs() < 1e-12);
        assert_eq!(state.body_rate, imu.gyro);

        // P = F P F' + Q with q = 0.01.
        // P00 = 1 + 0.25*0.04 + 0.01*0.125/3, P01 = 0.5*0.04 + 0.01*0.125,
        // P11 = 0.04 + 0.01*0.5.
        let p = filter.covariance().unwrap();
        assert!((p[(0, 0)] - (1.0 + 0.01 + 0.01 * 0.125 / 3.0)).abs() < 1e-12);
        assert!((p[(0, 1)] - (0.02 + 0.00125)).abs() < 1e-12);
        assert!((p[(1, 0)] - p[(0, 1)]).abs() < 1e-15);
        assert!((p[(1, 1)] - 0.045).abs() < 1e-12);
    }

    #[test]
    fn repeated_measurements_average_like_a_weighted_mean() {
        // With no motion and no process noise the filter must return the
        // inverse-variance weighted mean of the prior and the measurements.
        let config = VerticalKalmanConfig {
            lidar_offsets: vec![0.0],
            lidar_std: 0.5,
            gate_sigma: 100.0,
            ..VerticalKalmanConfig::default()
        };
        let mut filter = VerticalKalmanFilter::new(config);
        let (prior, prior_var, meas_var): (f64, f64, f64) = (-3.0, 4.0, 0.25);
        filter
            .initialize(&initial(prior, 0.0, prior_var.sqrt(), 0.0))
            .unwrap();

        let ranges = [2.9, 3.3, 3.05, 2.8, 3.2, 3.1];
        for range in ranges {
            let lidar = Measurement::Lidar { index: 0, range };
            filter.update(&Timestamped::new(at(0.0), lidar)).unwrap();
        }

        let information = 1.0 / prior_var + ranges.len() as f64 / meas_var;
        let mean = (prior / prior_var - ranges.iter().sum::<f64>() / meas_var) / information;
        let state = filter.nav_state().unwrap();
        assert!((state.position.z - mean).abs() < 1e-12);
        assert!((state.position_std.z.powi(2) - 1.0 / information).abs() < 1e-12);
    }

    #[test]
    fn lidar_uses_mount_offset_and_range_limits() {
        let mut filter = VerticalKalmanFilter::new(VerticalKalmanConfig::default());
        filter.initialize(&initial(-2.0, 0.0, 10.0, 1.0)).unwrap();

        // Default mounts are 1.14 m below the IMU: 1 m range puts the IMU at
        // 2.14 m.
        let t = at(0.0);
        let reading = |index, range| Timestamped::new(t, Measurement::Lidar { index, range });
        for _ in 0..50 {
            filter.update(&reading(2, 1.0)).unwrap();
        }
        assert!((filter.nav_state().unwrap().position.z + 2.14).abs() < 1e-3);

        let invalid = Ok(UpdateOutcome::Rejected(Rejection::InvalidReading));
        assert_eq!(filter.update(&reading(0, -1.0)), invalid);
        assert_eq!(filter.update(&reading(0, 0.1)), invalid);
        assert_eq!(filter.update(&reading(0, 9.0)), invalid);
        assert_eq!(filter.update(&reading(0, f64::NAN)), invalid);
        assert_eq!(filter.update(&reading(4, 1.0)), invalid);
    }

    #[test]
    fn rejects_outliers_without_changing_the_estimate() {
        let mut filter = VerticalKalmanFilter::new(VerticalKalmanConfig::default());
        filter.initialize(&initial(-2.0, 0.0, 0.1, 0.1)).unwrap();
        let before = filter.nav_state().unwrap();

        let far = Measurement::GpsPosition {
            position: Vector3::new(0.0, 0.0, -30.0),
        };
        let outcome = filter.update(&Timestamped::new(at(0.0), far)).unwrap();
        assert!(matches!(
            outcome,
            UpdateOutcome::Rejected(Rejection::Outlier { .. })
        ));
        assert_eq!(filter.nav_state().unwrap(), before);
    }

    #[test]
    fn ignores_the_magnetometer() {
        let mut filter = VerticalKalmanFilter::new(VerticalKalmanConfig::default());
        filter.initialize(&initial(-2.0, 0.0, 1.0, 1.0)).unwrap();
        let mag = Measurement::Magnetometer {
            field: Vector3::new(0.2, 0.0, 0.4),
        };
        assert_eq!(
            filter.update(&Timestamped::new(at(0.0), mag)),
            Ok(UpdateOutcome::NotUsed)
        );
    }

    #[test]
    fn reports_only_the_vertical_channel_as_estimated() {
        let mut filter = VerticalKalmanFilter::new(VerticalKalmanConfig::default());
        let mut start = initial(-2.0, 0.0, 1.0, 0.5);
        start.position.x = 4.0;
        start.velocity.y = -1.0;
        filter.initialize(&start).unwrap();

        let state = filter.nav_state().unwrap();
        assert_eq!(state.position, Vector3::new(4.0, 0.0, -2.0));
        assert_eq!(state.velocity, Vector3::new(0.0, -1.0, 0.0));
        assert_eq!(state.position_std.z, 1.0);
        assert_eq!(state.velocity_std.z, 0.5);
        for std in [
            state.position_std.x,
            state.position_std.y,
            state.velocity_std.x,
            state.velocity_std.y,
        ] {
            assert_eq!(std, f64::INFINITY);
        }
        assert!(state.attitude_std.iter().all(|s| s.is_infinite()));
        assert!(state.body_rate_std.iter().all(|s| s.is_infinite()));
    }

    #[test]
    fn rejects_an_initial_state_without_vertical_values() {
        let mut filter = VerticalKalmanFilter::new(VerticalKalmanConfig::default());
        let start = NavState::unknown(Duration::ZERO);
        assert!(matches!(
            filter.initialize(&start),
            Err(NavError::InvalidInitialState(_))
        ));
        assert!(filter.nav_state().is_none());
    }
}
