//! GMEKF: the hopper's 6DOF navigation filter, ported from MATLAB.
//!
//! A multiplicative extended Kalman filter on position, velocity, body
//! rate, attitude, accelerometer and gyro biases and a barometer bias. Each
//! IMU sample drives the prediction: the accelerometer the translation, the
//! gyro (minus the estimated gyro bias) the rotation. The barometer, GPS,
//! magnetometer and configured LiDARs correct it when they have a reading.
//!
//! [`math`] holds the equations, one function per MATLAB function, and is
//! tested against the MATLAB filter itself (see `testdata/`). This file
//! wraps them in the [`Estimator`] interface.

pub mod math;

use std::time::Duration;

use nalgebra::{DMatrix, Matrix3, Quaternion, SMatrix, Vector3};

use self::math::{
    Corrected, Covariance, Dynamics, ProcessNoise, StateVector, ACCEL_BIAS, BARO_BIAS, BARO_GATE,
    E_ACCEL_BIAS, E_ATT, E_BARO_BIAS, E_GYRO_BIAS, E_POS, E_RATE, E_VEL, GPS_3DOF_GATE, GPS_GATE,
    GYRO_BIAS, MAG_GATE, POS, QUAT, RATE, VEL,
};
use super::{
    kalman::barometric_altitude, Estimator, ImuSample, Measurement, NavError, NavState, Rejection,
    Timestamped, UpdateOutcome,
};

/// Tuning and vehicle constants for [`Gmekf`].
///
/// IMU noise is per sample (at the IMU rate); the bias random walks are
/// continuous-time power spectral densities, as in the MATLAB filter's `qc`.
#[derive(Debug, Clone, PartialEq)]
pub struct GmekfConfig {
    /// Gravity, m/s^2, pointing down.
    pub gravity: f64,
    /// Inertia tensor in body axes, kg m^2.
    pub inertia: Matrix3<f64>,
    /// Accelerometer white noise per sample, per body axis, m/s^2.
    pub accel_std: Vector3<f64>,
    /// Gyro white noise per sample, per body axis, rad/s.
    pub gyro_std: Vector3<f64>,
    /// Accelerometer bias random walk PSD, (m/s^2)^2/s.
    pub accel_bias_psd: Vector3<f64>,
    /// Gyro bias random walk PSD, (rad/s)^2/s.
    pub gyro_bias_psd: Vector3<f64>,
    /// Barometer bias random walk PSD, m^2/s.
    pub baro_bias_psd: f64,
    /// Initial accelerometer bias uncertainty, m/s^2.
    pub accel_bias_std: f64,
    /// Initial gyro bias uncertainty, rad/s.
    pub gyro_bias_std: f64,
    /// Initial barometer bias uncertainty, m.
    pub baro_bias_std: f64,
    /// Barometric height noise, m.
    pub baro_std: f64,
    /// Pressure at ground level at the pad, Pa.
    pub baro_reference_pressure: f64,
    /// Temperature at ground level at the pad, K.
    pub baro_reference_temperature: f64,
    /// GPS position noise per NED axis, m.
    pub gps_position_std: Vector3<f64>,
    /// GPS velocity noise per NED axis, m/s.
    pub gps_velocity_std: Vector3<f64>,
    /// GPS antenna position relative to the IMU, body axes, m.
    pub gps_lever_arm: Vector3<f64>,
    /// Earth's magnetic field in NED at the pad, Gauss. `None` turns the
    /// magnetometer off.
    pub mag_field_ned: Option<Vector3<f64>>,
    /// Magnetometer noise per body axis, Gauss.
    pub mag_std: f64,
    /// LiDAR beam mounts, indexed by `Measurement::Lidar::index`. Sensor
    /// positions are relative to the IMU, which is the filter's origin, in
    /// body axes; beam directions are unit vectors in body axes.
    pub lidar_mounts: Vec<LidarMount>,
    /// Largest allowed difference between a measurement's time and the
    /// current estimate time.
    pub max_measurement_age: Duration,
    /// Longest LiDAR range used, m. The range noise is calibrated from 0.2 m
    /// through 4.0 m; beyond 4.0 m up to this limit the 4.0 m value is used.
    /// 4.0 keeps the calibrated range. The CTV hops to about 4.5 m, so the
    /// range is lost for the top of the hop unless this is raised.
    pub lidar_max_range: f64,
    /// Shortest LiDAR range used, m. The range noise is calibrated from 0.2 m;
    /// below that, down to this limit, the 0.2 m value is used. 0.2 keeps the
    /// calibrated range. The sensor reads down to about 0.05 m, and on the ground
    /// the beams read about 0.10 m: without them the last 20 cm of a landing, and
    /// the vehicle at rest, are measured by the barometer and GPS alone.
    pub lidar_min_range: f64,
}

/// Geometry for one LiDAR beam used by the GMEKF range model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LidarMount {
    /// Unit beam direction in body axes.
    pub beam_dir_body: Vector3<f64>,
    /// LiDAR position relative to the IMU, in body axes, m.
    pub position_body: Vector3<f64>,
    /// The IMU's height above the ground when the state's down coordinate is
    /// zero (on the pad), m.
    pub height_imu: f64,
}

impl Default for GmekfConfig {
    /// Starting values for the CTV: ASM330LHBG1 IMU at 208 Hz, MS5611
    /// barometer, standalone GPS. The IMU noise is set well above the
    /// datasheet (about 7 mm/s^2 and 0.04 deg/s per sample at 208 Hz) to
    /// cover vibration. Tune before using the estimate for control.
    fn default() -> Self {
        let deg = std::f64::consts::PI / 180.0;
        GmekfConfig {
            gravity: 9.795, // Atlanta
            // CTV principal moments, body x along the vehicle.
            inertia: Matrix3::from_diagonal(&Vector3::new(0.0123, 0.1032, 0.1038)),
            accel_std: Vector3::repeat(0.3),
            gyro_std: Vector3::repeat(0.1 * deg),
            accel_bias_psd: Vector3::repeat(4e-4_f64.powi(2) / 100.0),
            gyro_bias_psd: Vector3::repeat((3.0 * deg / 3600.0).powi(2) / 100.0),
            baro_bias_psd: 1e-3,
            accel_bias_std: 0.2,
            gyro_bias_std: 0.5 * deg,
            baro_bias_std: 2.0,
            baro_std: 0.5,
            baro_reference_pressure: 101_325.0,
            baro_reference_temperature: 288.15,
            // Measured on the CTV receiver (static, no RTK). Its error is
            // time-correlated (~100 s), which this white-noise model ignores.
            gps_position_std: Vector3::new(1.50, 1.61, 4.43),
            gps_velocity_std: Vector3::repeat(0.1),
            gps_lever_arm: Vector3::zeros(),
            mag_field_ned: None,
            // nav_sensor_params: 3 mGauss per axis, variance inflated x10.
            mag_std: 9.5e-3,
            // LiDAR geometry must be measured before these updates are used.
            lidar_mounts: Vec::new(),
            max_measurement_age: Duration::from_millis(200),
            lidar_max_range: 4.0,
            lidar_min_range: 0.2,
        }
    }
}

impl GmekfConfig {
    fn process_noise(&self) -> ProcessNoise {
        let mut diag = nalgebra::SVector::<f64, 13>::zeros();
        diag.fixed_rows_mut::<3>(0)
            .copy_from(&self.accel_std.component_mul(&self.accel_std));
        diag.fixed_rows_mut::<3>(3)
            .copy_from(&self.gyro_std.component_mul(&self.gyro_std));
        diag.fixed_rows_mut::<3>(6).copy_from(&self.accel_bias_psd);
        diag.fixed_rows_mut::<3>(9).copy_from(&self.gyro_bias_psd);
        diag[12] = self.baro_bias_psd;
        ProcessNoise::from_diagonal(&diag)
    }
}

/// Per-sensor results of the last IMU step, in the MATLAB filter's
/// `diagnostics` layout: slots 0..8 are barometer, gyro (always -1: the
/// gyro drives the prediction, it is not a correction), GPS, LiDAR 1-4 and
/// magnetometer (1 accepted, 0 gated out, -1 no reading); slots 8..16 are
/// their NIS (-1 none).
pub type Diagnostics = [f64; 16];

const DIAG_BARO: usize = 0;
const DIAG_GPS: usize = 2;
const DIAG_MAG: usize = 7;
const DIAG_LIDAR: usize = 3;
const LIDAR_GATE: f64 = 6.635;

#[derive(Debug, Clone, Copy)]
struct Filter {
    time: Duration,
    x: StateVector,
    p: Covariance,
    diagnostics: Diagnostics,
    /// Time of the last reading used, per sensor, to skip repeats.
    last_baro: Option<Duration>,
    last_gps: Option<Duration>,
    last_mag: Option<Duration>,
    last_lidar: [Option<Duration>; 4],
}

/// The GMEKF behind the [`Estimator`] interface. See the module
/// documentation.
#[derive(Debug, Clone)]
pub struct Gmekf {
    config: GmekfConfig,
    dynamics: Dynamics,
    qc: ProcessNoise,
    filter: Option<Filter>,
}

impl Gmekf {
    /// Creates an uninitialized filter. Panics if the inertia tensor is not
    /// invertible.
    pub fn new(config: GmekfConfig) -> Self {
        let dynamics = Dynamics::new(config.inertia, config.gravity)
            .expect("GMEKF inertia tensor must be invertible");
        let qc = config.process_noise();
        Gmekf {
            config,
            dynamics,
            qc,
            filter: None,
        }
    }

    /// The filter's configuration.
    pub fn config(&self) -> &GmekfConfig {
        &self.config
    }

    /// The nominal state (20) and error covariance (19 x 19). See [`math`]
    /// for the layout.
    pub fn estimate(&self) -> Option<(StateVector, Covariance)> {
        let f = self.filter.as_ref()?;
        Some((f.x, f.p))
    }

    /// Per-sensor results of the current IMU step.
    pub fn diagnostics(&self) -> Option<Diagnostics> {
        Some(self.filter.as_ref()?.diagnostics)
    }

    /// Like [`Estimator::initialize`], then sets the gyro bias estimate
    /// (body, rad/s) and the heading uncertainty (rad). The attitude error is
    /// in NED axes, so `heading_std` widens only the rotation about down:
    /// leveling from the accelerometer gives tilt but not heading.
    pub fn initialize_on_pad(
        &mut self,
        initial: &NavState,
        gyro_bias: Vector3<f64>,
        heading_std: f64,
    ) -> Result<(), NavError> {
        self.initialize(initial)?;
        let f = self.filter.as_mut().ok_or(NavError::NotInitialized)?;
        f.x.fixed_rows_mut::<3>(GYRO_BIAS).copy_from(&gyro_bias);
        let down = E_ATT + 2;
        f.p.row_mut(down).fill(0.0);
        f.p.column_mut(down).fill(0.0);
        f.p[(down, down)] = heading_std.powi(2);
        Ok(())
    }

    /// Accelerometer bias (m/s^2), gyro bias (rad/s), both in body axes, and
    /// barometer bias (m).
    pub fn biases(&self) -> Option<(Vector3<f64>, Vector3<f64>, f64)> {
        let (x, _) = self.estimate()?;
        Some((
            x.fixed_rows::<3>(ACCEL_BIAS).into_owned(),
            x.fixed_rows::<3>(GYRO_BIAS).into_owned(),
            x[BARO_BIAS],
        ))
    }

    /// Checks the measurement time and returns the filter, or the outcome to
    /// report instead.
    fn filter_for(
        &mut self,
        time: Duration,
        last: fn(&mut Filter) -> &mut Option<Duration>,
    ) -> Result<Result<&mut Filter, UpdateOutcome>, NavError> {
        let age = self.config.max_measurement_age;
        let f = self.filter.as_mut().ok_or(NavError::NotInitialized)?;
        let previous = *last(f);
        if time.abs_diff(f.time) > age || previous.is_some_and(|p| time <= p) {
            return Ok(Err(UpdateOutcome::Rejected(Rejection::OutOfTimeWindow)));
        }
        *last(f) = Some(time);
        Ok(Ok(f))
    }

    /// Corrects the estimate with a barometric height above the pad, m.
    pub fn update_baro_height(
        &mut self,
        time: Duration,
        height: f64,
    ) -> Result<UpdateOutcome, NavError> {
        if !height.is_finite() {
            self.filter.ok_or(NavError::NotInitialized)?;
            return Ok(UpdateOutcome::Rejected(Rejection::InvalidReading));
        }
        let r = SMatrix::<f64, 1, 1>::new(self.config.baro_std.powi(2));
        let f = match self.filter_for(time, |f| &mut f.last_baro)? {
            Ok(f) => f,
            Err(outcome) => return Ok(outcome),
        };
        let c = math::update(
            &f.x,
            &f.p,
            Some(math::baro_model(&f.x, height)),
            &r,
            BARO_GATE,
        );
        Ok(apply(f, DIAG_BARO, c))
    }

    /// Corrects the estimate with a GPS fix: antenna position in NED (m) and,
    /// if given, antenna velocity in NED (m/s). Position and velocity are
    /// fused together, as in the MATLAB filter.
    pub fn update_gps(
        &mut self,
        time: Duration,
        position: Vector3<f64>,
        velocity: Option<Vector3<f64>>,
    ) -> Result<UpdateOutcome, NavError> {
        let finite = position
            .iter()
            .chain(velocity.iter().flatten())
            .all(|v| v.is_finite());
        if !finite {
            self.filter.ok_or(NavError::NotInitialized)?;
            return Ok(UpdateOutcome::Rejected(Rejection::InvalidReading));
        }
        let config = self.config.clone();
        let f = match self.filter_for(time, |f| &mut f.last_gps)? {
            Ok(f) => f,
            Err(outcome) => return Ok(outcome),
        };
        Ok(gps_update(&config, f, position, velocity))
    }

    /// Corrects the estimate with GPS velocity alone (NED, m/s).
    fn update_gps_velocity(
        &mut self,
        time: Duration,
        velocity: Vector3<f64>,
    ) -> Result<UpdateOutcome, NavError> {
        if !velocity.iter().all(|v| v.is_finite()) {
            self.filter.ok_or(NavError::NotInitialized)?;
            return Ok(UpdateOutcome::Rejected(Rejection::InvalidReading));
        }
        let config = self.config.clone();
        let age = config.max_measurement_age;
        let f = self.filter.as_mut().ok_or(NavError::NotInitialized)?;
        if time.abs_diff(f.time) > age {
            return Ok(UpdateOutcome::Rejected(Rejection::OutOfTimeWindow));
        }
        let (innovation, h) = math::gps_model(
            &f.x,
            &f.x.fixed_rows::<3>(POS).into_owned(),
            &velocity,
            &config.gps_lever_arm,
        );
        let innovation = innovation.fixed_rows::<3>(3).into_owned();
        let h = h.fixed_rows::<3>(3).into_owned();
        let r = Matrix3::from_diagonal(
            &config
                .gps_velocity_std
                .component_mul(&config.gps_velocity_std),
        );
        let c = math::update(&f.x, &f.p, Some((innovation, h)), &r, GPS_3DOF_GATE);
        Ok(apply(f, DIAG_GPS, c))
    }

    /// Corrects the estimate with a magnetometer reading in body axes, Gauss.
    fn update_mag(
        &mut self,
        time: Duration,
        field: Vector3<f64>,
    ) -> Result<UpdateOutcome, NavError> {
        self.filter.ok_or(NavError::NotInitialized)?;
        let Some(field_ned) = self.config.mag_field_ned else {
            return Ok(UpdateOutcome::NotUsed);
        };
        if !field.iter().all(|v| v.is_finite()) {
            return Ok(UpdateOutcome::Rejected(Rejection::InvalidReading));
        }
        let config = self.config.clone();
        let f = match self.filter_for(time, |f| &mut f.last_mag)? {
            Ok(f) => f,
            Err(outcome) => return Ok(outcome),
        };
        let r = Matrix3::identity() * config.mag_std.powi(2);
        let c = math::update(
            &f.x,
            &f.p,
            Some(math::mag_model(&f.x, &field, &field_ned)),
            &r,
            MAG_GATE,
        );
        Ok(apply(f, DIAG_MAG, c))
    }

    /// Corrects the estimate with one LiDAR range, in meters.
    fn update_lidar(
        &mut self,
        time: Duration,
        index: usize,
        range: f64,
    ) -> Result<UpdateOutcome, NavError> {
        self.filter.as_ref().ok_or(NavError::NotInitialized)?;
        if index >= 4 {
            return Ok(UpdateOutcome::Rejected(Rejection::InvalidReading));
        }
        let Some(mount) = self.config.lidar_mounts.get(index).copied() else {
            return Ok(UpdateOutcome::NotUsed);
        };
        let calibrated = if range > 4.0 && range <= self.config.lidar_max_range {
            math::lidar_covariance(4.0)
        } else if range < 0.2 && range >= self.config.lidar_min_range {
            math::lidar_covariance(0.2)
        } else {
            math::lidar_covariance(range)
        };
        let Some(variance) = calibrated else {
            return Ok(UpdateOutcome::Rejected(Rejection::InvalidReading));
        };

        let age = self.config.max_measurement_age;
        let f = self.filter.as_mut().ok_or(NavError::NotInitialized)?;
        let previous = f.last_lidar[index];
        if time.abs_diff(f.time) > age || previous.is_some_and(|last| time <= last) {
            return Ok(UpdateOutcome::Rejected(Rejection::OutOfTimeWindow));
        }
        f.last_lidar[index] = Some(time);

        let r = SMatrix::<f64, 1, 1>::new(variance);
        let c = math::update(
            &f.x,
            &f.p,
            math::lidar_model(
                &f.x,
                range,
                &mount.beam_dir_body,
                &mount.position_body,
                mount.height_imu,
            ),
            &r,
            LIDAR_GATE,
        );
        Ok(apply(f, DIAG_LIDAR + index, c))
    }
}

// Free functions so they can borrow the config and the filter state
// separately.

fn gps_update(
    config: &GmekfConfig,
    f: &mut Filter,
    position: Vector3<f64>,
    velocity: Option<Vector3<f64>>,
) -> UpdateOutcome {
    let pos_var = config
        .gps_position_std
        .component_mul(&config.gps_position_std);
    let c = match velocity {
        Some(velocity) => {
            let vel_var = config
                .gps_velocity_std
                .component_mul(&config.gps_velocity_std);
            let mut diag = nalgebra::SVector::<f64, 6>::zeros();
            diag.fixed_rows_mut::<3>(0).copy_from(&pos_var);
            diag.fixed_rows_mut::<3>(3).copy_from(&vel_var);
            let r = SMatrix::<f64, 6, 6>::from_diagonal(&diag);
            let model = math::gps_model(&f.x, &position, &velocity, &config.gps_lever_arm);
            math::update(&f.x, &f.p, Some(model), &r, GPS_GATE)
        }
        None => {
            let vel = f.x.fixed_rows::<3>(VEL).into_owned();
            let (innovation, h) = math::gps_model(&f.x, &position, &vel, &config.gps_lever_arm);
            let model = (
                innovation.fixed_rows::<3>(0).into_owned(),
                h.fixed_rows::<3>(0).into_owned(),
            );
            let r = Matrix3::from_diagonal(&pos_var);
            math::update(&f.x, &f.p, Some(model), &r, GPS_3DOF_GATE)
        }
    };
    apply(f, DIAG_GPS, c)
}

fn record(f: &mut Filter, slot: usize, c: &Corrected) {
    f.diagnostics[slot] = if c.accepted { 1.0 } else { 0.0 };
    f.diagnostics[slot + 8] = c.nis;
}

fn apply(f: &mut Filter, slot: usize, c: Corrected) -> UpdateOutcome {
    record(f, slot, &c);
    f.x = c.x;
    f.p = c.p;
    if c.accepted {
        UpdateOutcome::Applied { nis: c.nis }
    } else if c.nis < 0.0 {
        UpdateOutcome::Rejected(Rejection::InvalidReading)
    } else {
        UpdateOutcome::Rejected(Rejection::Outlier { nis: c.nis })
    }
}

impl Estimator for Gmekf {
    fn initialize(&mut self, initial: &NavState) -> Result<(), NavError> {
        let q = initial.attitude;
        let values = [initial.position, initial.velocity, initial.body_rate];
        let stds = [
            initial.position_std,
            initial.velocity_std,
            initial.body_rate_std,
            initial.attitude_std,
        ];
        let finite = |v: &Vector3<f64>| v.iter().all(|c| c.is_finite());
        if !values.iter().all(finite) || !q.coords.iter().all(|c| c.is_finite()) {
            return Err(NavError::InvalidInitialState(
                "position, velocity, body rate and attitude must be finite",
            ));
        }
        if !stds.iter().all(finite) {
            return Err(NavError::InvalidInitialState(
                "position, velocity, body rate and attitude std must be finite",
            ));
        }
        let norm = q.norm();
        if norm < 1e-9 {
            return Err(NavError::InvalidInitialState("attitude quaternion is zero"));
        }

        let mut x = StateVector::zeros();
        x.fixed_rows_mut::<3>(POS).copy_from(&initial.position);
        x.fixed_rows_mut::<3>(VEL).copy_from(&initial.velocity);
        x.fixed_rows_mut::<3>(RATE).copy_from(&initial.body_rate);
        x.fixed_rows_mut::<4>(QUAT)
            .copy_from(&nalgebra::Vector4::new(
                q.w / norm,
                q.i / norm,
                q.j / norm,
                q.k / norm,
            ));

        let sq = |v: Vector3<f64>| Matrix3::from_diagonal(&v.component_mul(&v));
        let mut p = Covariance::zeros();
        p.fixed_view_mut::<3, 3>(E_POS, E_POS)
            .copy_from(&sq(initial.position_std));
        p.fixed_view_mut::<3, 3>(E_VEL, E_VEL)
            .copy_from(&sq(initial.velocity_std));
        p.fixed_view_mut::<3, 3>(E_RATE, E_RATE)
            .copy_from(&sq(initial.body_rate_std));
        // The attitude error is in inertial axes; NavState gives body axes.
        let c = math::quaternion_to_rotation(&x.fixed_rows::<4>(QUAT).into_owned());
        p.fixed_view_mut::<3, 3>(E_ATT, E_ATT)
            .copy_from(&(c * sq(initial.attitude_std) * c.transpose()));
        let cfg = &self.config;
        p.fixed_view_mut::<3, 3>(E_ACCEL_BIAS, E_ACCEL_BIAS)
            .copy_from(&(Matrix3::identity() * cfg.accel_bias_std.powi(2)));
        p.fixed_view_mut::<3, 3>(E_GYRO_BIAS, E_GYRO_BIAS)
            .copy_from(&(Matrix3::identity() * cfg.gyro_bias_std.powi(2)));
        p[(E_BARO_BIAS, E_BARO_BIAS)] = cfg.baro_bias_std.powi(2);

        self.filter = Some(Filter {
            time: initial.time,
            x,
            p,
            diagnostics: [-1.0; 16],
            last_baro: None,
            last_gps: None,
            last_mag: None,
            last_lidar: [None; 4],
        });
        Ok(())
    }

    fn reset(&mut self) {
        self.filter = None;
    }

    fn predict(&mut self, imu: &Timestamped<ImuSample>) -> Result<(), NavError> {
        let finite = |v: &Vector3<f64>| v.iter().all(|c| c.is_finite());
        if !(finite(&imu.value.accel) && finite(&imu.value.gyro)) {
            return Err(NavError::InvalidSample { time: imu.time });
        }
        let (dynamics, qc) = (self.dynamics, self.qc);
        let f = self.filter.as_mut().ok_or(NavError::NotInitialized)?;
        if imu.time <= f.time {
            return Err(NavError::TimeNotIncreasing {
                current: f.time,
                sample: imu.time,
            });
        }

        // The time step comes from the sample times, so it follows the IMU's
        // actual rate.
        let dt = (imu.time - f.time).as_secs_f64();
        let (x, p) = math::gmekf_predict(
            &f.x,
            &f.p,
            &imu.value.accel,
            &imu.value.gyro,
            true,
            &dynamics,
            dt,
            &qc,
        );
        f.x = x;
        f.p = p;
        f.time = imu.time;
        f.diagnostics = [-1.0; 16];
        Ok(())
    }

    fn update(
        &mut self,
        measurement: &Timestamped<Measurement>,
    ) -> Result<UpdateOutcome, NavError> {
        self.filter.ok_or(NavError::NotInitialized)?;
        let time = measurement.time;
        match measurement.value {
            Measurement::Barometer { pressure, .. } => {
                if !(pressure.is_finite() && pressure > 0.0) {
                    return Ok(UpdateOutcome::Rejected(Rejection::InvalidReading));
                }
                let height = barometric_altitude(
                    pressure,
                    self.config.baro_reference_pressure,
                    self.config.baro_reference_temperature,
                );
                self.update_baro_height(time, height)
            }
            Measurement::GpsPosition { position } => self.update_gps(time, position, None),
            Measurement::GpsVelocity { velocity } => self.update_gps_velocity(time, velocity),
            Measurement::Magnetometer { field } => self.update_mag(time, field),
            Measurement::Lidar { index, range } => self.update_lidar(time, index, range),
        }
    }

    fn nav_state(&self) -> Option<NavState> {
        let time = self.filter?.time;
        let (x, p) = self.estimate()?;
        let v3 = |o: usize| x.fixed_rows::<3>(o).into_owned();
        let std3 =
            |o: usize| Vector3::new(p[(o, o)], p[(o + 1, o + 1)], p[(o + 2, o + 2)]).map(f64::sqrt);
        let q = x.fixed_rows::<4>(QUAT).into_owned();
        // Attitude error covariance in body axes.
        let c = math::quaternion_to_rotation(&q);
        let att_body = c.transpose() * p.fixed_view::<3, 3>(E_ATT, E_ATT) * c;

        Some(NavState {
            time,
            position: v3(POS),
            velocity: v3(VEL),
            attitude: Quaternion::new(q[0], q[1], q[2], q[3]),
            body_rate: v3(RATE),
            position_std: std3(E_POS),
            velocity_std: std3(E_VEL),
            attitude_std: att_body.diagonal().map(f64::sqrt),
            body_rate_std: std3(E_RATE),
        })
    }

    fn covariance(&self) -> Option<DMatrix<f64>> {
        let (_, p) = self.estimate()?;
        Some(DMatrix::from_iterator(19, 19, p.iter().copied()))
    }
}

#[cfg(test)]
mod tests;
#[cfg(test)]
mod replay;
