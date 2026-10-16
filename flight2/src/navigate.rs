//! Navigation: turns sensor measurements into a state estimate for the
//! control loop.
//!
//! [`Estimator`] is the interface every navigation filter implements, the
//! same way `Controller` in `control.rs` is the interface for control laws.
//! The main loop feeds each IMU sample to [`Estimator::predict`], feeds every
//! other sensor reading to [`Estimator::update`], and reads the result with
//! [`Estimator::nav_state`]. [`NavState::to_control_state`] turns that into
//! the controller's input.
//!
//! Conventions used by every estimator:
//! - Navigation frame: local North-East-Down, origin at ground level at the
//!   pad, same as the 6DOF Simulink model.
//! - Body frame: as in the 6DOF model. Body x points up while the vehicle
//!   stands on the pad.
//! - Position and velocity are those of the IMU. Moving them to the CG is
//!   not handled yet.
//! - Time is the time since the start of the control loop, the same clock as
//!   `ControlState::time`. Every input carries the time it was measured.
//! - Units are SI (m, m/s, rad, rad/s, Pa). Convert driver units, such as
//!   the IMU's deg/s, before handing samples to an estimator.
//!
//! This module only depends on `nalgebra` and luna's `ControlState`, so it
//! does not need any flight computer hardware to build or test.

use std::{error::Error, fmt, time::Duration};

use common::comm::ctv::ControlState;
use nalgebra::{DMatrix, Quaternion, Vector3};

pub mod kalman;

#[cfg(test)]
pub mod testing;

/// A value together with the time it was measured.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Timestamped<T> {
    /// Measurement time, since the start of the control loop.
    pub time: Duration,
    /// The measured value.
    pub value: T,
}

impl<T> Timestamped<T> {
    /// Wraps `value` measured at `time`.
    pub fn new(time: Duration, value: T) -> Self {
        Timestamped { time, value }
    }
}

/// One IMU sample in the body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Specific force (acceleration minus gravity) in m/s^2. At rest on the
    /// pad this reads about +9.81 along the up axis.
    pub accel: Vector3<f64>,
    /// Angular rate in rad/s.
    pub gyro: Vector3<f64>,
}

impl ImuSample {
    /// Builds a sample from the flight computer's IMU units: acceleration in
    /// m/s^2 and angular rate in deg/s (see `common::comm::fc_sensors::Imu`).
    pub fn from_deg_per_s(accel: Vector3<f64>, gyro_deg_per_s: Vector3<f64>) -> Self {
        ImuSample {
            accel,
            gyro: gyro_deg_per_s.map(f64::to_radians),
        }
    }
}

/// A reading from any sensor other than the IMU.
///
/// Each estimator uses the kinds it supports and answers
/// [`UpdateOutcome::NotUsed`] for the rest.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Measurement {
    /// Static pressure in Pa and temperature in degrees Celsius.
    Barometer { pressure: f64, temperature: f64 },
    /// Range in m from one downward-facing LiDAR. `index` selects the
    /// sensor's mounting position in the estimator's configuration.
    Lidar { index: usize, range: f64 },
    /// GPS antenna position in the navigation frame, in m. Only send fixes
    /// that are valid.
    GpsPosition { position: Vector3<f64> },
    /// GPS velocity in the navigation frame, in m/s. Only send fixes that
    /// are valid.
    GpsVelocity { velocity: Vector3<f64> },
    /// Magnetic field in the body frame, in Gauss.
    Magnetometer { field: Vector3<f64> },
}

/// The estimated vehicle state with its uncertainty.
///
/// Each `*_std` field is the 1-sigma uncertainty of the matching component.
/// It is `f64::INFINITY` for components the estimator does not estimate;
/// their values are then held from initialization or passed through from a
/// sensor, and must not be trusted as an estimate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavState {
    /// Time of the estimate, since the start of the control loop.
    pub time: Duration,
    /// Position in the navigation frame (NED), m.
    pub position: Vector3<f64>,
    /// Velocity in the navigation frame (NED), m/s.
    pub velocity: Vector3<f64>,
    /// Attitude quaternion, same convention as `ControlState::attitude`.
    pub attitude: Quaternion<f64>,
    /// Angular rate in the body frame, rad/s.
    pub body_rate: Vector3<f64>,
    /// Position uncertainty (NED), m.
    pub position_std: Vector3<f64>,
    /// Velocity uncertainty (NED), m/s.
    pub velocity_std: Vector3<f64>,
    /// Attitude uncertainty as small rotation angles about the body axes, rad.
    pub attitude_std: Vector3<f64>,
    /// Angular rate uncertainty, rad/s.
    pub body_rate_std: Vector3<f64>,
}

impl NavState {
    /// A state at `time` with zero position, velocity and rate, identity
    /// attitude and every uncertainty infinite. Useful as a starting point
    /// for building an initial state.
    pub fn unknown(time: Duration) -> Self {
        let inf = Vector3::repeat(f64::INFINITY);
        NavState {
            time,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            attitude: Quaternion::identity(),
            body_rate: Vector3::zeros(),
            position_std: inf,
            velocity_std: inf,
            attitude_std: inf,
            body_rate_std: inf,
        }
    }

    /// Converts this estimate into the controller's input. The uncertainty
    /// is dropped.
    pub fn to_control_state(self) -> ControlState {
        ControlState {
            time: self.time,
            position: self.position,
            velocity: self.velocity,
            body_rate: self.body_rate,
            attitude: self.attitude,
        }
    }
}

/// What an estimator did with a measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UpdateOutcome {
    /// The measurement was used. `nis` is its normalized innovation squared,
    /// which averages to the measurement's dimension for a consistent filter.
    Applied { nis: f64 },
    /// This estimator does not use this kind of measurement.
    NotUsed,
    /// The measurement was checked and thrown away.
    Rejected(Rejection),
}

/// Why a measurement was thrown away.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Rejection {
    /// The measurement time is too far from the estimator's current time.
    OutOfTimeWindow,
    /// The reading itself is invalid, for example out of the sensor's range
    /// or not finite.
    InvalidReading,
    /// The reading disagrees with the estimate by more than the gate allows.
    Outlier { nis: f64 },
}

/// Errors from using an estimator incorrectly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavError {
    /// [`Estimator::initialize`] has not been called since creation or the
    /// last [`Estimator::reset`].
    NotInitialized,
    /// An IMU sample was not newer than the estimator's current time.
    TimeNotIncreasing { current: Duration, sample: Duration },
    /// The initial state is missing a value or uncertainty the estimator
    /// needs.
    InvalidInitialState(&'static str),
}

impl fmt::Display for NavError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NavError::NotInitialized => write!(f, "estimator is not initialized"),
            NavError::TimeNotIncreasing { current, sample } => write!(
                f,
                "IMU sample at {sample:?} is not newer than the estimate at {current:?}"
            ),
            NavError::InvalidInitialState(reason) => write!(f, "invalid initial state: {reason}"),
        }
    }
}

impl Error for NavError {}

/// A navigation filter.
pub trait Estimator {
    /// Starts the filter from `initial`, for example the known state on the
    /// pad. Values and uncertainties of the components the filter estimates
    /// must be finite. Calling this again restarts the filter.
    fn initialize(&mut self, initial: &NavState) -> Result<(), NavError>;

    /// Forgets the current estimate. The filter must be initialized again
    /// before use.
    fn reset(&mut self);

    /// Propagates the estimate to the IMU sample's time using the sample.
    /// Samples must be strictly increasing in time.
    fn predict(&mut self, imu: &Timestamped<ImuSample>) -> Result<(), NavError>;

    /// Corrects the estimate with one measurement taken close to the
    /// current estimate time.
    fn update(&mut self, measurement: &Timestamped<Measurement>)
        -> Result<UpdateOutcome, NavError>;

    /// The current estimate, or `None` if the filter is not initialized.
    fn nav_state(&self) -> Option<NavState>;

    /// The covariance of the filter's internal state, for logging and
    /// debugging. Its size and ordering are specific to each filter. `None`
    /// if the filter is not initialized.
    fn covariance(&self) -> Option<DMatrix<f64>>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_sample_converts_degrees_to_radians() {
        let sample = ImuSample::from_deg_per_s(
            Vector3::new(9.8, 0.1, -0.2),
            Vector3::new(180.0, -90.0, 0.0),
        );
        assert_eq!(sample.accel, Vector3::new(9.8, 0.1, -0.2));
        assert!(
            (sample.gyro - Vector3::new(std::f64::consts::PI, -std::f64::consts::FRAC_PI_2, 0.0))
                .norm()
                < 1e-15
        );
    }

    #[test]
    fn nav_state_converts_to_control_state() {
        let mut state = NavState::unknown(Duration::from_millis(1500));
        state.position = Vector3::new(1.0, 2.0, -3.0);
        state.velocity = Vector3::new(0.1, 0.2, -0.3);
        state.body_rate = Vector3::new(0.01, 0.02, 0.03);
        state.attitude = Quaternion::new(0.5, 0.5, -0.5, 0.5);

        let control = state.to_control_state();
        assert_eq!(control.time, state.time);
        assert_eq!(control.position, state.position);
        assert_eq!(control.velocity, state.velocity);
        assert_eq!(control.body_rate, state.body_rate);
        assert_eq!(control.attitude, state.attitude);
    }

    #[test]
    fn unknown_state_has_infinite_uncertainty() {
        let state = NavState::unknown(Duration::ZERO);
        for std in [
            state.position_std,
            state.velocity_std,
            state.attitude_std,
            state.body_rate_std,
        ] {
            assert!(std.iter().all(|s| *s == f64::INFINITY));
        }
        assert_eq!(state.attitude, Quaternion::identity());
    }
}
