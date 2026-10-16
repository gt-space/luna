//! Shared test scenarios and checks for any [`Estimator`].
//!
//! A [`Scenario`] simulates the vehicle's true motion and the sensor readings
//! it produces. The `check_*` functions run an estimator through a scenario
//! and assert on the result. They only look at the components the estimator
//! reports a finite uncertainty for, so the same checks apply to the basic
//! vertical filter and to a full navigation filter. Each filter's own tests
//! call them with the scenarios and tolerances that fit it.
//!
//! The scenarios are vertical only for now: the vehicle stays upright and
//! does not rotate. Add horizontal motion and attitude scenarios, with
//! attitude errors in the checks, once a filter estimates them.

use std::time::Duration;

use nalgebra::{UnitQuaternion, Vector3};

use super::{Estimator, ImuSample, Measurement, NavError, NavState, Timestamped, UpdateOutcome};

/// Standard gravity, m/s^2.
const G0: f64 = 9.80665;
/// Specific gas constant of dry air, J/(kg K).
const R_AIR: f64 = 287.05;
/// Temperature lapse rate of the standard troposphere, K/m.
const LAPSE_RATE: f64 = 0.0065;
/// Pad pressure, Pa, and temperature, K. Matches the vertical filter's
/// default barometer reference.
const PAD_PRESSURE: f64 = 101_325.0;
const PAD_TEMPERATURE: f64 = 288.15;

/// Small deterministic random number generator (xorshift64* with a
/// Box-Muller transform), so every run of a test sees the same noise.
#[derive(Debug, Clone)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform on the open interval (0, 1).
    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal sample.
    pub fn normal(&mut self) -> f64 {
        let (u1, u2) = (self.uniform(), self.uniform());
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }

    fn normal3(&mut self, std: f64) -> Vector3<f64> {
        Vector3::new(self.normal(), self.normal(), self.normal()) * std
    }
}

/// White noise added to each simulated sensor reading (1-sigma per sample).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorNoise {
    /// Accelerometer, m/s^2.
    pub accel_std: f64,
    /// Gyroscope, rad/s.
    pub gyro_std: f64,
    /// Barometer, Pa.
    pub baro_std: f64,
    /// LiDAR range, m.
    pub lidar_std: f64,
    /// GPS position, each axis, m.
    pub gps_position_std: f64,
    /// GPS velocity, each axis, m/s.
    pub gps_velocity_std: f64,
}

impl SensorNoise {
    /// Perfect sensors.
    pub fn none() -> Self {
        SensorNoise {
            accel_std: 0.0,
            gyro_std: 0.0,
            baro_std: 0.0,
            lidar_std: 0.0,
            gps_position_std: 0.0,
            gps_velocity_std: 0.0,
        }
    }

    /// Roughly the white noise of the hopper's sensors at their sample rates
    /// (IMU at 500 Hz, standalone GPS).
    pub fn typical() -> Self {
        SensorNoise {
            accel_std: 0.02,
            gyro_std: 0.1_f64.to_radians(),
            baro_std: 1.2,
            lidar_std: 0.03,
            gps_position_std: 1.0,
            gps_velocity_std: 0.05,
        }
    }

    /// Barometer noise expressed as altitude at the pad, m.
    pub fn baro_altitude_std(&self) -> f64 {
        let density = PAD_PRESSURE / (R_AIR * PAD_TEMPERATURE);
        self.baro_std / (density * G0)
    }
}

/// True vertical motion of the IMU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Profile {
    /// Sitting still at `height` m above the ground.
    Stationary { height: f64 },
    /// Starts at rest at `start_height`, climbs smoothly by `apex` m and
    /// comes back down to rest over `duration` s, then stays there.
    Hop {
        start_height: f64,
        apex: f64,
        duration: f64,
    },
}

impl Profile {
    /// Height (m) and upward velocity (m/s) at time `t` s.
    fn at(&self, t: f64) -> (f64, f64) {
        match *self {
            Profile::Stationary { height } => (height, 0.0),
            Profile::Hop {
                start_height,
                apex,
                duration,
            } => {
                if t >= duration {
                    return (start_height, 0.0);
                }
                let w = 2.0 * std::f64::consts::PI / duration;
                let height = start_height + 0.5 * apex * (1.0 - (w * t).cos());
                let velocity = 0.5 * apex * w * (w * t).sin();
                (height, velocity)
            }
        }
    }
}

/// True state at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Truth {
    pub time: Duration,
    /// NED position, m.
    pub position: Vector3<f64>,
    /// NED velocity, m/s.
    pub velocity: Vector3<f64>,
}

/// A simulated flight: true motion, sensor rates and layout, and noise.
#[derive(Debug, Clone, PartialEq)]
pub struct Scenario {
    pub profile: Profile,
    /// Length of the run, s.
    pub duration: f64,
    /// Estimation loop period; one IMU sample per step.
    pub imu_period: Duration,
    pub baro_period: Duration,
    pub lidar_period: Duration,
    pub gps_period: Duration,
    /// Height of each LiDAR above the IMU, m (negative when below).
    pub lidar_offsets: Vec<f64>,
    /// Valid LiDAR range, m. Outside it the sensor reports -1.
    pub lidar_min_range: f64,
    pub lidar_max_range: f64,
    pub noise: SensorNoise,
    pub seed: u64,
}

impl Scenario {
    fn new(profile: Profile, duration: f64) -> Self {
        Scenario {
            profile,
            duration,
            // 500 Hz, the target estimation rate.
            imu_period: Duration::from_millis(2),
            baro_period: Duration::from_millis(10),
            lidar_period: Duration::from_millis(10),
            gps_period: Duration::from_millis(200),
            lidar_offsets: vec![-1.14; 4],
            lidar_min_range: 0.2,
            lidar_max_range: 8.0,
            noise: SensorNoise::none(),
            seed: 1,
        }
    }

    /// Vehicle standing still with the IMU `height` m above the ground, for
    /// `duration` s, with perfect sensors.
    pub fn stationary(height: f64, duration: f64) -> Self {
        Scenario::new(Profile::Stationary { height }, duration)
    }

    /// A hop of `apex` m lasting `hop_duration` s from an IMU height of
    /// `start_height` m, followed by 2 s at rest, with perfect sensors.
    pub fn hop(start_height: f64, apex: f64, hop_duration: f64) -> Self {
        let profile = Profile::Hop {
            start_height,
            apex,
            duration: hop_duration,
        };
        Scenario::new(profile, hop_duration + 2.0)
    }

    pub fn with_noise(mut self, noise: SensorNoise) -> Self {
        self.noise = noise;
        self
    }

    pub fn with_seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    /// True state at `time`.
    pub fn truth(&self, time: Duration) -> Truth {
        let (height, velocity) = self.profile.at(time.as_secs_f64());
        Truth {
            time,
            position: Vector3::new(0.0, 0.0, -height),
            velocity: Vector3::new(0.0, 0.0, -velocity),
        }
    }

    /// The true state at the start, with the given uncertainties on position
    /// and velocity, 0.01 rad on attitude and 0.01 rad/s on body rate.
    /// The attitude is upright (90 degree pitch, as in the 6DOF model).
    pub fn initial_state(&self, position_std: f64, velocity_std: f64) -> NavState {
        let truth = self.truth(Duration::ZERO);
        let mut state = NavState::unknown(Duration::ZERO);
        state.position = truth.position;
        state.velocity = truth.velocity;
        state.attitude =
            UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0).into_inner();
        state.position_std = Vector3::repeat(position_std);
        state.velocity_std = Vector3::repeat(velocity_std);
        state.attitude_std = Vector3::repeat(0.01);
        state.body_rate_std = Vector3::repeat(0.01);
        state
    }

    fn steps(&self, period: Duration) -> u64 {
        let steps = period.as_nanos() / self.imu_period.as_nanos();
        assert!(
            steps > 0 && period.as_nanos().is_multiple_of(self.imu_period.as_nanos()),
            "sensor periods must be multiples of the IMU period"
        );
        steps as u64
    }

    /// Runs `estimator` from `initial` through the scenario. After every IMU
    /// step and the measurements due at that time, calls `visit` with the
    /// truth and the estimate.
    ///
    /// Panics if the estimator returns an error.
    pub fn run<E: Estimator + ?Sized>(
        &self,
        estimator: &mut E,
        initial: &NavState,
        mut visit: impl FnMut(&Truth, &NavState),
    ) -> RunStats {
        estimator
            .initialize(initial)
            .expect("estimator rejected the initial state");

        let mut rng = Rng::new(self.seed);
        let mut stats = RunStats::default();
        let noise = &self.noise;
        let (baro_every, lidar_every, gps_every) = (
            self.steps(self.baro_period),
            self.steps(self.lidar_period),
            self.steps(self.gps_period),
        );
        let dt = self.imu_period.as_secs_f64();
        let total = (self.duration / dt).round() as u64;
        let mut previous_velocity_up = self.profile.at(0.0).1;

        for k in 1..=total {
            let time = self.imu_period * k as u32;
            let (height, velocity_up) = self.profile.at(time.as_secs_f64());
            // Like a real IMU, report the average acceleration over the
            // sample interval, so integrating it gives the true velocity.
            let accel_up = (velocity_up - previous_velocity_up) / dt;
            previous_velocity_up = velocity_up;

            // Upright vehicle: body x is up, so the accelerometer reads the
            // upward specific force along x.
            let imu = ImuSample {
                accel: Vector3::new(accel_up + G0, 0.0, 0.0) + rng.normal3(noise.accel_std),
                gyro: rng.normal3(noise.gyro_std),
            };
            estimator
                .predict(&Timestamped::new(time, imu))
                .expect("predict failed");

            let mut measurements = Vec::new();
            if k % baro_every == 0 {
                let pressure = PAD_PRESSURE
                    * (1.0 - LAPSE_RATE * height / PAD_TEMPERATURE).powf(G0 / (LAPSE_RATE * R_AIR))
                    + noise.baro_std * rng.normal();
                measurements.push(Measurement::Barometer {
                    pressure,
                    temperature: PAD_TEMPERATURE - LAPSE_RATE * height - 273.15,
                });
            }
            if k % lidar_every == 0 {
                for (index, offset) in self.lidar_offsets.iter().enumerate() {
                    let distance = height + offset;
                    let range = if (self.lidar_min_range..=self.lidar_max_range).contains(&distance)
                    {
                        distance + noise.lidar_std * rng.normal()
                    } else {
                        -1.0
                    };
                    measurements.push(Measurement::Lidar { index, range });
                }
            }
            if k % gps_every == 0 {
                measurements.push(Measurement::GpsPosition {
                    position: Vector3::new(0.0, 0.0, -height) + rng.normal3(noise.gps_position_std),
                });
                measurements.push(Measurement::GpsVelocity {
                    velocity: Vector3::new(0.0, 0.0, -velocity_up)
                        + rng.normal3(noise.gps_velocity_std),
                });
            }

            for measurement in measurements {
                let outcome = estimator
                    .update(&Timestamped::new(time, measurement))
                    .expect("update failed");
                stats.record(outcome);
            }

            let estimate = estimator.nav_state().expect("estimator lost its state");
            visit(&self.truth(time), &estimate);
        }
        stats
    }
}

/// What happened to the measurements in a run.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RunStats {
    pub applied: usize,
    pub not_used: usize,
    pub rejected: usize,
    /// Sum of the normalized innovation squared of applied measurements.
    pub nis_sum: f64,
}

impl RunStats {
    fn record(&mut self, outcome: UpdateOutcome) {
        match outcome {
            UpdateOutcome::Applied { nis } => {
                self.applied += 1;
                self.nis_sum += nis;
            }
            UpdateOutcome::NotUsed => self.not_used += 1,
            UpdateOutcome::Rejected(_) => self.rejected += 1,
        }
    }

    /// Mean normalized innovation squared of the applied measurements.
    pub fn mean_nis(&self) -> f64 {
        self.nis_sum / self.applied as f64
    }
}

/// One estimated component's error against the truth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComponentError {
    pub name: &'static str,
    pub error: f64,
    pub std: f64,
}

/// Errors of the estimated position and velocity components (those with a
/// finite uncertainty).
pub fn estimated_errors(truth: &Truth, estimate: &NavState) -> Vec<ComponentError> {
    const NAMES: [&str; 6] = ["north", "east", "down", "v_north", "v_east", "v_down"];
    (0..6)
        .map(|i| {
            let (value, true_value, std) = if i < 3 {
                (
                    estimate.position[i],
                    truth.position[i],
                    estimate.position_std[i],
                )
            } else {
                (
                    estimate.velocity[i - 3],
                    truth.velocity[i - 3],
                    estimate.velocity_std[i - 3],
                )
            };
            ComponentError {
                name: NAMES[i],
                error: value - true_value,
                std,
            }
        })
        .filter(|c| c.std.is_finite())
        .collect()
}

/// Checks the rules every estimator must follow for time and
/// initialization.
pub fn check_time_rules<E: Estimator + ?Sized>(estimator: &mut E, scenario: &Scenario) {
    let imu = |seconds: f64| {
        Timestamped::new(
            Duration::from_secs_f64(seconds),
            ImuSample {
                accel: Vector3::new(G0, 0.0, 0.0),
                gyro: Vector3::zeros(),
            },
        )
    };
    let baro = |seconds: f64| {
        Timestamped::new(
            Duration::from_secs_f64(seconds),
            Measurement::Barometer {
                pressure: PAD_PRESSURE,
                temperature: 15.0,
            },
        )
    };

    estimator.reset();
    assert!(
        estimator.nav_state().is_none(),
        "state available before initialize"
    );
    assert!(
        estimator.covariance().is_none(),
        "covariance available before initialize"
    );
    assert_eq!(estimator.predict(&imu(1.0)), Err(NavError::NotInitialized));
    assert_eq!(estimator.update(&baro(1.0)), Err(NavError::NotInitialized));

    let mut initial = scenario.initial_state(1.0, 0.5);
    initial.time = Duration::from_secs(20);
    estimator.initialize(&initial).unwrap();

    for seconds in [20.0, 19.5] {
        assert!(
            matches!(
                estimator.predict(&imu(seconds)),
                Err(NavError::TimeNotIncreasing { .. })
            ),
            "accepted an IMU sample at {seconds} s after an estimate at 20 s"
        );
    }
    estimator.predict(&imu(20.001)).unwrap();
    assert_eq!(
        estimator.nav_state().unwrap().time,
        Duration::from_secs_f64(20.001)
    );

    let before = estimator.nav_state();
    for seconds in [10.0, 30.0] {
        let outcome = estimator.update(&baro(seconds)).unwrap();
        assert!(
            matches!(
                outcome,
                UpdateOutcome::Rejected(super::Rejection::OutOfTimeWindow) | UpdateOutcome::NotUsed
            ),
            "measurement at {seconds} s used by an estimate at 20 s: {outcome:?}"
        );
    }
    assert_eq!(
        estimator.nav_state(),
        before,
        "out-of-window measurement changed the state"
    );

    estimator.reset();
    assert!(
        estimator.nav_state().is_none(),
        "state available after reset"
    );
}

/// Checks that the estimate right after initialization is the initial
/// state for every component the estimator estimates.
pub fn check_initialization<E: Estimator + ?Sized>(estimator: &mut E, scenario: &Scenario) {
    let initial = scenario.initial_state(1.0, 0.5);
    estimator.initialize(&initial).unwrap();
    let state = estimator.nav_state().unwrap();
    assert_eq!(state.time, initial.time);

    let truth = scenario.truth(initial.time);
    let errors = estimated_errors(&truth, &state);
    assert!(
        !errors.is_empty(),
        "estimator estimates no position or velocity component"
    );
    for c in &errors {
        assert!(
            c.error.abs() < 1e-12,
            "{} starts {} away from the initial state",
            c.name,
            c.error
        );
    }
    for i in 0..3 {
        for (std, initial_std) in [
            (state.position_std[i], initial.position_std[i]),
            (state.velocity_std[i], initial.velocity_std[i]),
        ] {
            assert!(
                std.is_infinite() || (std - initial_std).abs() < 1e-9,
                "initial uncertainty {std} does not match {initial_std}"
            );
        }
    }
}

/// Runs a stationary scenario from the exact initial state. Every estimated
/// component must stay within 4 sigma of the truth after the first second
/// and end within `position_tolerance` (m) and `velocity_tolerance` (m/s).
pub fn check_stationary<E: Estimator + ?Sized>(
    estimator: &mut E,
    scenario: &Scenario,
    position_tolerance: f64,
    velocity_tolerance: f64,
) {
    assert!(matches!(scenario.profile, Profile::Stationary { .. }));
    let initial = scenario.initial_state(0.5, 0.2);
    let mut last = Vec::new();
    scenario.run(estimator, &initial, |truth, estimate| {
        let errors = estimated_errors(truth, estimate);
        if truth.time > Duration::from_secs(1) {
            for c in &errors {
                assert!(
                    c.error.abs() <= 4.0 * c.std,
                    "{} error {} exceeds 4 sigma ({}) at {:?}",
                    c.name,
                    c.error,
                    c.std,
                    truth.time
                );
            }
        }
        last = errors;
    });

    for c in last {
        let tolerance = if c.name.starts_with("v_") {
            velocity_tolerance
        } else {
            position_tolerance
        };
        assert!(
            c.error.abs() < tolerance,
            "final {} error {} exceeds {tolerance}",
            c.name,
            c.error
        );
    }
}

/// Starts the estimator `position_offset` m and `velocity_offset` m/s away
/// from the truth on every axis (with 2x that as the initial uncertainty)
/// and runs the scenario. Returns the largest final position error of the
/// estimated components, which must be smaller than the initial offset.
pub fn check_convergence<E: Estimator + ?Sized>(
    estimator: &mut E,
    scenario: &Scenario,
    position_offset: f64,
    velocity_offset: f64,
) -> f64 {
    let mut initial = scenario.initial_state(2.0 * position_offset, 2.0 * velocity_offset);
    initial.position += Vector3::repeat(position_offset);
    initial.velocity += Vector3::repeat(velocity_offset);

    let mut final_error = f64::NAN;
    scenario.run(estimator, &initial, |truth, estimate| {
        final_error = estimated_errors(truth, estimate)
            .iter()
            .filter(|c| !c.name.starts_with("v_"))
            .map(|c| c.error.abs())
            .fold(0.0, f64::max);
    });
    assert!(
        final_error < position_offset,
        "position error grew from {position_offset} m to {final_error} m"
    );
    final_error
}

/// RMS errors over a run, m and m/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RmsError {
    pub position: f64,
    pub velocity: f64,
}

/// Runs the scenario from the exact initial state. At least 99 % of the
/// samples must have every estimated component within `n_sigma` of the
/// truth. Returns the RMS position and velocity errors of the estimated
/// components.
pub fn check_tracking<E: Estimator + ?Sized>(
    estimator: &mut E,
    scenario: &Scenario,
    n_sigma: f64,
) -> RmsError {
    let initial = scenario.initial_state(0.1, 0.05);
    let (mut samples, mut outside) = (0usize, 0usize);
    let (mut position_sq, mut position_n, mut velocity_sq, mut velocity_n) = (0.0, 0, 0.0, 0);

    scenario.run(estimator, &initial, |truth, estimate| {
        let errors = estimated_errors(truth, estimate);
        samples += 1;
        if errors.iter().any(|c| c.error.abs() > n_sigma * c.std) {
            outside += 1;
        }
        for c in errors {
            if c.name.starts_with("v_") {
                velocity_sq += c.error * c.error;
                velocity_n += 1;
            } else {
                position_sq += c.error * c.error;
                position_n += 1;
            }
        }
    });

    let fraction_outside = outside as f64 / samples as f64;
    assert!(
        fraction_outside <= 0.01,
        "{:.2} % of samples outside {n_sigma} sigma",
        100.0 * fraction_outside
    );
    RmsError {
        position: (position_sq / position_n.max(1) as f64).sqrt(),
        velocity: (velocity_sq / velocity_n.max(1) as f64).sqrt(),
    }
}

/// Result of a consistency check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Consistency {
    /// Average of error^2 / std^2 over the estimated position components,
    /// all samples and all runs. 1 for a consistent filter.
    pub position_nees: f64,
    /// Same for the velocity components.
    pub velocity_nees: f64,
    /// Mean normalized innovation squared of all applied measurements.
    pub mean_nis: f64,
}

/// Monte Carlo consistency check: runs the scenario `runs` times with
/// different noise and a random initial error drawn from the initial
/// uncertainty. The filter's reported uncertainty must match its actual
/// errors: the average normalized error squared of each estimated position
/// and velocity component must lie between 0.5 and 1.5 (1 when consistent).
/// The scenario's noise should match what the filter is tuned for.
pub fn check_consistency<E: Estimator + ?Sized>(
    estimator: &mut E,
    scenario: &Scenario,
    runs: u64,
) -> Consistency {
    let (mut position_sum, mut position_n, mut velocity_sum, mut velocity_n) = (0.0, 0, 0.0, 0);
    let (mut nis_sum, mut applied) = (0.0, 0);

    for run in 0..runs {
        let seeded = scenario
            .clone()
            .with_seed(scenario.seed.wrapping_add(1000 * run));
        let mut rng = Rng::new(seeded.seed ^ 0xA5A5);
        let mut initial = seeded.initial_state(1.0, 0.5);
        initial.position += rng.normal3(1.0);
        initial.velocity += rng.normal3(0.5);

        let stats = seeded.run(estimator, &initial, |truth, estimate| {
            for c in estimated_errors(truth, estimate) {
                let normalized = (c.error / c.std).powi(2);
                if c.name.starts_with("v_") {
                    velocity_sum += normalized;
                    velocity_n += 1;
                } else {
                    position_sum += normalized;
                    position_n += 1;
                }
            }
        });
        nis_sum += stats.nis_sum;
        applied += stats.applied;
    }

    let result = Consistency {
        position_nees: position_sum / position_n.max(1) as f64,
        velocity_nees: velocity_sum / velocity_n.max(1) as f64,
        mean_nis: nis_sum / applied.max(1) as f64,
    };
    for (name, nees) in [
        ("position", result.position_nees),
        ("velocity", result.velocity_nees),
    ] {
        assert!(
            (0.5..=1.5).contains(&nees),
            "{name} normalized error squared averages {nees}, expected about 1 ({result:?})"
        );
    }
    result
}
