//! Pieces of the software-in-the-loop (SIL) run that need no hardware and no
//! plant: a stand-in for the navigation filter, and the scoring of a flight.
//!
//! The SIL bench (`sim_plant.rs`) flies the generated plant with the real
//! flight logic ([`crate::gnc::Gnc`]) and the real engine-controller link. The
//! plant gives truth, in the controller's own frame. Handing that straight to
//! the controller would skip the one thing the navigation estimate always
//! differs in: where its origin is, which way it thinks north is, and that it
//! is the IMU's position and not the vehicle reference point's. [`SilNav`]
//! turns truth into an estimate with those differences, so the frame
//! conversion ([`crate::nav_frame::NavFrame`]) is exercised in every run.
//!
//! It is not a navigation filter and has no sensor noise, delay or bias. The
//! sensors and the GMEKF are not in this loop.

use std::{str::FromStr, time::Duration};

use common::comm::{ctv::ActuatorDemand, ec_link::Outputs};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::{
    gnc::{Fault, Mode},
    nav_frame::tilt_deg,
    navigate::NavState,
};

/// The plant's truth state: position NED (3), velocity NED (3), body rates
/// (3), quaternion scalar first (4). The same as `hopper_plant::State`.
pub type State = [f64; 13];

/// What a landing has to meet. These are hopper's CTV requirements in
/// `ctv_inputs.m`, all of them tagged ASSUMPTION there.
pub const MAX_IMPACT_SPEED_MPS: f64 = 0.5;
pub const MAX_LANDING_OFFSET_M: f64 = 0.5;
/// Hopper's Monte Carlo bound on the touchdown tilt, degrees.
pub const MAX_TOUCHDOWN_TILT_DEG: f64 = 2.0;

/// Truth seen through a navigation filter's frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SilNav {
    /// The filter's heading relative to the schedule's, about the NED
    /// vertical.
    heading: UnitQuaternion<f64>,
    /// Where the filter's origin puts the pad, NED, m.
    origin: Vector3<f64>,
    /// The vehicle reference point relative to the IMU, body axes, m.
    lever_body: Vector3<f64>,
}

impl SilNav {
    pub fn new(heading_offset_deg: f64, origin: Vector3<f64>, lever_body: Vector3<f64>) -> Self {
        SilNav {
            heading: UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                heading_offset_deg.to_radians(),
            ),
            origin,
            lever_body,
        }
    }

    /// No offsets: the filter agrees with the schedule's frame.
    pub fn identity() -> Self {
        SilNav::new(0.0, Vector3::zeros(), Vector3::zeros())
    }

    /// The estimate for the truth state `x` at plant time `t`.
    pub fn estimate(&self, t: f64, x: &State) -> NavState {
        let truth_attitude =
            UnitQuaternion::from_quaternion(Quaternion::new(x[9], x[10], x[11], x[12]));
        let attitude = self.heading * truth_attitude;
        let reference_point = self.origin + self.heading * Vector3::new(x[0], x[1], x[2]);
        let body_rate = Vector3::new(x[6], x[7], x[8]);

        NavState {
            time: Duration::from_secs_f64(t.max(0.0)),
            // The estimate is the IMU's position and velocity: the truth's are the
            // reference point's, which moves with the rotation about the IMU too
            position: reference_point - attitude * self.lever_body,
            velocity: self.heading * Vector3::new(x[3], x[4], x[5])
                - attitude * body_rate.cross(&self.lever_body),
            attitude: *attitude.quaternion(),
            body_rate,
            position_std: Vector3::repeat(0.05),
            velocity_std: Vector3::repeat(0.05),
            attitude_std: Vector3::repeat(0.005),
            body_rate_std: Vector3::repeat(0.005),
        }
    }
}

/// How far the estimate is from the truth, accumulated over a window and
/// reported once a second: root-mean-square error of the body rate, the
/// velocity and the position, and the attitude's angle error.
/// How many truth samples back the lag search looks.
pub const LAG_SAMPLES: usize = 150;

#[derive(Debug)]
pub struct EstimateError {
    /// Squared rate error against the truth `k` samples ago.
    lag_error: [f64; LAG_SAMPLES],
    rate: [f64; 3],
    velocity: [f64; 3],
    position: [f64; 3],
    attitude: f64,
    att_sum: [f64; 3],
    att_sq: [f64; 3],
    truth_rate: [f64; 3],
    count: u32,
    last_report: Option<std::time::Instant>,
}

impl Default for EstimateError {
    fn default() -> Self {
        EstimateError {
            lag_error: [0.0; LAG_SAMPLES],
            rate: [0.0; 3],
            velocity: [0.0; 3],
            position: [0.0; 3],
            attitude: 0.0,
            att_sum: [0.0; 3],
            att_sq: [0.0; 3],
            truth_rate: [0.0; 3],
            count: 0,
            last_report: None,
        }
    }
}

impl EstimateError {
    /// Compares the estimate's rate with the truth rate `k` samples ago, for
    /// every `k` the history holds: the `k` with the least error is how late
    /// the estimate is, in IMU samples.
    pub fn add_lags(
        &mut self,
        rate: &Vector3<f64>,
        history: &std::collections::VecDeque<Vector3<f64>>,
    ) {
        let n = history.len();

        for k in 0..n.min(LAG_SAMPLES) {
            self.lag_error[k] += (rate - history[n - 1 - k]).norm_squared();
        }
    }

    pub fn add(&mut self, estimate: &NavState, truth: &NavState) {
        for i in 0..3 {
            self.rate[i] += (estimate.body_rate[i] - truth.body_rate[i]).powi(2);
            self.truth_rate[i] += truth.body_rate[i].powi(2);
            self.velocity[i] += (estimate.velocity[i] - truth.velocity[i]).powi(2);
            self.position[i] += (estimate.position[i] - truth.position[i]).powi(2);
        }

        let q = UnitQuaternion::from_quaternion(estimate.attitude);
        let t = UnitQuaternion::from_quaternion(truth.attitude);

        self.attitude += (q.inverse() * t).angle().powi(2);

        // the error rotation in the truth's body axes: about x is heading, y and z tilt
        let e = (t.inverse() * q).scaled_axis();

        for i in 0..3 {
            self.att_sum[i] += e[i];
            self.att_sq[i] += e[i] * e[i];
        }
        self.count += 1;
    }

    /// The line for the log, once a second.
    pub fn report(&mut self, now: std::time::Instant) -> Option<String> {
        let due = self
            .last_report
            .is_none_or(|at| now.saturating_duration_since(at) >= Duration::from_millis(250));

        if !due || self.count == 0 {
            return None;
        }

        let n = f64::from(self.count);
        let rms = |sum: [f64; 3]| sum.map(|s| (s / n).sqrt());
        let rate = rms(self.rate);
        let truth_rate = rms(self.truth_rate);
        let att_mean = self.att_sum.map(|v| (v / n).to_degrees());
        let att_dev = [0, 1, 2].map(|i| {
            ((self.att_sq[i] / n - (self.att_sum[i] / n).powi(2)).max(0.0))
                .sqrt()
                .to_degrees()
        });
        let velocity = rms(self.velocity);
        let position = rms(self.position);
        let best = self
            .lag_error
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(b.1))
            .map_or(0, |(k, _)| k);
        let line = format!(
            "GNC: estimate lags truth by {best} samples; error rms: rate [{:.4} {:.4} {:.4}] rad/s (truth rate rms [{:.3} {:.3} {:.3}]), attitude {:.3} deg (mean [{:.2} {:.2} {:.2}] deg, scatter [{:.3} {:.3} {:.3}] deg), velocity [{:.3} {:.3} {:.3}] m/s, position [{:.3} {:.3} {:.3}] m",
            rate[0],
            rate[1],
            rate[2],
            truth_rate[0],
            truth_rate[1],
            truth_rate[2],
            (self.attitude / n).sqrt().to_degrees(),
            att_mean[0],
            att_mean[1],
            att_mean[2],
            att_dev[0],
            att_dev[1],
            att_dev[2],
            velocity[0],
            velocity[1],
            velocity[2],
            position[0],
            position[1],
            position[2],
        );

        *self = EstimateError {
            last_report: Some(now),
            ..EstimateError::default()
        };

        Some(line)
    }
}

/// How long the EC takes to act on what the flight logic sends: the lag at
/// which the actuator values the EC reports applied best match the demands sent,
/// found by sliding one against the other. It is the round trip as the flight
/// computer sees it (demand out, applied back in the EC's status), a few tens of
/// milliseconds on the real link.
#[derive(Debug, Default)]
pub struct ApplyLag {
    /// Per control step: the demand and the applied value of the A1 extension
    /// (mm) and the throttle.
    history: Vec<[f64; 4]>,
}

impl ApplyLag {
    /// The most steps looked back.
    const MAX_LAG: usize = 20;
    /// Steps to gather before a report.
    pub const WINDOW: usize = 200;

    pub fn push(&mut self, demand: &ActuatorDemand, applied: &Outputs) {
        self.history.push([
            demand.tvc_extension_mm[0],
            f64::from(applied.tvc_extension_mm[0]),
            demand.main_throttle,
            f64::from(applied.main_throttle),
        ]);
    }

    /// The report line once there is a window of steps (and starts a new one),
    /// with `step_ms` the control period.
    pub fn report(&mut self, step_ms: f64) -> Option<String> {
        if self.history.len() < Self::WINDOW {
            return None;
        }

        let history = std::mem::take(&mut self.history);
        let best = |demand: usize, applied: usize| -> Option<(f64, f64)> {
            let mean = |k: usize, from: usize| {
                history[from..].iter().map(|r| r[k]).sum::<f64>() / (history.len() - from) as f64
            };
            let mut found: Option<(usize, f64)> = None;
            let mut spread = 0.0;

            for lag in 0..=Self::MAX_LAG {
                // the applied value `lag` steps after the demand
                let n = history.len() - Self::MAX_LAG;
                let (md, ma) = (mean(demand, 0), mean(applied, 0));
                let error: f64 = (0..n)
                    .map(|i| ((history[i][demand] - md) - (history[i + lag][applied] - ma)).powi(2))
                    .sum::<f64>()
                    / n as f64;

                spread = history
                    .iter()
                    .map(|r| (r[demand] - md).powi(2))
                    .sum::<f64>()
                    / history.len() as f64;

                if found.is_none_or(|(_, e)| error < e) {
                    found = Some((lag, error));
                }
            }

            // a demand that hardly moved says nothing about the lag
            found
                .filter(|_| spread > 1e-6)
                .map(|(lag, e)| (lag as f64 * step_ms, e.sqrt()))
        };
        let show = |v: Option<(f64, f64)>| {
            v.map_or("n/a".to_string(), |(lag, e)| {
                format!("{lag:.0} ms (residual {e:.3})")
            })
        };

        Some(format!(
            "GNC: EC applies the A1 demand {} after it is sent; the throttle {}",
            show(best(0, 1)),
            show(best(2, 3))
        ))
    }
}

/// A fault to inject into a SIL run, at a mission time in seconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SilFault {
    /// No estimate at all, from `at` for `duration` s.
    NavLoss { at: f64, duration: f64 },
    /// An estimate that is too old to use.
    NavStale { at: f64, duration: f64 },
    /// An estimate with a NaN position.
    NavNan { at: f64, duration: f64 },
    /// The attitude estimate is tipped by `deg` about north from `at` on, as a
    /// bad accelerometer would: the controller then tilts the vehicle for real.
    TiltBias { at: f64, deg: f64 },
    /// An operator abort to land.
    Abort { at: f64 },
    /// An operator terminate.
    Terminate { at: f64 },
}

impl FromStr for SilFault {
    type Err = String;

    /// `nav-loss:T[:DUR]`, `nav-stale:T[:DUR]`, `nav-nan:T[:DUR]`,
    /// `tilt-bias:T:DEG`, `abort:T`, `terminate:T`. A missing duration is
    /// forever.
    fn from_str(text: &str) -> Result<Self, String> {
        let mut parts = text.split(':');
        let kind = parts.next().unwrap_or_default();
        let number = |part: Option<&str>, what: &str| -> Result<Option<f64>, String> {
            part.map(|p| {
                p.trim()
                    .parse::<f64>()
                    .ok()
                    .filter(|v| v.is_finite())
                    .ok_or_else(|| format!("{what} {p:?} is not a finite number"))
            })
            .transpose()
        };
        let at = number(parts.next(), "time")?.ok_or("a fault needs a time, like nav-loss:8")?;
        let extra = number(parts.next(), "value")?;

        if parts.next().is_some() {
            return Err(format!("too many fields in {text:?}"));
        }

        let duration = extra.unwrap_or(f64::INFINITY);

        match kind {
            "nav-loss" => Ok(SilFault::NavLoss { at, duration }),
            "nav-stale" => Ok(SilFault::NavStale { at, duration }),
            "nav-nan" => Ok(SilFault::NavNan { at, duration }),
            "tilt-bias" => Ok(SilFault::TiltBias {
                at,
                deg: extra.ok_or("tilt-bias needs degrees, like tilt-bias:8:30")?,
            }),
            "abort" if extra.is_none() => Ok(SilFault::Abort { at }),
            "terminate" if extra.is_none() => Ok(SilFault::Terminate { at }),
            other => Err(format!(
                "unknown fault {other:?}; use nav-loss, nav-stale, nav-nan, tilt-bias, abort or terminate"
            )),
        }
    }
}

/// What the faults did to this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Manual {
    pub abort: bool,
    pub terminate: bool,
}

/// The faults of a run, applied to the nav input each tick.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FaultPlan {
    faults: Vec<SilFault>,
    fired: Vec<bool>,
}

impl FaultPlan {
    pub fn new(faults: Vec<SilFault>) -> Self {
        let fired = vec![false; faults.len()];

        FaultPlan { faults, fired }
    }

    pub fn is_empty(&self) -> bool {
        self.faults.is_empty()
    }

    /// Changes `nav` as the faults active at mission time `t` do, and returns
    /// the operator commands that fall due (each once).
    pub fn apply(&mut self, t: f64, nav: &mut Option<(NavState, Duration)>) -> Manual {
        let mut manual = Manual::default();
        let active = |at: f64, duration: f64| t >= at && t < at + duration;

        for (i, fault) in self.faults.iter().enumerate() {
            match *fault {
                SilFault::NavLoss { at, duration } if active(at, duration) => *nav = None,
                SilFault::NavStale { at, duration } if active(at, duration) => {
                    if let Some((_, age)) = nav {
                        *age = Duration::from_millis(500);
                    }
                }
                SilFault::NavNan { at, duration } if active(at, duration) => {
                    if let Some((estimate, _)) = nav {
                        estimate.position.x = f64::NAN;
                    }
                }
                SilFault::TiltBias { at, deg } if t >= at => {
                    if let Some((estimate, _)) = nav {
                        let tip =
                            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), deg.to_radians());
                        let tipped = tip * UnitQuaternion::from_quaternion(estimate.attitude);

                        estimate.attitude = *tipped.quaternion();
                    }
                }
                SilFault::Abort { at } if t >= at && !self.fired[i] => {
                    self.fired[i] = true;
                    manual.abort = true;
                }
                SilFault::Terminate { at } if t >= at && !self.fired[i] => {
                    self.fired[i] = true;
                    manual.terminate = true;
                }
                _ => {}
            }
        }

        manual
    }
}

/// How a run is expected to end.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Expect {
    /// A normal hop: landed within the requirements, no fault.
    #[default]
    Nominal,
    /// A fault made the vehicle abort to land, and it still landed within the
    /// requirements.
    AbortLand,
    /// The vehicle was terminated (R4), the EC safed. No landing requirements.
    Safed,
}

impl FromStr for Expect {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, String> {
        match text {
            "nominal" => Ok(Expect::Nominal),
            "abort-land" => Ok(Expect::AbortLand),
            "safed" => Ok(Expect::Safed),
            other => Err(format!(
                "unknown expectation {other:?}; use nominal, abort-land or safed"
            )),
        }
    }
}

/// What a run did, gathered tick by tick, scored at the end.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Tracker {
    pub ticks: u64,
    pub max_altitude_m: f64,
    pub max_tilt_deg: f64,
    pub max_horizontal_m: f64,
    /// The last truth state before the run ended.
    pub last: Option<State>,
    /// The truth state one tick before `last`: the impact velocity is the
    /// speed on the way in, not the 0 the plant stops at.
    pub previous: Option<State>,
    /// Altitude at which the flight logic cut the engine.
    pub cut_altitude_m: Option<f64>,
    pub faults: Vec<Fault>,
    pub final_mode: Option<Mode>,
    /// The plant's own touchdown stop fired.
    pub plant_touchdown: bool,
    /// Mission time of the end, s.
    pub end_time_s: f64,
}

impl Tracker {
    pub fn add(&mut self, x: &State, t: f64) {
        self.ticks += 1;
        self.max_altitude_m = self.max_altitude_m.max(-x[2]);
        self.max_horizontal_m = self.max_horizontal_m.max(x[0].hypot(x[1]));
        self.max_tilt_deg = self.max_tilt_deg.max(truth_tilt_deg(x));
        self.previous = self.last;
        self.last = Some(*x);
        self.end_time_s = t;
    }

    /// What the flight achieved, or why it did not count as a landing.
    pub fn outcome(&self) -> Outcome {
        let Some(last) = self.last else {
            return Outcome::default();
        };
        let speed = |x: &State| (x[3] * x[3] + x[4] * x[4] + x[5] * x[5]).sqrt();

        Outcome {
            landing_offset_m: Some(last[0].hypot(last[1])),
            // the faster of the last two samples: the plant freezes at touchdown
            impact_speed_mps: Some(
                self.previous
                    .map_or(speed(&last), |p| speed(&p).max(speed(&last))),
            ),
            touchdown_tilt_deg: Some(truth_tilt_deg(&last)),
            final_altitude_m: Some(-last[2]),
        }
    }

    /// Why this run does not pass, empty if it does. A pass is a landing that
    /// meets the requirements, with the engine cut by the flight logic at the
    /// pad and no fault latched.
    pub fn failures(&self) -> Vec<String> {
        self.failures_for(Expect::Nominal)
    }

    /// Like `failures`, against what the run was meant to do.
    pub fn failures_for(&self, expect: Expect) -> Vec<String> {
        let mut failures = Vec::new();
        let outcome = self.outcome();

        if self.last.is_none() {
            return vec!["the run never flew".to_string()];
        }

        if expect == Expect::Safed {
            if self.final_mode != Some(Mode::Safed) {
                failures.push(format!(
                    "the flight logic ended in {:?}, not Safed",
                    self.final_mode
                ));
            }

            if self.faults.is_empty() {
                failures.push("safed with no fault latched".to_string());
            }

            return failures;
        }

        let landed = self.final_mode == Some(Mode::Landed);

        if !landed {
            failures.push(format!(
                "the flight logic ended in {:?}, not Landed",
                self.final_mode
            ));
        }

        if landed && self.cut_altitude_m.is_none() {
            failures.push("landed without a recorded engine cut".to_string());
        }

        match expect {
            Expect::Nominal if !self.faults.is_empty() => {
                failures.push(format!("faults latched: {:?}", self.faults));
            }
            Expect::AbortLand if self.faults.is_empty() => {
                failures.push("expected a fault to abort to land, none latched".to_string());
            }
            _ => {}
        }

        if let Some(offset) = outcome.landing_offset_m {
            if offset > MAX_LANDING_OFFSET_M {
                failures.push(format!(
                    "landing offset {offset:.2} m exceeds {MAX_LANDING_OFFSET_M} m"
                ));
            }
        }

        if let Some(speed) = outcome.impact_speed_mps {
            if speed > MAX_IMPACT_SPEED_MPS {
                failures.push(format!(
                    "impact speed {speed:.2} m/s exceeds {MAX_IMPACT_SPEED_MPS} m/s"
                ));
            }
        }

        if let Some(tilt) = outcome.touchdown_tilt_deg {
            if tilt > MAX_TOUCHDOWN_TILT_DEG {
                failures.push(format!(
                    "touchdown tilt {tilt:.2} deg exceeds {MAX_TOUCHDOWN_TILT_DEG} deg"
                ));
            }
        }

        failures
    }

    /// Lines for the end of the run: one `key=value` line a script can parse,
    /// and the verdict.
    pub fn report(&self, expect: Expect) -> Vec<String> {
        let o = self.outcome();
        let f = |v: Option<f64>, digits: usize| {
            v.map_or("n/a".to_string(), |v| format!("{v:.digits$}"))
        };
        let failures = self.failures_for(expect);

        vec![
            format!(
                "SIL RESULT: mode={:?} time_s={:.2} apex_m={:.3} max_tilt_deg={:.2} max_horizontal_m={:.3} \
                 landing_offset_m={} impact_speed_mps={} touchdown_tilt_deg={} cut_altitude_m={} faults={:?}",
                self.final_mode,
                self.end_time_s,
                self.max_altitude_m,
                self.max_tilt_deg,
                self.max_horizontal_m,
                f(o.landing_offset_m, 3),
                f(o.impact_speed_mps, 3),
                f(o.touchdown_tilt_deg, 2),
                f(self.cut_altitude_m, 4),
                self.faults,
            ),
            if failures.is_empty() {
                format!("SIL VERDICT: PASS (expected {expect:?})")
            } else {
                format!("SIL VERDICT: FAIL (expected {expect:?}: {})", failures.join("; "))
            },
        ]
    }
}

/// How a landing went.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Outcome {
    pub landing_offset_m: Option<f64>,
    pub impact_speed_mps: Option<f64>,
    pub touchdown_tilt_deg: Option<f64>,
    pub final_altitude_m: Option<f64>,
}

/// The tilt of the body x axis from vertical for a truth state, degrees.
pub fn truth_tilt_deg(x: &State) -> f64 {
    tilt_deg(&UnitQuaternion::from_quaternion(Quaternion::new(
        x[9], x[10], x[11], x[12],
    )))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_apply_lag_is_how_late_the_applied_value_follows_the_demand() {
        let mut lag = ApplyLag::default();
        // a demand that wanders, and the EC applying it 3 steps (30 ms) late
        let demand = |k: usize| 5.0 + 3.0 * (k as f64 * 0.37).sin() + 2.0 * (k as f64 * 0.11).cos();

        for k in 0..ApplyLag::WINDOW {
            let applied_k = k.saturating_sub(3);
            let d = ActuatorDemand {
                tvc_extension_mm: [demand(k), 0.0],
                rcs_duty: [0.0; 4],
                main_throttle: 0.5,
            };
            let a = Outputs {
                tvc_extension_mm: [demand(applied_k) as f32, 0.0],
                rcs_duty: [0.0; 4],
                main_throttle: 0.5,
            };

            lag.push(&d, &a);
        }

        let line = lag.report(10.0).expect("a window of steps");

        assert!(line.contains("A1 demand 30 ms"), "{line}");
        // the throttle never moved: it says nothing
        assert!(line.contains("throttle n/a"), "{line}");
        // and a new window starts empty
        assert!(lag.report(10.0).is_none());
    }
    use crate::nav_frame::{vertical_attitude, NavFrame};

    fn upright(z: f64, vz: f64) -> State {
        let h = std::f64::consts::FRAC_1_SQRT_2;

        [0.0, 0.0, z, 0.0, 0.0, vz, 0.0, 0.0, 0.0, h, 0.0, h, 0.0]
    }

    #[test]
    fn the_vertical_truth_attitude_is_the_frames_vertical() {
        let q = vertical_attitude();
        let x = upright(0.0, 0.0);

        assert!((x[9] - q.w).abs() < 1e-12 && (x[11] - q.j).abs() < 1e-12);
        assert!(truth_tilt_deg(&x) < 1e-9);
    }

    /// The frame conversion undoes whatever the estimate's frame is.
    #[test]
    fn truth_through_the_estimate_and_the_pad_frame_comes_back_as_truth() {
        let nav = SilNav::new(
            137.0,
            Vector3::new(35.0, -12.0, -3.0),
            Vector3::new(-0.3, 0.02, -0.01),
        );
        let pad = nav.estimate(0.0, &upright(0.0, 0.0));
        let frame = NavFrame::tare(&pad, Vector3::new(-0.3, 0.02, -0.01)).unwrap();

        // 1.5 m up, 0.2 m east, climbing, tilted 4 deg about north
        let tilt = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 4f64.to_radians())
            * vertical_attitude();
        let q = tilt.quaternion();
        let truth: State = [
            0.0, 0.2, -1.5, 0.0, 0.01, -0.8, 0.01, -0.02, 0.03, q.w, q.i, q.j, q.k,
        ];
        let seen = frame.control_state(&nav.estimate(2.5, &truth));

        assert!(
            (seen.position - Vector3::new(0.0, 0.2, -1.5)).norm() < 1e-9,
            "{seen:?}"
        );
        assert!(
            (seen.velocity - Vector3::new(0.0, 0.01, -0.8)).norm() < 1e-9,
            "{seen:?}"
        );
        assert_eq!(seen.body_rate, Vector3::new(0.01, -0.02, 0.03));

        let dot = seen.attitude.coords.dot(&q.coords);

        assert!((dot.abs() - 1.0).abs() < 1e-9, "{dot}");
    }

    #[test]
    fn the_estimate_really_is_in_another_frame() {
        let nav = SilNav::new(90.0, Vector3::new(10.0, 0.0, 0.0), Vector3::zeros());
        let e = nav.estimate(0.0, &upright(-1.0, 0.0));

        // 90 deg turns the schedule's north into the filter's east, and the
        // origin adds 10 m north
        assert!((e.position - Vector3::new(10.0, 0.0, -1.0)).norm() < 1e-9);

        let h = std::f64::consts::FRAC_1_SQRT_2;
        let moved = nav.estimate(
            0.0,
            &[2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, h, 0.0, h, 0.0],
        );

        assert!(
            (moved.position - Vector3::new(10.0, 2.0, 0.0)).norm() < 1e-9,
            "{:?}",
            moved.position
        );
    }

    fn landed_tracker() -> Tracker {
        let mut t = Tracker::default();

        t.add(&upright(-0.05, 0.3), 9.9);

        let mut last = upright(-0.004, 0.2);

        last[0] = 0.1;
        t.add(&last, 10.0);

        t.final_mode = Some(Mode::Landed);
        t.cut_altitude_m = Some(0.004);

        t
    }

    #[test]
    fn a_soft_landing_on_the_pad_passes() {
        let t = landed_tracker();

        assert_eq!(t.failures(), Vec::<String>::new());
        assert!(t.report(Expect::Nominal).last().unwrap().contains("PASS"));
    }

    #[test]
    fn the_impact_speed_is_the_speed_before_the_plant_stops() {
        let mut t = Tracker::default();

        t.add(&upright(-0.01, 0.9), 9.9);
        // the plant freezes at touchdown with zero velocity
        t.add(&upright(0.0, 0.0), 10.0);
        t.final_mode = Some(Mode::Landed);
        t.cut_altitude_m = Some(0.004);

        assert!(t.outcome().impact_speed_mps.unwrap() > 0.85);
        assert!(t.failures().iter().any(|f| f.contains("impact speed")));
    }

    #[test]
    fn each_requirement_can_fail_alone() {
        let mut far = landed_tracker();

        far.last.as_mut().unwrap()[0] = 2.0;

        assert!(far.failures().iter().any(|f| f.contains("landing offset")));

        let mut tipped = landed_tracker();
        let q = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 5f64.to_radians())
            * vertical_attitude();

        tipped.last.as_mut().unwrap()[9..13].copy_from_slice(&[q.w, q.i, q.j, q.k]);

        assert!(tipped.failures().iter().any(|f| f.contains("tilt")));

        let mut faulted = landed_tracker();

        faulted.faults.push(Fault::Tilt);

        assert!(faulted.failures().iter().any(|f| f.contains("faults")));

        let mut not_landed = landed_tracker();

        not_landed.final_mode = Some(Mode::Safed);

        assert!(not_landed
            .failures()
            .iter()
            .any(|f| f.contains("not Landed")));

        let mut no_cut = landed_tracker();

        no_cut.cut_altitude_m = None;

        assert!(no_cut.failures().iter().any(|f| f.contains("engine cut")));
    }

    #[test]
    fn faults_parse() {
        assert_eq!(
            "nav-loss:8".parse(),
            Ok(SilFault::NavLoss {
                at: 8.0,
                duration: f64::INFINITY
            })
        );
        assert_eq!(
            "nav-stale:3:0.5".parse(),
            Ok(SilFault::NavStale {
                at: 3.0,
                duration: 0.5
            })
        );
        assert_eq!(
            "tilt-bias:8:30".parse(),
            Ok(SilFault::TiltBias { at: 8.0, deg: 30.0 })
        );
        assert_eq!("abort:4".parse(), Ok(SilFault::Abort { at: 4.0 }));
        assert_eq!("terminate:4.5".parse(), Ok(SilFault::Terminate { at: 4.5 }));

        for bad in [
            "nav-loss",
            "nav-loss:x",
            "tilt-bias:8",
            "abort:4:1",
            "boom:1",
            "nav-loss:1:2:3",
            "nav-nan:NaN",
        ] {
            assert!(bad.parse::<SilFault>().is_err(), "{bad}");
        }
    }

    fn some_nav() -> Option<(NavState, Duration)> {
        Some((
            SilNav::identity().estimate(1.0, &upright(0.0, 0.0)),
            Duration::ZERO,
        ))
    }

    #[test]
    fn faults_change_the_nav_input_only_while_active() {
        let mut plan = FaultPlan::new(vec![
            "nav-loss:2:1".parse().unwrap(),
            "nav-stale:5:1".parse().unwrap(),
            "nav-nan:7:1".parse().unwrap(),
        ]);

        let mut nav = some_nav();
        plan.apply(1.9, &mut nav);
        assert!(nav.is_some());

        plan.apply(2.5, &mut nav);
        assert!(nav.is_none());

        let mut nav = some_nav();
        plan.apply(3.1, &mut nav);
        assert!(nav.is_some(), "the loss ended");

        plan.apply(5.5, &mut nav);
        assert!(nav.unwrap().1 > Duration::from_millis(100));

        let mut nav = some_nav();
        plan.apply(7.5, &mut nav);
        assert!(nav.unwrap().0.position.x.is_nan());
    }

    #[test]
    fn a_tilt_bias_tips_the_attitude_estimate() {
        let mut plan = FaultPlan::new(vec!["tilt-bias:2:30".parse().unwrap()]);
        let mut nav = some_nav();

        plan.apply(1.0, &mut nav);
        let before = tilt_deg(&UnitQuaternion::from_quaternion(nav.unwrap().0.attitude));

        plan.apply(2.0, &mut nav);
        let after = tilt_deg(&UnitQuaternion::from_quaternion(nav.unwrap().0.attitude));

        assert!(
            before < 1e-6 && (after - 30.0).abs() < 1e-6,
            "{before} {after}"
        );
    }

    #[test]
    fn operator_commands_fire_once() {
        let mut plan = FaultPlan::new(vec![
            "abort:2".parse().unwrap(),
            "terminate:4".parse().unwrap(),
        ]);
        let mut nav = some_nav();

        assert_eq!(plan.apply(1.0, &mut nav), Manual::default());
        assert_eq!(
            plan.apply(2.0, &mut nav),
            Manual {
                abort: true,
                terminate: false
            }
        );
        assert_eq!(plan.apply(2.01, &mut nav), Manual::default());
        assert_eq!(
            plan.apply(4.0, &mut nav),
            Manual {
                abort: false,
                terminate: true
            }
        );
        assert_eq!(plan.apply(4.01, &mut nav), Manual::default());
    }

    #[test]
    fn expectations_score_the_other_outcomes() {
        // an abort to land that landed softly
        let mut aborted = landed_tracker();

        aborted.faults.push(Fault::Tilt);

        assert!(aborted.failures_for(Expect::AbortLand).is_empty());
        assert!(!aborted.failures_for(Expect::Nominal).is_empty());
        assert!(!landed_tracker().failures_for(Expect::AbortLand).is_empty());

        // a terminate
        let mut safed = landed_tracker();

        safed.final_mode = Some(Mode::Safed);
        safed.faults.push(Fault::ManualTerminate);
        safed.last.as_mut().unwrap()[0] = 40.0; // no landing requirements apply

        assert!(safed.failures_for(Expect::Safed).is_empty());
        assert!(!landed_tracker().failures_for(Expect::Safed).is_empty());
    }

    #[test]
    fn a_run_that_never_flew_fails() {
        assert_eq!(
            Tracker::default().failures(),
            vec!["the run never flew".to_string()]
        );
    }

    #[test]
    fn the_report_is_one_parseable_line() {
        let line = &landed_tracker().report(Expect::Nominal)[0];

        assert!(line.starts_with("SIL RESULT: "));
        assert!(line.contains("landing_offset_m=0.100"), "{line}");
        assert!(!line.contains('\n'));
    }
}
