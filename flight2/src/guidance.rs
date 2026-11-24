//! The flight computer's side of [`Gnc`]: when it runs, what starts it, and
//! what its output becomes.
//!
//! `Gnc` is the flight logic and knows nothing of the main loop. `Guidance`
//! is the part that does:
//!
//! - **A fixed period.** The main loop runs as fast as it can; the controller
//!   is stepped every [`PERIOD`] on a schedule that does not drift, and a
//!   period that was missed is counted and not made up for (stepping twice in
//!   a row would feed the controller one estimate twice).
//! - **Who owns the engine controller.** While the vehicle is flying, only
//!   `Guidance` commands the EC. A sequence's `Ctv.control` or `Ctv.actuators`
//!   would otherwise fight the controller for it, so the main loop drops them
//!   while [`Guidance::owns_ec`] is true. Arm, disarm and terminate still pass.
//! - **Start and abort.** A sequence's `Ctv.start_flight` arms the flight
//!   logic from the current estimate and starts the mission clock. A
//!   terminate always takes effect; an abort in flight lands.
//! - **The allocation.** The controller's command becomes actuator demands
//!   through the vehicle's measured [`Allocation`]. There is no default one:
//!   without a vehicle config the flight logic is not started.
//!
//! It is plain logic like `Gnc`, so it is tested without hardware.

use std::{
    fmt,
    path::PathBuf,
    time::{Duration, Instant},
};

use common::comm::ctv::{ActuatorDemand, CtvCheck, CtvMode, CtvNav, CtvSample, CtvStatus};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    bench_config::BenchConfig,
    ctv_allocation::Allocation,
    ctv_controller::CtvLqrController,
    gain_manifest,
    gnc::{Command, EcInput, EcRequest, Gnc, GncConfig, GncOutput, Mode},
    nav_frame::{tilt_deg, MAX_TARE_TILT_DEG},
    navigate::NavState,
};

/// The control period, 100 Hz as the gain schedule's closed-loop checks in the
/// hopper sim assume. The command has to reach the EC within about 1 ms of
/// reading the state: a late command is what costs touchdown tilt margin.
pub const PERIOD: Duration = Duration::from_millis(10);

/// Something the operator asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Request {
    /// `Ctv.start_flight`: arm from the current estimate and start the clock.
    Start,
    /// An abort while flying: land.
    Abort,
    /// A terminate: safe the EC.
    Terminate,
    /// Back to idle after a landing or a terminate. Not while flying.
    Reset,
    /// Keep the EC's demand stream alive with the safe demand while on the
    /// ground (`true`), or stop (`false`). An EC can only be armed while
    /// demands are flowing, and it safes within 30 ms of the last one, so the
    /// arm has to be preceded by a stream and followed by one until the flight
    /// starts.
    Keepalive(bool),
}

/// What to configure it with.
#[derive(Debug, Clone)]
pub struct GuidanceConfig {
    /// The gain folder exported by hopper (`gains.json` and its tables).
    pub gains_dir: PathBuf,
    /// The vehicle config with the measured allocation (`bench_config.rs`).
    pub vehicle_config: PathBuf,
    /// Accept gains exported from a modified hopper tree. Never in flight.
    pub allow_dirty: bool,
    /// The reference point relative to the IMU, body axes, m.
    pub imu_lever: [f64; 3],
}

/// What the main loop knows that the flight logic does not, for the
/// operator's status and the pre-flight checks.
#[derive(Debug, Clone)]
pub struct Outside {
    /// The navigation estimate, or why there is none.
    pub nav: Result<NavState, String>,
    /// A session with the EC is open.
    pub ec_connected: bool,
    /// The EC's state, if it has reported one.
    pub ec_state: Option<common::comm::ec_link::EcState>,
    /// Why the EC safed, for display; empty if it has not.
    pub ec_safing_reason: String,
    /// No EC driver is faulted or not ready.
    pub ec_drivers_ok: bool,
    /// The filter's latest output as it is, even if unhealthy: its uncertainty
    /// and its sensors' results are what the operator wants then.
    pub nav_state: Option<NavState>,
    /// The filter's health, for display.
    pub nav_health: String,
    /// What the filter did with each sensor's latest reading (`Diagnostics`).
    pub nav_diagnostics: [f64; 16],
}

/// Tilt from vertical of an attitude quaternion, degrees.
fn attitude_tilt(q: &nalgebra::Quaternion<f64>) -> f64 {
    tilt_deg(&UnitQuaternion::from_quaternion(*q))
}

/// How the filter is doing, from what the main loop read of it.
fn nav_status(outside: &Outside) -> CtvNav {
    let std3 = |v: &Vector3<f64>| [v.x as f32, v.y as f32, v.z as f32];
    let mut nav = CtvNav {
        health: outside.nav_health.clone(),
        sensor_results: [-1; 7],
        ..CtvNav::default()
    };

    if let Some(state) = outside.nav_state.as_ref() {
        nav.has_estimate = true;
        nav.position_std_m = std3(&state.position_std);
        nav.velocity_std_mps = std3(&state.velocity_std);
        // body x is up on the pad: the tilt is about the other two axes
        nav.tilt_std_deg = state.attitude_std.y.max(state.attitude_std.z).to_degrees() as f32;
        nav.heading_std_deg = state.attitude_std.x.to_degrees() as f32;
    }

    // the filter's slots: baro 0, gps 2, lidar 3 to 6, mag 7
    let d = &outside.nav_diagnostics;
    let result = |slot: usize| d[slot] as i8;

    nav.sensor_results = [
        result(0),
        result(2),
        result(7),
        result(3),
        result(4),
        result(5),
        result(6),
    ];

    if d[2] != -1.0 {
        nav.has_gps_nis = true;
        nav.gps_nis = d[10] as f32;
    }

    nav
}

/// What a due tick asks of the main loop.
#[derive(Debug, Clone, PartialEq)]
pub struct Step {
    /// Demands for the EC, if the flight logic is commanding it.
    pub demand: Option<ActuatorDemand>,
    /// A request for the EC.
    pub ec_request: Option<EcRequest>,
    /// Lines to print.
    pub log: Vec<String>,
    /// This period, for the operator's plots.
    pub sample: CtvSample,
}

/// A schedule of equally spaced ticks.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Periodic {
    period: Duration,
    next: Instant,
    /// Periods that went by without a tick.
    missed: u64,
}

impl Periodic {
    fn new(period: Duration, now: Instant) -> Self {
        Periodic {
            period,
            next: now + period,
            missed: 0,
        }
    }

    /// Whether a tick is due at `now`. If so the next one is one period after
    /// the last due time, not after `now`, so lateness does not accumulate.
    /// More than a period late skips the ones missed.
    fn due(&mut self, now: Instant) -> bool {
        if now < self.next {
            return false;
        }

        self.next += self.period;

        if now >= self.next {
            // a whole period or more late: skip ahead
            let behind = now.duration_since(self.next).as_nanos() / self.period.as_nanos().max(1);

            self.missed += 1 + behind as u64;
            self.next = now + self.period;
        }

        true
    }
}

/// The flight logic in the main loop.
pub struct Guidance {
    /// When it started: the clock the plots' samples are on.
    epoch: Instant,
    gnc: Gnc,
    allocation: Allocation,
    schedule: Periodic,
    start_requested: bool,
    was_commanding: bool,
    /// Missed periods already reported in a log line.
    reported_missed: u64,
    /// The controller and commit of the gains, for the operator.
    gains_label: String,
    /// The latest line from the flight logic, for the operator.
    last_event: String,
    /// Send the safe demand while idle (see `Request::Keepalive`).
    keepalive: bool,
    /// A sequence has commanded the EC lately: it has the demand, not us.
    external_demand: bool,
    /// How old the estimate was when each control step used it, over the
    /// current report window.
    age_sum: Duration,
    age_max: Duration,
    age_count: u32,
    last_age_report: Option<Instant>,
}

impl fmt::Debug for Guidance {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Guidance")
            .field("mode", &self.gnc.mode())
            .field("missed_ticks", &self.schedule.missed)
            .finish()
    }
}

impl Guidance {
    /// Loads the gains (checked against their `gains.json`) and the vehicle
    /// config.
    pub fn new(config: &GuidanceConfig, now: Instant) -> Result<Self, String> {
        let policy = gain_manifest::Policy {
            controller: "lqr_ctv_tuning",
            plant_commit: None,
            allow_dirty: config.allow_dirty,
        };
        let manifest = gain_manifest::verify(&config.gains_dir, &policy)?;
        let controller = CtvLqrController::from_gain_dir(&config.gains_dir, &policy)
            .map_err(|e| e.to_string())?;
        let vehicle = BenchConfig::load(Some(&config.vehicle_config))?;

        println!(
            "Guidance: gains {} ({} from hopper commit {}), vehicle config {}",
            config.gains_dir.display(),
            manifest.controller,
            manifest.git_commit,
            config.vehicle_config.display()
        );

        let gnc_config = GncConfig {
            lever_body: Vector3::from(config.imu_lever),
            ..GncConfig::default()
        };

        let mut guidance =
            Guidance::with_parts(Gnc::new(gnc_config, controller), vehicle.allocation(), now);

        guidance.gains_label = format!("{} @ {}", manifest.controller, manifest.git_commit);

        Ok(guidance)
    }

    /// From parts, for tests and for `new`.
    pub fn with_parts(gnc: Gnc, allocation: Allocation, now: Instant) -> Self {
        Guidance {
            epoch: now,
            gnc,
            allocation,
            schedule: Periodic::new(PERIOD, now),
            start_requested: false,
            was_commanding: false,
            reported_missed: 0,
            gains_label: "test gains".to_string(),
            last_event: String::new(),
            keepalive: false,
            external_demand: false,
            age_sum: Duration::ZERO,
            age_max: Duration::ZERO,
            age_count: 0,
            last_age_report: None,
        }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn mode(&self) -> Mode {
        self.gnc.mode()
    }

    /// Ticks that were missed so far.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn missed_ticks(&self) -> u64 {
        self.schedule.missed
    }

    /// Starts the control schedule from `now`. The main loop calls it as it
    /// begins, so the time spent starting up is not counted as missed periods.
    pub fn restart_clock(&mut self, now: Instant) {
        self.schedule = Periodic::new(PERIOD, now);
        self.reported_missed = 0;
    }

    /// Records how old the estimate is at this control step: the time between
    /// the IMU sample it is built from and now. Delay in the estimate is delay
    /// in the loop, which costs stability, so it is reported once a second while
    /// flying.
    pub fn note_estimate_age(&mut self, age: Option<Duration>) {
        if let Some(age) = age {
            self.age_sum += age;
            self.age_max = self.age_max.max(age);
            self.age_count += 1;
        }
    }

    /// Tells it whether a sequence has commanded the EC lately. While one has,
    /// the idle keep-alive stays out of its way.
    pub fn set_external_demand(&mut self, recent: bool) {
        self.external_demand = recent;
    }

    /// Whether the flight logic is commanding the EC, so that nothing else may.
    pub fn owns_ec(&self) -> bool {
        self.gnc.mode().flying()
    }

    /// An operator request. It takes effect on the next due tick.
    pub fn request(&mut self, request: Request) {
        match request {
            Request::Start => self.start_requested = true,
            Request::Abort => {
                if self.gnc.mode().flying() {
                    self.gnc.abort_land();
                }
            }
            Request::Terminate => self.gnc.terminate(),
            Request::Reset => self.gnc.reset(),
            Request::Keepalive(on) => self.keepalive = on,
        }
    }

    /// Runs the flight logic if a period is due.
    ///
    /// `nav` is the latest estimate, or `None` if there is no usable one.
    pub fn step(&mut self, now: Instant, nav: Option<NavState>, ec: EcInput) -> Option<Step> {
        if !self.schedule.due(now) {
            return None;
        }

        let mut log = Vec::new();

        // A period that went by without a tick is worth a line: a command that
        // reaches the EC late is what costs the touchdown margin
        if self.schedule.missed > self.reported_missed {
            log.push(format!(
                "GNC: {} control period(s) missed so far",
                self.schedule.missed
            ));
            self.reported_missed = self.schedule.missed;
        }

        if self.start_requested {
            self.start_requested = false;
            self.start(now, nav.as_ref(), ec, &mut log);
        }

        let input = nav.map(|estimate| (estimate, Duration::ZERO));
        let out = self.gnc.tick(now, &input, ec);
        let sample = self.sample(now, input.as_ref().map(|(estimate, _)| estimate), &out);

        for event in &out.events {
            log.push(format!(
                "GNC t = {:.2} s: {event}",
                out.mission_time.as_secs_f64()
            ));
        }

        // the operator's last event is the last thing that happened, not the
        // running count of missed periods
        if let Some(line) = log
            .iter()
            .rev()
            .find(|l| !l.contains("control period") && !l.contains("estimate age"))
        {
            self.last_event.clone_from(line);
        }

        // the estimate's age, once a second while flying
        if out.mode.flying() && self.age_count > 0 {
            let report_due = self
                .last_age_report
                .is_none_or(|at| now.saturating_duration_since(at) >= Duration::from_secs(1));

            if report_due {
                let mean = self.age_sum / self.age_count;

                log.push(format!(
                    "GNC: estimate age at use: mean {:.1} ms, max {:.1} ms over {} steps",
                    mean.as_secs_f64() * 1e3,
                    self.age_max.as_secs_f64() * 1e3,
                    self.age_count
                ));
                self.last_age_report = Some(now);
                self.age_sum = Duration::ZERO;
                self.age_max = Duration::ZERO;
                self.age_count = 0;
            }
        }

        if matches!(out.mode, Mode::Landed | Mode::Safed) {
            // the EC is disarmed or safed: nothing to keep alive
            self.keepalive = false;
        }

        // The flight logic commands the EC while flying, and for the tick it
        // stops, so the last demand it leaves is the safe one
        let commanding = out.mode.flying() || self.was_commanding;
        self.was_commanding = out.mode.flying();

        let idle_stream =
            self.keepalive && !self.external_demand && matches!(out.mode, Mode::Idle | Mode::Armed);
        let demand = if commanding {
            Some(match out.command {
                Command::Control(u) => self.allocation.demand(&u),
                Command::Safe => self.allocation.safe(),
            })
        } else if idle_stream {
            Some(self.allocation.safe())
        } else {
            None
        };

        Some(Step {
            demand,
            ec_request: out.ec_request,
            log,
            sample,
        })
    }

    /// This period's numbers for the plots; NaN for what is not available.
    fn sample(&self, now: Instant, nav: Option<&NavState>, out: &GncOutput) -> CtvSample {
        let v3 = |v: &Vector3<f64>| [v.x as f32, v.y as f32, v.z as f32];
        // the pad frame is north, east, down; the plots show up
        let up = |v: &Vector3<f64>| [v.x as f32, v.y as f32, -v.z as f32];
        let mut sample = CtvSample {
            time_s: now.saturating_duration_since(self.epoch).as_secs_f64(),
            mission_s: out.mission_time.as_secs_f32(),
            ..CtvSample::default()
        };

        if let Some(estimate) = nav {
            sample.available |= CtvSample::ESTIMATE;
            sample.position_ned_m = v3(&estimate.position);
            sample.velocity_ned_mps = v3(&estimate.velocity);
            sample.body_rate_dps = v3(&estimate.body_rate.map(f64::to_degrees));
            sample.tilt_deg = attitude_tilt(&estimate.attitude) as f32;
        }

        if out.mode.flying() {
            if let Some(state) = out.control_state {
                let (reference_position, reference_velocity) =
                    self.gnc.reference_at(state.time.as_secs_f64());

                sample.available |= CtvSample::CONTROLLER;
                sample.pad_position_m = up(&state.position);
                sample.pad_velocity_mps = up(&state.velocity);
                sample.reference_position_m = up(&Vector3::from(reference_position));
                sample.reference_velocity_mps = up(&Vector3::from(reference_velocity));
            }

            if let Command::Control(u) = out.command {
                sample.available |= CtvSample::COMMAND;
                sample.command = [
                    u.thrust as f32,
                    u.tvc_pitch.to_degrees() as f32,
                    u.tvc_yaw.to_degrees() as f32,
                    u.rcs_torque as f32,
                ];
            }
        }

        sample
    }

    /// What to show the operator: the mode, the flight numbers, the fault and
    /// the pre-flight checks.
    pub fn status(&self, outside: &Outside) -> CtvStatus {
        let mode = match self.gnc.mode() {
            Mode::Idle => CtvMode::Idle,
            Mode::Armed => CtvMode::Armed,
            Mode::Flight => CtvMode::Flight,
            Mode::AbortLand => CtvMode::AbortLand,
            Mode::Landed => CtvMode::Landed,
            Mode::Safed => CtvMode::Safed,
        };
        let flying = self.gnc.mode().flying();
        let state = self.gnc.last_state().filter(|_| {
            matches!(
                self.gnc.mode(),
                Mode::Flight | Mode::AbortLand | Mode::Landed | Mode::Safed
            )
        });
        let preflight = self.preflight(outside);
        let can_start = preflight.iter().all(|check| check.ok);

        CtvStatus {
            mode,
            mission_time_s: self.gnc.mission_time().as_secs_f64(),
            schedule_time_s: state.map_or(0.0, |s| s.time.as_secs_f64()),
            schedule_end_s: self.gnc.schedule_end(),
            altitude_m: state.map_or(0.0, |s| -s.position.z),
            tilt_deg: state
                .map(|s| attitude_tilt(&s.attitude))
                .or_else(|| {
                    outside
                        .nav
                        .as_ref()
                        .ok()
                        .map(|n| attitude_tilt(&n.attitude))
                })
                .unwrap_or(0.0),
            speed_mps: state.map_or(0.0, |s| s.velocity.norm()),
            ec_connected: outside.ec_connected,
            ec_state: outside
                .ec_state
                .filter(|_| outside.ec_connected)
                .map_or_else(String::new, |s| format!("{s:?}")),
            ec_safing_reason: outside.ec_safing_reason.clone(),
            fault: self
                .gnc
                .latched()
                .map_or_else(String::new, |(fault, response)| {
                    format!("{fault:?} ({response:?})")
                }),
            last_event: self.last_event.clone(),
            gains: self.gains_label.clone(),
            missed_ticks: self.schedule.missed,
            preflight,
            can_start: can_start && !flying,
            nav: nav_status(outside),
            tvc_limit_deg: self.allocation.max_tvc_angle_deg() as f32,
            samples: Vec::new(),
        }
    }

    /// The checks before a start, each with what it found.
    pub fn preflight(&self, outside: &Outside) -> Vec<CtvCheck> {
        let check = |name: &str, ok: bool, detail: String| CtvCheck {
            name: name.to_string(),
            ok,
            detail,
        };
        let armed =
            outside.ec_connected && outside.ec_state == Some(common::comm::ec_link::EcState::Armed);
        let state = outside
            .ec_state
            .filter(|_| outside.ec_connected)
            .map_or("no status".to_string(), |s| format!("{s:?}"));
        let mode = self.gnc.mode();
        let tilt = outside
            .nav
            .as_ref()
            .ok()
            .map(|n| attitude_tilt(&n.attitude));

        vec![
            check("Flight logic", true, self.gains_label.clone()),
            match &outside.nav {
                Ok(_) => check("Navigation estimate", true, "healthy".to_string()),
                Err(why) => check("Navigation estimate", false, why.clone()),
            },
            check(
                "Vehicle upright",
                tilt.is_some_and(|t| t <= MAX_TARE_TILT_DEG),
                tilt.map_or("no estimate".to_string(), |t| {
                    format!("{t:.1} deg from vertical (limit {MAX_TARE_TILT_DEG})")
                }),
            ),
            check(
                "EC link",
                outside.ec_connected,
                if outside.ec_connected {
                    "session open".to_string()
                } else {
                    "no session".to_string()
                },
            ),
            check(
                "EC drivers",
                outside.ec_connected && outside.ec_drivers_ok,
                if !outside.ec_connected {
                    "no session".to_string()
                } else if outside.ec_drivers_ok {
                    "ready".to_string()
                } else {
                    "a driver is faulted or not ready".to_string()
                },
            ),
            check("EC armed", armed, state),
            check("Not already flying", !mode.flying(), format!("{mode:?}")),
        ]
    }

    fn start(&mut self, now: Instant, nav: Option<&NavState>, ec: EcInput, log: &mut Vec<String>) {
        // A new start after a landing or a terminate begins a new flight
        if matches!(self.gnc.mode(), Mode::Landed | Mode::Safed) {
            self.gnc.reset();
        }

        let Some(estimate) = nav else {
            log.push("GNC: start refused: no usable navigation estimate".to_string());
            self.last_event
                .clone_from(log.last().unwrap_or(&String::new()));

            return;
        };

        match self
            .gnc
            .arm(estimate)
            .and_then(|()| self.gnc.start(now, ec))
        {
            Ok(()) => {
                log.push("GNC: flight started".to_string());
                self.last_event
                    .clone_from(log.last().unwrap_or(&String::new()));
            }
            Err(why) => {
                // arm succeeded but the EC was not ready: stand down again
                let _ = self.gnc.disarm();

                log.push(format!("GNC: start refused: {why}"));
                self.last_event
                    .clone_from(log.last().unwrap_or(&String::new()));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{MatrixXx1, MatrixXx4, UnitQuaternion};

    use super::*;
    use crate::{
        ctv_controller::{CtvK1Flat, CtvXRef},
        nav_frame::vertical_attitude,
    };

    fn controller() -> CtvLqrController {
        let tgrid = MatrixXx1::from_column_slice(&[0.0, 2.0, 4.0, 6.0]);
        let mut xref = CtvXRef::zeros(4);
        let h = std::f64::consts::FRAC_1_SQRT_2;

        for i in 0..4 {
            xref[(i, 9)] = h;
            xref[(i, 11)] = h;
        }

        xref[(1, 2)] = -2.0;
        xref[(2, 2)] = -2.0;

        let unom = MatrixXx4::from_row_slice(&[
            20.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0,
        ]);

        CtvLqrController::new(tgrid, xref, CtvK1Flat::zeros(4), MatrixXx4::zeros(4), unom)
    }

    fn guidance(now: Instant) -> Guidance {
        Guidance::with_parts(
            Gnc::new(GncConfig::default(), controller()),
            Allocation::PLACEHOLDER,
            now,
        )
    }

    fn nav(altitude: f64) -> NavState {
        NavState {
            time: Duration::ZERO,
            position: Vector3::new(0.0, 0.0, -altitude),
            velocity: Vector3::zeros(),
            attitude: *vertical_attitude().quaternion(),
            body_rate: Vector3::zeros(),
            position_std: Vector3::repeat(0.1),
            velocity_std: Vector3::repeat(0.1),
            attitude_std: Vector3::repeat(0.01),
            body_rate_std: Vector3::repeat(0.01),
        }
    }

    const EC_OK: EcInput = EcInput {
        connected: true,
        armed: true,
        safing: false,
    };

    fn ms(t0: Instant, ms: u64) -> Instant {
        t0 + Duration::from_millis(ms)
    }

    #[test]
    fn it_ticks_every_period_and_not_in_between() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        assert!(g.step(ms(t0, 3), Some(nav(0.0)), EC_OK).is_none());
        assert!(g.step(ms(t0, 9), Some(nav(0.0)), EC_OK).is_none());
        assert!(g.step(ms(t0, 10), Some(nav(0.0)), EC_OK).is_some());
        assert!(g.step(ms(t0, 12), Some(nav(0.0)), EC_OK).is_none());
        assert!(g.step(ms(t0, 20), Some(nav(0.0)), EC_OK).is_some());
        assert_eq!(g.missed_ticks(), 0);
    }

    #[test]
    fn lateness_does_not_accumulate() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        // 4 ms late, then on time: the next tick is still due at 20 ms
        assert!(g.step(ms(t0, 14), Some(nav(0.0)), EC_OK).is_some());
        assert!(g.step(ms(t0, 19), Some(nav(0.0)), EC_OK).is_none());
        assert!(g.step(ms(t0, 20), Some(nav(0.0)), EC_OK).is_some());
        assert_eq!(g.missed_ticks(), 0);
    }

    #[test]
    fn a_missed_period_is_counted_and_not_made_up() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        assert!(g.step(ms(t0, 10), Some(nav(0.0)), EC_OK).is_some());
        // nothing until 45 ms: the tick due at 20 ms runs late, and the ones
        // due at 30 and 40 ms are skipped
        assert!(g.step(ms(t0, 45), Some(nav(0.0)), EC_OK).is_some());
        assert_eq!(g.missed_ticks(), 2);
        // one tick, not a burst: the next is a period after now
        assert!(g.step(ms(t0, 46), Some(nav(0.0)), EC_OK).is_none());
        assert!(g.step(ms(t0, 55), Some(nav(0.0)), EC_OK).is_some());
    }

    #[test]
    fn before_a_start_a_period_carries_only_the_estimate() {
        let t0 = Instant::now();
        let mut g = guidance(t0);
        let sample = g.step(ms(t0, 10), Some(nav(0.5)), EC_OK).unwrap().sample;

        assert!((sample.time_s - 0.01).abs() < 1e-9, "{}", sample.time_s);
        // the estimate's position is north, east, down: 0.5 m up is -0.5 down
        assert!((sample.position_ned_m[2] + 0.5).abs() < 1e-6);
        assert!(sample.tilt_deg.abs() < 1e-3);
        // nothing was flying: no controller input, reference or command
        assert_eq!(sample.available, CtvSample::ESTIMATE);
    }

    #[test]
    fn in_flight_a_period_carries_the_controllers_state_reference_and_command() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Start);
        g.step(ms(t0, 10), Some(nav(0.0)), EC_OK).unwrap();

        // a second in, 0.3 m up and not moving
        let sample = g.step(ms(t0, 1010), Some(nav(0.3)), EC_OK).unwrap().sample;

        assert!(
            (sample.mission_s - 1.0).abs() < 0.02,
            "{}",
            sample.mission_s
        );
        // the pad frame is north, east, up
        assert!(
            (sample.pad_position_m[2] - 0.3).abs() < 1e-5,
            "{:?}",
            sample.pad_position_m
        );
        // the schedule is climbing at this point, so the reference is above the ground
        assert!(
            sample.reference_position_m[2] > 0.0,
            "{:?}",
            sample.reference_position_m
        );
        assert_eq!(
            sample.available,
            CtvSample::ESTIMATE | CtvSample::CONTROLLER | CtvSample::COMMAND
        );
        assert!(
            sample.command.iter().all(|v| v.is_finite()),
            "{:?}",
            sample.command
        );
        // thrust holds the vehicle's weight, order 10 N
        assert!(sample.command[0] > 1.0, "{:?}", sample.command);
    }

    #[test]
    fn the_status_reports_the_filters_uncertainty_and_what_it_did_with_each_sensor() {
        let t0 = Instant::now();
        let g = guidance(t0);
        let mut outside = outside(Ok(nav(0.0)), true);
        let mut state = nav(0.0);

        state.position_std = Vector3::new(0.02, 0.03, 0.01);
        state.velocity_std = Vector3::new(0.05, 0.06, 0.04);
        state.attitude_std = Vector3::new(0.02, 0.01, 0.005);
        outside.nav_state = Some(state);
        outside.nav_health = "Healthy".to_string();
        // baro taken, gps rejected with a NIS of 40, lidar 1 taken, 2 to 4 none, mag taken
        let mut d = [-1.0; 16];

        d[0] = 1.0;
        d[2] = 0.0;
        d[10] = 40.0;
        d[3] = 1.0;
        d[7] = 1.0;
        outside.nav_diagnostics = d;

        let nav = g.status(&outside).nav;

        assert_eq!(nav.health, "Healthy");
        assert!(nav.has_estimate && nav.has_gps_nis);
        assert_eq!(nav.position_std_m, [0.02, 0.03, 0.01]);
        assert!((nav.heading_std_deg - 0.02_f32.to_degrees()).abs() < 1e-4);
        assert!((nav.tilt_std_deg - 0.01_f32.to_degrees()).abs() < 1e-4);
        // baro, gps, mag, then the four LiDARs
        assert_eq!(nav.sensor_results, [1, 0, 1, 1, -1, -1, -1]);
        assert_eq!(nav.gps_nis, 40.0);
    }

    #[test]
    fn with_no_estimate_or_gps_update_yet_the_status_says_so() {
        let t0 = Instant::now();
        let g = guidance(t0);
        let status = g.status(&outside(Ok(nav(0.0)), true));

        assert!(!status.nav.has_gps_nis);
        assert!(!status.nav.has_estimate);
        assert_eq!(
            status.tvc_limit_deg,
            Allocation::PLACEHOLDER.max_tvc_angle_deg() as f32
        );
    }

    /// The status goes to the operator as JSON and is read back by Rust tools too
    /// (servo's clients, the SIL operator): every value has to survive it, which a
    /// NaN does not.
    #[test]
    fn a_status_with_samples_survives_a_round_trip_through_json() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Start);
        g.step(ms(t0, 10), Some(nav(0.0)), EC_OK).unwrap();

        let sample = g.step(ms(t0, 20), Some(nav(0.1)), EC_OK).unwrap().sample;
        let mut status = g.status(&outside(Ok(nav(0.1)), true));

        status.samples = vec![sample, CtvSample::default()];

        let json = serde_json::to_string(&status).unwrap();
        let back: CtvStatus = serde_json::from_str(&json).unwrap();

        assert_eq!(back, status);
        assert!(!json.contains("null"), "{json}");
    }

    #[test]
    fn before_a_start_it_commands_nothing() {
        let t0 = Instant::now();
        let mut g = guidance(t0);
        let step = g.step(ms(t0, 10), Some(nav(0.0)), EC_OK).unwrap();

        assert_eq!(step.demand, None);
        assert_eq!(step.ec_request, None);
        assert!(!g.owns_ec());
    }

    #[test]
    fn a_start_arms_from_the_estimate_and_takes_ownership_of_the_ec() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Start);

        let step = g.step(ms(t0, 10), Some(nav(0.0)), EC_OK).unwrap();

        assert_eq!(g.mode(), Mode::Flight);
        assert!(g.owns_ec());
        assert!(step.demand.is_some());
        assert!(step.log.iter().any(|l| l.contains("flight started")));
    }

    #[test]
    fn a_start_without_an_estimate_or_an_armed_ec_is_refused() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Start);

        let step = g.step(ms(t0, 10), None, EC_OK).unwrap();

        assert_eq!(g.mode(), Mode::Idle);
        assert!(step
            .log
            .iter()
            .any(|l| l.contains("no usable navigation estimate")));
        assert_eq!(step.demand, None);

        g.request(Request::Start);

        let not_armed = EcInput {
            armed: false,
            ..EC_OK
        };
        let step = g.step(ms(t0, 20), Some(nav(0.0)), not_armed).unwrap();

        assert_eq!(g.mode(), Mode::Idle, "stood down, not left armed");
        assert!(step.log.iter().any(|l| l.contains("EC is not armed")));
        assert!(!g.owns_ec());
    }

    #[test]
    fn a_terminate_safes_the_ec_and_releases_it() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Start);
        g.step(ms(t0, 10), Some(nav(0.0)), EC_OK);
        g.request(Request::Terminate);

        let step = g.step(ms(t0, 20), Some(nav(2.0)), EC_OK).unwrap();

        assert_eq!(step.ec_request, Some(EcRequest::Terminate));
        assert_eq!(g.mode(), Mode::Safed);
        assert!(!g.owns_ec());
        // the demand of the tick it stopped is the safe one
        assert_eq!(step.demand, Some(Allocation::PLACEHOLDER.safe()));

        // and then it sends nothing
        let after = g.step(ms(t0, 30), Some(nav(2.0)), EC_OK).unwrap();

        assert_eq!(after.demand, None);
    }

    #[test]
    fn a_reset_leaves_safed_but_not_a_flight() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Start);
        g.step(ms(t0, 10), Some(nav(0.0)), EC_OK);
        g.request(Request::Reset);
        g.step(ms(t0, 20), Some(nav(0.0)), EC_OK);

        assert_eq!(g.mode(), Mode::Flight, "not while flying");

        g.request(Request::Terminate);
        g.step(ms(t0, 30), Some(nav(0.0)), EC_OK);

        assert_eq!(g.mode(), Mode::Safed);

        g.request(Request::Reset);
        g.step(ms(t0, 40), Some(nav(0.0)), EC_OK);

        assert_eq!(g.mode(), Mode::Idle);
    }

    #[test]
    fn the_keepalive_streams_the_safe_demand_on_the_ground_until_something_ends_it() {
        let t0 = Instant::now();
        let mut g = guidance(t0);
        let safe = Allocation::PLACEHOLDER.safe();

        // off by default: nothing is sent
        assert_eq!(
            g.step(ms(t0, 10), Some(nav(0.0)), EC_OK).unwrap().demand,
            None
        );

        g.request(Request::Keepalive(true));

        for k in 2..6 {
            assert_eq!(
                g.step(ms(t0, k * 10), Some(nav(0.0)), EC_OK)
                    .unwrap()
                    .demand,
                Some(safe)
            );
        }

        // a sequence that commands the EC has it
        g.set_external_demand(true);
        assert_eq!(
            g.step(ms(t0, 60), Some(nav(0.0)), EC_OK).unwrap().demand,
            None
        );

        g.set_external_demand(false);
        assert_eq!(
            g.step(ms(t0, 70), Some(nav(0.0)), EC_OK).unwrap().demand,
            Some(safe)
        );

        g.request(Request::Keepalive(false));
        assert_eq!(
            g.step(ms(t0, 80), Some(nav(0.0)), EC_OK).unwrap().demand,
            None
        );
    }

    #[test]
    fn the_keepalive_ends_with_a_flight() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Keepalive(true));
        g.request(Request::Start);
        g.step(ms(t0, 10), Some(nav(0.0)), EC_OK);
        g.request(Request::Terminate);
        g.step(ms(t0, 20), Some(nav(0.0)), EC_OK);

        // Safed: no stream, even though the keepalive was left on by the operator
        assert_eq!(
            g.step(ms(t0, 30), Some(nav(0.0)), EC_OK).unwrap().demand,
            None
        );

        g.request(Request::Reset);
        assert_eq!(
            g.step(ms(t0, 40), Some(nav(0.0)), EC_OK).unwrap().demand,
            None,
            "it ended with the flight"
        );
    }

    #[test]
    fn restarting_the_clock_forgets_the_time_spent_starting_up() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.restart_clock(ms(t0, 8000));

        assert!(g.step(ms(t0, 8010), Some(nav(0.0)), EC_OK).is_some());
        assert_eq!(g.missed_ticks(), 0);
    }

    #[test]
    fn a_terminate_works_when_not_flying() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Terminate);

        let step = g.step(ms(t0, 10), None, EC_OK).unwrap();

        assert_eq!(step.ec_request, Some(EcRequest::Terminate));
    }

    #[test]
    fn an_abort_on_the_ground_does_nothing_here_and_in_flight_it_lands() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Abort);

        let step = g.step(ms(t0, 10), Some(nav(0.0)), EC_OK).unwrap();

        assert_eq!(step.ec_request, None);
        assert_eq!(g.mode(), Mode::Idle);

        g.request(Request::Start);
        g.step(ms(t0, 20), Some(nav(0.0)), EC_OK);
        g.step(ms(t0, 30), Some(nav(1.0)), EC_OK);
        g.request(Request::Abort);
        g.step(ms(t0, 40), Some(nav(1.0)), EC_OK);

        assert_eq!(g.mode(), Mode::AbortLand);
        assert!(g.owns_ec(), "still flying it down");
    }

    #[test]
    fn a_new_start_after_a_landing_begins_a_new_flight() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Start);
        g.step(ms(t0, 10), Some(nav(0.0)), EC_OK);
        g.request(Request::Terminate);
        g.step(ms(t0, 20), Some(nav(0.0)), EC_OK);

        assert_eq!(g.mode(), Mode::Safed);

        g.request(Request::Start);
        g.step(ms(t0, 30), Some(nav(0.0)), EC_OK);

        assert_eq!(g.mode(), Mode::Flight);
    }

    fn outside(nav: Result<NavState, String>, armed: bool) -> Outside {
        Outside {
            nav,
            ec_connected: true,
            ec_state: Some(if armed {
                common::comm::ec_link::EcState::Armed
            } else {
                common::comm::ec_link::EcState::Disarmed
            }),
            ec_safing_reason: String::new(),
            ec_drivers_ok: true,
            nav_state: None,
            nav_health: "Healthy".to_string(),
            nav_diagnostics: [-1.0; 16],
        }
    }

    #[test]
    fn before_arming_the_ec_the_preflight_says_so() {
        let t0 = Instant::now();
        let g = guidance(t0);
        let status = g.status(&outside(Ok(nav(0.0)), false));

        assert_eq!(status.mode, CtvMode::Idle);
        assert!(!status.can_start);

        let failing: Vec<&str> = status
            .preflight
            .iter()
            .filter(|c| !c.ok)
            .map(|c| c.name.as_str())
            .collect();

        assert_eq!(failing, vec!["EC armed"]);
    }

    #[test]
    fn everything_ready_means_it_can_start() {
        let t0 = Instant::now();
        let g = guidance(t0);
        let status = g.status(&outside(Ok(nav(0.0)), true));

        assert!(status.can_start, "{:?}", status.preflight);
        assert!(status.preflight.iter().all(|c| c.ok));
    }

    #[test]
    fn each_missing_thing_is_named() {
        let t0 = Instant::now();
        let g = guidance(t0);

        let no_nav = g.status(&outside(Err("still leveling".to_string()), true));
        let names: Vec<_> = no_nav
            .preflight
            .iter()
            .filter(|c| !c.ok)
            .map(|c| c.name.clone())
            .collect();

        assert_eq!(names, vec!["Navigation estimate", "Vehicle upright"]);
        assert!(no_nav.preflight[1].detail.contains("leveling"));
        assert!(!no_nav.can_start);

        let mut tipped = nav(0.0);

        tipped.attitude =
            *(UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 10f64.to_radians())
                * crate::nav_frame::vertical_attitude())
            .quaternion();

        let status = g.status(&outside(Ok(tipped), true));

        assert!(status
            .preflight
            .iter()
            .any(|c| c.name == "Vehicle upright" && !c.ok));

        let mut no_ec = outside(Ok(nav(0.0)), true);

        no_ec.ec_connected = false;

        let status = g.status(&no_ec);
        let bad: Vec<_> = status
            .preflight
            .iter()
            .filter(|c| !c.ok)
            .map(|c| c.name.as_str())
            .collect();

        assert_eq!(bad, vec!["EC link", "EC drivers", "EC armed"]);

        let mut faulted = outside(Ok(nav(0.0)), true);

        faulted.ec_drivers_ok = false;

        assert!(g
            .status(&faulted)
            .preflight
            .iter()
            .any(|c| c.name == "EC drivers" && !c.ok));
    }

    #[test]
    fn the_status_follows_a_flight() {
        let t0 = Instant::now();
        let mut g = guidance(t0);
        let world = outside(Ok(nav(0.0)), true);

        g.request(Request::Start);
        g.step(ms(t0, 10), Some(nav(0.0)), EC_OK);

        for k in 2..=101 {
            g.step(ms(t0, k * 10), Some(nav(1.5)), EC_OK);
        }

        let status = g.status(&world);

        assert_eq!(status.mode, CtvMode::Flight);
        assert!((status.altitude_m - 1.5).abs() < 1e-9);
        assert!(
            (status.mission_time_s - 1.0).abs() < 1e-6,
            "{}",
            status.mission_time_s
        );
        assert_eq!(status.schedule_end_s, 6.0);
        assert!(!status.can_start, "not while flying");
        assert!(status.last_event.contains("flight started"));

        g.request(Request::Terminate);
        g.step(ms(t0, 1020), Some(nav(1.5)), EC_OK);

        let status = g.status(&world);

        assert_eq!(status.mode, CtvMode::Safed);
        assert!(status.fault.contains("ManualTerminate"), "{}", status.fault);
        assert!(status.last_event.contains("Safed"), "{}", status.last_event);
    }

    #[test]
    fn the_demand_is_the_allocation_of_the_controllers_command() {
        let t0 = Instant::now();
        let mut g = guidance(t0);

        g.request(Request::Start);

        let step = g.step(ms(t0, 10), Some(nav(0.0)), EC_OK).unwrap();
        let wanted = Allocation::PLACEHOLDER.demand(&common::comm::ctv::ControlVector {
            thrust: 20.0,
            tvc_pitch: 0.0,
            tvc_yaw: 0.0,
            rcs_torque: 0.0,
        });

        assert_eq!(step.demand, Some(wanted));
    }
}
