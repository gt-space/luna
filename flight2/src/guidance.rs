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

use common::comm::ctv::ActuatorDemand;
use nalgebra::Vector3;

use crate::{
    bench_config::BenchConfig,
    ctv_allocation::Allocation,
    ctv_controller::CtvLqrController,
    gain_manifest,
    gnc::{Command, EcInput, EcRequest, Gnc, GncConfig, Mode},
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

/// What a due tick asks of the main loop.
#[derive(Debug, Clone, PartialEq)]
pub struct Step {
    /// Demands for the EC, if the flight logic is commanding it.
    pub demand: Option<ActuatorDemand>,
    /// A request for the EC.
    pub ec_request: Option<EcRequest>,
    /// Lines to print.
    pub log: Vec<String>,
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
    gnc: Gnc,
    allocation: Allocation,
    schedule: Periodic,
    start_requested: bool,
    was_commanding: bool,
    /// Missed periods already reported in a log line.
    reported_missed: u64,
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

        Ok(Guidance::with_parts(
            Gnc::new(gnc_config, controller),
            vehicle.allocation(),
            now,
        ))
    }

    /// From parts, for tests and for `new`.
    pub fn with_parts(gnc: Gnc, allocation: Allocation, now: Instant) -> Self {
        Guidance {
            gnc,
            allocation,
            schedule: Periodic::new(PERIOD, now),
            start_requested: false,
            was_commanding: false,
            reported_missed: 0,
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

        for event in &out.events {
            log.push(format!("GNC t = {:.2} s: {event}", out.mission_time.as_secs_f64()));
        }

        // The flight logic commands the EC while flying, and for the tick it
        // stops, so the last demand it leaves is the safe one
        let commanding = out.mode.flying() || self.was_commanding;
        self.was_commanding = out.mode.flying();

        let demand = commanding.then(|| match out.command {
            Command::Control(u) => self.allocation.demand(&u),
            Command::Safe => self.allocation.safe(),
        });

        Some(Step {
            demand,
            ec_request: out.ec_request,
            log,
        })
    }

    fn start(&mut self, now: Instant, nav: Option<&NavState>, ec: EcInput, log: &mut Vec<String>) {
        // A new start after a landing or a terminate begins a new flight
        if matches!(self.gnc.mode(), Mode::Landed | Mode::Safed) {
            self.gnc.reset();
        }

        let Some(estimate) = nav else {
            log.push("GNC: start refused: no usable navigation estimate".to_string());

            return;
        };

        match self.gnc.arm(estimate).and_then(|()| self.gnc.start(now, ec)) {
            Ok(()) => log.push("GNC: flight started".to_string()),
            Err(why) => {
                // arm succeeded but the EC was not ready: stand down again
                let _ = self.gnc.disarm();

                log.push(format!("GNC: start refused: {why}"));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{MatrixXx1, MatrixXx4};

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
        assert!(step.log.iter().any(|l| l.contains("no usable navigation estimate")));
        assert_eq!(step.demand, None);

        g.request(Request::Start);

        let not_armed = EcInput { armed: false, ..EC_OK };
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
