//! Guidance, navigation and control for the CTV: what runs every control
//! period between the navigation estimate and the engine controller.
//!
//! [`Gnc`] is plain logic: no sockets, no threads, no clock of its own. The
//! caller hands it the time, the navigation estimate and what the engine
//! controller (EC) reports on every tick, and gets back the command to
//! send and any request for the EC. That makes the same code run in the flight
//! computer's main loop and in software-in-the-loop (SIL) against the plant.
//!
//! ```text
//!   estimate -> NavFrame (pad tare) -> ControlState -> CtvLqrController -> command
//!                                           |
//!                                           +-> monitors -> faults -> response -> mode
//! ```
//!
//! # Modes
//!
//! ```text
//!   Idle --arm--> Armed --start--> Flight --(fault R2)--> AbortLand
//!     ^              |                |                       |
//!     +---disarm-----+                +--cutoff at the pad----+--> Landed
//!   any mode --(R4, or terminate)--> Safed
//! ```
//!
//! - `arm` takes the pad pose from the navigation estimate ([`NavFrame`]).
//!   It fails if the estimate is unusable or the vehicle is not upright.
//! - `start` begins the mission clock at schedule time 0, which is what the
//!   gain schedule is indexed by. It needs the EC armed.
//! - In flight the controller is stepped every tick. The command is held for
//!   a short while if the estimate is late, and the response becomes R4
//!   (uncontrollable) if it stays lost.
//! - The engine is cut when the vehicle is back down at the cutoff altitude
//!   (5 mm in the hopper sim), then `Landed`: the EC is disarmed.
//!
//! # Faults and responses
//!
//! Modeled on hopper's FDIR ladder (`docs/FDIR_ARCHITECTURE.md`), at a much
//! smaller scale. A fault latches at a response level and only ever escalates:
//!
//! - R2, abort to land: the schedule clock jumps to the point on the descent
//!   whose altitude is the vehicle's, so the reference is continuous in height
//!   and ends on the pad.
//! - R4, safe: the EC is terminated (throttle closed, gimbal safed). It is
//!   permitted on the pad, below the cut height `h_cut`, or when the vehicle
//!   is uncontrollable. Above `h_cut` and still controllable, it is held back
//!   to R2, because cutting the engine in the air is worse.
//!
//! The thresholds are provisional, like the ones in hopper's catalog: they
//! need a Monte Carlo false-alarm against missed-detection study.

use std::{
    fmt,
    time::{Duration, Instant},
};

use common::comm::ctv::{ControlState, ControlVector};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    ctv_controller::{CtvLqrController, StepError},
    nav_frame::{tilt_deg, FrameError, NavFrame},
    navigate::NavState,
};

/// Settings. The defaults are provisional (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GncConfig {
    /// An estimate older than this is not used.
    pub nav_max_age: Duration,
    /// The command is held this long without a usable estimate. Past it the
    /// vehicle is uncontrollable (R4).
    pub nav_loss_limit: Duration,
    /// Tilt from vertical that aborts to land, degrees.
    pub tilt_r2_deg: f64,
    /// Tilt from vertical past which the vehicle is uncontrollable, degrees.
    pub tilt_uncontrollable_deg: f64,
    /// Distance from the reference that aborts to land, m (each of the
    /// horizontal and vertical errors).
    pub position_error_r2_m: f64,
    /// The engine is cut at this altitude on the way down, m (hopper's
    /// `engine_cutoff_alt`).
    pub cutoff_altitude_m: f64,
    /// R4 in flight is only allowed below this altitude, m.
    pub h_cut_m: f64,
    /// Above this altitude the vehicle has left the pad, m. The cutoff only
    /// counts after that, so it cannot fire on the pad.
    pub left_pad_altitude_m: f64,
    /// Past the end of the schedule by this long, abort to land.
    pub schedule_overrun: Duration,
    /// The controller's reference point relative to the IMU, body axes, m.
    pub lever_body: Vector3<f64>,
}

impl Default for GncConfig {
    fn default() -> Self {
        GncConfig {
            nav_max_age: Duration::from_millis(50),
            nav_loss_limit: Duration::from_millis(200),
            tilt_r2_deg: 20.0,
            tilt_uncontrollable_deg: 60.0,
            position_error_r2_m: 3.0,
            cutoff_altitude_m: 0.005,
            h_cut_m: 1.5,
            left_pad_altitude_m: 0.3,
            schedule_overrun: Duration::from_secs(5),
            lever_body: Vector3::zeros(),
        }
    }
}

/// What the engine controller reports each tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EcInput {
    /// A session with the EC is up.
    pub connected: bool,
    /// The EC is armed (has output authority).
    pub armed: bool,
    /// The EC has safed itself.
    pub safing: bool,
}

/// The mode of the flight logic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// On the pad, not armed. Safe outputs.
    Idle,
    /// Pad pose taken, waiting for `start`.
    Armed,
    /// Flying the schedule.
    Flight,
    /// A fault latched R2: flying down from the vehicle's altitude.
    AbortLand,
    /// Engine cut at the pad, EC disarmed.
    Landed,
    /// Terminated (R4): EC safed.
    Safed,
}

impl Mode {
    /// Whether the controller is running.
    pub fn flying(self) -> bool {
        matches!(self, Mode::Flight | Mode::AbortLand)
    }
}

/// The response levels, as in hopper's FDIR.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Response {
    R2,
    R4,
}

/// Why a response was latched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    /// No usable estimate for longer than the loss limit.
    NavLost,
    /// Tilted past the abort limit.
    Tilt,
    /// Tilted past the uncontrollable limit.
    Uncontrollable,
    /// Too far from the reference.
    PositionError,
    /// Past the end of the schedule and still flying.
    ScheduleOverrun,
    /// The EC safed itself or dropped out in flight.
    EcLost,
    /// An operator abort.
    ManualAbort,
    /// An operator terminate.
    ManualTerminate,
}

/// What `Gnc` asks of the EC, once per transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EcRequest {
    /// Output authority off: fans and ESC off, TVC holds. A normal landing.
    Disarm,
    /// Safe immediately and latch.
    Terminate,
}

/// The command for this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    /// Thrust, gimbal angles and roll torque, to go through the allocation.
    Control(ControlVector),
    /// The safe outputs: throttle closed, gimbal centered, fans off.
    Safe,
}

/// Something worth logging.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Event {
    ModeChange { from: Mode, to: Mode },
    Latched { fault: Fault, response: Response },
    /// R4 was asked for but is not allowed here; R2 is flown instead.
    R4Held { altitude_m: f64 },
    /// The abort-to-land reference starts at this schedule time.
    AbortReference { schedule_s: f64 },
    /// The estimate was unusable this tick.
    NavUnusable,
    /// The controller refused the state.
    ControllerError(StepError),
    EngineCut { altitude_m: f64 },
}

impl fmt::Display for Event {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Event::ModeChange { from, to } => write!(f, "mode {from:?} -> {to:?}"),
            Event::Latched { fault, response } => write!(f, "latched {fault:?} at {response:?}"),
            Event::R4Held { altitude_m } => write!(
                f,
                "R4 held back at {altitude_m:.2} m (above the cut height and controllable); flying R2"
            ),
            Event::AbortReference { schedule_s } => {
                write!(f, "abort reference from schedule t = {schedule_s:.2} s")
            }
            Event::NavUnusable => write!(f, "estimate unusable this tick"),
            Event::ControllerError(e) => write!(f, "controller refused the state: {e}"),
            Event::EngineCut { altitude_m } => {
                write!(f, "engine cut at {altitude_m:.3} m")
            }
        }
    }
}

/// What `tick` returns.
#[derive(Debug, Clone, PartialEq)]
pub struct GncOutput {
    pub mode: Mode,
    /// Mission time, from `start`. Zero before.
    pub mission_time: Duration,
    pub command: Command,
    pub ec_request: Option<EcRequest>,
    pub events: Vec<Event>,
    /// The state the controller was given, if it ran.
    pub control_state: Option<ControlState>,
}

/// Why `arm` or `start` was refused.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RefusedReason {
    WrongMode(Mode),
    NoEstimate,
    Frame(FrameError),
    EcNotArmed,
}

impl fmt::Display for RefusedReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RefusedReason::WrongMode(m) => write!(f, "not allowed in mode {m:?}"),
            RefusedReason::NoEstimate => write!(f, "no usable navigation estimate"),
            RefusedReason::Frame(e) => write!(f, "{e}"),
            RefusedReason::EcNotArmed => write!(f, "the EC is not armed"),
        }
    }
}

impl std::error::Error for RefusedReason {}

/// The flight logic.
#[derive(Debug)]
pub struct Gnc {
    config: GncConfig,
    controller: CtvLqrController,
    mode: Mode,
    frame: Option<NavFrame>,
    /// Mission clock start.
    t0: Option<Instant>,
    /// Added to the mission time to get the schedule time (abort to land).
    schedule_shift: f64,
    last_good: Option<Instant>,
    last_command: Option<ControlVector>,
    left_pad: bool,
    latched: Option<(Fault, Response)>,
    /// A manual request waiting for the next tick.
    manual: Option<Fault>,
    /// The last tilt, for logs.
    uncontrollable: bool,
}

/// A usable estimate: not too old.
type NavInput = Option<(NavState, Duration)>;

impl Gnc {
    pub fn new(config: GncConfig, controller: CtvLqrController) -> Self {
        Gnc {
            config,
            controller,
            mode: Mode::Idle,
            frame: None,
            t0: None,
            schedule_shift: 0.0,
            last_good: None,
            last_command: None,
            left_pad: false,
            latched: None,
            manual: None,
            uncontrollable: false,
        }
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn config(&self) -> &GncConfig {
        &self.config
    }

    /// The end of the gain schedule, s.
    pub fn schedule_end(&self) -> f64 {
        self.controller.schedule_end()
    }

    /// The fault that put the logic in its current response, if any.
    pub fn latched(&self) -> Option<(Fault, Response)> {
        self.latched
    }

    /// Takes the pad pose from `estimate`, which must be of the vehicle
    /// standing on the pad.
    pub fn arm(&mut self, estimate: &NavState) -> Result<(), RefusedReason> {
        if self.mode != Mode::Idle {
            return Err(RefusedReason::WrongMode(self.mode));
        }

        self.frame = Some(
            NavFrame::tare(estimate, self.config.lever_body).map_err(RefusedReason::Frame)?,
        );
        self.mode = Mode::Armed;

        Ok(())
    }

    /// Back to idle from armed.
    pub fn disarm(&mut self) -> Result<(), RefusedReason> {
        if self.mode != Mode::Armed {
            return Err(RefusedReason::WrongMode(self.mode));
        }

        self.frame = None;
        self.mode = Mode::Idle;

        Ok(())
    }

    /// Starts the mission clock at schedule time 0 and the controller.
    pub fn start(&mut self, now: Instant, ec: EcInput) -> Result<(), RefusedReason> {
        if self.mode != Mode::Armed {
            return Err(RefusedReason::WrongMode(self.mode));
        }

        if !(ec.connected && ec.armed && !ec.safing) {
            return Err(RefusedReason::EcNotArmed);
        }

        self.t0 = Some(now);
        self.last_good = Some(now);
        self.last_command = None;
        self.schedule_shift = 0.0;
        self.left_pad = false;
        self.latched = None;
        self.uncontrollable = false;
        self.mode = Mode::Flight;

        Ok(())
    }

    /// Back to idle after a landing or a terminate, ready to arm again. Not
    /// allowed while flying.
    pub fn reset(&mut self) {
        if self.mode.flying() {
            return;
        }

        self.mode = Mode::Idle;
        self.frame = None;
        self.t0 = None;
        self.schedule_shift = 0.0;
        self.last_good = None;
        self.last_command = None;
        self.left_pad = false;
        self.latched = None;
        self.manual = None;
        self.uncontrollable = false;
    }

    /// An operator abort to land. Takes effect on the next tick.
    pub fn abort_land(&mut self) {
        self.manual.get_or_insert(Fault::ManualAbort);
    }

    /// An operator terminate: safe the EC. Always accepted, in any mode.
    pub fn terminate(&mut self) {
        self.manual = Some(Fault::ManualTerminate);
    }

    /// One control period. `nav` is the latest estimate with its age, if
    /// there is one.
    pub fn tick(&mut self, now: Instant, nav: &NavInput, ec: EcInput) -> GncOutput {
        let mut out = GncOutput {
            mode: self.mode,
            mission_time: self.t0.map_or(Duration::ZERO, |t0| now.saturating_duration_since(t0)),
            command: Command::Safe,
            ec_request: None,
            events: Vec::new(),
            control_state: None,
        };

        // An operator terminate is accepted in every mode
        if self.manual == Some(Fault::ManualTerminate) {
            self.manual = None;

            if self.mode != Mode::Safed {
                self.latch(Fault::ManualTerminate, Response::R4, &mut out);
                self.go(Mode::Safed, &mut out);
                out.ec_request = Some(EcRequest::Terminate);
            }

            out.mode = self.mode;

            return out;
        }

        if self.mode.flying() {
            self.fly(now, nav, ec, &mut out);
        }

        out.mode = self.mode;

        out
    }

    fn go(&mut self, to: Mode, out: &mut GncOutput) {
        if self.mode != to {
            out.events.push(Event::ModeChange {
                from: self.mode,
                to,
            });
            self.mode = to;
        }
    }

    /// Latches `fault` at `response` if that is a higher response than the
    /// one already latched.
    fn latch(&mut self, fault: Fault, response: Response, out: &mut GncOutput) {
        if self.latched.is_none_or(|(_, r)| response > r) {
            self.latched = Some((fault, response));
            out.events.push(Event::Latched { fault, response });
        }
    }

    fn fly(&mut self, now: Instant, nav: &NavInput, ec: EcInput, out: &mut GncOutput) {
        let t0 = self.t0.unwrap_or(now);
        let mission = now.saturating_duration_since(t0).as_secs_f64();
        let frame = self.frame;

        // The EC safed itself or went away: it has taken the engine away, and
        // there is no actuation left to command, whatever the altitude
        if ec.safing || !ec.connected || !ec.armed {
            self.uncontrollable = true;
            self.latch(Fault::EcLost, Response::R4, out);
        }

        // The estimate, in the pad frame, stepped through the controller
        let mut state = None;
        let mut command = None;

        if let (Some((estimate, age)), Some(frame)) = (nav, frame) {
            if *age <= self.config.nav_max_age {
                let mut s = frame.control_state(estimate);

                s.time = Duration::from_secs_f64((mission + self.schedule_shift).max(0.0));

                match self.controller.try_step(s) {
                    Ok(u) => {
                        state = Some(s);
                        command = Some(u);
                        self.last_good = Some(now);
                        self.last_command = Some(u);
                    }
                    Err(e) => out.events.push(Event::ControllerError(e)),
                }
            }
        }

        if state.is_none() {
            out.events.push(Event::NavUnusable);

            let since = now.saturating_duration_since(self.last_good.unwrap_or(t0));

            if since > self.config.nav_loss_limit {
                self.uncontrollable = true;
                self.latch(Fault::NavLost, Response::R4, out);
            } else {
                // hold the last command through a short gap
                command = self.last_command;
            }
        }

        let mut altitude = None;

        if let Some(s) = state {
            let alt = -s.position.z;
            let attitude = UnitQuaternion::from_quaternion(s.attitude);
            let tilt = tilt_deg(&attitude);

            altitude = Some(alt);

            if alt > self.config.left_pad_altitude_m {
                self.left_pad = true;
            }

            if tilt >= self.config.tilt_uncontrollable_deg {
                self.uncontrollable = true;
                self.latch(Fault::Uncontrollable, Response::R4, out);
            } else if tilt >= self.config.tilt_r2_deg {
                self.latch(Fault::Tilt, Response::R2, out);
            }

            let reference = self.controller.reference_state(s.time.as_secs_f64());
            let horizontal = (s.position.x - reference[0]).hypot(s.position.y - reference[1]);
            let vertical = (s.position.z - reference[2]).abs();

            if horizontal > self.config.position_error_r2_m
                || vertical > self.config.position_error_r2_m
            {
                self.latch(Fault::PositionError, Response::R2, out);
            }

            if s.time.as_secs_f64() > self.controller.schedule_end() + self.config.schedule_overrun.as_secs_f64() {
                self.latch(Fault::ScheduleOverrun, Response::R2, out);
            }

            // The engine cut at the pad on the way down
            if self.left_pad
                && alt <= self.config.cutoff_altitude_m
                && s.velocity.z >= -0.05
            {
                out.events.push(Event::EngineCut { altitude_m: alt });
                out.control_state = state;
                self.go(Mode::Landed, out);
                out.ec_request = Some(EcRequest::Disarm);

                return;
            }
        }

        if self.manual == Some(Fault::ManualAbort) {
            self.manual = None;
            self.latch(Fault::ManualAbort, Response::R2, out);
        }

        // Respond to what is latched
        if let Some((_, response)) = self.latched {
            let alt = altitude.unwrap_or(f64::INFINITY);
            let allowed = self.uncontrollable
                || !self.left_pad
                || (altitude.is_some() && alt <= self.config.h_cut_m);

            if response == Response::R4 && allowed {
                self.go(Mode::Safed, out);
                out.ec_request = Some(EcRequest::Terminate);
                out.control_state = state;

                return;
            }

            if response == Response::R4 {
                out.events.push(Event::R4Held { altitude_m: alt });
            }

            if self.mode == Mode::Flight {
                self.go(Mode::AbortLand, out);

                if let Some(alt) = altitude {
                    if let Some(t) = self.controller.descent_time_at_altitude(alt) {
                        let now_schedule = mission + self.schedule_shift;

                        if t > now_schedule {
                            self.schedule_shift = t - mission;
                        }

                        out.events.push(Event::AbortReference {
                            schedule_s: mission + self.schedule_shift,
                        });
                    }
                }
            }
        }

        out.control_state = state;
        out.command = command.map_or(Command::Safe, Command::Control);
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Quaternion, MatrixXx1, MatrixXx4};

    use super::*;
    use crate::{
        ctv_controller::{CtvK1Flat, CtvXRef},
        nav_frame::vertical_attitude,
    };

    /// A schedule up 2 m in 2 s, hover to 4 s, down to the pad at 6 s, with
    /// zero gains and a constant 20 N: the command is always the nominal.
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
            20.0, 0.0, 0.0, 0.0, //
            20.0, 0.0, 0.0, 0.0, //
            20.0, 0.0, 0.0, 0.0, //
            20.0, 0.0, 0.0, 0.0,
        ]);

        CtvLqrController::new(tgrid, xref, CtvK1Flat::zeros(4), MatrixXx4::zeros(4), unom)
    }

    const EC_OK: EcInput = EcInput {
        connected: true,
        armed: true,
        safing: false,
    };

    fn estimate(altitude: f64, tilt_deg: f64) -> NavState {
        let tilt = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), tilt_deg.to_radians())
            * vertical_attitude();

        NavState {
            time: Duration::ZERO,
            position: Vector3::new(0.0, 0.0, -altitude),
            velocity: Vector3::zeros(),
            attitude: *tilt.quaternion(),
            body_rate: Vector3::zeros(),
            position_std: Vector3::repeat(0.1),
            velocity_std: Vector3::repeat(0.1),
            attitude_std: Vector3::repeat(0.01),
            body_rate_std: Vector3::repeat(0.01),
        }
    }

    fn fresh(estimate: NavState) -> NavInput {
        Some((estimate, Duration::from_millis(5)))
    }

    /// An armed and started Gnc, and the start instant.
    fn flying() -> (Gnc, Instant) {
        let mut gnc = Gnc::new(GncConfig::default(), controller());
        let t0 = Instant::now();

        gnc.arm(&estimate(0.0, 0.0)).unwrap();
        gnc.start(t0, EC_OK).unwrap();

        (gnc, t0)
    }

    fn at(t0: Instant, seconds: f64) -> Instant {
        t0 + Duration::from_secs_f64(seconds)
    }

    #[test]
    fn idle_and_armed_command_the_safe_outputs() {
        let mut gnc = Gnc::new(GncConfig::default(), controller());
        let now = Instant::now();

        assert_eq!(gnc.tick(now, &fresh(estimate(0.0, 0.0)), EC_OK).command, Command::Safe);

        gnc.arm(&estimate(0.0, 0.0)).unwrap();

        assert_eq!(gnc.mode(), Mode::Armed);
        assert_eq!(gnc.tick(now, &fresh(estimate(0.0, 0.0)), EC_OK).command, Command::Safe);
    }

    #[test]
    fn arm_is_refused_when_not_upright_or_bad() {
        let mut gnc = Gnc::new(GncConfig::default(), controller());

        assert!(matches!(
            gnc.arm(&estimate(0.0, 10.0)),
            Err(RefusedReason::Frame(FrameError::NotUpright { .. }))
        ));
        assert_eq!(gnc.mode(), Mode::Idle);

        let mut bad = estimate(0.0, 0.0);

        bad.attitude = Quaternion::new(f64::NAN, 0.0, 0.0, 0.0);

        assert!(matches!(
            gnc.arm(&bad),
            Err(RefusedReason::Frame(FrameError::InvalidEstimate))
        ));
    }

    #[test]
    fn start_needs_an_armed_ec_and_an_armed_gnc() {
        let mut gnc = Gnc::new(GncConfig::default(), controller());
        let now = Instant::now();

        assert!(matches!(
            gnc.start(now, EC_OK),
            Err(RefusedReason::WrongMode(Mode::Idle))
        ));

        gnc.arm(&estimate(0.0, 0.0)).unwrap();

        for bad in [
            EcInput { armed: false, ..EC_OK },
            EcInput { connected: false, ..EC_OK },
            EcInput { safing: true, ..EC_OK },
        ] {
            assert_eq!(gnc.start(now, bad), Err(RefusedReason::EcNotArmed));
        }

        assert!(gnc.start(now, EC_OK).is_ok());
        assert_eq!(gnc.mode(), Mode::Flight);
    }

    #[test]
    fn flight_runs_the_controller_on_the_mission_clock() {
        let (mut gnc, t0) = flying();
        let out = gnc.tick(at(t0, 1.0), &fresh(estimate(0.0, 0.0)), EC_OK);

        assert_eq!(out.mode, Mode::Flight);
        assert_eq!(out.mission_time, Duration::from_secs(1));
        assert_eq!(out.control_state.unwrap().time, Duration::from_secs(1));
        assert!(matches!(out.command, Command::Control(u) if (u.thrust - 20.0).abs() < 1e-9));
    }

    #[test]
    fn the_estimates_own_clock_is_ignored() {
        let (mut gnc, t0) = flying();
        let mut e = estimate(0.0, 0.0);

        e.time = Duration::from_secs(9999);

        let out = gnc.tick(at(t0, 0.5), &fresh(e), EC_OK);

        assert_eq!(out.control_state.unwrap().time, Duration::from_millis(500));
    }

    #[test]
    fn a_short_gap_holds_the_last_command() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(0.0, 0.0)), EC_OK);

        let out = gnc.tick(at(t0, 1.1), &None, EC_OK);

        assert_eq!(out.mode, Mode::Flight);
        assert!(out.events.contains(&Event::NavUnusable));
        assert!(matches!(out.command, Command::Control(u) if (u.thrust - 20.0).abs() < 1e-9));
    }

    #[test]
    fn a_stale_estimate_is_not_used() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(0.0, 0.0)), EC_OK);

        let stale = Some((estimate(0.0, 0.0), Duration::from_millis(80)));
        let out = gnc.tick(at(t0, 1.01), &stale, EC_OK);

        assert!(out.events.contains(&Event::NavUnusable));
        assert!(out.control_state.is_none());
    }

    #[test]
    fn losing_the_estimate_for_too_long_is_r4_and_terminates_the_ec() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(5.0, 0.0)), EC_OK);

        let out = gnc.tick(at(t0, 1.3), &None, EC_OK);

        // uncontrollable, so allowed even high up
        assert_eq!(out.mode, Mode::Safed);
        assert_eq!(out.ec_request, Some(EcRequest::Terminate));
        assert_eq!(out.command, Command::Safe);
        assert_eq!(gnc.latched(), Some((Fault::NavLost, Response::R4)));
    }

    #[test]
    fn a_non_finite_estimate_is_treated_as_lost() {
        let (mut gnc, t0) = flying();
        let mut bad = estimate(0.0, 0.0);

        bad.position.x = f64::NAN;

        gnc.tick(at(t0, 1.0), &fresh(estimate(0.0, 0.0)), EC_OK);

        // frame.control_state passes the NaN to the controller, which refuses it
        let out = gnc.tick(at(t0, 1.01), &fresh(bad), EC_OK);

        assert!(out
            .events
            .contains(&Event::ControllerError(StepError::NonFiniteState)));
        assert_eq!(out.mode, Mode::Flight);

        let out = gnc.tick(at(t0, 1.3), &fresh(bad), EC_OK);

        assert_eq!(out.mode, Mode::Safed);
    }

    #[test]
    fn tilt_aborts_to_land_and_does_not_cut_the_engine_in_the_air() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(2.0, 0.0)), EC_OK);

        let out = gnc.tick(at(t0, 1.01), &fresh(estimate(2.0, 30.0)), EC_OK);

        assert_eq!(out.mode, Mode::AbortLand);
        assert_eq!(out.ec_request, None);
        assert!(matches!(out.command, Command::Control(_)));
        assert_eq!(gnc.latched(), Some((Fault::Tilt, Response::R2)));
    }

    #[test]
    fn an_uncontrollable_tilt_terminates_even_high_up() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(2.0, 0.0)), EC_OK);

        let out = gnc.tick(at(t0, 1.01), &fresh(estimate(2.0, 75.0)), EC_OK);

        assert_eq!(out.mode, Mode::Safed);
        assert_eq!(out.ec_request, Some(EcRequest::Terminate));
    }

    #[test]
    fn r4_in_the_air_but_controllable_is_held_back_to_r2() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(2.0, 0.0)), EC_OK);
        gnc.terminate_for_test_fault(Fault::EcLost);

        let out = gnc.tick(at(t0, 1.01), &fresh(estimate(2.0, 0.0)), EC_OK);

        assert!(out.events.iter().any(|e| matches!(e, Event::R4Held { .. })));
        assert_eq!(out.mode, Mode::AbortLand);
    }

    #[test]
    fn r4_is_allowed_on_the_pad_and_below_the_cut_height() {
        for altitude in [0.0, 1.0] {
            let (mut gnc, t0) = flying();

            if altitude > 0.3 {
                gnc.tick(at(t0, 1.0), &fresh(estimate(2.0, 0.0)), EC_OK);
            }

            gnc.terminate_for_test_fault(Fault::EcLost);

            let out = gnc.tick(at(t0, 1.01), &fresh(estimate(altitude, 0.0)), EC_OK);

            assert_eq!(out.mode, Mode::Safed, "altitude {altitude}");
            assert_eq!(out.ec_request, Some(EcRequest::Terminate));
        }
    }

    #[test]
    fn the_ec_safing_in_flight_is_a_fault() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 0.5), &fresh(estimate(0.0, 0.0)), EC_OK);

        let safing = EcInput {
            safing: true,
            armed: false,
            ..EC_OK
        };
        let out = gnc.tick(at(t0, 0.51), &fresh(estimate(0.0, 0.0)), safing);

        assert_eq!(out.mode, Mode::Safed);
        assert_eq!(gnc.latched(), Some((Fault::EcLost, Response::R4)));
    }

    #[test]
    fn the_ec_dropping_out_high_up_is_terminal_not_held_back() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(3.0, 0.0)), EC_OK);

        let gone = EcInput {
            connected: false,
            armed: false,
            safing: false,
        };
        let out = gnc.tick(at(t0, 1.01), &fresh(estimate(3.0, 0.0)), gone);

        assert_eq!(out.mode, Mode::Safed);
        assert!(!out.events.iter().any(|e| matches!(e, Event::R4Held { .. })));
    }

    #[test]
    fn abort_to_land_continues_the_reference_from_the_vehicles_altitude() {
        let (mut gnc, t0) = flying();

        // Climbing at 1 m, t = 1 s: ascending reference
        gnc.tick(at(t0, 1.0), &fresh(estimate(1.0, 0.0)), EC_OK);
        gnc.abort_land();

        let out = gnc.tick(at(t0, 1.01), &fresh(estimate(1.0, 0.0)), EC_OK);

        assert_eq!(out.mode, Mode::AbortLand);

        // 1 m on the way down is schedule t = 5 s, so the clock jumped from 1.01
        let jump = out
            .events
            .iter()
            .find_map(|e| match e {
                Event::AbortReference { schedule_s } => Some(*schedule_s),
                _ => None,
            })
            .unwrap();

        assert!((jump - 5.0).abs() < 1e-9, "{jump}");

        let next = gnc.tick(at(t0, 1.02), &fresh(estimate(1.0, 0.0)), EC_OK);

        assert!((next.control_state.unwrap().time.as_secs_f64() - 5.01).abs() < 1e-6);
    }

    #[test]
    fn the_engine_is_cut_at_the_pad_on_the_way_down_only() {
        let (mut gnc, t0) = flying();

        // On the pad at the start: no cutoff, it has not left yet
        let out = gnc.tick(at(t0, 0.1), &fresh(estimate(0.0, 0.0)), EC_OK);

        assert_eq!(out.mode, Mode::Flight);

        gnc.tick(at(t0, 2.0), &fresh(estimate(2.0, 0.0)), EC_OK);
        gnc.tick(at(t0, 5.5), &fresh(estimate(0.4, 0.0)), EC_OK);

        let out = gnc.tick(at(t0, 5.9), &fresh(estimate(0.004, 0.0)), EC_OK);

        assert_eq!(out.mode, Mode::Landed);
        assert_eq!(out.ec_request, Some(EcRequest::Disarm));
        assert!(out.events.iter().any(|e| matches!(e, Event::EngineCut { .. })));

        // After landing: safe outputs, no more requests
        let after = gnc.tick(at(t0, 6.0), &fresh(estimate(0.0, 0.0)), EC_OK);

        assert_eq!(after.command, Command::Safe);
        assert_eq!(after.ec_request, None);
    }

    #[test]
    fn no_cutoff_while_still_climbing_through_the_cutoff_altitude() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(1.0, 0.0)), EC_OK);

        let mut low_and_rising = estimate(0.004, 0.0);

        low_and_rising.velocity = Vector3::new(0.0, 0.0, -1.0);

        let out = gnc.tick(at(t0, 1.1), &fresh(low_and_rising), EC_OK);

        assert_eq!(out.mode, Mode::Flight);
    }

    #[test]
    fn a_position_error_aborts_to_land() {
        let (mut gnc, t0) = flying();
        let mut off = estimate(0.5, 0.0);

        off.position.x = 4.0;

        let out = gnc.tick(at(t0, 0.5), &fresh(off), EC_OK);

        assert_eq!(out.mode, Mode::AbortLand);
        assert_eq!(gnc.latched(), Some((Fault::PositionError, Response::R2)));
    }

    #[test]
    fn past_the_schedule_end_aborts_to_land() {
        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(2.0, 0.0)), EC_OK);

        let out = gnc.tick(at(t0, 11.5), &fresh(estimate(2.0, 0.0)), EC_OK);

        assert_eq!(gnc.latched().map(|l| l.0), Some(Fault::ScheduleOverrun));
        assert_eq!(out.mode, Mode::AbortLand);
    }

    #[test]
    fn terminate_is_accepted_in_every_mode() {
        for armed in [false, true] {
            let mut gnc = Gnc::new(GncConfig::default(), controller());

            if armed {
                gnc.arm(&estimate(0.0, 0.0)).unwrap();
            }

            gnc.terminate();

            let out = gnc.tick(Instant::now(), &None, EC_OK);

            assert_eq!(out.mode, Mode::Safed);
            assert_eq!(out.ec_request, Some(EcRequest::Terminate));
        }

        let (mut gnc, t0) = flying();

        gnc.tick(at(t0, 1.0), &fresh(estimate(3.0, 0.0)), EC_OK);
        gnc.terminate();

        let out = gnc.tick(at(t0, 1.01), &fresh(estimate(3.0, 0.0)), EC_OK);

        assert_eq!(out.mode, Mode::Safed);
        assert_eq!(out.ec_request, Some(EcRequest::Terminate));
        assert_eq!(out.command, Command::Safe);
    }

    #[test]
    fn terminate_requests_only_once() {
        let (mut gnc, t0) = flying();

        gnc.terminate();
        gnc.tick(at(t0, 0.1), &fresh(estimate(0.0, 0.0)), EC_OK);

        let again = gnc.tick(at(t0, 0.11), &fresh(estimate(0.0, 0.0)), EC_OK);

        assert_eq!(again.ec_request, None);
        assert_eq!(again.mode, Mode::Safed);
    }

    #[test]
    fn reset_clears_a_landing_or_a_terminate_but_not_a_flight() {
        let (mut gnc, t0) = flying();

        gnc.reset();
        assert_eq!(gnc.mode(), Mode::Flight, "not while flying");

        gnc.terminate();
        gnc.tick(at(t0, 0.1), &fresh(estimate(0.0, 0.0)), EC_OK);
        assert_eq!(gnc.mode(), Mode::Safed);

        gnc.reset();
        assert_eq!(gnc.mode(), Mode::Idle);
        assert_eq!(gnc.latched(), None);
        assert!(gnc.arm(&estimate(0.0, 0.0)).is_ok());
        assert!(gnc.start(at(t0, 1.0), EC_OK).is_ok());
    }

    #[test]
    fn disarm_returns_to_idle_and_arm_can_be_repeated() {
        let mut gnc = Gnc::new(GncConfig::default(), controller());

        gnc.arm(&estimate(0.0, 0.0)).unwrap();
        gnc.disarm().unwrap();

        assert_eq!(gnc.mode(), Mode::Idle);
        assert!(gnc.arm(&estimate(0.0, 0.0)).is_ok());
    }

    impl Gnc {
        /// Latches `fault` at R4 as the next tick would, for the tests.
        fn terminate_for_test_fault(&mut self, fault: Fault) {
            self.latched = Some((fault, Response::R4));
        }
    }
}
