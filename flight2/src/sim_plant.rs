//! `--sim-plant`: the FC flies a precompiled vehicle plant while its
//! commands also drive the real engine controller (EC) on the bench.
//!
//! The plant is the hopper sim's harness (`models/hopper_6dof.slx`,
//! CONTROLLER=fsw, VEHICLE=ctv) as generated C, linked through
//! `sitl/hopper_plant`. Every 10 ms, in lockstep with the plant:
//!
//! ```text
//!   plant state x(t) --> SilNav --> Gnc (NavFrame, CtvLqrController) --> u --> Allocation --> caps --+--> EC link
//!                                                                      \--> plant
//! ```
//!
//! The plant flies the same capped command the EC is told (the capped
//! demand converted back to thrust, gimbal angles and roll torque), so a run
//! shows what the capped vehicle does. The controller's own uncapped command
//! and the capped demand are both logged, with a flag for every cap that
//! acted. The caps, thrusts, ESC map and PA-12 geometry come
//! from the bench config file (`--sim-config`, see `bench_config.rs`).
//!
//! Run phases, driven by commands typed on stdin:
//! - IDLE: EC session up but disarmed, safe demand at 100 Hz, plant parked at t
//!   = 0. `start` arms the EC and, once it reports Armed, starts t = 0.
//! - RUN: closed loop as above.
//! - LANDED: the plant's touchdown stop, the end of the schedule, or a limit.
//!   Safe demand; the plant freezes at its last state.
//! - SAFED: after `stop` (any phase): Terminate, so the EC latches safing.
//!   `reset` clears it (ClearSafing) and goes back to IDLE.
//!
//! `quit`, Ctrl-C, SIGTERM and panics all send Terminate before exiting. If
//! the FC dies without sending anything, the EC's own demand timeout safes
//! it.

use std::{
    fs,
    io::{self, BufRead, BufWriter, Write},
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, OnceLock,
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use common::comm::{
    ctv::{ActuatorDemand, ControlVector},
    ec_link::EcState,
};
use hopper_plant::{Plant, State, FSW_PERIOD, INFO_PATH, PLANT_GIT_COMMIT};
use nalgebra::Vector3;

use crate::{
    bench_config::{BenchConfig, CAP_FAN, CAP_THROTTLE, CAP_TVC},
    ctv_controller::CtvLqrController,
    ec_link::{EcLinkHandle, Request},
    gain_manifest,
    gnc::{Command, EcInput, EcRequest, Event, Gnc, GncConfig, Mode},
    sil::{FaultPlan, SilNav, Tracker},
};

/// How long to wait for the EC to report Armed after `start`
const ARM_TIMEOUT: Duration = Duration::from_secs(2);
/// How long to wait for the EC to report Safing after Terminate
const SAFE_TIMEOUT: Duration = Duration::from_secs(1);
/// Seconds past the end of the gain schedule before the run counts as over
const SCHEDULE_MARGIN: f64 = 5.0;
/// Horizontal or vertical distance from the pad that ends a run, m
const POSITION_LIMIT_M: f64 = 25.0;

/// Command-line options for `--sim-plant`.
#[derive(Debug, Clone)]
pub(crate) struct SimPlantConfig {
    /// export_plant_gains output (tgrid, xref, k1flat, k2grid, unom .csv)
    pub gains: PathBuf,
    /// Bench config JSON (caps, thrusts, ESC map, PA-12 geometry;
    /// bench_config.rs); defaults when absent
    pub config: Option<PathBuf>,
    /// Per-tick CSV log; default `sim_plant_<unix time>.csv`
    pub log: Option<PathBuf>,
    /// Controller the gains must have been exported for (`gains.json`)
    pub gains_controller: String,
    /// Accept gains or a plant from a modified (`-dirty`) hopper tree
    pub allow_dirty: bool,
    /// Start by itself, fly one hop, print the score and exit
    pub auto: bool,
    /// The navigation frame's heading relative to the schedule's, degrees
    pub heading_offset_deg: f64,
    /// Where the navigation frame puts the pad, NED, m
    pub origin_offset: [f64; 3],
    /// The vehicle reference point relative to the IMU, body axes, m
    pub imu_lever: [f64; 3],
    /// Give the plant the roll torque even though no fan is driven
    pub rcs: bool,
    /// Faults to inject, by mission time
    pub faults: Vec<crate::sil::SilFault>,
    /// How the run is meant to end
    pub expect: crate::sil::Expect,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    #[default]
    Idle,
    Arming(Instant),
    Run,
    Landed,
    Stopping(Instant),
    Safed,
}

impl Phase {
    fn name(&self) -> &'static str {
        match self {
            Phase::Idle => "IDLE",
            Phase::Arming(_) => "ARMING",
            Phase::Run => "RUN",
            Phase::Landed => "LANDED",
            Phase::Stopping(_) => "STOPPING",
            Phase::Safed => "SAFED",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Op {
    Start,
    Stop,
    Reset,
    Status,
    Quit,
}

static EC: OnceLock<Arc<EcLinkHandle>> = OnceLock::new();

/// Runs the bench test until `quit`. Never returns.
pub(crate) fn run(config: SimPlantConfig, ec: Option<EcLinkHandle>) -> ! {
    match run_inner(config, ec) {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("sim-plant: {e}");
            terminate_ec();
            std::process::exit(1);
        }
    }
}

fn run_inner(config: SimPlantConfig, ec: Option<EcLinkHandle>) -> Result<i32, String> {
    let ec = Arc::new(ec.ok_or("the EC link didn't start; nothing to drive")?);
    let _ = EC.set(ec.clone());
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        terminate_ec();
        default_hook(info);
    }));
    let quit = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, quit.clone())
            .map_err(|e| format!("signal handler: {e}"))?;
    }

    // The plant first: a build without one should say so, not complain about
    // gains it has nothing to compare with
    let mut plant = Plant::new().map_err(|e| e.to_string())?;
    let policy = gain_manifest::Policy {
        controller: &config.gains_controller,
        plant_commit: Some(PLANT_GIT_COMMIT),
        allow_dirty: config.allow_dirty,
    };
    let manifest = gain_manifest::verify(&config.gains, &policy)?;
    let gains = Gains::load(&config.gains, &policy)?;
    let t_end = gains.t_end + SCHEDULE_MARGIN;
    let controller = gains.controller;
    let lever = Vector3::from(config.imu_lever);
    let gnc_config = GncConfig {
        lever_body: lever,
        ..GncConfig::default()
    };
    // What the navigation filter would hand over: truth in its own frame
    let sil_nav = SilNav::new(
        config.heading_offset_deg,
        Vector3::from(config.origin_offset),
        lever,
    );
    let mut gnc = Gnc::new(gnc_config, controller.clone());
    // The flight logic runs on the plant's clock, not the wall clock, so a
    // run does not depend on how late the host schedules a tick. The EC link
    // and the demand timeouts stay on real time.
    let sim_epoch = Instant::now();
    let sim_now = |plant_time: f64| sim_epoch + Duration::from_secs_f64(plant_time.max(0.0));
    let mut tracker = Tracker::default();
    let mut plan = FaultPlan::new(config.faults.clone());
    let mut exit_code = 0;
    let mut auto_started = false;
    let bench = BenchConfig::load(config.config.as_deref())?;
    let caps = bench.caps;
    let allocation = bench.allocation();
    let (center, stroke) = allocation.tvc_center_and_stroke_mm();
    let safe = allocation.safe();
    let log_path = config.log.unwrap_or_else(|| {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        PathBuf::from(format!("sim_plant_{}.csv", now.as_secs()))
    });
    let log = Log::open(&log_path)?;

    println!("sim-plant: plant {INFO_PATH}");
    println!(
        "sim-plant: gains {} (schedule to {:.2} s), controller {}, hopper commit {}",
        config.gains.display(),
        gains.t_end,
        manifest.controller,
        manifest.git_commit
    );
    match &config.config {
        Some(p) => println!("sim-plant: bench config {}", p.display()),
        None => println!("sim-plant: bench config: defaults (no --sim-config)"),
    }
    for line in bench.describe() {
        println!("sim-plant:   {line}");
    }
    for axis in bench.tvc_short_axes() {
        println!(
            "sim-plant: WARNING: the {axis} actuator's stroke doesn't reach +-{} deg;              commands past it are clamped by the geometry",
            bench.tvc.max_angle_deg
        );
    }
    println!(
        "sim-plant: nav frame: heading offset {} deg, pad at {:?} m, IMU lever {:?} m; roll torque {}",
        config.heading_offset_deg,
        config.origin_offset,
        config.imu_lever,
        if config.rcs {
            "given to the plant"
        } else {
            "NOT given to the plant (the FC drives no fans)"
        }
    );
    println!("sim-plant: log   {}", log_path.display());
    println!("sim-plant: commands: start | stop | reset | status | quit");
    let ops = spawn_stdin_reader();

    let period = Duration::from_secs_f64(FSW_PERIOD);
    let mut phase = Phase::Idle;
    let mut deadline = Instant::now() + period;
    let mut last_x: Option<State> = None;
    let mut tick: u64 = 0;
    let mut stats = RunStats::default();
    // A run is open from RUN until its summary prints, even if it starts and
    // ends in the same tick
    let mut run_open = false;
    println!("sim-plant: {} (EC disarmed, safe outputs)", phase.name());

    loop {
        let start = Instant::now();
        let late_us = start
            .saturating_duration_since(deadline - period)
            .as_micros() as i64;
        let snapshot = ec.snapshot();
        let ec_state = snapshot.status.map(|s| s.ec_state);

        let mut op = ops.try_recv().ok();
        if quit.load(Ordering::Relaxed) {
            op = Some(Op::Quit);
        }
        // --sim-auto: start once the EC is up, quit when the run is over
        if config.auto && op.is_none() {
            if phase == Phase::Idle && !auto_started && snapshot.connected && tick >= 100 {
                auto_started = true;
                op = Some(Op::Start);
            } else if auto_started && matches!(phase, Phase::Landed | Phase::Safed) {
                op = Some(Op::Quit);
            }
        }
        let next = match (phase, op) {
            (_, Some(Op::Quit)) => {
                if run_open {
                    stats.print();
                }
                println!("sim-plant: quitting; terminating the EC");
                let _ = ec.request(Request::Terminate);
                wait_for(&ec, EcState::Safing, SAFE_TIMEOUT);
                log.close();
                return Ok(exit_code);
            }
            (_, Some(Op::Status)) => {
                println!(
                    "sim-plant: {} t = {:.2} s, EC connected {} state {:?}",
                    phase.name(),
                    plant.time(),
                    snapshot.connected,
                    ec_state
                );
                phase
            }
            (Phase::Idle, Some(Op::Start)) if !snapshot.connected => {
                println!("sim-plant: no EC session yet; not starting");
                phase
            }
            (Phase::Idle, Some(Op::Start)) => match ec.request(Request::Arm) {
                Ok(()) => Phase::Arming(start),
                Err(e) => {
                    println!("sim-plant: arm request failed: {e}");
                    phase
                }
            },
            (Phase::Safed, Some(Op::Reset)) => {
                let _ = ec.request(Request::ClearSafing);
                plant.reset().map_err(|e| e.to_string())?;
                last_x = None;
                Phase::Idle
            }
            (Phase::Safed, Some(_)) => {
                println!("sim-plant: safed; `reset` first");
                phase
            }
            (Phase::Stopping(_) | Phase::Safed, None) => phase,
            (_, Some(Op::Stop)) => {
                let _ = ec.request(Request::Terminate);
                Phase::Stopping(start)
            }
            (_, Some(op)) => {
                println!("sim-plant: `{op:?}` does nothing in {}", phase.name());
                phase
            }
            (_, None) => phase,
        };
        let was_running = phase == Phase::Run;
        phase = advance(next, ec_state, start, &mut plant)?;
        if phase == Phase::Run && !was_running {
            stats = RunStats::default();
            tracker = Tracker::default();
            plan = FaultPlan::new(config.faults.clone());
            run_open = true;
            ec.take_max_demand_gap(); // measure this run's sends only

            // The flight logic takes the pad pose from the estimate at t = 0
            // and starts its mission clock with the EC armed
            gnc = Gnc::new(gnc_config, controller.clone());
            let pad = plant.state().map_err(|e| e.to_string())?;
            let started = gnc
                .arm(&sil_nav.estimate(plant.time(), &pad))
                .and_then(|()| gnc.start(sim_now(plant.time()), ec_input(snapshot.connected, ec_state)));
            if let Err(why) = started {
                println!("sim-plant: the flight logic refused to start: {why}; IDLE");
                let _ = ec.request(Request::Disarm);
                phase = Phase::Idle;
                run_open = false;
                exit_code = 2;
            }
        }

        let mut row = Row {
            tick,
            phase,
            late_us,
            ec_connected: snapshot.connected,
            ec_state,
            max_demand_gap_us: snapshot.max_demand_gap.as_micros() as u32,
            ..Row::default()
        };
        let demand = if phase == Phase::Run {
            let x = plant.state().map_err(|e| e.to_string())?;
            let t0 = Instant::now();
            let estimate = sil_nav.estimate(plant.time(), &x);
            let mut nav = Some((estimate, Duration::ZERO));
            let manual = plan.apply(plant.time(), &mut nav);
            if manual.abort {
                println!("sim-plant: t = {:.2} s: operator abort to land", plant.time());
                gnc.abort_land();
            }
            if manual.terminate {
                println!("sim-plant: t = {:.2} s: operator terminate", plant.time());
                gnc.terminate();
            }
            let out = gnc.tick(sim_now(plant.time()), &nav, ec_input(snapshot.connected, ec_state));
            row.ctrl_us = t0.elapsed().as_micros() as u32;
            row.gnc_mode = Some(out.mode);
            for event in &out.events {
                println!("sim-plant: t = {:.2} s: {event}", plant.time());
                match event {
                    Event::Latched { fault, .. } => tracker.faults.push(*fault),
                    Event::EngineCut { altitude_m } => tracker.cut_altitude_m = Some(*altitude_m),
                    _ => {}
                }
            }
            if let Some(request) = out.ec_request {
                let result = ec.request(match request {
                    EcRequest::Disarm => Request::Disarm,
                    EcRequest::Terminate => Request::Terminate,
                });
                if let Err(e) = result {
                    println!("sim-plant: EC {request:?} request failed: {e}");
                }
            }
            let (wanted, sent, acted) = match out.command {
                Command::Control(u) => {
                    row.u = Some(u);
                    let wanted = allocation.demand(&u);
                    let (sent, acted) = caps.apply(&wanted, center, stroke);
                    (wanted, sent, acted)
                }
                Command::Safe => (safe, safe, 0),
            };
            // The plant flies what the EC is told, caps included, so a run
            // shows what the capped vehicle does. The FC drives no fans, so
            // the plant gets no roll torque unless asked.
            let mut flown = allocation.command_for(&sent);
            if !config.rcs {
                flown.rcs_torque = 0.0;
            }
            let t1 = Instant::now();
            let period_end =
                plant.run_period(&[flown.thrust, flown.tvc_pitch, flown.tvc_yaw, flown.rcs_torque]);
            row.plant_us = t1.elapsed().as_micros() as u32;
            let period_end = period_end.map_err(|e| e.to_string())?;
            row.t = Some(plant.time());
            row.x = Some(x);
            row.wanted = Some(wanted);
            row.capped = acted;
            stats.add(&row, period);
            tracker.add(&x, plant.time());
            last_x = Some(x);
            if let Some(why) = run_over(&period_end, &x, t_end) {
                println!(
                    "sim-plant: LANDED at t = {:.2} s ({why}); safe outputs, plant frozen",
                    plant.time()
                );
                tracker.plant_touchdown = period_end.touchdown;
                tracker.final_mode = Some(gnc.mode());
                phase = Phase::Landed;
            }
            sent
        } else {
            row.t = Some(plant.time());
            row.x = last_x;
            safe
        };
        if let Err(reason) = ec.set_demand(&demand) {
            println!("sim-plant: EC link rejected the demand ({reason:?}); sending safe");
            let _ = ec.set_demand(&safe);
        }
        if run_open && phase != Phase::Run {
            stats.print();
            run_open = false;
            if tracker.ticks > 0 {
                tracker.final_mode.get_or_insert(gnc.mode());
                for line in tracker.report(config.expect) {
                    println!("sim-plant: {line}");
                }
                if config.auto {
                    exit_code = i32::from(!tracker.failures_for(config.expect).is_empty()) * 2;
                }
            }
        }
        row.sent = Some(demand);
        row.applied = snapshot.status.map(|s| s.applied);
        row.safing_reason = snapshot.status.map(|s| format!("{:?}", s.safing_reason));
        log.write(&row);

        tick += 1;
        let now = Instant::now();
        if now < deadline {
            thread::sleep(deadline - now);
        } else {
            // overran: count it, don't try to catch up
            deadline = now;
        }
        deadline += period;
    }
}

/// Transitions that follow from the EC's reported state and the clock
fn advance(
    phase: Phase,
    ec: Option<EcState>,
    now: Instant,
    plant: &mut Plant,
) -> Result<Phase, String> {
    Ok(match phase {
        Phase::Arming(_) if ec == Some(EcState::Armed) => {
            plant.reset().map_err(|e| e.to_string())?;
            println!("sim-plant: EC armed; RUN, t = 0");
            Phase::Run
        }
        Phase::Arming(since) if now - since > ARM_TIMEOUT => {
            println!(
                "sim-plant: EC didn't report Armed within {ARM_TIMEOUT:?} (state {ec:?}); IDLE"
            );
            Phase::Idle
        }
        // In RUN the flight logic sees the EC leave Armed and responds (R4);
        // the run then ends on the plant's own events, so it does not depend on
        // how soon the EC's status reaches us
        Phase::Landed if ec != Some(EcState::Armed) => {
            println!("sim-plant: EC left Armed (state {ec:?}); SAFED, `reset` to clear");
            Phase::Safed
        }
        Phase::Stopping(_) if ec == Some(EcState::Safing) => {
            println!("sim-plant: EC safed; SAFED, `reset` to clear");
            Phase::Safed
        }
        Phase::Stopping(since) if now - since > SAFE_TIMEOUT => {
            println!("sim-plant: EC didn't report Safing within {SAFE_TIMEOUT:?} (state {ec:?}); SAFED anyway");
            Phase::Safed
        }
        other => other,
    })
}

/// Why the run is over, if it is
fn run_over(period: &hopper_plant::Period, x: &State, t_end: f64) -> Option<&'static str> {
    if period.touchdown {
        Some("touchdown")
    } else if period.time > t_end {
        Some("end of the gain schedule")
    } else if x.iter().any(|v| !v.is_finite()) {
        Some("non-finite plant state")
    } else if x[..3].iter().any(|p| p.abs() > POSITION_LIMIT_M) {
        Some("position limit")
    } else {
        None
    }
}

fn wait_for(ec: &EcLinkHandle, state: EcState, timeout: Duration) {
    let until = Instant::now() + timeout;
    while Instant::now() < until {
        if ec.snapshot().status.map(|s| s.ec_state) == Some(state) {
            return;
        }
        thread::sleep(Duration::from_millis(5));
    }
    eprintln!("sim-plant: EC didn't report {state:?} within {timeout:?}");
}

fn terminate_ec() {
    if let Some(ec) = EC.get() {
        let _ = ec.request(Request::Terminate);
        // give the link thread a few send periods
        thread::sleep(Duration::from_millis(100));
    }
}

fn spawn_stdin_reader() -> mpsc::Receiver<Op> {
    let (tx, rx) = mpsc::channel();
    thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            let Ok(line) = line else { break };
            let op = match line.trim() {
                "start" => Op::Start,
                "stop" => Op::Stop,
                "reset" => Op::Reset,
                "status" => Op::Status,
                "quit" | "exit" => Op::Quit,
                "" => continue,
                other => {
                    println!("sim-plant: unknown command `{other}` (start | stop | reset | status | quit)");
                    continue;
                }
            };
            if tx.send(op).is_err() {
                break;
            }
        }
    });
    rx
}

struct Gains {
    controller: CtvLqrController,
    t_end: f64,
}

impl Gains {
    /// The gain set in `dir`, once `gains.json` has been checked against
    /// `policy`
    fn load(dir: &Path, policy: &gain_manifest::Policy) -> Result<Gains, String> {
        let controller = CtvLqrController::from_gain_dir(dir, policy).map_err(|e| e.to_string())?;

        Ok(Gains {
            t_end: controller.schedule_end(),
            controller,
        })
    }
}

/// What the EC reports, in the flight logic's terms
///
/// `connected` is the link's own flag: after the link drops the last Status is
/// still there, with the state it had, so it says nothing about the EC now.
fn ec_input(connected: bool, state: Option<EcState>) -> EcInput {
    EcInput {
        connected,
        armed: connected && state == Some(EcState::Armed),
        safing: connected && state == Some(EcState::Safing),
    }
}

/// What a run did, printed when it leaves RUN
#[derive(Debug, Default)]
struct RunStats {
    ticks: u64,
    throttle_capped: u64,
    fan_capped: u64,
    tvc_capped: u64,
    max_late_us: i64,
    overruns: u64,
    max_ctrl_us: u32,
    max_plant_us: u32,
    max_demand_gap_us: u32,
}

impl RunStats {
    fn add(&mut self, r: &Row, period: Duration) {
        self.ticks += 1;
        self.throttle_capped += u64::from(r.capped & CAP_THROTTLE != 0);
        self.fan_capped += u64::from(r.capped & CAP_FAN != 0);
        self.tvc_capped += u64::from(r.capped & CAP_TVC != 0);
        self.max_late_us = self.max_late_us.max(r.late_us);
        self.overruns += u64::from(r.late_us > period.as_micros() as i64);
        self.max_ctrl_us = self.max_ctrl_us.max(r.ctrl_us);
        self.max_plant_us = self.max_plant_us.max(r.plant_us);
        self.max_demand_gap_us = self.max_demand_gap_us.max(r.max_demand_gap_us);
    }

    fn summary(&self) -> Vec<String> {
        let pct = |n: u64| 100.0 * n as f64 / self.ticks.max(1) as f64;
        vec![
            format!(
                "run summary: {} ticks; caps acted on throttle {} ({:.0}%), fans {} ({:.0}%), TVC {} ({:.0}%)",
                self.ticks,
                self.throttle_capped,
                pct(self.throttle_capped),
                self.fan_capped,
                pct(self.fan_capped),
                self.tvc_capped,
                pct(self.tvc_capped)
            ),
            format!(
                "run summary: tick lateness max {} us, {} overruns; controller max {} us, plant max {} us",
                self.max_late_us, self.overruns, self.max_ctrl_us, self.max_plant_us
            ),
            format!(
                "run summary: longest gap between demand sends {:.1} ms (EC demand timeout 30 ms)",
                f64::from(self.max_demand_gap_us) / 1000.0
            ),
        ]
    }

    fn print(&self) {
        for line in self.summary() {
            println!("sim-plant: {line}");
        }
    }
}

/// One CSV row per tick
#[derive(Default)]
struct Row {
    tick: u64,
    phase: Phase,
    late_us: i64,
    ctrl_us: u32,
    plant_us: u32,
    t: Option<f64>,
    x: Option<State>,
    gnc_mode: Option<Mode>,
    u: Option<ControlVector>,
    wanted: Option<ActuatorDemand>,
    sent: Option<ActuatorDemand>,
    capped: u8,
    ec_connected: bool,
    ec_state: Option<EcState>,
    safing_reason: Option<String>,
    applied: Option<common::comm::ec_link::Outputs>,
    /// Longest gap between demand sends so far this run
    max_demand_gap_us: u32,
}

const HEADER: &str = "tick,wall_s,phase,late_us,ctrl_us,plant_us,t,\
pN,pE,pD,vN,vE,vD,p,q,r,q0,q1,q2,q3,\
u_thrust,u_tvc_pitch,u_tvc_yaw,u_rcs,\
want_tvc1_mm,want_tvc2_mm,want_fan1,want_fan2,want_fan3,want_fan4,want_throttle,\
sent_tvc1_mm,sent_tvc2_mm,sent_fan1,sent_fan2,sent_fan3,sent_fan4,sent_throttle,\
cap_throttle,cap_fan,cap_tvc,ec_connected,ec_state,safing_reason,\
applied_tvc1_mm,applied_tvc2_mm,applied_fan1,applied_fan2,applied_fan3,applied_fan4,applied_throttle,max_demand_gap_us,gnc_mode";

struct Log {
    tx: Option<mpsc::Sender<String>>,
    writer: Option<thread::JoinHandle<()>>,
    epoch: Instant,
}

impl Log {
    fn open(path: &Path) -> Result<Log, String> {
        let file = fs::File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let (tx, rx) = mpsc::channel::<String>();
        let writer = thread::spawn(move || {
            let mut w = BufWriter::new(file);
            let _ = writeln!(w, "{HEADER}");
            for line in rx {
                let _ = writeln!(w, "{line}");
            }
            let _ = w.flush();
        });
        Ok(Log {
            tx: Some(tx),
            writer: Some(writer),
            epoch: Instant::now(),
        })
    }

    fn write(&self, r: &Row) {
        let mut s = format!(
            "{},{:.6},{},{},{},{},",
            r.tick,
            self.epoch.elapsed().as_secs_f64(),
            r.phase.name(),
            r.late_us,
            r.ctrl_us,
            r.plant_us
        );
        let mut push = |v: Option<f64>| {
            if let Some(v) = v {
                s.push_str(&format!("{v:.9}"));
            }
            s.push(',');
        };
        push(r.t);
        for i in 0..13 {
            push(r.x.map(|x| x[i]));
        }
        push(r.u.map(|u| u.thrust));
        push(r.u.map(|u| u.tvc_pitch));
        push(r.u.map(|u| u.tvc_yaw));
        push(r.u.map(|u| u.rcs_torque));
        for d in [r.wanted, r.sent] {
            for i in 0..2 {
                push(d.map(|d| d.tvc_extension_mm[i]));
            }
            for i in 0..4 {
                push(d.map(|d| d.rcs_duty[i]));
            }
            push(d.map(|d| d.main_throttle));
        }
        s.push_str(&format!(
            "{},{},{},{},{:?},{},",
            (r.capped & CAP_THROTTLE != 0) as u8,
            (r.capped & CAP_FAN != 0) as u8,
            (r.capped & CAP_TVC != 0) as u8,
            r.ec_connected as u8,
            r.ec_state,
            r.safing_reason.as_deref().unwrap_or("")
        ));
        let a = r.applied;
        let fields: Vec<String> = (0..2)
            .map(|i| a.map(|a| f64::from(a.tvc_extension_mm[i])))
            .chain((0..4).map(|i| a.map(|a| f64::from(a.rcs_duty[i]))))
            .chain(std::iter::once(a.map(|a| f64::from(a.main_throttle))))
            .map(|v| v.map(|v| format!("{v:.6}")).unwrap_or_default())
            .collect();
        s.push_str(&fields.join(","));
        s.push_str(&format!(",{}", r.max_demand_gap_us));
        s.push_str(&format!(",{}", r.gnc_mode.map_or(String::new(), |m| format!("{m:?}"))));
        if let Some(tx) = &self.tx {
            let _ = tx.send(s);
        }
    }

    fn close(mut self) {
        self.tx.take();
        if let Some(w) = self.writer.take() {
            let _ = w.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn run_summary_counts_cap_hits() {
        let mut stats = RunStats::default();
        let period = Duration::from_millis(10);
        for (capped, late_us, gap) in [
            (CAP_THROTTLE, 100, 5_000),
            (CAP_THROTTLE | CAP_FAN, 20_000, 12_300),
            (0, 50, 7_000),
        ] {
            stats.add(
                &Row {
                    capped,
                    late_us,
                    max_demand_gap_us: gap,
                    ..Row::default()
                },
                period,
            );
        }
        let s = stats.summary();
        assert!(s[0].contains("3 ticks"), "{s:?}");
        assert!(s[0].contains("throttle 2 (67%)"), "{s:?}");
        assert!(s[0].contains("fans 1 (33%)"), "{s:?}");
        assert!(s[1].contains("1 overruns"), "{s:?}");
        assert!(s[2].contains("12.3 ms"), "{s:?}");
    }

    #[test]
    fn landed_on_touchdown_or_limits() {
        let x = [0.0; 13];
        let p = |time, touchdown| hopper_plant::Period { time, touchdown };
        assert_eq!(run_over(&p(3.0, true), &x, 26.0), Some("touchdown"));
        assert_eq!(
            run_over(&p(27.0, false), &x, 26.0),
            Some("end of the gain schedule")
        );
        let mut far = x;
        far[0] = 30.0;
        assert_eq!(run_over(&p(3.0, false), &far, 26.0), Some("position limit"));
        assert_eq!(run_over(&p(3.0, false), &x, 26.0), None);
    }
}
