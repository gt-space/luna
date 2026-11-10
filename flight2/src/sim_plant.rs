//! `--sim-plant`: the FC flies a precompiled vehicle plant while its
//! commands also drive the real engine controller (EC) on the bench.
//!
//! The plant is the hopper sim's harness (`models/hopper_6dof.slx`,
//! CONTROLLER=fsw, VEHICLE=ctv) as generated C, linked through
//! `sitl/hopper_plant`. Every 10 ms, in lockstep with the plant:
//!
//! ```text
//!   plant state x(t) --> LqrController --> u --> plant (uncapped)
//!                                          \--> Allocation --> caps --> EC link
//! ```
//!
//! The plant flies the controller's own command so the flight stays
//! nominal; the EC gets the capped version. Both are logged, with a flag for
//! every cap that acted.
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
    ctv::{ActuatorDemand, ControlState, ControlVector},
    ec_link::EcState,
};
use hopper_plant::{Plant, State, FSW_PERIOD, INFO_PATH};
use nalgebra::{Const, Dyn, Matrix, MatrixXx1, MatrixXx4, Quaternion, VecStorage, Vector3};
use serde::Deserialize;

use crate::{
    control::{lqr::LqrController, Controller},
    ctv_allocation::Allocation,
    ec_link::{EcLinkHandle, Request},
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
    /// Caps JSON; defaults when absent
    pub caps: Option<PathBuf>,
    /// Per-tick CSV log; default `sim_plant_<unix time>.csv`
    pub log: Option<PathBuf>,
}

/// Limits on what reaches the EC. The plant never sees them.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct Caps {
    /// Max main throttle, 0..1; None = uncapped
    pub throttle: Option<f64>,
    /// Max duty per RCS fan, 0..1
    pub fan_duty: f64,
    /// Max gimbal actuator travel from center, as a fraction of the stroke
    /// left on that side, 0..1. ASSUMPTION: what "70%" means for the TVC.
    pub tvc_travel: f64,
}

impl Default for Caps {
    fn default() -> Self {
        Caps {
            throttle: None,
            fan_duty: 0.7,
            tvc_travel: 0.7,
        }
    }
}

/// Which caps acted on a demand (bits)
const CAP_THROTTLE: u8 = 1;
const CAP_FAN: u8 = 2;
const CAP_TVC: u8 = 4;

impl Caps {
    fn load(path: Option<&Path>) -> Result<Caps, String> {
        let caps = match path {
            None => Caps::default(),
            Some(p) => {
                let text = fs::read_to_string(p).map_err(|e| format!("{}: {e}", p.display()))?;
                serde_json::from_str(&text).map_err(|e| format!("{}: {e}", p.display()))?
            }
        };
        let fraction = |v: f64| (0.0..=1.0).contains(&v);
        if !(caps.throttle.is_none_or(fraction)
            && fraction(caps.fan_duty)
            && fraction(caps.tvc_travel))
        {
            return Err(format!("caps must be within 0..1: {caps:?}"));
        }
        Ok(caps)
    }

    fn describe(&self) -> String {
        let throttle = match self.throttle {
            None => "uncapped".to_string(),
            Some(t) => format!("{:.0}% of full", t * 100.0),
        };
        format!(
            "throttle: {throttle}; fans: {:.0}% duty; TVC: {:.0}% of travel from center (ASSUMPTION)",
            self.fan_duty * 100.0,
            self.tvc_travel * 100.0
        )
    }

    /// The demand the EC gets, and which caps acted
    fn apply(
        &self,
        d: &ActuatorDemand,
        center_mm: [f64; 2],
        stroke_mm: [f64; 2],
    ) -> (ActuatorDemand, u8) {
        let mut out = *d;
        let mut acted = 0;
        if let Some(max) = self.throttle {
            if out.main_throttle > max {
                out.main_throttle = max;
                acted |= CAP_THROTTLE;
            }
        }
        for duty in &mut out.rcs_duty {
            if *duty > self.fan_duty {
                *duty = self.fan_duty;
                acted |= CAP_FAN;
            }
        }
        for i in 0..2 {
            let lo = center_mm[i] - self.tvc_travel * center_mm[i];
            let hi = center_mm[i] + self.tvc_travel * (stroke_mm[i] - center_mm[i]);
            let e = out.tvc_extension_mm[i];
            if e < lo || e > hi {
                out.tvc_extension_mm[i] = e.clamp(lo, hi);
                acted |= CAP_TVC;
            }
        }
        (out, acted)
    }
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
        Ok(()) => std::process::exit(0),
        Err(e) => {
            eprintln!("sim-plant: {e}");
            terminate_ec();
            std::process::exit(1);
        }
    }
}

fn run_inner(config: SimPlantConfig, ec: Option<EcLinkHandle>) -> Result<(), String> {
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

    let gains = Gains::load(&config.gains)?;
    let t_end = gains.t_end + SCHEDULE_MARGIN;
    let mut controller = gains.controller;
    let caps = Caps::load(config.caps.as_deref())?;
    let allocation = Allocation::CTV_SIM;
    let (center, stroke) = allocation.tvc_center_and_stroke_mm();
    let safe = allocation.safe();
    let mut plant = Plant::new().map_err(|e| e.to_string())?;
    let log_path = config.log.unwrap_or_else(|| {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        PathBuf::from(format!("sim_plant_{}.csv", now.as_secs()))
    });
    let log = Log::open(&log_path)?;

    println!("sim-plant: plant {INFO_PATH}");
    println!(
        "sim-plant: gains {} (schedule to {:.2} s)",
        config.gains.display(),
        gains.t_end
    );
    println!("sim-plant: caps  {}", caps.describe());
    println!("sim-plant: log   {}", log_path.display());
    println!("sim-plant: commands: start | stop | reset | status | quit");
    let ops = spawn_stdin_reader();

    let period = Duration::from_secs_f64(FSW_PERIOD);
    let mut phase = Phase::Idle;
    let mut deadline = Instant::now() + period;
    let mut last_x: Option<State> = None;
    let mut tick: u64 = 0;
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
        let next = match (phase, op) {
            (_, Some(Op::Quit)) => {
                println!("sim-plant: quitting; terminating the EC");
                let _ = ec.request(Request::Terminate);
                wait_for(&ec, EcState::Safing, SAFE_TIMEOUT);
                log.close();
                return Ok(());
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
        phase = advance(next, ec_state, start, &mut plant)?;

        let mut row = Row {
            tick,
            phase,
            late_us,
            ec_connected: snapshot.connected,
            ec_state,
            ..Row::default()
        };
        let demand = if phase == Phase::Run {
            let x = plant.state().map_err(|e| e.to_string())?;
            let t0 = Instant::now();
            let u = controller.step(control_state(plant.time(), &x));
            row.ctrl_us = t0.elapsed().as_micros() as u32;
            let wanted = allocation.demand(&u);
            let (sent, acted) = caps.apply(&wanted, center, stroke);
            let t1 = Instant::now();
            let period_end = plant.run_period(&[u.thrust, u.tvc_pitch, u.tvc_yaw, u.rcs_torque]);
            row.plant_us = t1.elapsed().as_micros() as u32;
            let period_end = period_end.map_err(|e| e.to_string())?;
            row.t = Some(plant.time());
            row.x = Some(x);
            row.u = Some(u);
            row.wanted = Some(wanted);
            row.capped = acted;
            last_x = Some(x);
            if let Some(why) = run_over(&period_end, &x, t_end) {
                println!(
                    "sim-plant: LANDED at t = {:.2} s ({why}); safe outputs, plant frozen",
                    plant.time()
                );
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
        Phase::Run | Phase::Landed if ec != Some(EcState::Armed) => {
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

fn control_state(t: f64, x: &State) -> ControlState {
    ControlState {
        time: Duration::from_secs_f64(t.max(0.0)),
        position: Vector3::new(x[0], x[1], x[2]),
        velocity: Vector3::new(x[3], x[4], x[5]),
        body_rate: Vector3::new(x[6], x[7], x[8]),
        attitude: Quaternion::new(x[9], x[10], x[11], x[12]),
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
    controller: LqrController,
    t_end: f64,
}

impl Gains {
    fn load(dir: &Path) -> Result<Gains, String> {
        let tgrid: MatrixXx1<f64> = MatrixXx1::from_column_slice(&column(dir, "tgrid")?);
        let n = tgrid.nrows();
        let xref = Matrix::<f64, Dyn, Const<13>, VecStorage<f64, Dyn, Const<13>>>::from_row_slice(
            &table(dir, "xref", 13, n)?,
        );
        let k1flat = Matrix::<f64, Dyn, Const<52>, VecStorage<f64, Dyn, Const<52>>>::from_row_slice(
            &table(dir, "k1flat", 52, n)?,
        );
        let k2grid = MatrixXx4::from_row_slice(&table(dir, "k2grid", 4, n)?);
        let unom = MatrixXx4::from_row_slice(&table(dir, "unom", 4, n)?);
        Ok(Gains {
            t_end: tgrid[n - 1],
            controller: LqrController::new(tgrid, xref, k1flat, k2grid, unom),
        })
    }
}

fn column(dir: &Path, name: &str) -> Result<Vec<f64>, String> {
    let rows = rows(dir, name)?;
    Ok(rows.into_iter().map(|r| r[0]).collect())
}

fn table(dir: &Path, name: &str, cols: usize, n: usize) -> Result<Vec<f64>, String> {
    let rows = rows(dir, name)?;
    if rows.len() != n || rows.iter().any(|r| r.len() != cols) {
        return Err(format!("{name}.csv should be {n} x {cols}"));
    }
    Ok(rows.concat())
}

fn rows(dir: &Path, name: &str) -> Result<Vec<Vec<f64>>, String> {
    let path = dir.join(format!("{name}.csv"));
    let text = fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    text.lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            l.split(',')
                .map(|v| {
                    v.trim()
                        .parse::<f64>()
                        .map_err(|e| format!("{}: {e}", path.display()))
                })
                .collect()
        })
        .collect()
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
    u: Option<ControlVector>,
    wanted: Option<ActuatorDemand>,
    sent: Option<ActuatorDemand>,
    capped: u8,
    ec_connected: bool,
    ec_state: Option<EcState>,
    safing_reason: Option<String>,
    applied: Option<common::comm::ec_link::Outputs>,
}

const HEADER: &str = "tick,wall_s,phase,late_us,ctrl_us,plant_us,t,\
pN,pE,pD,vN,vE,vD,p,q,r,q0,q1,q2,q3,\
u_thrust,u_tvc_pitch,u_tvc_yaw,u_rcs,\
want_tvc1_mm,want_tvc2_mm,want_fan1,want_fan2,want_fan3,want_fan4,want_throttle,\
sent_tvc1_mm,sent_tvc2_mm,sent_fan1,sent_fan2,sent_fan3,sent_fan4,sent_throttle,\
cap_throttle,cap_fan,cap_tvc,ec_connected,ec_state,safing_reason,\
applied_tvc1_mm,applied_tvc2_mm,applied_fan1,applied_fan2,applied_fan3,applied_fan4,applied_throttle";

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

    fn demand(tvc: [f64; 2], fans: [f64; 4], throttle: f64) -> ActuatorDemand {
        ActuatorDemand {
            tvc_extension_mm: tvc,
            rcs_duty: fans,
            main_throttle: throttle,
        }
    }

    #[test]
    fn default_caps_leave_throttle_alone() {
        let (out, acted) = Caps::default().apply(
            &demand([13.45, 13.45], [0.0; 4], 0.95),
            [13.45; 2],
            [26.9; 2],
        );
        assert_eq!(out.main_throttle, 0.95);
        assert_eq!(acted, 0);
        assert!(Caps::default().describe().contains("throttle: uncapped"));
    }

    #[test]
    fn fans_and_tvc_capped_at_70_percent() {
        let caps = Caps::default();
        // center 10 mm on a 26.9 mm stroke: 7 mm below, 11.83 mm above allowed
        let (out, acted) = caps.apply(
            &demand([0.0, 26.9], [0.9, 0.0, 0.9, 0.5], 0.5),
            [10.0; 2],
            [26.9; 2],
        );
        assert_eq!(out.rcs_duty, [0.7, 0.0, 0.7, 0.5]);
        assert!((out.tvc_extension_mm[0] - 3.0).abs() < 1e-12);
        assert!((out.tvc_extension_mm[1] - (10.0 + 0.7 * 16.9)).abs() < 1e-12);
        assert_eq!(acted, CAP_FAN | CAP_TVC);
    }

    #[test]
    fn throttle_cap_from_json() {
        let caps: Caps = serde_json::from_str(r#"{"throttle": 0.7}"#).unwrap();
        let (out, acted) = caps.apply(&demand([5.0; 2], [0.0; 4], 0.9), [5.0; 2], [10.0; 2]);
        assert_eq!(out.main_throttle, 0.7);
        assert_eq!(acted, CAP_THROTTLE);
        assert_eq!(caps.fan_duty, 0.7);
    }

    #[test]
    fn unknown_or_out_of_range_caps_are_rejected() {
        assert!(serde_json::from_str::<Caps>(r#"{"thrust": 0.5}"#).is_err());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("caps.json");
        fs::write(&path, r#"{"fan_duty": 1.5}"#).unwrap();
        assert!(Caps::load(Some(&path)).is_err());
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
