//! Bench modes for the engine controller, with nothing else running (no
//! navigation, Servo, sequences or plant):
//!
//! - `--ec-check`: read-only. Opens a session, prints the EC's Status and
//!   exits; never arms and never sends a demand.
//! - `--ec-bench`: drives the TVC actuators (mm) and the main ESC (throttle)
//!   from commands typed on stdin. Only the channels under test are enabled in
//!   the demand; the EC holds the rest safe. Type `help` for the commands.
//!
//! Limits here are the FC's own; the EC clamps again to its bench profile.

use std::{
    io::{self, BufRead},
    process,
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};

use common::comm::ec_link::{channel, Outputs, TVC_STROKE_MM};

use crate::ec_link::{EcLinkHandle, LinkSnapshot, Request};

/// Highest throttle the bench accepts; the EC's profile caps it again.
pub(crate) const THROTTLE_CAP: f32 = 1.0;
/// Largest single `jog` step, so a typo can't swing the gimbal.
pub(crate) const MAX_JOG_MM: f32 = 2.0;
const TVC: u16 = channel::TVC_A1 | channel::TVC_A2;
const TICK: Duration = Duration::from_millis(20);
const WATCH_PERIOD: Duration = Duration::from_secs(1);
const CHECK_WAIT: Duration = Duration::from_secs(3);

const HELP: &str = "\
commands:
  arm | disarm | clear | term      EC transactions (clear = ClearSafing, term = Terminate)
  tvc <a1_mm> [a2_mm]              TVC extension, 0..26.9 mm (a2 = a1 if left out)
  jog a1|a2 <mm> | jog <a1> <a2>   move TVC by a small step (max 2 mm), starting
                                   from the measured position the first time
  pos                              commanded vs measured TVC position (for trimming)
  throttle <0..1>                  main ESC throttle
  stop                             throttle 0, ESC still enabled
  release tvc|esc|all              stop driving those channels (EC holds them safe)
  status | watch on|off            print EC status now / every second
  quit                             Terminate, then exit";

/// One typed command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Command {
    Request(Request),
    Tvc(f32, f32),
    Jog(f32, f32),
    Pos,
    Throttle(f32),
    Release(u16),
    Status,
    Watch(bool),
    Help,
    Quit,
}

pub(crate) fn parse(line: &str) -> Result<Command, String> {
    let words: Vec<&str> = line.split_whitespace().collect();
    let number = |s: &str| -> Result<f32, String> {
        s.parse::<f32>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| format!("'{s}' isn't a number"))
    };
    match words.as_slice() {
        ["arm"] => Ok(Command::Request(Request::Arm)),
        ["disarm"] => Ok(Command::Request(Request::Disarm)),
        ["clear"] => Ok(Command::Request(Request::ClearSafing)),
        ["term"] => Ok(Command::Request(Request::Terminate)),
        ["tvc", a1] => {
            let a1 = number(a1)?;
            Ok(Command::Tvc(a1, a1))
        }
        ["tvc", a1, a2] => Ok(Command::Tvc(number(a1)?, number(a2)?)),
        ["jog", "a1", d] => Ok(Command::Jog(number(d)?, 0.0)),
        ["jog", "a2", d] => Ok(Command::Jog(0.0, number(d)?)),
        ["jog", d1, d2] => Ok(Command::Jog(number(d1)?, number(d2)?)),
        ["pos"] => Ok(Command::Pos),
        ["throttle", t] => Ok(Command::Throttle(number(t)?)),
        ["stop"] => Ok(Command::Throttle(0.0)),
        ["release", "tvc"] => Ok(Command::Release(TVC)),
        ["release", "esc"] => Ok(Command::Release(channel::MAIN_ESC)),
        ["release", "all"] => Ok(Command::Release(channel::ALL)),
        ["status"] => Ok(Command::Status),
        ["watch", "on"] => Ok(Command::Watch(true)),
        ["watch", "off"] => Ok(Command::Watch(false)),
        ["help"] | [] => Ok(Command::Help),
        ["quit"] | ["exit"] => Ok(Command::Quit),
        _ => Err(format!("unknown command '{}'; type help", line.trim())),
    }
}

/// What the bench is asking for: values plus the channels under test.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct Bench {
    pub outputs: Outputs,
    pub mask: u16,
}

impl Bench {
    /// Applies an output command, clamped to the bench limits. Returns a note
    /// when a value was clamped.
    pub(crate) fn apply(&mut self, command: Command) -> Option<String> {
        match command {
            Command::Tvc(a1, a2) => {
                let clamp = |v: f32| v.clamp(0.0, TVC_STROKE_MM);
                self.outputs.tvc_extension_mm = [clamp(a1), clamp(a2)];
                self.mask |= TVC;
                (clamp(a1) != a1 || clamp(a2) != a2)
                    .then(|| format!("TVC clamped to 0..{TVC_STROKE_MM} mm"))
            }
            Command::Throttle(t) => {
                let clamped = t.clamp(0.0, THROTTLE_CAP);
                self.outputs.main_throttle = clamped;
                self.mask |= channel::MAIN_ESC;
                (clamped != t).then(|| format!("throttle clamped to 0..{THROTTLE_CAP}"))
            }
            Command::Release(bits) => {
                self.mask &= !bits;
                if bits & channel::MAIN_ESC != 0 {
                    self.outputs.main_throttle = 0.0;
                }
                None
            }
            _ => None,
        }
    }

    /// Moves the TVC by `d1`/`d2` mm. The first jog starts from `measured`
    /// (the EC's valid positions), never from 0; without it the jog is
    /// refused. Steps are limited to [`MAX_JOG_MM`].
    pub(crate) fn jog(
        &mut self,
        d1: f32,
        d2: f32,
        measured: Option<[f32; 2]>,
    ) -> Result<Option<String>, String> {
        if d1.abs() > MAX_JOG_MM || d2.abs() > MAX_JOG_MM {
            return Err(format!("jog steps are at most {MAX_JOG_MM} mm"));
        }
        let start = if self.mask & TVC == TVC {
            self.outputs.tvc_extension_mm
        } else {
            measured.ok_or("no valid measured TVC position yet; use `tvc <mm>` first")?
        };
        Ok(self.apply(Command::Tvc(start[0] + d1, start[1] + d2)))
    }
}

/// Both TVC positions, if the EC reports them valid.
pub(crate) fn measured_tvc(snapshot: &LinkSnapshot) -> Option<[f32; 2]> {
    let m = snapshot.status?.measurements;
    (m.validity_mask & 0b11 == 0b11).then_some(m.tvc_position_mm)
}

/// One line for trimming: commanded vs measured per actuator.
pub(crate) fn format_pos(bench: &Bench, snapshot: &LinkSnapshot) -> String {
    let Some(s) = snapshot.status else {
        return "no Status yet".into();
    };
    let m = &s.measurements;
    let side = |i: usize| {
        let valid = m.validity_mask & (1 << i) != 0;
        format!(
            "A{}: cmd {:.2} mm, measured {:.2} mm ({} counts){}",
            i + 1,
            bench.outputs.tvc_extension_mm[i],
            m.tvc_position_mm[i],
            m.tvc_position_counts[i],
            on_off(valid)
        )
    };
    format!("{} | {}", side(0), side(1))
}

fn on_off(valid: bool) -> &'static str {
    if valid {
        ""
    } else {
        " (invalid)"
    }
}

/// The operator's view of the newest Status.
pub(crate) fn format_status(snapshot: &LinkSnapshot) -> String {
    let Some(s) = snapshot.status else {
        return format!(
            "EC: {} , no Status yet",
            if snapshot.connected {
                "connected"
            } else {
                "not connected"
            }
        );
    };
    let m = &s.measurements;
    let pos_valid = |i: usize| m.validity_mask & (1 << i) != 0;
    let not_ready = (s.driver_faults >> 8) & 0x7F;
    format!(
        "EC {:?}{}{} | authority {} | safing reason {:?} | last reject {:?}\n\
         demands: accepted {} applied {} (fresh {}) | limited {:#04x}\n\
         drivers: faulted {:#04x} not ready {:#04x} interlock open {}\n\
         applied: TVC {:.2} / {:.2} mm, throttle {:.3}\n\
         measured: A1 {:.2} mm ({} counts, {} ms old, feedback {}, fault {:#x}){}\n\
         \x20         A2 {:.2} mm ({} counts, {} ms old, feedback {}, fault {:#x}){}\n\
         link: crc errors {} rejected {} stale {} deadline misses {}",
        s.ec_state,
        if s.session_valid { "" } else { " (no session)" },
        if snapshot.connected {
            ""
        } else {
            " [link down]"
        },
        s.output_authority,
        s.safing_reason,
        s.last_reject_reason,
        s.last_accepted_demand_seq,
        s.last_applied_demand_seq,
        s.demand_fresh,
        s.limited_mask,
        s.driver_faults & 0x7F,
        not_ready,
        s.driver_faults & (1 << 16) != 0,
        s.applied.tvc_extension_mm[0],
        s.applied.tvc_extension_mm[1],
        s.applied.main_throttle,
        m.tvc_position_mm[0],
        m.tvc_position_counts[0],
        m.tvc_position_age_ms[0],
        m.pa12_feedback_state[0],
        m.pa12_fault_flags[0],
        on_off(pos_valid(0)),
        m.tvc_position_mm[1],
        m.tvc_position_counts[1],
        m.tvc_position_age_ms[1],
        m.pa12_feedback_state[1],
        m.pa12_fault_flags[1],
        on_off(pos_valid(1)),
        s.counters.crc_errors,
        s.counters.rejected,
        s.counters.stale,
        s.counters.deadline_miss,
    )
}

fn require(link: Option<EcLinkHandle>) -> EcLinkHandle {
    link.unwrap_or_else(|| {
        eprintln!("No EC link, nothing to do.");
        process::exit(1);
    })
}

/// `--ec-check`: waits for a session and a Status, prints it, exits 0 (1 if
/// none arrived). Sends no demand and no transaction.
pub(crate) fn check(link: Option<EcLinkHandle>) -> ! {
    let link = require(link);
    let start = Instant::now();
    while start.elapsed() < CHECK_WAIT {
        let snapshot = link.snapshot();
        if snapshot.connected && snapshot.status.is_some() {
            thread::sleep(Duration::from_millis(100)); // let a fresh Status
                                                       // land
            println!("{}", format_status(&link.snapshot()));
            process::exit(0);
        }
        thread::sleep(TICK);
    }
    println!("{}", format_status(&link.snapshot()));
    eprintln!("No Status from the EC within {CHECK_WAIT:?}.");
    process::exit(1);
}

/// `--ec-bench`: typed commands drive TVC and throttle; see [`HELP`].
pub(crate) fn run(link: Option<EcLinkHandle>) -> ! {
    let link = require(link);
    println!("EC bench. Throttle cap {THROTTLE_CAP}, TVC 0..{TVC_STROKE_MM} mm.\n{HELP}");

    let (tx, rx) = mpsc::channel::<String>();
    thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            match line {
                Ok(line) => {
                    if tx.send(line).is_err() {
                        return;
                    }
                }
                Err(_) => break,
            }
        }
        let _ = tx.send("quit".into()); // stdin closed: shut down safely
    });

    let mut bench = Bench::default();
    let mut watch = false;
    let mut last_watch = Instant::now();
    loop {
        while let Ok(line) = rx.try_recv() {
            match parse(&line) {
                Ok(Command::Request(request)) => {
                    if let Err(e) = link.request(request) {
                        println!("{request:?} not queued: {e}");
                    }
                }
                Ok(Command::Status) => println!("{}", format_status(&link.snapshot())),
                Ok(Command::Pos) => println!("{}", format_pos(&bench, &link.snapshot())),
                Ok(Command::Jog(d1, d2)) => {
                    match bench.jog(d1, d2, measured_tvc(&link.snapshot())) {
                        Ok(note) => {
                            if let Some(note) = note {
                                println!("{note}");
                            }
                            println!(
                                "TVC -> {:.2} / {:.2} mm (type `pos` once it has moved)",
                                bench.outputs.tvc_extension_mm[0],
                                bench.outputs.tvc_extension_mm[1]
                            );
                        }
                        Err(e) => println!("{e}"),
                    }
                }
                Ok(Command::Watch(on)) => watch = on,
                Ok(Command::Help) => println!("{HELP}"),
                Ok(Command::Quit) => {
                    let _ = link.request(Request::Terminate);
                    thread::sleep(Duration::from_millis(300));
                    println!("{}", format_status(&link.snapshot()));
                    process::exit(0);
                }
                Ok(command) => {
                    if let Some(note) = bench.apply(command) {
                        println!("{note}");
                    }
                    println!(
                        "demand: TVC {:.2} / {:.2} mm, throttle {:.3}, channels {:#04x}",
                        bench.outputs.tvc_extension_mm[0],
                        bench.outputs.tvc_extension_mm[1],
                        bench.outputs.main_throttle,
                        bench.mask
                    );
                }
                Err(e) => println!("{e}"),
            }
        }

        // Resent every tick so it stays fresh; the EC times out without it.
        if let Err(reason) = link.set_demand_masked(bench.outputs, bench.mask) {
            println!("demand not sent: {reason:?}");
        }
        if watch && last_watch.elapsed() >= WATCH_PERIOD {
            last_watch = Instant::now();
            println!("{}", format_status(&link.snapshot()));
        }
        thread::sleep(TICK);
    }
}

#[cfg(test)]
mod tests {
    use common::comm::ec_link::Status;

    use super::*;

    #[test]
    fn parses_commands() {
        assert_eq!(parse("arm"), Ok(Command::Request(Request::Arm)));
        assert_eq!(parse(" term "), Ok(Command::Request(Request::Terminate)));
        assert_eq!(parse("tvc 12.5"), Ok(Command::Tvc(12.5, 12.5)));
        assert_eq!(parse("tvc 10 14"), Ok(Command::Tvc(10.0, 14.0)));
        assert_eq!(parse("throttle 0.2"), Ok(Command::Throttle(0.2)));
        assert_eq!(parse("stop"), Ok(Command::Throttle(0.0)));
        assert_eq!(
            parse("release esc"),
            Ok(Command::Release(channel::MAIN_ESC))
        );
        assert_eq!(parse("watch on"), Ok(Command::Watch(true)));
        assert!(parse("throttle NaN").is_err());
        assert!(parse("throttle inf").is_err());
        assert!(parse("tvc").is_err());
        assert!(parse("fly").is_err());
    }

    #[test]
    fn only_channels_under_test_are_enabled_and_values_are_clamped() {
        let mut bench = Bench::default();
        assert_eq!(bench.mask, 0);

        assert!(bench.apply(Command::Throttle(1.3)).is_some());
        assert_eq!(bench.outputs.main_throttle, THROTTLE_CAP);
        assert!(bench.apply(Command::Throttle(0.9)).is_none());
        assert_eq!(bench.mask, channel::MAIN_ESC);

        assert!(bench.apply(Command::Tvc(-1.0, 30.0)).is_some());
        assert_eq!(bench.outputs.tvc_extension_mm, [0.0, TVC_STROKE_MM]);
        assert_eq!(bench.mask, channel::MAIN_ESC | TVC);
        assert!(bench.apply(Command::Tvc(12.0, 13.0)).is_none());

        bench.apply(Command::Release(channel::MAIN_ESC));
        assert_eq!(bench.mask, TVC);
        assert_eq!(bench.outputs.main_throttle, 0.0);
        // Never enables an RCS fan.
        assert_eq!(bench.mask & !(TVC | channel::MAIN_ESC), 0);
    }

    #[test]
    fn jog_starts_from_the_measured_position_and_is_limited() {
        assert_eq!(parse("jog a1 0.5"), Ok(Command::Jog(0.5, 0.0)));
        assert_eq!(parse("jog a2 -0.2"), Ok(Command::Jog(0.0, -0.2)));
        assert_eq!(parse("jog 0.1 0.3"), Ok(Command::Jog(0.1, 0.3)));
        assert_eq!(parse("pos"), Ok(Command::Pos));

        let mut bench = Bench::default();
        // No measurement and no TVC command yet: refused, nothing enabled.
        assert!(bench.jog(0.5, 0.0, None).is_err());
        assert_eq!(bench.mask, 0);

        // First jog starts from where the actuators are, not from 0 mm.
        assert!(bench.jog(0.5, 0.0, Some([13.0, 14.0])).is_ok());
        assert_eq!(bench.outputs.tvc_extension_mm, [13.5, 14.0]);
        assert_eq!(bench.mask, TVC);

        // Later jogs build on the commanded value.
        bench.jog(0.0, -1.0, Some([99.0, 99.0])).unwrap();
        assert_eq!(bench.outputs.tvc_extension_mm, [13.5, 13.0]);

        assert!(bench.jog(2.5, 0.0, None).is_err());
        assert_eq!(bench.outputs.tvc_extension_mm, [13.5, 13.0]);
    }

    #[test]
    fn status_without_a_session_says_so() {
        let text = format_status(&LinkSnapshot::default());
        assert!(text.contains("not connected"));
        let snap = LinkSnapshot {
            connected: true,
            status: Some(Status::default()),
            max_demand_gap: Duration::ZERO,
        };
        assert!(format_status(&snap).contains("authority false"));
    }
}
