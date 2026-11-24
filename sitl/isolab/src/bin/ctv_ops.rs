//! The operator of a CTV SIL run: plays what the CTV operations page does, over
//! servo's HTTP and websocket API, and judges the flight by the status the
//! flight computer reports back.
//!
//! ```text
//! ctv_ops [--servo HOST:PORT] [--via commands|sequence] [--expect landed|abort-land|safed]
//!         [--expect-fault TEXT] [--abort-at S] [--terminate-at S] [--reinit-nav]
//!         [--ready-timeout S] [--flight-timeout S]
//! ```
//!
//! It waits for the pre-flight checks to pass, arms the EC, starts the flight,
//! follows it, and prints one `OPS RESULT` line and a verdict. With
//! `--reinit-nav` it first re-initializes the navigation filter and waits for it
//! to level again and the checks to pass (`--via commands` only). Exit status 0
//! means the run ended as expected.
//!
//! `--via commands` uses `POST /operator/ctv-command`, which is what the GUI
//! buttons send. `--via sequence` saves and runs a Python sequence instead,
//! which exercises servo's sequence route, the flight computer's sequence
//! runner and the Python `Ctv` class.

use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use common::comm::{
    ctv::{CtvMode, CtvStatus},
    VehicleState,
};
use futures_util::StreamExt;
use reqwest::Client;
use serde_json::json;
use tokio::time::{sleep, timeout};
use tokio_tungstenite::{connect_async, tungstenite::Message};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Via {
    Commands,
    Sequence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Expect {
    Landed,
    AbortLand,
    Safed,
}

struct Args {
    servo: String,
    via: Via,
    expect: Expect,
    /// The latched fault must contain this, so a run that ends the right way for
    /// the wrong reason does not pass
    expect_fault: Option<String>,
    abort_at: Option<f64>,
    terminate_at: Option<f64>,
    /// Re-initialize the navigation filter before arming
    reinit_nav: bool,
    ready_timeout: Duration,
    flight_timeout: Duration,
}

fn parse_args() -> Result<Args> {
    let mut args = Args {
        servo: "127.0.0.1:7200".to_string(),
        via: Via::Commands,
        expect: Expect::Landed,
        expect_fault: None,
        abort_at: None,
        terminate_at: None,
        reinit_nav: false,
        ready_timeout: Duration::from_secs(180),
        flight_timeout: Duration::from_secs(60),
    };
    let mut it = std::env::args().skip(1);

    while let Some(flag) = it.next() {
        let mut value = |name: &str| it.next().ok_or_else(|| anyhow!("{name} needs a value"));
        let seconds = |text: String, name: &str| -> Result<f64> {
            text.parse::<f64>()
                .ok()
                .filter(|v| v.is_finite() && *v >= 0.0)
                .ok_or_else(|| anyhow!("{name} wants seconds, got {text:?}"))
        };

        match flag.as_str() {
            "--servo" => args.servo = value("--servo")?,
            "--via" => {
                args.via = match value("--via")?.as_str() {
                    "commands" => Via::Commands,
                    "sequence" => Via::Sequence,
                    other => bail!("--via is commands or sequence, not {other:?}"),
                }
            }
            "--expect" => {
                args.expect = match value("--expect")?.as_str() {
                    "landed" => Expect::Landed,
                    "abort-land" => Expect::AbortLand,
                    "safed" => Expect::Safed,
                    other => bail!("--expect is landed, abort-land or safed, not {other:?}"),
                }
            }
            "--expect-fault" => args.expect_fault = Some(value("--expect-fault")?),
            "--abort-at" => args.abort_at = Some(seconds(value("--abort-at")?, "--abort-at")?),
            "--terminate-at" => {
                args.terminate_at = Some(seconds(value("--terminate-at")?, "--terminate-at")?)
            }
            "--reinit-nav" => args.reinit_nav = true,
            "--ready-timeout" => {
                args.ready_timeout = Duration::from_secs_f64(seconds(value("--ready-timeout")?, "--ready-timeout")?)
            }
            "--flight-timeout" => {
                args.flight_timeout =
                    Duration::from_secs_f64(seconds(value("--flight-timeout")?, "--flight-timeout")?)
            }
            other => bail!("unknown flag {other:?}"),
        }
    }

    Ok(args)
}

type Socket = tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>>;

/// The next vehicle state from servo's telemetry stream.
async fn next_state(socket: &mut Socket, wait: Duration) -> Result<VehicleState> {
    let text = timeout(wait, async {
        loop {
            match socket.next().await {
                None => bail!("telemetry websocket closed"),
                Some(Err(e)) => bail!("telemetry websocket: {e}"),
                Some(Ok(Message::Text(text))) => return Ok(text),
                Some(Ok(_)) => {}
            }
        }
    })
    .await
    .map_err(|_| anyhow!("no telemetry in {wait:?}"))??;

    serde_json::from_str(&text).context("a telemetry frame did not decode as a VehicleState")
}

fn base64(bytes: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();

    for chunk in bytes.chunks(3) {
        let n = chunk.iter().enumerate().fold(0u32, |n, (i, b)| n | (u32::from(*b) << (16 - 8 * i)));

        for i in 0..4 {
            if i <= chunk.len() {
                out.push(T[((n >> (18 - 6 * i)) & 63) as usize] as char);
            } else {
                out.push('=');
            }
        }
    }

    out
}

struct Operator {
    http: Client,
    base: String,
}

impl Operator {
    async fn command(&self, name: &str) -> Result<()> {
        println!("OPS: operator command {name}");

        self.http
            .post(format!("{}/operator/ctv-command", self.base))
            .json(&json!({ "command": name }))
            .send()
            .await?
            .error_for_status()
            .with_context(|| format!("servo refused the {name} command"))?;

        Ok(())
    }

    /// Arms the EC and starts the flight with a Python sequence.
    async fn run_sequence(&self) -> Result<()> {
        println!("OPS: operator runs the sequence 'ctv_hop' (arm, wait, start flight)");

        let script = "import time\nctv = Ctv()\nctv.arm()\ntime.sleep(2.0)\nctv.start_flight()\n";

        self.http
            .put(format!("{}/operator/sequence", self.base))
            .json(&json!({ "name": "ctv_hop", "configuration_id": null, "script": base64(script.as_bytes()) }))
            .send()
            .await?
            .error_for_status()
            .context("servo refused to save the sequence")?;

        self.http
            .post(format!("{}/operator/run-sequence", self.base))
            .json(&json!({ "name": "ctv_hop", "force": true }))
            .send()
            .await?
            .error_for_status()
            .context("servo refused to run the sequence")?;

        Ok(())
    }
}

/// Reads states until `done` is true of the CTV status, or `limit` passes.
async fn until(
    socket: &mut Socket,
    limit: Duration,
    what: &str,
    mut seen: impl FnMut(&CtvStatus),
    done: impl Fn(&CtvStatus) -> bool,
) -> Result<CtvStatus> {
    let deadline = Instant::now() + limit;
    let mut last = None;
    let mut next_report = Instant::now() + Duration::from_secs(15);

    while Instant::now() + Duration::from_millis(250) < deadline {
        let state = next_state(socket, Duration::from_secs(5)).await?;

        seen(&state.ctv);

        if done(&state.ctv) {
            return Ok(state.ctv);
        }

        if Instant::now() >= next_report {
            println!("OPS: still waiting for {what}: {}", describe(&state.ctv));
            next_report = Instant::now() + Duration::from_secs(15);
        }

        last = Some(state.ctv);
    }

    bail!(
        "timed out after {limit:?} waiting for {what}; last status: {}",
        last.map_or("none".to_string(), |s| describe(&s))
    )
}

fn describe(s: &CtvStatus) -> String {
    let failing: Vec<String> = s
        .preflight
        .iter()
        .filter(|c| !c.ok)
        .map(|c| format!("{} ({})", c.name, c.detail))
        .collect();

    format!(
        "mode {:?}, EC {} {}, t {:.2} s, alt {:.2} m, fault '{}', not ready: [{}], last event '{}'",
        s.mode,
        if s.ec_connected { "connected" } else { "not connected" },
        s.ec_state,
        s.mission_time_s,
        s.altitude_m,
        s.fault,
        failing.join("; "),
        s.last_event
    )
}

#[tokio::main]
async fn main() {
    match run().await {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("OPS ERROR: {e:#}");
            println!("OPS VERDICT: FAIL ({e:#})");
            std::process::exit(2);
        }
    }
}

async fn run() -> Result<i32> {
    let args = parse_args()?;
    let base = format!("http://{}", args.servo);
    let http = Client::new();
    let operator = Operator {
        http: http.clone(),
        base: base.clone(),
    };

    // servo up
    let deadline = Instant::now() + Duration::from_secs(60);

    loop {
        match http.get(format!("{base}/operator/mappings")).send().await {
            Ok(r) if r.status().is_success() => break,
            _ if Instant::now() > deadline => bail!("servo's HTTP server did not come up at {}", args.servo),
            _ => sleep(Duration::from_millis(250)).await,
        }
    }

    let (mut socket, _) = connect_async(format!("ws://{}/data/forward", args.servo))
        .await
        .context("telemetry websocket")?;
    let started = Instant::now();
    let mut max_altitude: f64 = 0.0;
    let mut events = 0usize;
    let mut previous: Option<(CtvMode, String, String)> = None;

    let mut note = |s: &CtvStatus, started: Instant| {
        max_altitude = max_altitude.max(s.altitude_m);

        let key = (s.mode, s.ec_state.clone(), s.fault.clone());

        if previous.as_ref() != Some(&key) {
            events += 1;
            println!(
                "OPS: +{:.1}s mode {:?}, EC {}, fault '{}', mission {:.2} s",
                started.elapsed().as_secs_f64(),
                s.mode,
                if s.ec_connected { s.ec_state.as_str() } else { "not connected" },
                s.fault,
                s.mission_time_s
            );
            previous = Some(key);
        }
    };

    // 1. the pre-flight checks that do not depend on arming the EC
    println!("OPS: waiting for the pre-flight checks (the navigation filter levels on the pad first)");

    let ready = until(
        &mut socket,
        args.ready_timeout,
        "the pre-flight checks",
        |s| note(s, started),
        |s| {
            s.mode == CtvMode::Idle
                && s.preflight
                    .iter()
                    .filter(|c| c.name != "EC armed")
                    .all(|c| c.ok)
        },
    )
    .await?;

    println!("OPS: ready: {}", describe(&ready));

    if args.reinit_nav {
        if args.via != Via::Commands {
            bail!("--reinit-nav needs --via commands");
        }

        // the way back from a bad estimate: throw the filter away, level again
        operator.command("reinit_nav").await?;

        let navigation_leveling = |s: &CtvStatus| {
            s.preflight
                .iter()
                .any(|c| c.name == "Navigation estimate" && !c.ok && c.detail.to_lowercase().contains("leveling"))
        };

        until(
            &mut socket,
            Duration::from_secs(10),
            "the navigation estimate to level again after the re-initialize",
            |s| note(s, started),
            navigation_leveling,
        )
        .await?;
        println!("OPS: navigation is leveling again");

        let ready = until(
            &mut socket,
            args.ready_timeout,
            "the pre-flight checks after the re-initialize",
            |s| note(s, started),
            |s| {
                s.mode == CtvMode::Idle
                    && s.preflight
                        .iter()
                        .filter(|c| c.name != "EC armed")
                        .all(|c| c.ok)
            },
        )
        .await?;

        println!("OPS: navigation healthy again: {}", describe(&ready));
    }

    // 2. arm and start
    if args.via == Via::Commands {
        operator.command("arm_ec").await?;

        until(
            &mut socket,
            Duration::from_secs(15),
            "the EC to arm",
            |s| note(s, started),
            |s| s.ec_state.eq_ignore_ascii_case("armed"),
        )
        .await?;

        until(
            &mut socket,
            Duration::from_secs(10),
            "can_start",
            |s| note(s, started),
            |s| s.can_start,
        )
        .await?;

        operator.command("start_flight").await?;
    } else {
        operator.run_sequence().await?;
    }

    until(
        &mut socket,
        Duration::from_secs(15),
        "the flight to start",
        |s| note(s, started),
        |s| matches!(s.mode, CtvMode::Flight | CtvMode::AbortLand | CtvMode::Landed | CtvMode::Safed),
    )
    .await?;

    // 3. fly
    let mut abort_sent = args.abort_at.is_none();
    let mut terminate_sent = args.terminate_at.is_none();
    let mut final_status = None;
    let flight_deadline = Instant::now() + args.flight_timeout;

    while Instant::now() < flight_deadline {
        let state = next_state(&mut socket, Duration::from_secs(5)).await?;
        let s = state.ctv;

        note(&s, started);

        if !abort_sent && s.mission_time_s >= args.abort_at.unwrap_or(f64::MAX) {
            abort_sent = true;
            operator.command("abort_land").await?;
        }

        if !terminate_sent && s.mission_time_s >= args.terminate_at.unwrap_or(f64::MAX) {
            terminate_sent = true;
            operator.command("terminate").await?;
        }

        if matches!(s.mode, CtvMode::Landed | CtvMode::Safed) {
            // let the status settle
            sleep(Duration::from_secs(1)).await;
            final_status = Some(s);

            break;
        }
    }

    let s = final_status.ok_or_else(|| anyhow!("the flight did not end within {:?}", args.flight_timeout))?;

    println!(
        "OPS RESULT: mode={:?} fault='{}' max_altitude_m={:.2} mission_s={:.2} ec_state={} gains='{}' events={events} via={:?}",
        s.mode, s.fault, max_altitude, s.mission_time_s, s.ec_state, s.gains, args.via
    );

    let ok = match args.expect {
        Expect::Landed => s.mode == CtvMode::Landed && s.fault.is_empty(),
        Expect::AbortLand => s.mode == CtvMode::Landed && !s.fault.is_empty(),
        Expect::Safed => s.mode == CtvMode::Safed,
    } && args.expect_fault.as_ref().is_none_or(|text| s.fault.contains(text.as_str()));

    if ok {
        println!("OPS VERDICT: PASS (expected {:?})", args.expect);

        Ok(0)
    } else {
        println!("OPS VERDICT: FAIL (expected {:?}; {})", args.expect, describe(&s));

        Ok(2)
    }
}
