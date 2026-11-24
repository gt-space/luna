//! `--sil-bridge`: the vehicle of the multi-process SIL run.
//!
//! The flight computer runs as itself (its main loop, the servo link, the
//! sequences, the GMEKF, the flight logic) and the engine controller runs as
//! `sil_ec`. This process is everything outside them: the generated hopper
//! plant, flown in real time at its own 1 ms step, and the sensors.
//!
//! ```text
//!   sil_ec --(applied outputs, UDP)--> bridge --(sensor samples, UDP)--> flight computer
//!   what the EC really drives          the plant                         --sil-feed
//! ```
//!
//! The plant flies on what the EC applied (the ESC pulse on the wire and the
//! actuators' real positions, through the vehicle's allocation), not on what
//! the flight computer asked for, so the EC's limits, its actuator lag and any
//! fault in it are in the flight.
//!
//! Sensors come from hopper's own Sensors block, built into the plant: its
//! noise, rates, valid flags and fault gates. The bridge sends a sample when a
//! sensor's stamp changes, as a driver reports a new reading.
//!
//! The plant has no ground: a plant stepped with no thrust falls through the
//! pad. So the vehicle waits on the pad until the EC applies throttle, which is
//! the flight logic's first demand, and until then the sensors read the vehicle
//! at rest, from the same model parameters the plant uses. That is when the
//! navigation filter levels. At release the plant starts at t = 0, which is the
//! schedule's t = 0, and flies until touchdown.

use std::{
    fs,
    io::{BufWriter, Write},
    net::{SocketAddr, UdpSocket},
    path::{Path, PathBuf},
    thread,
    time::{Duration, Instant},
};

use common::comm::{ctv::ActuatorDemand, GpsState};
use hopper_plant::{layout, Plant, Sensors, State, INFO_PATH, STEP};
use serde_json::Value;

use crate::{
    bench_config::BenchConfig,
    ctv_allocation::Allocation,
    sil::Tracker,
    sil_feed::SilFeedSender,
    sil_sensors::{pressure_at, SensorConfig, SensorModel, SilSample, Stamped},
};

/// Options for `--sil-bridge`.
#[derive(Debug, Clone)]
pub(crate) struct BridgeConfig {
    /// The flight computer's `--sil-feed` address.
    pub feed_to: SocketAddr,
    /// The UDP port `sil_ec --applied-udp` sends to.
    pub applied_port: u16,
    /// The vehicle config (thrust at full throttle, TVC geometry), the same
    /// file the flight computer gets as `--gnc-vehicle-config`.
    pub vehicle_config: Option<PathBuf>,
    /// The per-tick CSV of the flight.
    pub log: Option<PathBuf>,
    /// Throttle above which the vehicle leaves the pad.
    pub release_throttle: f64,
    /// Give up if the flight lasts this long, s.
    pub max_flight_s: f64,
    /// Give up if nothing releases the vehicle in this long, s.
    pub max_pad_s: f64,
    /// What the plant flies on.
    pub command_source: CommandSource,
    /// A UDP port to take the flight computer's RCS fan duties from
    /// (`--sil-roll-to`). Without it the plant gets no roll torque, as the
    /// vehicle gets none: the EC link never enables the fans.
    pub roll_port: Option<u16>,
    /// After the flight is over, keep sending the sensors of the vehicle at
    /// rest until the process is stopped, instead of exiting. For flying by hand
    /// from the GUI: the flight computer's estimate stays healthy and the pages
    /// keep working. The result is still printed once.
    pub keep_running: bool,
}

/// What the plant flies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CommandSource {
    /// What the EC applied: the ESC pulse on the wire and the actuators' real
    /// positions.
    Applied,
    /// What the EC asked of its drivers: the driver's throttle and the actuator
    /// goals, without the ESC's frame and the actuators' travel time. For
    /// telling the EC's own lag from the rest of the loop's.
    Goals,
}

impl std::str::FromStr for CommandSource {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, String> {
        match text {
            "applied" => Ok(CommandSource::Applied),
            "goals" => Ok(CommandSource::Goals),
            other => Err(format!("{other:?} is not applied or goals")),
        }
    }
}

/// What the EC says it is driving: "A <t_ms> <STATE> <esc_us> <a1_mm> <a2_mm>".
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Applied {
    pub armed: bool,
    pub esc_us: f64,
    pub tvc_mm: [f64; 2],
    /// The actuator goals and the ESC driver's throttle, if the EC sends them
    pub goals: Option<([f64; 2], f64)>,
}

impl Applied {
    pub(crate) fn parse(line: &str) -> Option<Applied> {
        let mut fields = line.split_whitespace();

        if fields.next()? != "A" {
            return None;
        }

        let _t_ms = fields.next()?;
        let state = fields.next()?;
        let number = |field: Option<&str>| field?.parse::<f64>().ok().filter(|v| v.is_finite());
        let esc_us = number(fields.next())?;
        let a1 = number(fields.next())?;
        let a2 = number(fields.next())?;

        let goals = (|| {
            let g1 = number(fields.next())?;
            let g2 = number(fields.next())?;
            let throttle = number(fields.next())?;

            Some(([g1, g2], throttle))
        })();

        Some(Applied {
            armed: state == "ARMED",
            esc_us,
            tvc_mm: [a1, a2],
            goals,
        })
    }

    /// Throttle from the ESC pulse on the wire.
    pub(crate) fn throttle(&self, min_us: f64, max_us: f64) -> f64 {
        ((self.esc_us - min_us) / (max_us - min_us)).clamp(0.0, 1.0)
    }

    /// The plant's command: thrust from the throttle, gimbal angles from where
    /// the actuators are. The RCS fans are not driven by the EC, so the roll
    /// torque is only what `rcs_duty` asks for, which comes from the flight
    /// computer directly and only when it was started with `--sil-roll-to`.
    pub(crate) fn plant_command(
        &self,
        allocation: &Allocation,
        min_us: f64,
        max_us: f64,
        source: CommandSource,
        rcs_duty: [f64; 4],
    ) -> [f64; 4] {
        let (tvc_extension_mm, main_throttle) = match (source, self.goals) {
            (CommandSource::Goals, Some((goals, throttle))) => (goals, throttle.clamp(0.0, 1.0)),
            _ => (self.tvc_mm, self.throttle(min_us, max_us)),
        };
        let command = allocation.command_for(&ActuatorDemand {
            tvc_extension_mm,
            rcs_duty,
            main_throttle,
        });

        [
            command.thrust,
            command.tvc_pitch,
            command.tvc_yaw,
            command.rcs_torque,
        ]
    }
}

/// The RCS fan duties of a roll datagram: `R d1 d2 d3 d4`, each 0 to 1. The
/// flight computer's `--sil-roll-to` sends them.
pub(crate) fn parse_rcs_duty(line: &str) -> Option<[f64; 4]> {
    let mut fields = line.split_whitespace();

    if fields.next()? != "R" {
        return None;
    }

    let mut duty = [0.0; 4];

    for d in &mut duty {
        *d = fields
            .next()?
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())?
            .clamp(0.0, 1.0);
    }

    Some(duty)
}

/// Which sensor samples have been sent, to send each once.
#[derive(Debug, Clone, Default)]
pub(crate) struct Sent {
    stamp: [Option<u64>; 9],
    fixes: i64,
}

/// The samples to send for the Sensors block's outputs now: one per sensor
/// whose stamp changed and whose reading is valid, converted to the flight
/// computer's units.
pub(crate) fn convert(sensors: &Sensors, sent: &mut Sent, site: &SensorConfig) -> Vec<SilSample> {
    let m = &sensors.meas;
    let take = |r: std::ops::Range<usize>| [m[r.start], m[r.start + 1], m[r.start + 2]];
    let mut out = Vec::new();

    if fresh(sent, sensors, layout::S_IMU) {
        out.push(SilSample::Imu {
            accel: take(layout::ACCEL),
            gyro_dps: take(layout::GYRO).map(f64::to_degrees),
        });
    }

    if fresh(sent, sensors, layout::S_BARO) {
        out.push(SilSample::Baro {
            pressure_pa: pressure_at(site.pad_altitude_m + m[layout::BARO]),
            temperature_c: site.temperature_c,
        });
    }

    if fresh(sent, sensors, layout::S_MAG) {
        // micro-tesla to Gauss
        out.push(SilSample::Mag {
            gauss: take(layout::MAG).map(|ut| ut / 100.0),
        });
    }

    for (beam, sensor) in layout::S_LIDAR.enumerate() {
        if fresh(sent, sensors, sensor) {
            out.push(SilSample::Lidar {
                id: beam as u8 + 1,
                range_m: m[layout::LIDAR.start + beam],
            });
        }
    }

    if fresh(sent, sensors, layout::S_GPS) {
        let [north, east, down] = take(layout::GPS_POS);
        let [vn, ve, vd] = take(layout::GPS_VEL);
        let latitude = site.pad_latitude_deg + north / METERS_PER_DEGREE;

        sent.fixes += 1;

        out.push(SilSample::Gps(GpsState {
            latitude_deg: latitude,
            longitude_deg: site.pad_longitude_deg
                + east / (METERS_PER_DEGREE * latitude.to_radians().cos()),
            altitude_m: site.pad_altitude_m - down,
            north_mps: vn,
            east_mps: ve,
            down_mps: vd,
            timestamp_unix_ms: Some(1_700_000_000_000 + sent.fixes * 200),
            has_fix: m[layout::GPS_FIX] >= 3.0,
            num_satellites: m[layout::GPS_SATS].clamp(0.0, 255.0) as u8,
        }));
    }

    out
}

/// Whether `sensor` has a new valid sample: its stamp changed since the last
/// time, and its reading is valid. Notes the stamp either way.
fn fresh(sent: &mut Sent, sensors: &Sensors, sensor: usize) -> bool {
    let bits = sensors.stamp[sensor].to_bits();
    let new = sent.stamp[sensor] != Some(bits);

    sent.stamp[sensor] = Some(bits);

    new && sensors.valid[sensor]
}

/// Meters per degree of latitude, as in `sil_sensors`.
const METERS_PER_DEGREE: f64 = 111_320.0;

/// A plant with a ground: the vehicle is down once it has been higher than this, m ...
const ON_THE_GROUND_AFTER_M: f64 = 0.5;
/// ... and is lower than this, m, and slower than this, m/s ...
const ON_THE_GROUND_M: f64 = 0.05;
const ON_THE_GROUND_SPEED_MPS: f64 = 0.05;

/// How long the sensors keep reading a vehicle at rest after the plant's touchdown
/// stop before the run is over, s: long enough for the flight logic to see it down.
const REST_S: f64 = 2.0;

/// The CTV's centre of mass above its base, m: `cg_height` in `plant.json`
/// (older plants do not carry it). Hopper's sensor geometry (antenna, LiDAR
/// mounts) is from the base; its sensor models work from the centre of mass,
/// where they put the IMU.
pub(crate) const CTV_CG_HEIGHT_M: f64 = 0.349_826_344_169_466_1;

/// The sensor settings of the plant that was built, so the vehicle at rest on
/// the pad reads what the plant would read.
pub(crate) fn site_from_plant(info: &Value) -> SensorConfig {
    let params = &info["sensor_params"];
    let number = |v: &Value, default: f64| v.as_f64().unwrap_or(default);
    let first = |v: &Value, default: f64| v.get(0).and_then(Value::as_f64).unwrap_or(default);
    let mut site = SensorConfig::default();

    let imu_period = number(&params["imu"]["period"], 0.002);

    site.imu_hz = 1.0 / imu_period;
    site.baro_hz = 1.0 / number(&params["baro"]["period"], 0.05);
    site.mag_hz = 1.0 / number(&params["mag"]["period"], 0.02);
    site.gps_hz = 1.0 / number(&params["gps"]["period"], 0.2);
    site.accel_std = first(&params["imu"]["accel_sigma"], site.accel_std);
    site.gyro_std_dps =
        first(&params["imu"]["gyro_sigma"], site.gyro_std_dps.to_radians()).to_degrees();
    site.baro_alt_std_m = number(&params["baro"]["sigma"], site.baro_alt_std_m);
    site.mag_std_gauss = number(&params["mag"]["sigma"], site.mag_std_gauss * 100.0) / 100.0;
    site.gps_position_std = [0, 1, 2].map(|i| {
        params["gps"]["sigma"]
            .get(i)
            .and_then(Value::as_f64)
            .unwrap_or(site.gps_position_std[i])
    });
    site.gps_velocity_std = first(&params["gps"]["vel_sigma"], site.gps_velocity_std);
    // hopper's receiver has a fix from the start
    site.gps_fix_after_s = 0.0;

    // and its antenna sits off the vehicle's axis. The plant gives it from the
    // base; its sensor models take it from the centre of mass, which is where
    // they also put the IMU, so from the CG is from the IMU here too.
    if let Some(antenna) = params["gps"]["antenna"].as_array() {
        let component = |row: usize| {
            antenna
                .get(row)
                .and_then(|r| r.get(0))
                .and_then(Value::as_f64)
                .unwrap_or(0.0)
        };

        let com = info["cg_height"].as_f64().unwrap_or(CTV_CG_HEIGHT_M);

        site.gps_antenna_body = [component(0) - com, component(1), component(2)];
        // and the plant's GPS is against the CG's real position, which is the
        // controller's position less this: the pad must report the same
        site.gps_position_offset = [0.0, 0.0, -com];
    }

    if let Some(field) = params["mag"]["field_ned"].as_array() {
        if field.len() == 3 {
            site.mag_field_ned = [0, 1, 2].map(|i| field[i].as_f64().unwrap_or(0.0) / 100.0);
        }
    }

    site
}

fn pace(next: &mut Instant) {
    *next += Duration::from_secs_f64(STEP);

    let now = Instant::now();

    if *next > now {
        let wait = *next - now;

        if wait > Duration::from_micros(300) {
            thread::sleep(wait - Duration::from_micros(200));
        }

        while Instant::now() < *next {
            std::hint::spin_loop();
        }
    } else if now - *next > Duration::from_millis(50) {
        // far behind (a stalled host): do not try to catch up
        *next = now;
    }
}

/// Runs the vehicle. Never returns.
pub(crate) fn run(config: BridgeConfig) -> ! {
    match run_inner(&config) {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("sil-bridge: {e}");
            std::process::exit(1);
        }
    }
}

struct Log {
    out: BufWriter<fs::File>,
}

impl Log {
    fn open(path: &Path) -> Result<Log, String> {
        let file = fs::File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut out = BufWriter::new(file);

        writeln!(
            out,
            "bridge_t,phase,plant_t,pN,pE,pD,vN,vE,vD,p,q,r,q0,q1,q2,q3,applied_throttle,a1_mm,a2_mm,ec_armed"
        )
        .map_err(|e| e.to_string())?;

        Ok(Log { out })
    }

    fn row(
        &mut self,
        bridge_t: f64,
        phase: &str,
        plant_t: f64,
        x: &State,
        applied: Option<&Applied>,
        thr: f64,
    ) {
        let xs: Vec<String> = x.iter().map(|v| format!("{v:.6}")).collect();
        let _ = writeln!(
            self.out,
            "{bridge_t:.4},{phase},{plant_t:.4},{},{thr:.4},{:.3},{:.3},{}",
            xs.join(","),
            applied.map_or(0.0, |a| a.tvc_mm[0]),
            applied.map_or(0.0, |a| a.tvc_mm[1]),
            u8::from(applied.is_some_and(|a| a.armed)),
        );
    }
}

fn run_inner(config: &BridgeConfig) -> Result<i32, String> {
    let info_text = fs::read_to_string(INFO_PATH).map_err(|e| format!("{INFO_PATH}: {e}"))?;
    let info: Value = serde_json::from_str(&info_text).map_err(|e| format!("{INFO_PATH}: {e}"))?;
    let site = site_from_plant(&info);
    let vehicle = BenchConfig::load(config.vehicle_config.as_deref())?;
    let allocation = vehicle.allocation();
    let (min_us, max_us) = (1000.0, 2000.0);

    let mut plant = Plant::new().map_err(|e| e.to_string())?;

    if plant.sensors().map_err(|e| e.to_string())?.is_none() {
        return Err(
            "this plant has no sensor outputs; rebuild it with hopper's sensor outports"
                .to_string(),
        );
    }

    let applied_socket = UdpSocket::bind(("0.0.0.0", config.applied_port))
        .map_err(|e| format!("applied port {}: {e}", config.applied_port))?;

    applied_socket
        .set_nonblocking(true)
        .map_err(|e| e.to_string())?;

    // SIL only: the roll torque the flight computer asks for, which the EC would
    // not carry to the fans
    let roll_socket = config
        .roll_port
        .map(|port| {
            let socket =
                UdpSocket::bind(("0.0.0.0", port)).map_err(|e| format!("roll port {port}: {e}"))?;

            socket.set_nonblocking(true).map_err(|e| e.to_string())?;

            Ok::<_, String>(socket)
        })
        .transpose()?;
    let feed = SilFeedSender::new(config.feed_to).map_err(|e| e.to_string())?;
    let mut log = match &config.log {
        Some(path) => Some(Log::open(path)?),
        None => None,
    };

    // The vehicle at rest on the pad, as the plant starts
    let pad = plant.state().map_err(|e| e.to_string())?;
    let mut pad_sensors = SensorModel::new(site.clone());

    println!("sil-bridge: plant {INFO_PATH}");
    println!(
        "sil-bridge: sensors at {:.0} Hz IMU, {:.0} Hz baro, {:.0} Hz magnetometer, {:.0} Hz GPS",
        site.imu_hz, site.baro_hz, site.mag_hz, site.gps_hz
    );
    println!(
        "sil-bridge: sending to the flight computer at {}, listening for the EC on UDP {}",
        config.feed_to, config.applied_port
    );
    if let Some(port) = config.roll_port {
        println!(
            "sil-bridge: the RCS fans follow the flight computer's duties on UDP {port}              (a SIL convenience: the EC does not drive them)"
        );
    }

    println!("sil-bridge: on the pad; the vehicle is released when the EC applies throttle");

    let mut applied: Option<Applied> = None;
    let mut rcs_duty = [0.0_f64; 4];
    let mut buffer = [0u8; 256];
    let mut sent = Sent::default();
    let mut tracker = Tracker::default();
    let mut bridge_t = 0.0_f64;
    let mut flying = false;
    let mut release_t = 0.0;
    let mut next = Instant::now();
    let mut x = pad;
    let mut ticks: u64 = 0;
    let mut end_reason = None;
    // After the plant's touchdown stop: the vehicle at rest on the ground, as
    // the sensors read it, since when and with what sensors. The plant has no
    // ground, so without this the sensors just stop.
    let mut rest: Option<(f64, SensorModel, State)> = None;
    // A plant with a ground has no touchdown stop: the vehicle is down when it has
    // been up and is low and still for a while
    let mut peak_altitude = 0.0_f64;
    let mut still_since: Option<f64> = None;
    // `keep_running`: the flight's result has been printed, and the vehicle rests
    let mut finished = false;

    loop {
        // What the EC is driving: the newest datagram wins
        while let Ok(size) = applied_socket.recv(&mut buffer) {
            if let Some(a) = std::str::from_utf8(&buffer[..size])
                .ok()
                .and_then(Applied::parse)
            {
                applied = Some(a);
            }
        }

        if let Some(socket) = &roll_socket {
            while let Ok(size) = socket.recv(&mut buffer) {
                if let Some(duty) = std::str::from_utf8(&buffer[..size])
                    .ok()
                    .and_then(parse_rcs_duty)
                {
                    rcs_duty = duty;
                }
            }
        }

        let throttle = applied.as_ref().map_or(0.0, |a| a.throttle(min_us, max_us));
        let armed = applied.as_ref().is_some_and(|a| a.armed);

        if !flying && rest.is_none() {
            if armed && throttle > config.release_throttle {
                plant.reset().map_err(|e| e.to_string())?;
                sent = Sent::default();
                flying = true;
                release_t = bridge_t;
                println!("sil-bridge: released at throttle {throttle:.2}; t = 0 (bridge time {bridge_t:.2} s)");
            } else {
                // at rest on the pad
                for sample in pad_sensors.step(bridge_t, &pad) {
                    let imu = matches!(sample, SilSample::Imu { .. });

                    feed.send(&Stamped {
                        t_s: bridge_t,
                        sample,
                    });

                    if imu {
                        feed.send(&Stamped {
                            t_s: bridge_t,
                            sample: SilSample::Truth { x: pad },
                        });
                    }
                }

                if bridge_t > config.max_pad_s {
                    return Err(format!(
                        "nothing released the vehicle in {:.0} s",
                        config.max_pad_s
                    ));
                }
            }
        }

        if let Some((since, model, resting)) = rest.as_mut() {
            for sample in model.step(bridge_t, resting) {
                let imu = matches!(sample, SilSample::Imu { .. });

                feed.send(&Stamped {
                    t_s: bridge_t,
                    sample,
                });

                if imu {
                    feed.send(&Stamped {
                        t_s: bridge_t,
                        sample: SilSample::Truth { x: *resting },
                    });
                }
            }

            if !finished && bridge_t - *since > REST_S {
                end_reason = Some("touchdown");
            }
        } else if flying {
            let u = applied.as_ref().map_or([0.0; 4], |a| {
                a.plant_command(&allocation, min_us, max_us, config.command_source, rcs_duty)
            });

            plant.set_command(&u);

            let period = plant.step().map_err(|e| e.to_string())?;

            x = plant.state().map_err(|e| e.to_string())?;
            tracker.add(&x, plant.time());

            if let Some(sensors) = plant.sensors().map_err(|e| e.to_string())? {
                for sample in convert(&sensors, &mut sent, &site) {
                    let imu = matches!(sample, SilSample::Imu { .. });

                    feed.send(&Stamped {
                        t_s: bridge_t,
                        sample,
                    });

                    if imu {
                        feed.send(&Stamped {
                            t_s: bridge_t,
                            sample: SilSample::Truth { x },
                        });
                    }
                }
            }

            peak_altitude = peak_altitude.max(-x[2]);

            let speed = (x[3] * x[3] + x[4] * x[4] + x[5] * x[5]).sqrt();

            still_since = if peak_altitude > ON_THE_GROUND_AFTER_M
                && -x[2] <= ON_THE_GROUND_M
                && speed <= ON_THE_GROUND_SPEED_MPS
            {
                still_since.or(Some(bridge_t))
            } else {
                None
            };

            if period.touchdown {
                // keep feeding the sensors, now of a vehicle at rest, for a while
                let mut resting = x;

                resting[3..9].fill(0.0);
                println!(
                    "sil-bridge: touchdown at plant t = {:.2} s; the vehicle rests",
                    plant.time()
                );
                rest = Some((
                    bridge_t,
                    SensorModel::starting_at(site.clone(), bridge_t),
                    resting,
                ));
            } else if still_since.is_some_and(|since| bridge_t - since >= REST_S) {
                // down on the plant's own ground: the sensors have been reading it all along
                end_reason = Some("touchdown");
            } else if x.iter().any(|v| !v.is_finite()) {
                end_reason = Some("non-finite plant state");
            } else if x[..3].iter().any(|p| p.abs() > 60.0) {
                end_reason = Some("position limit");
            } else if bridge_t - release_t > config.max_flight_s {
                end_reason = Some("flight time limit");
            }
        }

        if ticks.is_multiple_of(10) {
            if let Some(log) = log.as_mut() {
                log.row(
                    bridge_t,
                    if flying { "FLIGHT" } else { "PAD" },
                    plant.time(),
                    &x,
                    applied.as_ref(),
                    throttle,
                );
            }
        }

        ticks += 1;
        bridge_t += STEP;

        if let Some(why) = end_reason {
            tracker.plant_touchdown = why == "touchdown";
            println!(
                "sil-bridge: flight over at plant t = {:.2} s ({why})",
                plant.time()
            );

            // The plant-only outcome: the verdict on the whole system is the
            // flight logic's mode from the operator's side
            let o = tracker.outcome();

            println!(
                "BRIDGE RESULT: reason={why} time_s={:.2} apex_m={:.3} max_tilt_deg={:.2} max_horizontal_m={:.3} landing_offset_m={:.3} impact_speed_mps={:.3} touchdown_tilt_deg={:.2}",
                plant.time(),
                tracker.max_altitude_m,
                tracker.max_tilt_deg,
                tracker.max_horizontal_m,
                o.landing_offset_m.unwrap_or(f64::NAN),
                o.impact_speed_mps.unwrap_or(f64::NAN),
                o.touchdown_tilt_deg.unwrap_or(f64::NAN),
            );

            // keep the log whole, and give the flight computer a moment to see
            // the end before the run is torn down
            if let Some(log) = log.as_mut() {
                let _ = log.out.flush();
            }

            if config.keep_running && why != "non-finite plant state" {
                // stay up: the vehicle rests where it ended, and its sensors keep coming
                if rest.is_none() {
                    let mut resting = x;

                    resting[3..9].fill(0.0);
                    rest = Some((
                        bridge_t,
                        SensorModel::starting_at(site.clone(), bridge_t),
                        resting,
                    ));
                }

                flying = false;
                finished = true;
                end_reason = None;
                println!("sil-bridge: staying up with the vehicle at rest (stop the stack to end)");
                pace(&mut next);

                continue;
            }

            thread::sleep(Duration::from_millis(500));

            let landed_softly = why == "touchdown"
                && o.impact_speed_mps
                    .is_some_and(|v| v <= crate::sil::MAX_IMPACT_SPEED_MPS)
                && o.landing_offset_m
                    .is_some_and(|v| v <= crate::sil::MAX_LANDING_OFFSET_M)
                && o.touchdown_tilt_deg
                    .is_some_and(|v| v <= crate::sil::MAX_TOUCHDOWN_TILT_DEG);

            return Ok(if landed_softly { 0 } else { 2 });
        }

        pace(&mut next);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sensors(stamps: [f64; 9]) -> Sensors {
        let mut meas = [0.0; 26];

        meas[layout::ACCEL.start] = 9.8;
        meas[layout::GYRO.start] = 0.1;
        meas[layout::BARO] = 2.0;
        meas[layout::GPS_POS.start] = 111.32;
        meas[layout::GPS_POS.start + 2] = -2.0;
        meas[layout::GPS_FIX] = 3.0;
        meas[layout::GPS_SATS] = 12.0;
        meas[layout::MAG.start] = 22.6;
        meas[layout::MAG.start + 2] = 42.0;
        meas[layout::GPS_VEL.start + 2] = -1.0;

        Sensors {
            meas,
            valid: [true; 9],
            stamp: stamps,
        }
    }

    #[test]
    fn a_line_from_the_ec_parses() {
        let a = Applied::parse("A 5917 ARMED 1500 13.504 14.250\n").unwrap();

        assert_eq!(a.goals, None, "an older EC sends no goals");

        let with_goals =
            Applied::parse("A 5917 ARMED 1500 13.504 14.250 15.0 16.0 0.62\n").unwrap();

        assert_eq!(with_goals.goals, Some(([15.0, 16.0], 0.62)));
        assert_eq!(with_goals.tvc_mm, a.tvc_mm);

        assert!(a.armed);
        assert_eq!(a.esc_us, 1500.0);
        assert_eq!(a.tvc_mm, [13.504, 14.25]);
        assert_eq!(a.throttle(1000.0, 2000.0), 0.5);

        assert!(!Applied::parse("A 1 DISARMED 1000 0 0").unwrap().armed);
        assert!(!Applied::parse("A 1 SAFING 1000 0 0").unwrap().armed);

        for bad in [
            "",
            "B 1 ARMED 1500 1 2",
            "A 1 ARMED",
            "A 1 ARMED x 1 2",
            "A 1 ARMED 1500 NaN 2",
        ] {
            assert_eq!(Applied::parse(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn the_throttle_is_the_pulse_inside_its_range() {
        let a = |us| Applied {
            armed: true,
            esc_us: us,
            tvc_mm: [0.0; 2],
            goals: None,
        };

        assert_eq!(a(900.0).throttle(1000.0, 2000.0), 0.0);
        assert_eq!(a(1000.0).throttle(1000.0, 2000.0), 0.0);
        assert!((a(1730.0).throttle(1000.0, 2000.0) - 0.73).abs() < 1e-12);
        assert_eq!(a(2500.0).throttle(1000.0, 2000.0), 1.0);
    }

    #[test]
    fn roll_torque_reaches_the_plant_only_from_the_duties_the_flight_computer_sends() {
        let allocation = Allocation::PLACEHOLDER;
        let (center, _) = allocation.tvc_center_and_stroke_mm();
        let applied = Applied {
            armed: true,
            esc_us: 1500.0,
            tvc_mm: center,
            goals: None,
        };
        let torque = |duty| {
            applied.plant_command(&allocation, 1000.0, 2000.0, CommandSource::Applied, duty)[3]
        };

        assert_eq!(torque([0.0; 4]), 0.0);
        assert!(torque([0.5, 0.0, 0.5, 0.0]) > 0.0);
        assert!(torque([0.0, 0.5, 0.0, 0.5]) < 0.0);
        // the pairs oppose each other
        assert!(torque([0.5; 4]).abs() < 1e-12);

        assert_eq!(
            parse_rcs_duty("R 0.5000 0.0000 0.5000 0.0000\n"),
            Some([0.5, 0.0, 0.5, 0.0])
        );
        // duties are clamped to what a fan can do; anything else is ignored
        assert_eq!(parse_rcs_duty("R 3 -1 0 0"), Some([1.0, 0.0, 0.0, 0.0]));
        assert_eq!(parse_rcs_duty("R 0.5 0.5"), None);
        assert_eq!(parse_rcs_duty("R 0.5 NaN 0 0"), None);
        assert_eq!(parse_rcs_duty("A 0.5 0.5 0 0"), None);
    }

    #[test]
    fn the_plant_flies_on_the_thrust_and_the_gimbal_the_ec_applied() {
        let allocation = Allocation::PLACEHOLDER;
        let (center, _) = allocation.tvc_center_and_stroke_mm();
        let centered = Applied {
            armed: true,
            esc_us: 1500.0,
            tvc_mm: center,
            goals: None,
        };
        let u = centered.plant_command(
            &allocation,
            1000.0,
            2000.0,
            CommandSource::Applied,
            [0.0; 4],
        );

        // placeholder: 100 N at full throttle
        assert!((u[0] - 50.0).abs() < 1e-9, "{u:?}");
        assert!(
            u[1].abs() < 1e-9 && u[2].abs() < 1e-9 && u[3] == 0.0,
            "{u:?}"
        );

        // an actuator that has only got part of the way is where it is
        let part = Applied {
            tvc_mm: [center[0] + 2.0, center[1]],
            ..centered
        };

        assert!(
            part.plant_command(
                &allocation,
                1000.0,
                2000.0,
                CommandSource::Applied,
                [0.0; 4]
            )[1] > 0.0
        );
    }

    #[test]
    fn the_goals_source_flies_on_what_the_drivers_were_told() {
        let allocation = Allocation::PLACEHOLDER;
        let (center, _) = allocation.tvc_center_and_stroke_mm();
        let a = Applied {
            armed: true,
            esc_us: 1100.0,
            tvc_mm: center,
            goals: Some(([center[0] + 3.0, center[1]], 0.5)),
        };
        let applied = a.plant_command(
            &allocation,
            1000.0,
            2000.0,
            CommandSource::Applied,
            [0.0; 4],
        );
        let goals = a.plant_command(&allocation, 1000.0, 2000.0, CommandSource::Goals, [0.0; 4]);

        assert!(
            (applied[0] - 10.0).abs() < 1e-9 && applied[1].abs() < 1e-9,
            "{applied:?}"
        );
        assert!(
            (goals[0] - 50.0).abs() < 1e-9 && goals[1] > 0.0,
            "{goals:?}"
        );

        // an EC that sends no goals: the applied values
        let none = Applied { goals: None, ..a };

        assert_eq!(
            none.plant_command(&allocation, 1000.0, 2000.0, CommandSource::Goals, [0.0; 4]),
            applied
        );
    }

    #[test]
    fn each_new_sample_goes_out_once_in_the_flight_computers_units() {
        let site = SensorConfig::default();
        let mut sent = Sent::default();
        let first = convert(&sensors([0.002; 9]), &mut sent, &site);

        // the IMU, barometer, magnetometer, the four LiDAR beams, the GPS
        assert_eq!(first.len(), 8, "{first:?}");

        let SilSample::Imu { accel, gyro_dps } = first[0] else {
            panic!("{first:?}")
        };

        assert_eq!(accel[0], 9.8);
        assert!((gyro_dps[0] - 0.1f64.to_degrees()).abs() < 1e-12);

        let SilSample::Baro { pressure_pa, .. } = first[1] else {
            panic!()
        };

        assert!((pressure_pa - pressure_at(322.0)).abs() < 1e-9);

        let SilSample::Mag { gauss } = first[2] else {
            panic!()
        };

        assert!((gauss[0] - 0.226).abs() < 1e-12 && (gauss[2] - 0.42).abs() < 1e-12);

        assert!(first[3..7].iter().enumerate().all(
            |(beam, s)| matches!(s, SilSample::Lidar { id, .. } if usize::from(*id) == beam + 1)
        ));

        let SilSample::Gps(ref gps) = first[7] else {
            panic!()
        };

        assert!(gps.has_fix && gps.num_satellites == 12);
        assert!((gps.latitude_deg - (site.pad_latitude_deg + 0.001)).abs() < 1e-9);
        assert!((gps.altitude_m - (site.pad_altitude_m + 2.0)).abs() < 1e-9);
        assert_eq!(gps.down_mps, -1.0);

        // the same stamps again: nothing new
        assert!(convert(&sensors([0.002; 9]), &mut sent, &site).is_empty());

        // only the IMU has taken a new sample
        let mut stamps = [0.002; 9];

        stamps[layout::S_IMU] = 0.004;

        let next = convert(&sensors(stamps), &mut sent, &site);

        assert_eq!(next.len(), 1);
        assert!(matches!(next[0], SilSample::Imu { .. }));
    }

    #[test]
    fn an_invalid_sensor_sends_nothing_which_is_how_a_dropout_looks() {
        let site = SensorConfig::default();
        let mut sent = Sent::default();
        let mut s = sensors([1.0; 9]);

        s.valid[layout::S_GPS] = false;
        s.valid[layout::S_IMU] = false;

        let out = convert(&s, &mut sent, &site);

        // the barometer, the magnetometer and the four LiDAR beams are left
        assert_eq!(out.len(), 6);
        assert!(out
            .iter()
            .all(|s| !matches!(s, SilSample::Imu { .. } | SilSample::Gps(_))));
    }

    #[test]
    fn the_pad_model_takes_the_plants_sensor_settings() {
        let info: Value = serde_json::from_str(
            r#"{ "sensor_params": {
                "imu": {"period": 0.002, "accel_sigma": [0.05,0.05,0.05], "gyro_sigma": [0.005,0.005,0.005]},
                "baro": {"period": 0.05, "sigma": 0.1},
                "mag": {"period": 0.02, "sigma": 0.5, "field_ned": [22.6, -2.5, 42]},
                "gps": {"period": 0.2, "sigma": [0.02, 0.02, 0.05], "vel_sigma": [0.05, 0.05, 0.1]} } }"#,
        )
        .unwrap();
        let site = site_from_plant(&info);

        assert_eq!(site.imu_hz, 500.0);
        assert_eq!(site.baro_hz, 20.0);
        assert_eq!(site.mag_hz, 50.0);
        assert_eq!(site.gps_hz, 5.0);
        assert!((site.gyro_std_dps - 0.005f64.to_degrees()).abs() < 1e-12);
        assert!((site.mag_std_gauss - 0.005).abs() < 1e-12);
        assert_eq!(site.mag_field_ned, [0.226, -0.025, 0.42]);
        assert_eq!(site.gps_position_std, [0.02, 0.02, 0.05]);
        assert_eq!(site.gps_fix_after_s, 0.0);
    }

    #[test]
    fn the_pad_model_has_the_plants_gps_antenna() {
        let info: Value = serde_json::from_str(
            r#"{ "sensor_params": { "gps": { "period": 0.2,
                "antenna": [[0.54921, 0.54921], [0, 0], [0.1, -0.1]] } } }"#,
        )
        .unwrap();

        // from the base in the plant's parameters, from the centre of mass for the filter
        let lever = site_from_plant(&info).gps_antenna_body;

        assert!(
            (lever[0] - (0.54921 - CTV_CG_HEIGHT_M)).abs() < 1e-12,
            "{lever:?}"
        );
        assert_eq!(&lever[1..], &[0.0, 0.1]);
        // and the plant's GPS is against the CG's real position, which the pad must match
        assert_eq!(
            site_from_plant(&info).gps_position_offset,
            [0.0, 0.0, -CTV_CG_HEIGHT_M]
        );
    }
}
