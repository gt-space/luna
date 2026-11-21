//! Shadow navigation: runs the GMEKF on the flight computer's own sensors,
//! logs its estimate, and publishes it with a health flag for the control
//! loop ([`ShadowNav::estimate`]). Nothing here feeds control by itself: the
//! control loop decides whether to use the estimate.
//!
//! The filter runs on its own thread. The main loop hands it copies of the
//! IMU, barometer, magnetometer and GPS samples through a bounded channel with
//! `try_send`, so a slow or failed filter can never block or crash the main
//! loop; when the channel is full, samples are dropped and counted.
//!
//! On startup the thread levels: it averages the IMU and barometer over
//! `level_seconds` (the vehicle must sit still on the pad), sets the pad
//! pressure and temperature as the barometer reference, the average gyro as
//! the gyro bias and the attitude from the gravity direction. Heading comes
//! from the average magnetometer reading against `mag_field_ned`, or is
//! arbitrary (with a large uncertainty) without the magnetometer. Position is
//! zero at the IMU on the pad; the first good GPS fix after that sets the GPS
//! origin.
//!
//! Health ([`NavHealth`]): `Leveling` until the filter starts; `Healthy`
//! while every check passes; `Unhealthy` while one fails (uncertainty above
//! a limit, the barometer or GPS rejected many times in a row, or no new IMU
//! step for `imu_timeout_ms`); `Failed` once the estimate goes non-finite.
//! A failure is latched until restart: leveling again in flight would be
//! wrong, since leveling assumes the vehicle is still.
//!
//! Enabled with `--shadow-nav`; `--shadow-nav-config FILE` reads a JSON
//! [`ShadowNavConfig`]. The log is `shadow_nav_<unix time>.csv` in the first
//! log directory.

use std::{
    fs::File,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    sync::{
        mpsc::{self, Receiver, SyncSender, TrySendError},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use common::comm::{fc_sensors::Imu, GpsState};
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use serde::Deserialize;

use crate::navigate::{
    gmekf::{Diagnostics, Gmekf, GmekfConfig},
    Estimator, ImuSample, Measurement, NavState, Timestamped,
};

/// Settings for the shadow filter. Every field is optional in the JSON
/// file; missing ones keep the defaults.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ShadowNavConfig {
    /// How the IMU is mounted: which IMU axis each body axis points along
    /// (for example `["-z", "+y", "+x"]`), or a rotation matrix from IMU to
    /// body axes. IMU axes = body axes until the CTV mounting is verified.
    pub imu_to_body: ImuMounting,
    /// How the magnetometer is mounted, in the same form as `imu_to_body`.
    /// Magnetometer axes = body axes until the mounting is verified.
    pub mag_to_body: ImuMounting,
    /// Fuse the magnetometer for heading. Turn it off if it reads nonsense,
    /// for example near the motors before a hard/soft-iron calibration.
    pub use_magnetometer: bool,
    /// Earth's magnetic field at the pad in NED, Gauss. Default: the Atlanta
    /// value of the MATLAB filter's sensor parameters.
    pub mag_field_ned: [f64; 3],
    /// Shortest time between fused magnetometer readings, s. The LIS2MDL
    /// runs at 10 Hz and the driver returns the same reading until a new
    /// one is ready, so faster fusing would count one reading many times.
    pub mag_interval_s: f64,
    /// Initial heading uncertainty when the magnetometer set the heading,
    /// deg.
    pub mag_heading_std_deg: f64,
    /// GPS antenna position from the IMU, body axes, m.
    pub gps_lever_arm: [f64; 3],
    /// Fuse GPS velocity as well as position. Off by default: the
    /// receiver's velocity errors are time-correlated.
    pub use_gps_velocity: bool,
    /// Fewest satellites for a GPS fix to be used.
    pub min_satellites: u8,
    /// Shortest time between fused GPS fixes, s. Standalone GPS error is a
    /// slow wander (correlation time ~100 s), not white noise; fusing every
    /// 20 Hz fix as independent makes the filter overconfident. 0 fuses
    /// every fix.
    pub gps_interval_s: f64,
    /// How long to average sensors on the pad before starting, s.
    pub level_seconds: f64,
    /// Write one log row every this many IMU samples.
    pub log_every: u32,
    /// Initial heading uncertainty without the magnetometer, deg.
    pub heading_std_deg: f64,
    /// Print a status line to the terminal every this many seconds; 0 turns
    /// it off.
    pub status_every_s: f64,
    /// Health limits (see [`NavHealth`]). Largest position uncertainty on
    /// any axis, m.
    pub max_position_std_m: f64,
    /// Largest velocity uncertainty on any axis, m/s.
    pub max_velocity_std_mps: f64,
    /// Largest tilt uncertainty (about body y and z), deg.
    pub max_tilt_std_deg: f64,
    /// Largest heading uncertainty (about body x, up on the pad), deg.
    pub max_heading_std_deg: f64,
    /// Barometer or GPS readings rejected in a row before the estimate is
    /// unhealthy.
    pub max_rejections_in_a_row: u32,
    /// Longest time without a new IMU step before the estimate is stale, ms.
    pub imu_timeout_ms: u64,
    /// Overrides for the filter tuning (see `GmekfConfig`). `None` keeps the
    /// filter default.
    pub gravity: Option<f64>,
    /// Accelerometer noise per sample, m/s^2.
    pub accel_std: Option<f64>,
    /// Gyro noise per sample, deg/s.
    pub gyro_std_dps: Option<f64>,
    pub baro_std: Option<f64>,
    pub gps_position_std: Option<[f64; 3]>,
    pub gps_velocity_std: Option<[f64; 3]>,
    /// Magnetometer noise per axis, Gauss.
    pub mag_std: Option<f64>,
}

impl Default for ShadowNavConfig {
    fn default() -> Self {
        ShadowNavConfig {
            imu_to_body: ImuMounting::default(),
            mag_to_body: ImuMounting::default(),
            use_magnetometer: true,
            // nav_sensor_params.m: [227.5; -22.4; 423.0] mGauss.
            mag_field_ned: [0.2275, -0.0224, 0.4230],
            mag_interval_s: 0.1,
            mag_heading_std_deg: 10.0,
            gps_lever_arm: [0.0; 3],
            use_gps_velocity: false,
            min_satellites: 6,
            gps_interval_s: 1.0,
            level_seconds: 2.0,
            log_every: 4,
            heading_std_deg: 90.0,
            status_every_s: 1.0,
            max_position_std_m: 5.0,
            max_velocity_std_mps: 0.5,
            max_tilt_std_deg: 3.0,
            max_heading_std_deg: 30.0,
            max_rejections_in_a_row: 10,
            imu_timeout_ms: 50,
            gravity: None,
            accel_std: None,
            gyro_std_dps: None,
            baro_std: None,
            gps_position_std: None,
            gps_velocity_std: None,
            mag_std: None,
        }
    }
}

impl ShadowNavConfig {
    /// Reads and checks a JSON config file.
    pub fn from_file(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
        let config: Self = serde_json::from_str(&text)
            .map_err(|e| format!("bad config {}: {e}", path.display()))?;
        config
            .imu_to_body
            .rotation()
            .map_err(|e| format!("bad imu_to_body in {}: {e}", path.display()))?;
        config
            .mag_to_body
            .rotation()
            .map_err(|e| format!("bad mag_to_body in {}: {e}", path.display()))?;
        Ok(config)
    }

    fn filter_config(&self) -> GmekfConfig {
        let mut c = GmekfConfig::default();
        if let Some(g) = self.gravity {
            c.gravity = g;
        }
        if let Some(s) = self.accel_std {
            c.accel_std = Vector3::repeat(s);
        }
        if let Some(s) = self.gyro_std_dps {
            c.gyro_std = Vector3::repeat(s.to_radians());
        }
        if let Some(s) = self.baro_std {
            c.baro_std = s;
        }
        if let Some(s) = self.gps_position_std {
            c.gps_position_std = Vector3::from(s);
        }
        if let Some(s) = self.gps_velocity_std {
            c.gps_velocity_std = Vector3::from(s);
        }
        c.gps_lever_arm = Vector3::from(self.gps_lever_arm);
        if self.use_magnetometer {
            c.mag_field_ned = Some(Vector3::from(self.mag_field_ned));
        }
        if let Some(s) = self.mag_std {
            c.mag_std = s;
        }
        c
    }
}

/// How the IMU sits in the body frame (body x up on the pad).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(untagged)]
pub enum ImuMounting {
    /// The IMU axis each body axis (x, y, z) points along, with a sign:
    /// `["-z", "+y", "+x"]` means body x is the IMU's -z, body y its +y and
    /// body z its +x. Read the axes off the IMU datasheet's diagram and the
    /// board's orientation in the vehicle.
    Axes([String; 3]),
    /// Rotation matrix from IMU axes to body axes, row by row.
    Matrix([[f64; 3]; 3]),
}

impl Default for ImuMounting {
    fn default() -> Self {
        ImuMounting::Axes(["+x".into(), "+y".into(), "+z".into()])
    }
}

impl ImuMounting {
    /// The rotation from IMU to body axes, or why it is not a rotation.
    pub fn rotation(&self) -> Result<Matrix3<f64>, String> {
        let r = match self {
            ImuMounting::Axes(axes) => {
                let mut r = Matrix3::zeros();
                for (row, axis) in axes.iter().enumerate() {
                    let (sign, column) = parse_axis(axis)?;
                    r[(row, column)] = sign;
                }
                r
            }
            ImuMounting::Matrix(m) => Matrix3::from_fn(|i, j| m[i][j]),
        };
        if (r * r.transpose() - Matrix3::identity()).amax() > 1e-6 {
            return Err(format!(
                "{self:?} is not a rotation (each IMU axis must be used once)"
            ));
        }
        if r.determinant() < 0.0 {
            return Err(format!(
                "{self:?} is a mirror image, not a rotation: flip the sign of one axis"
            ));
        }
        Ok(r)
    }

    /// Readable form for the startup message.
    fn describe(&self) -> String {
        match self {
            ImuMounting::Axes(a) => format!(
                "body x = IMU {}, body y = IMU {}, body z = IMU {}",
                a[0], a[1], a[2]
            ),
            ImuMounting::Matrix(m) => format!("IMU-to-body matrix {m:?}"),
        }
    }
}

/// Parses `"+x"`, `"-z"`, `"y"` into a sign and an axis index.
fn parse_axis(axis: &str) -> Result<(f64, usize), String> {
    let a = axis.trim().to_ascii_lowercase();
    let (sign, name) = match a.strip_prefix('-') {
        Some(rest) => (-1.0, rest),
        None => (1.0, a.strip_prefix('+').unwrap_or(&a)),
    };
    let index = match name {
        "x" => 0,
        "y" => 1,
        "z" => 2,
        _ => {
            return Err(format!(
                "unknown axis {axis:?}; use +x, -x, +y, -y, +z or -z"
            ))
        }
    };
    Ok((sign, index))
}

/// Tilt of body x from vertical and heading of body z (from north toward
/// east), deg. Body x points up on the pad, so body z is horizontal there.
fn tilt_and_heading(attitude: &UnitQuaternion<f64>) -> (f64, f64) {
    let up = attitude * Vector3::x();
    let tilt = (-up.z).clamp(-1.0, 1.0).acos().to_degrees();
    let z = attitude * Vector3::z();
    (tilt, z.y.atan2(z.x).to_degrees())
}

/// Whether the estimate can be used for control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NavHealth {
    /// Still averaging the sensors on the pad.
    Leveling,
    /// Every check passes.
    Healthy,
    /// A check fails; the estimate may recover.
    Unhealthy(&'static str),
    /// The estimate went non-finite. Latched until restart.
    Failed(&'static str),
}

impl NavHealth {
    /// Log column code: 0 leveling, 1 healthy, 2 unhealthy, 3 failed.
    fn code(&self) -> f64 {
        match self {
            NavHealth::Leveling => 0.0,
            NavHealth::Healthy => 1.0,
            NavHealth::Unhealthy(_) => 2.0,
            NavHealth::Failed(_) => 3.0,
        }
    }
}

/// The filter's latest output, shared with the main loop.
#[derive(Debug, Clone, Copy)]
struct Published {
    state: Option<NavState>,
    health: NavHealth,
    /// When the IMU sample of the latest step was read.
    imu_read_at: Option<Instant>,
}

/// The filter's latest estimate as the main loop sees it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavReading {
    /// The estimate; `None` while leveling.
    pub state: Option<NavState>,
    /// Its health, including the staleness check at read time.
    pub health: NavHealth,
    /// When the IMU sample of the estimate was read (the estimate's time on
    /// the main loop's clock).
    pub imu_read_at: Option<Instant>,
}

/// Health checks on a running filter, apart from staleness (checked when
/// the main loop reads the estimate).
fn assess(
    config: &ShadowNavConfig,
    state: &NavState,
    baro_rejected: u32,
    gps_rejected: u32,
) -> NavHealth {
    let max = |v: &Vector3<f64>| v.iter().copied().fold(0.0, f64::max);
    let a = &state.attitude_std;
    if max(&state.position_std) > config.max_position_std_m {
        return NavHealth::Unhealthy("position uncertainty too large");
    }
    if max(&state.velocity_std) > config.max_velocity_std_mps {
        return NavHealth::Unhealthy("velocity uncertainty too large");
    }
    if a.y.max(a.z) > config.max_tilt_std_deg.to_radians() {
        return NavHealth::Unhealthy("tilt uncertainty too large");
    }
    if a.x > config.max_heading_std_deg.to_radians() {
        return NavHealth::Unhealthy("heading uncertainty too large");
    }
    if baro_rejected >= config.max_rejections_in_a_row {
        return NavHealth::Unhealthy("barometer rejected repeatedly");
    }
    if gps_rejected >= config.max_rejections_in_a_row {
        return NavHealth::Unhealthy("GPS rejected repeatedly");
    }
    NavHealth::Healthy
}

/// Counts readings rejected in a row from one IMU step's results: reset on
/// an accepted reading, +1 on a rejected one, unchanged without one.
fn count_rejections(count: &mut u32, result: f64) {
    if result == 1.0 {
        *count = 0;
    } else if result == 0.0 {
        *count += 1;
    }
}

/// One sensor sample for the shadow filter, stamped when it was read.
#[derive(Debug, Clone)]
enum Input {
    Imu {
        at: Instant,
        imu: Imu,
    },
    Baro {
        at: Instant,
        pressure: f64,
        temperature: f64,
    },
    Gps {
        at: Instant,
        gps: GpsState,
    },
    Mag {
        at: Instant,
        /// Sensor axes, Gauss.
        field: Vector3<f64>,
    },
}

/// The main loop's end of the shadow filter.
pub struct ShadowNav {
    tx: SyncSender<Input>,
    output: Arc<Mutex<Published>>,
    imu_timeout: Duration,
    warned_full: bool,
    warned_gone: bool,
}

impl ShadowNav {
    /// Starts the shadow filter thread, logging to `log_dir`.
    pub fn spawn(config: ShadowNavConfig, log_dir: &Path) -> Result<Self, String> {
        config.imu_to_body.rotation()?;
        config.mag_to_body.rotation()?;
        let unix = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let path = log_dir.join(format!("shadow_nav_{unix}.csv"));
        let file =
            File::create(&path).map_err(|e| format!("cannot create {}: {e}", path.display()))?;
        let (tx, rx) = mpsc::sync_channel(4096);
        let output = Arc::new(Mutex::new(Published::leveling()));
        let imu_timeout = Duration::from_millis(config.imu_timeout_ms);
        let shared = output.clone();
        thread::Builder::new()
            .name("shadow-nav".into())
            .spawn(move || run(config, rx, BufWriter::new(file), path, shared))
            .map_err(|e| format!("cannot start the shadow-nav thread: {e}"))?;
        Ok(ShadowNav {
            tx,
            output,
            imu_timeout,
            warned_full: false,
            warned_gone: false,
        })
    }

    /// The latest estimate and its health, as of `now` (the main loop's
    /// clock). Never blocks for longer than the filter takes to publish.
    // Read by the control loop once it is wired in.
    #[allow(dead_code)]
    pub fn reading(&self, now: Instant) -> NavReading {
        let published = match self.output.lock() {
            Ok(guard) => *guard,
            Err(_) => {
                return NavReading {
                    state: None,
                    health: NavHealth::Failed("filter thread panicked"),
                    imu_read_at: None,
                }
            }
        };
        read_published(&published, now, self.imu_timeout)
    }

    /// The estimate if it can be used for control right now, else why not.
    // Read by the control loop once it is wired in.
    #[allow(dead_code)]
    pub fn estimate(&self, now: Instant) -> Result<NavState, NavHealth> {
        let reading = self.reading(now);
        match (reading.health, reading.state) {
            (NavHealth::Healthy, Some(state)) => Ok(state),
            (NavHealth::Healthy, None) => Err(NavHealth::Leveling),
            (health, _) => Err(health),
        }
    }

    /// Hands an IMU sample read at `at` to the filter.
    pub fn imu(&mut self, at: Instant, imu: Imu) {
        self.send(Input::Imu { at, imu });
    }

    /// Hands a barometer sample (Pa, deg C) read at `at` to the filter.
    pub fn baro(&mut self, at: Instant, pressure: f64, temperature: f64) {
        self.send(Input::Baro {
            at,
            pressure,
            temperature,
        });
    }

    /// Hands a magnetometer reading (sensor axes, Gauss) read at `at` to
    /// the filter.
    pub fn mag(&mut self, at: Instant, field: [f64; 3]) {
        self.send(Input::Mag {
            at,
            field: Vector3::from(field),
        });
    }

    /// Hands a GPS fix received at `at` to the filter.
    pub fn gps(&mut self, at: Instant, gps: &GpsState) {
        self.send(Input::Gps {
            at,
            gps: gps.clone(),
        });
    }

    fn send(&mut self, input: Input) {
        match self.tx.try_send(input) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                if !self.warned_full {
                    eprintln!("shadow-nav: input queue full, dropping samples");
                    self.warned_full = true;
                }
            }
            Err(TrySendError::Disconnected(_)) => {
                if !self.warned_gone {
                    eprintln!("shadow-nav: filter thread stopped; shadow navigation is off");
                    self.warned_gone = true;
                }
            }
        }
    }
}

impl Published {
    fn leveling() -> Self {
        Published {
            state: None,
            health: NavHealth::Leveling,
            imu_read_at: None,
        }
    }
}

/// Adds the staleness check to what the filter published.
fn read_published(published: &Published, now: Instant, imu_timeout: Duration) -> NavReading {
    let stale = published
        .imu_read_at
        .is_some_and(|at| now.saturating_duration_since(at) > imu_timeout);
    let health = match published.health {
        NavHealth::Healthy if stale => NavHealth::Unhealthy("no new IMU step (stale)"),
        health => health,
    };
    NavReading {
        state: published.state,
        health,
        imu_read_at: published.imu_read_at,
    }
}

/// Averages taken on the pad before the filter starts.
#[derive(Default)]
struct Leveling {
    start: Option<Duration>,
    accel: Vector3<f64>,
    gyro: Vector3<f64>,
    imu_n: u32,
    pressure: f64,
    temperature: f64,
    baro_n: u32,
    /// Body axes, Gauss.
    mag: Vector3<f64>,
    mag_n: u32,
}

enum Phase {
    Leveling(Leveling),
    Running(Box<Gmekf>),
    /// The estimate went non-finite; nothing runs until restart.
    Failed,
}

/// Local NED origin for GPS fixes.
struct GpsOrigin {
    lat: f64,
    lon: f64,
    alt: f64,
    /// Meridian and prime-vertical radii plus altitude, m.
    r_north: f64,
    r_east: f64,
    /// Antenna NED position when the origin was set.
    offset: Vector3<f64>,
}

impl GpsOrigin {
    fn new(gps: &GpsState, offset: Vector3<f64>) -> Self {
        const A: f64 = 6_378_137.0; // WGS84
        const E2: f64 = 6.694_379_990_14e-3;
        let lat = gps.latitude_deg.to_radians();
        let s = 1.0 - E2 * lat.sin().powi(2);
        GpsOrigin {
            lat: gps.latitude_deg,
            lon: gps.longitude_deg,
            alt: gps.altitude_m,
            r_north: A * (1.0 - E2) / s.powf(1.5) + gps.altitude_m,
            r_east: (A / s.sqrt() + gps.altitude_m) * lat.cos(),
            offset,
        }
    }

    fn ned(&self, gps: &GpsState) -> Vector3<f64> {
        Vector3::new(
            (gps.latitude_deg - self.lat).to_radians() * self.r_north,
            (gps.longitude_deg - self.lon).to_radians() * self.r_east,
            -(gps.altitude_m - self.alt),
        ) + self.offset
    }
}

const HEADER: &str = "time_s,north_m,east_m,down_m,v_north,v_east,v_down,\
qw,qx,qy,qz,p_rad_s,q_rad_s,r_rad_s,\
north_std,east_std,down_std,v_north_std,v_east_std,v_down_std,\
att_x_std,att_y_std,att_z_std,\
accel_bias_x,accel_bias_y,accel_bias_z,gyro_bias_x,gyro_bias_y,gyro_bias_z,baro_bias,\
baro_ok,gyro_ok,gps_ok,lidar1_ok,lidar2_ok,lidar3_ok,lidar4_ok,mag_ok,\
baro_nis,gyro_nis,gps_nis,lidar1_nis,lidar2_nis,lidar3_nis,lidar4_nis,mag_nis,\
max_step_us,mean_step_us,health";

fn run(
    config: ShadowNavConfig,
    rx: Receiver<Input>,
    log: BufWriter<File>,
    path: PathBuf,
    output: Arc<Mutex<Published>>,
) {
    println!("shadow-nav: logging to {}", path.display());
    // A log failure stops the log, never the filter.
    let mut log = Some(log);
    if let Some(Err(e)) = log.as_mut().map(|l| writeln!(l, "{HEADER}")) {
        eprintln!("shadow-nav: cannot write the log: {e}; running without it");
        log = None;
    }
    let mut health = NavHealth::Leveling;
    let (mut baro_rejected, mut gps_rejected) = (0u32, 0u32);
    let publish = |state: Option<NavState>, health: NavHealth, at: Instant| {
        if let Ok(mut guard) = output.lock() {
            *guard = Published {
                state,
                health,
                imu_read_at: Some(at),
            };
        }
    };

    let epoch = Instant::now();
    let since = |at: Instant| at.checked_duration_since(epoch);
    let imu_to_body = match config.imu_to_body.rotation() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("shadow-nav: bad imu_to_body: {e}; stopping");
            return;
        }
    };
    let mag_to_body = match config.mag_to_body.rotation() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("shadow-nav: bad mag_to_body: {e}; stopping");
            return;
        }
    };
    println!(
        "shadow-nav: IMU mounting: {}",
        config.imu_to_body.describe()
    );
    if config.use_magnetometer {
        println!(
            "shadow-nav: magnetometer mounting: {}",
            config.mag_to_body.describe()
        );
    }
    let mut last_mag_used: Option<Duration> = None;
    let mut last_status: Option<Instant> = None;
    // Sensor results since the last log row and the last status line. The
    // filter clears its own at every prediction, so collect them first.
    let (mut log_diag, mut status_diag) = ([-1.0; 16], [-1.0; 16]);
    let mut phase = Phase::Leveling(Leveling::default());
    let mut origin: Option<GpsOrigin> = None;
    let mut last_fix_ms: Option<i64> = None;
    let mut last_gps_used: Option<Duration> = None;
    let (mut steps, mut step_sum, mut step_max) = (0u32, Duration::ZERO, Duration::ZERO);
    let mut last_flush = Instant::now();

    for input in rx.iter() {
        match input {
            Input::Imu { at, imu } => {
                let Some(time) = since(at) else { continue };
                let a = &imu.accelerometer;
                let g = &imu.gyroscope;
                let accel = imu_to_body * Vector3::new(a.x, a.y, a.z);
                let gyro = imu_to_body * Vector3::new(g.x, g.y, g.z).map(f64::to_radians);
                let sample = Timestamped::new(time, ImuSample { accel, gyro });

                match &mut phase {
                    Phase::Leveling(l) => {
                        let start = *l.start.get_or_insert(time);
                        l.accel += accel;
                        l.gyro += gyro;
                        l.imu_n += 1;
                        let leveled = (time - start).as_secs_f64() >= config.level_seconds;
                        if leveled && l.baro_n > 0 && l.imu_n > 10 {
                            match start_filter(&config, l, time) {
                                Ok(filter) => {
                                    phase = Phase::Running(Box::new(filter));
                                    origin = None;
                                    last_gps_used = None;
                                    last_mag_used = None;
                                    (baro_rejected, gps_rejected) = (0, 0);
                                }
                                Err(e) => {
                                    eprintln!("shadow-nav: cannot start: {e}; leveling again");
                                    phase = Phase::Leveling(Leveling::default());
                                }
                            }
                        }
                    }
                    Phase::Failed => {}
                    Phase::Running(filter) => {
                        if let Some(d) = filter.diagnostics() {
                            merge_diagnostics(&mut log_diag, &d);
                            merge_diagnostics(&mut status_diag, &d);
                            count_rejections(&mut baro_rejected, d[0]);
                            count_rejections(&mut gps_rejected, d[2]);
                        }
                        let start = Instant::now();
                        if let Err(e) = filter.predict(&sample) {
                            eprintln!("shadow-nav: predict: {e}");
                            continue;
                        }
                        let state = filter.nav_state();
                        let elapsed = start.elapsed();
                        steps += 1;
                        step_sum += elapsed;
                        step_max = step_max.max(elapsed);

                        let finite = state.is_some_and(|s| {
                            s.position
                                .iter()
                                .chain(s.velocity.iter())
                                .all(|v| v.is_finite())
                        });
                        let new_health = match state.as_ref() {
                            Some(s) if finite => assess(&config, s, baro_rejected, gps_rejected),
                            _ => NavHealth::Failed("estimate went non-finite"),
                        };
                        if new_health != health {
                            println!("shadow-nav: health {health:?} -> {new_health:?} at {time:?}");
                            health = new_health;
                        }
                        publish(state, health, at);
                        if !finite {
                            eprintln!(
                                "shadow-nav: estimate went non-finite at {time:?}; \
                                 stopped until restart"
                            );
                            phase = Phase::Failed;
                            continue;
                        }
                        let status_due = config.status_every_s > 0.0
                            && last_status
                                .is_none_or(|t| t.elapsed().as_secs_f64() >= config.status_every_s);
                        if status_due {
                            print_status(filter, &status_diag, &health);
                            last_status = Some(Instant::now());
                            status_diag = [-1.0; 16];
                        }
                        if steps >= config.log_every.max(1) {
                            let mean = step_sum / steps;
                            if let Some(Err(e)) = log
                                .as_mut()
                                .map(|l| write_row(l, filter, &log_diag, &health, step_max, mean))
                            {
                                eprintln!(
                                    "shadow-nav: cannot write the log: {e}; running without it"
                                );
                                log = None;
                            }
                            log_diag = [-1.0; 16];
                            (steps, step_sum, step_max) = (0, Duration::ZERO, Duration::ZERO);
                        }
                    }
                }
            }
            Input::Baro {
                at,
                pressure,
                temperature,
            } => {
                let Some(time) = since(at) else { continue };
                match &mut phase {
                    Phase::Leveling(l) => {
                        if pressure.is_finite() && temperature.is_finite() && l.start.is_some() {
                            l.pressure += pressure;
                            l.temperature += temperature;
                            l.baro_n += 1;
                        }
                    }
                    Phase::Failed => {}
                    Phase::Running(filter) => {
                        let m = Measurement::Barometer {
                            pressure,
                            temperature,
                        };
                        if let Err(e) = filter.update(&Timestamped::new(time, m)) {
                            eprintln!("shadow-nav: barometer update: {e}");
                        }
                    }
                }
            }
            Input::Gps { at, gps } => {
                let Some(time) = since(at) else { continue };
                let Phase::Running(filter) = &mut phase else {
                    continue;
                };
                if !gps.has_fix || gps.num_satellites < config.min_satellites {
                    continue;
                }
                // The GPS mailbox can hand over the same fix twice.
                if gps.timestamp_unix_ms.is_some() && gps.timestamp_unix_ms == last_fix_ms {
                    continue;
                }
                last_fix_ms = gps.timestamp_unix_ms;
                let interval = Duration::from_secs_f64(config.gps_interval_s.max(0.0));
                if last_gps_used.is_some_and(|last| time < last + interval) {
                    continue;
                }
                last_gps_used = Some(time);

                let origin = origin.get_or_insert_with(|| {
                    // Put the origin under the IMU: the antenna sits at the
                    // lever arm from it.
                    let state = filter.nav_state().expect("running filter has a state");
                    let attitude = UnitQuaternion::from_quaternion(state.attitude);
                    let lever = attitude * Vector3::from(config.gps_lever_arm);
                    println!(
                        "shadow-nav: GPS origin {:.7}, {:.7}, {:.2} m ({} satellites)",
                        gps.latitude_deg, gps.longitude_deg, gps.altitude_m, gps.num_satellites
                    );
                    GpsOrigin::new(&gps, state.position + lever)
                });
                let position = origin.ned(&gps);
                let velocity = config
                    .use_gps_velocity
                    .then(|| Vector3::new(gps.north_mps, gps.east_mps, gps.down_mps));
                if let Err(e) = filter.update_gps(time, position, velocity) {
                    eprintln!("shadow-nav: GPS update: {e}");
                }
            }

            Input::Mag { at, field } => {
                let Some(time) = since(at) else { continue };
                if !config.use_magnetometer || !field.iter().all(|v| v.is_finite()) {
                    continue;
                }
                let field = mag_to_body * field;
                match &mut phase {
                    Phase::Leveling(l) => {
                        if l.start.is_some() {
                            l.mag += field;
                            l.mag_n += 1;
                        }
                    }
                    Phase::Failed => {}
                    Phase::Running(filter) => {
                        let interval = Duration::from_secs_f64(config.mag_interval_s.max(0.0));
                        if last_mag_used.is_some_and(|last| time < last + interval) {
                            continue;
                        }
                        last_mag_used = Some(time);
                        let m = Measurement::Magnetometer { field };
                        if let Err(e) = filter.update(&Timestamped::new(time, m)) {
                            eprintln!("shadow-nav: magnetometer update: {e}");
                        }
                    }
                }
            }
        }

        if last_flush.elapsed() >= Duration::from_secs(1) {
            if let Some(l) = log.as_mut() {
                let _ = l.flush();
            }
            last_flush = Instant::now();
        }
    }
    if let Some(l) = log.as_mut() {
        let _ = l.flush();
    }
}

/// Builds and initializes the filter from the pad averages.
fn start_filter(config: &ShadowNavConfig, l: &Leveling, time: Duration) -> Result<Gmekf, String> {
    let accel = l.accel / l.imu_n as f64;
    let gyro_bias = l.gyro / l.imu_n as f64;
    let pressure = l.pressure / l.baro_n as f64;
    let temperature = l.temperature / l.baro_n as f64 + 273.15;
    if accel.norm() < 5.0 || accel.norm() > 15.0 {
        return Err(format!(
            "average specific force {:.2} m/s^2 is not gravity; is the vehicle still?",
            accel.norm()
        ));
    }

    // At rest the accelerometer reads +g along body up; the attitude maps
    // that direction to NED up.
    let up_ned = -Vector3::z();
    let mut attitude = UnitQuaternion::rotation_between(&accel, &up_ned).unwrap_or_else(|| {
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
    });
    let mut heading_std = config.heading_std_deg;
    if config.use_magnetometer {
        match align_heading(config, l, &attitude) {
            Some(aligned) => {
                attitude = aligned;
                heading_std = config.mag_heading_std_deg;
            }
            None => println!(
                "shadow-nav: no usable magnetometer reading while leveling; heading is arbitrary"
            ),
        }
    }

    let mut filter_config = config.filter_config();
    filter_config.baro_reference_pressure = pressure;
    filter_config.baro_reference_temperature = temperature;
    let mut filter = Gmekf::new(filter_config);

    let mut initial = NavState::unknown(time);
    initial.attitude = attitude.into_inner();
    initial.position_std = Vector3::repeat(0.1);
    initial.velocity_std = Vector3::repeat(0.05);
    initial.attitude_std = Vector3::repeat(0.02);
    initial.body_rate_std = Vector3::repeat(0.01);
    filter
        .initialize_on_pad(&initial, gyro_bias, heading_std.to_radians())
        .map_err(|e| e.to_string())?;

    let (tilt, _) = tilt_and_heading(&attitude);
    println!(
        "shadow-nav: started at {time:?}: pad {pressure:.0} Pa, {temperature:.1} K, \
         |g| {:.3} m/s^2, gyro bias {:.3?} deg/s, body x {tilt:.1} deg from vertical",
        accel.norm(),
        gyro_bias.map(f64::to_degrees).as_slice(),
    );
    Ok(filter)
}

/// Turns the leveled attitude about the vertical so the average
/// magnetometer reading points like `mag_field_ned`, and reports how the
/// reading compares with the reference. `None` without a usable reading.
fn align_heading(
    config: &ShadowNavConfig,
    l: &Leveling,
    leveled: &UnitQuaternion<f64>,
) -> Option<UnitQuaternion<f64>> {
    if l.mag_n == 0 {
        return None;
    }
    let measured = leveled * (l.mag / l.mag_n as f64);
    let reference = Vector3::from(config.mag_field_ned);
    let horizontal = |v: &Vector3<f64>| v.x.hypot(v.y);
    if horizontal(&measured) < 1e-3 || horizontal(&reference) < 1e-3 {
        return None;
    }
    let turn = reference.y.atan2(reference.x) - measured.y.atan2(measured.x);
    let aligned = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), turn) * leveled;

    let dip = |v: &Vector3<f64>| v.z.atan2(horizontal(v)).to_degrees();
    let ratio = measured.norm() / reference.norm();
    println!(
        "shadow-nav: magnetometer reads {:.3} G (reference {:.3} G), dip {:.1} deg \
         (reference {:.1} deg); heading set from it",
        measured.norm(),
        reference.norm(),
        dip(&measured),
        dip(&reference),
    );
    if !(0.7..=1.3).contains(&ratio) || (dip(&measured) - dip(&reference)).abs() > 15.0 {
        eprintln!(
            "shadow-nav: WARNING magnetometer disagrees with the Earth field (uncalibrated \
             hard/soft iron or wrong mag_to_body?); heading is likely wrong and \
             magnetometer updates may be GATED"
        );
    }
    Some(aligned)
}

/// One terminal line with the estimate, for watching it while moving the
/// vehicle by hand.
fn print_status(filter: &Gmekf, d: &Diagnostics, health: &NavHealth) {
    let (Some(s), Some((_, gb, bb))) = (filter.nav_state(), filter.biases()) else {
        return;
    };
    let (tilt, heading) = tilt_and_heading(&UnitQuaternion::from_quaternion(s.attitude));
    let flag = |i: usize| {
        if d[i] == 1.0 {
            "ok"
        } else if d[i] == 0.0 {
            "GATED"
        } else {
            "-"
        }
    };
    println!(
        "shadow-nav t {:7.2} s | NED {:+7.2} {:+7.2} {:+7.2} m (std {:.2} {:.2} {:.2}) | \
         v {:+6.2} {:+6.2} {:+6.2} m/s | tilt {:5.1} deg, heading {:+6.1} deg | \
         rate {:+6.1} {:+6.1} {:+6.1} deg/s | gyro bias {:+.3} {:+.3} {:+.3} deg/s, \
         baro bias {:+.2} m | baro {} gps {} mag {} | {:?}",
        s.time.as_secs_f64(),
        s.position.x,
        s.position.y,
        s.position.z,
        s.position_std.x,
        s.position_std.y,
        s.position_std.z,
        s.velocity.x,
        s.velocity.y,
        s.velocity.z,
        tilt,
        heading,
        s.body_rate.x.to_degrees(),
        s.body_rate.y.to_degrees(),
        s.body_rate.z.to_degrees(),
        gb.x.to_degrees(),
        gb.y.to_degrees(),
        gb.z.to_degrees(),
        bb,
        flag(0),
        flag(2),
        flag(7),
        health,
    );
}

/// Adds one IMU step's sensor results to `into`: the latest result of each
/// sensor that had a reading wins.
fn merge_diagnostics(into: &mut Diagnostics, step: &Diagnostics) {
    for i in 0..8 {
        if step[i] != -1.0 {
            into[i] = step[i];
            into[i + 8] = step[i + 8];
        }
    }
}

fn write_row(
    log: &mut impl Write,
    filter: &Gmekf,
    diag: &Diagnostics,
    health: &NavHealth,
    max_step: Duration,
    mean_step: Duration,
) -> std::io::Result<()> {
    let (Some(s), Some((ab, gb, bb))) = (filter.nav_state(), filter.biases()) else {
        return Ok(());
    };
    let q = s.attitude;
    let mut row = vec![s.time.as_secs_f64()];
    row.extend(s.position.iter());
    row.extend(s.velocity.iter());
    row.extend([q.w, q.i, q.j, q.k]);
    row.extend(s.body_rate.iter());
    row.extend(s.position_std.iter());
    row.extend(s.velocity_std.iter());
    row.extend(s.attitude_std.iter());
    row.extend(ab.iter());
    row.extend(gb.iter());
    row.push(bb);
    row.extend(diag);
    row.push(max_step.as_secs_f64() * 1e6);
    row.push(mean_step.as_secs_f64() * 1e6);
    row.push(health.code());
    let text: Vec<String> = row.iter().map(|v| format!("{v:.9}")).collect();
    writeln!(log, "{}", text.join(","))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fix(lat: f64, lon: f64, alt: f64) -> GpsState {
        GpsState {
            latitude_deg: lat,
            longitude_deg: lon,
            altitude_m: alt,
            north_mps: 0.0,
            east_mps: 0.0,
            down_mps: 0.0,
            timestamp_unix_ms: None,
            has_fix: true,
            num_satellites: 12,
        }
    }

    #[test]
    fn gps_to_ned_is_metres_from_the_origin() {
        let origin = GpsOrigin::new(&fix(33.7756, -84.3963, 300.0), Vector3::zeros());
        // 1e-5 deg of latitude is about 1.109 m at 33.8 deg N, of longitude
        // about 0.925 m.
        let ned = origin.ned(&fix(33.77561, -84.39629, 302.0));
        assert!((ned.x - 1.109).abs() < 0.002, "north {}", ned.x);
        assert!((ned.y - 0.925).abs() < 0.002, "east {}", ned.y);
        assert!((ned.z + 2.0).abs() < 1e-9, "down {}", ned.z);
    }

    #[test]
    fn config_file_fields_are_all_optional() {
        let c: ShadowNavConfig = serde_json::from_str("{}").unwrap();
        assert_eq!(c, ShadowNavConfig::default());
        let c: ShadowNavConfig =
            serde_json::from_str(r#"{"imu_to_body": [[0,0,-1],[0,1,0],[1,0,0]], "baro_std": 1.0}"#)
                .unwrap();
        assert_eq!(c.imu_to_body.rotation().unwrap()[(0, 2)], -1.0);
        assert_eq!(c.filter_config().baro_std, 1.0);
        assert!(serde_json::from_str::<ShadowNavConfig>(r#"{"typo": 1}"#).is_err());
    }

    #[test]
    fn health_checks_each_limit() {
        let config = ShadowNavConfig::default();
        let mut s = NavState::unknown(Duration::ZERO);
        s.position_std = Vector3::repeat(0.5);
        s.velocity_std = Vector3::repeat(0.05);
        s.attitude_std = Vector3::new(0.1, 0.01, 0.01);
        s.body_rate_std = Vector3::repeat(0.01);
        assert_eq!(assess(&config, &s, 0, 0), NavHealth::Healthy);

        let unhealthy = |s: &NavState, baro, gps| {
            matches!(assess(&config, s, baro, gps), NavHealth::Unhealthy(_))
        };
        let mut bad = s;
        bad.position_std.y = 6.0;
        assert!(unhealthy(&bad, 0, 0));
        let mut bad = s;
        bad.velocity_std.z = 0.6;
        assert!(unhealthy(&bad, 0, 0));
        let mut bad = s;
        bad.attitude_std.z = 4.0_f64.to_radians();
        assert!(unhealthy(&bad, 0, 0));
        let mut bad = s;
        bad.attitude_std.x = 31.0_f64.to_radians();
        assert!(unhealthy(&bad, 0, 0));
        assert!(unhealthy(&s, 10, 0));
        assert!(unhealthy(&s, 0, 10));
        assert_eq!(assess(&config, &s, 9, 9), NavHealth::Healthy);

        let mut n = 0;
        for result in [0.0, 0.0, -1.0, 0.0] {
            count_rejections(&mut n, result);
        }
        assert_eq!(n, 3, "no reading leaves the count alone");
        count_rejections(&mut n, 1.0);
        assert_eq!(n, 0);
    }

    #[test]
    fn a_stale_estimate_is_unhealthy() {
        let now = Instant::now();
        let healthy = Published {
            state: Some(NavState::unknown(Duration::ZERO)),
            health: NavHealth::Healthy,
            imu_read_at: Some(now),
        };
        let timeout = Duration::from_millis(50);
        assert_eq!(
            read_published(&healthy, now, timeout).health,
            NavHealth::Healthy
        );
        let later = now + Duration::from_millis(51);
        assert!(matches!(
            read_published(&healthy, later, timeout).health,
            NavHealth::Unhealthy(_)
        ));
        // A failure stays a failure, however fresh.
        let failed = Published {
            health: NavHealth::Failed("x"),
            ..healthy
        };
        assert_eq!(
            read_published(&failed, now, timeout).health,
            NavHealth::Failed("x")
        );
    }

    #[test]
    fn imu_axes_build_the_rotation() {
        let axes = |a: [&str; 3]| ImuMounting::Axes(a.map(String::from));
        assert_eq!(
            ImuMounting::default().rotation().unwrap(),
            Matrix3::identity()
        );

        // Body x along IMU -z, body y along IMU +y, body z along IMU +x: a
        // 90 deg rotation about y.
        let r = axes(["-z", "+y", "x"]).rotation().unwrap();
        assert_eq!(r * Vector3::new(0.0, 0.0, -1.0), Vector3::x());
        assert_eq!(r * Vector3::x(), Vector3::z());
        assert_eq!(
            r,
            Matrix3::new(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0)
        );
        let json: ShadowNavConfig =
            serde_json::from_str(r#"{"imu_to_body": ["-Z", "+y", "+x"]}"#).unwrap();
        assert_eq!(json.imu_to_body.rotation().unwrap(), r);

        // Mistakes are caught, not flown.
        assert!(axes(["+x", "+x", "+z"]).rotation().is_err()); // axis twice
        assert!(axes(["+x", "+y", "-z"])
            .rotation()
            .unwrap_err()
            .contains("mirror"));
        assert!(axes(["+x", "+y", "+w"]).rotation().is_err());
        let skewed = ImuMounting::Matrix([[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
        assert!(skewed.rotation().is_err());
    }

    #[test]
    fn tilt_and_heading_of_an_upright_vehicle() {
        let upright = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        let (tilt, _) = tilt_and_heading(&upright);
        assert!(tilt.abs() < 1e-9, "tilt {tilt}");
        let tipped = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 80.0_f64.to_radians());
        let (tilt, _) = tilt_and_heading(&tipped);
        assert!((tilt - 10.0).abs() < 1e-9, "tilt {tilt}");
    }

    /// A still vehicle with the magnetometer: the heading is set from it on
    /// the pad and the readings are fused.
    #[test]
    fn levels_with_the_magnetometer() {
        use common::comm::fc_sensors::Vector as V;
        let dir = tempfile::tempdir().unwrap();
        let config = ShadowNavConfig {
            level_seconds: 0.5,
            // No GPS here, so horizontal velocity is unobserved and its
            // uncertainty passes the default 0.5 m/s limit after ~2 s.
            max_velocity_std_mps: 10.0,
            ..ShadowNavConfig::default()
        };
        let (tx, rx) = mpsc::sync_channel(100_000);
        let path = dir.path().join("log.csv");
        let file = BufWriter::new(File::create(&path).unwrap());

        // Body x up, tilted 3 deg, heading 40 deg.
        let truth = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 40.0_f64.to_radians())
            * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 87.0_f64.to_radians());
        let accel = truth.inverse() * Vector3::new(0.0, 0.0, -9.795);
        let field = truth.inverse() * Vector3::from(config.mag_field_ned);
        let t0 = Instant::now() + Duration::from_millis(50);
        for k in 0..(208 * 3) {
            let at = t0 + Duration::from_secs_f64(k as f64 / 208.0);
            let imu = Imu {
                accelerometer: V {
                    x: accel.x,
                    y: accel.y,
                    z: accel.z,
                },
                gyroscope: V::default(),
            };
            tx.send(Input::Imu { at, imu }).unwrap();
            if k % 10 == 0 {
                tx.send(Input::Baro {
                    at,
                    pressure: 98_000.0,
                    temperature: 20.0,
                })
                .unwrap();
            }
            // Read faster than the 10 Hz output rate, like the worker.
            if k % 4 == 0 {
                tx.send(Input::Mag { at, field }).unwrap();
            }
        }
        drop(tx);
        let output = Arc::new(Mutex::new(Published::leveling()));
        run(config, rx, file, path.clone(), output.clone());

        let text = std::fs::read_to_string(&path).unwrap();
        let rows: Vec<Vec<f64>> = text
            .lines()
            .skip(1)
            .map(|l| l.split(',').map(|v| v.parse().unwrap()).collect())
            .collect();
        let last = rows.last().unwrap();
        let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            last[7], last[8], last[9], last[10],
        ));
        let error = q.angle_to(&truth).to_degrees();
        assert!(error < 0.5, "attitude {error} deg from the truth");
        // mag_ok is column 37: some readings accepted.
        let accepted = rows.iter().filter(|r| r[37] == 1.0).count();
        assert!(accepted > 5, "{accepted} magnetometer updates accepted");

        // The last published estimate is healthy and is the logged one.
        let published = *output.lock().unwrap();
        assert_eq!(published.health, NavHealth::Healthy);
        let state = published.state.unwrap();
        assert!(UnitQuaternion::from_quaternion(state.attitude).angle_to(&truth) < 0.01);
        assert_eq!(*last.last().unwrap(), 1.0, "health column");
    }

    /// Feeds a still vehicle through the thread's code path and checks that
    /// it levels, starts and stays near the pad.
    #[test]
    fn levels_and_tracks_a_still_vehicle() {
        let dir = tempfile::tempdir().unwrap();
        let config = ShadowNavConfig {
            level_seconds: 0.5,
            ..ShadowNavConfig::default()
        };
        let (tx, rx) = mpsc::sync_channel(100_000);
        let path = dir.path().join("log.csv");
        let file = BufWriter::new(File::create(&path).unwrap());

        // Sensor axes tilted 3 deg from upright, body x up.
        let t0 = Instant::now() + Duration::from_millis(50);
        let tilt = UnitQuaternion::from_euler_angles(0.0, 3.0_f64.to_radians(), 0.0);
        let f = tilt.inverse() * Vector3::new(9.795, 0.0, 0.0);
        for k in 0..(208 * 3) {
            let at = t0 + Duration::from_secs_f64(k as f64 / 208.0);
            let imu = Imu {
                accelerometer: common::comm::fc_sensors::Vector {
                    x: f.x,
                    y: f.y,
                    z: f.z,
                },
                gyroscope: common::comm::fc_sensors::Vector {
                    x: 0.2,
                    y: -0.1,
                    z: 0.05,
                },
            };
            tx.send(Input::Imu { at, imu }).unwrap();
            if k % 10 == 0 {
                tx.send(Input::Baro {
                    at,
                    pressure: 98_000.0,
                    temperature: 20.0,
                })
                .unwrap();
            }
        }
        drop(tx);
        let output = Arc::new(Mutex::new(Published::leveling()));
        run(config, rx, file, path.clone(), output.clone());

        let text = std::fs::read_to_string(&path).unwrap();
        let rows: Vec<Vec<f64>> = text
            .lines()
            .skip(1)
            .map(|l| l.split(',').map(|v| v.parse().unwrap()).collect())
            .collect();
        assert!(rows.len() > 100, "only {} log rows", rows.len());
        let last = rows.last().unwrap();
        let (pos, vel) = (&last[1..4], &last[4..7]);
        assert!(pos.iter().all(|p| p.abs() < 0.05), "drifted to {pos:?}");
        assert!(vel.iter().all(|v| v.abs() < 0.05), "velocity {vel:?}");
        // Gyro bias found from the pad average, deg/s -> rad/s.
        let gyro_bias = &last[26..29];
        assert!(
            (gyro_bias[0] - 0.2_f64.to_radians()).abs() < 1e-4,
            "gyro bias {gyro_bias:?}"
        );
    }
}
