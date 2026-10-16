#![allow(missing_docs)]

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::HashSet;

pub const SCHEMA_VERSION: u32 = 1;
pub const MAX_BUNDLE_BYTES: usize = 1024 * 1024;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Bundle {
    pub schema_version: u32,
    pub vehicle_id: String,
    pub mapping_configuration_id: String,
    pub compatibility: Compatibility,
    pub controller: Controller,
    pub navigation: Navigation,
    pub fdir: Fdir,
    pub tvc: [TvcAxis; 2],
    pub ec: EngineController,
    pub startup: Startup,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Compatibility {
    pub fc_firmware: String,
    pub ec_firmware: String,
    pub protocol_version: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Controller {
    pub max_thrust_n: f64,
    pub max_rcs_torque_nm: f64,
    pub law: ControlLaw,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ControlLaw {
    Pid {
        kp: [f64; 4],
        ki: [f64; 4],
        kd: [f64; 4],
    },
    Lqr {
        tgrid_s: Vec<f64>,
        xref: Vec<Vec<f64>>,
        k1flat: Vec<Vec<f64>>,
        k2grid: Vec<Vec<f64>>,
        unom: Vec<Vec<f64>>,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavigationKind {
    Disabled,
    VerticalKalman,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Navigation {
    pub kind: NavigationKind,
    pub up_axis: [f64; 3],
    pub gravity_m_s2: f64,
    pub accel_noise_density_m_s2_per_sqrt_hz: f64,
    pub baro_reference_pressure_pa: f64,
    pub baro_reference_temperature_k: f64,
    pub baro_std_m: f64,
    pub lidar_offsets_m: Vec<f64>,
    pub lidar_min_range_m: f64,
    pub lidar_max_range_m: f64,
    pub lidar_std_m: f64,
    pub gps_position_std_m: f64,
    pub gps_velocity_std_m_s: f64,
    pub max_measurement_age_ms: u32,
    pub gate_sigma: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultAction {
    Report,
    InhibitArm,
    Abort,
    Terminate,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FaultRule {
    pub fault_id: String,
    pub action: FaultAction,
    pub latched: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fdir {
    pub fault_catalog: Vec<String>,
    pub rules: Vec<FaultRule>,
    pub command_timeout_ms: u32,
    pub navigation_required: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TvcAxis {
    pub pivot_radius_m: f64,
    pub structural_distance_m: f64,
    pub centered_length_m: f64,
    pub retracted_length_m: f64,
    pub stroke_mm: f64,
    pub inverted: bool,
    pub max_angle_rad: f64,
    pub max_rate_rad_s: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Actuator {
    pub model: String,
    pub supply_voltage_v: f64,
    pub uart_port: u8,
    pub device_id: u8,
    pub baud: u32,
    pub feedback_return_mode: u8,
    pub calibration_id: String,
    pub min_extension_mm: f64,
    pub max_extension_mm: f64,
    pub min_position_counts: u16,
    pub max_position_counts: u16,
    pub response_timeout_ms: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fan {
    pub name: String,
    pub location: String,
    pub torque_direction: i8,
    pub calibration_id: String,
    pub startup_duty: f64,
    pub deadband_duty: f64,
    pub max_current_a: f64,
    pub duty_to_torque_nm: Vec<[f64; 2]>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Rcs {
    pub pwm_hz: u32,
    pub max_duty: f64,
    pub max_slew_per_s: f64,
    pub command_timeout_ms: u32,
    pub fans: [Fan; 4],
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Esc {
    pub model: String,
    pub firmware: String,
    pub calibration_id: String,
    pub pwm_hz: u32,
    pub min_pulse_us: u32,
    pub max_pulse_us: u32,
    pub disarmed_pulse_us: u32,
    pub max_throttle: f64,
    pub max_slew_per_s: f64,
    pub command_timeout_ms: u32,
    pub signal_loss_behavior: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EngineController {
    pub actuators: [Actuator; 2],
    pub rcs: Rcs,
    pub esc: Esc,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Workers {
    pub imu: bool,
    pub gps_reco: bool,
    pub magnetometer: bool,
    pub barometer: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Logging {
    pub enabled: bool,
    pub directories: Vec<String>,
    pub buffer_samples: u32,
    pub rotation_bytes: u64,
    pub fsync_rate: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Network {
    pub servo_address: String,
    pub fc_bind_address: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Timing {
    pub telemetry_interval_ms: u32,
    pub heartbeat_interval_ms: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Startup {
    pub workers: Workers,
    pub logging: Logging,
    pub network: Network,
    pub timing: Timing,
    pub abort_on_servo_disconnect: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ValidationError {
    pub path: String,
    pub reason: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Lifecycle {
    Candidate,
    Uploaded,
    Validation,
    Validated,
    Staged,
    Active,
    Rejected,
    PreviousKnownGood,
}

impl Lifecycle {
    pub fn transition(
        self,
        next: Self,
        fc_confirmed: bool,
        ec_confirmed: bool,
    ) -> Result<Self, &'static str> {
        use Lifecycle::*;
        let allowed = matches!(
            (self, next),
            (Candidate, Uploaded)
                | (Uploaded, Validation)
                | (Validation, Validated | Rejected)
                | (Validated, Staged)
                | (Staged, Active | Rejected)
                | (Active, PreviousKnownGood)
                | (PreviousKnownGood, Staged)
        );
        if !allowed {
            return Err("invalid configuration lifecycle transition");
        }
        if matches!(next, Staged | Active | PreviousKnownGood) && !(fc_confirmed && ec_confirmed) {
            return Err("FC and EC confirmation is required");
        }
        Ok(next)
    }
}

pub fn content_hash(value: &impl Serialize) -> Result<String, serde_json::Error> {
    let bytes = serde_json::to_vec(value)?;
    Ok(hash_bytes(&bytes))
}

pub fn hash_bytes(bytes: &[u8]) -> String {
    format!("sha256:{:x}", Sha256::digest(bytes))
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BatterySample {
    pub source_id: String,
    pub voltage_v: f64,
    pub current_a: f64,
    pub sample_time_s: f64,
    pub observed_age_ms: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FcReadout {
    pub protocol_version: u32,
    pub request_id: u64,
    pub boot_id: String,
    pub uptime_ms: u64,
    pub configuration: serde_json::Value,
    pub battery: Option<BatterySample>,
    pub legacy_abort: serde_json::Value,
}

impl FcReadout {
    pub fn valid_for(&self, request_id: u64) -> bool {
        self.protocol_version == 1
            && self.request_id == request_id
            && !self.boot_id.is_empty()
            && self.boot_id.len() <= 128
            && self.configuration.is_object()
            && self.battery.as_ref().is_none_or(|b| {
                [
                    b.voltage_v,
                    b.current_a,
                    b.sample_time_s,
                    b.observed_age_ms,
                    b.voltage_v * b.current_a,
                ]
                .iter()
                .all(|v| v.is_finite())
                    && b.voltage_v >= 0.0
                    && b.sample_time_s >= 0.0
                    && b.observed_age_ms >= 0.0
            })
    }
}

impl Bundle {
    pub fn parse(bytes: &[u8]) -> Result<Self, Vec<ValidationError>> {
        let error = |path: String, reason: String| vec![ValidationError { path, reason }];
        if bytes.len() > MAX_BUNDLE_BYTES {
            return Err(error("$".into(), "bundle exceeds 1 MiB".into()));
        }
        let mut deserializer = serde_json::Deserializer::from_slice(bytes);
        let bundle: Self = serde_path_to_error::deserialize(&mut deserializer)
            .map_err(|e| error(e.path().to_string(), e.inner().to_string()))?;
        deserializer
            .end()
            .map_err(|e| error("$".into(), e.to_string()))?;
        let errors = bundle.validate();
        if errors.is_empty() {
            Ok(bundle)
        } else {
            Err(errors)
        }
    }

    pub fn validate(&self) -> Vec<ValidationError> {
        let mut errors = Vec::new();
        let mut check = |ok: bool, path: String, reason: &str| {
            if !ok {
                errors.push(ValidationError {
                    path,
                    reason: reason.into(),
                });
            }
        };
        macro_rules! require {
            ($ok:expr, $path:expr, $why:expr) => {
                check($ok, $path.to_string(), $why)
            };
        }
        macro_rules! positive {
            ($value:expr, $path:expr) => {
                require!(
                    $value.is_finite() && $value > 0.0,
                    $path,
                    "must be finite and positive"
                )
            };
        }
        macro_rules! fraction {
            ($value:expr, $path:expr) => {
                require!(
                    $value.is_finite() && (0.0..=1.0).contains(&$value),
                    $path,
                    "must be finite and between 0 and 1"
                )
            };
        }
        require!(
            self.schema_version == SCHEMA_VERSION,
            "schema_version",
            "unsupported schema version"
        );
        for (path, value) in [
            ("vehicle_id", &self.vehicle_id),
            ("mapping_configuration_id", &self.mapping_configuration_id),
            ("compatibility.fc_firmware", &self.compatibility.fc_firmware),
            ("compatibility.ec_firmware", &self.compatibility.ec_firmware),
        ] {
            require!(
                !value.trim().is_empty() && value.len() <= 128,
                path,
                "must contain 1 to 128 bytes"
            );
        }
        require!(
            self.compatibility.protocol_version == 1,
            "compatibility.protocol_version",
            "unsupported protocol version"
        );
        positive!(self.controller.max_thrust_n, "controller.max_thrust_n");
        positive!(
            self.controller.max_rcs_torque_nm,
            "controller.max_rcs_torque_nm"
        );
        match &self.controller.law {
            ControlLaw::Pid { kp, ki, kd } => {
                for (name, values) in [("kp", kp), ("ki", ki), ("kd", kd)] {
                    for (i, value) in values.iter().enumerate() {
                        require!(
                            value.is_finite(),
                            format!("controller.law.{name}[{i}]"),
                            "must be finite"
                        );
                    }
                }
            }
            ControlLaw::Lqr {
                tgrid_s,
                xref,
                k1flat,
                k2grid,
                unom,
            } => {
                let n = tgrid_s.len();
                require!(
                    (2..=4096).contains(&n),
                    "controller.law.tgrid_s",
                    "must contain 2 to 4096 reference times"
                );
                for (i, time) in tgrid_s.iter().enumerate() {
                    require!(
                        time.is_finite() && *time >= 0.0 && (i == 0 || *time > tgrid_s[i - 1]),
                        format!("controller.law.tgrid_s[{i}]"),
                        "must be finite, nonnegative, and strictly increasing"
                    );
                }
                for (name, matrix, width) in [
                    ("xref", xref, 13),
                    ("k1flat", k1flat, 52),
                    ("k2grid", k2grid, 4),
                    ("unom", unom, 4),
                ] {
                    require!(
                        matrix.len() == n,
                        format!("controller.law.{name}"),
                        "row count must match tgrid_s"
                    );
                    for (i, row) in matrix.iter().enumerate() {
                        require!(
                            row.len() == width,
                            format!("controller.law.{name}[{i}]"),
                            &format!("must contain {width} columns")
                        );
                        for (j, value) in row.iter().enumerate() {
                            require!(
                                value.is_finite(),
                                format!("controller.law.{name}[{i}][{j}]"),
                                "must be finite"
                            );
                        }
                    }
                }
            }
        }
        let nav = &self.navigation;
        for (path, value) in [
            ("gravity_m_s2", nav.gravity_m_s2),
            (
                "accel_noise_density_m_s2_per_sqrt_hz",
                nav.accel_noise_density_m_s2_per_sqrt_hz,
            ),
            ("baro_reference_pressure_pa", nav.baro_reference_pressure_pa),
            (
                "baro_reference_temperature_k",
                nav.baro_reference_temperature_k,
            ),
            ("baro_std_m", nav.baro_std_m),
            ("lidar_min_range_m", nav.lidar_min_range_m),
            ("lidar_max_range_m", nav.lidar_max_range_m),
            ("lidar_std_m", nav.lidar_std_m),
            ("gps_position_std_m", nav.gps_position_std_m),
            ("gps_velocity_std_m_s", nav.gps_velocity_std_m_s),
            ("gate_sigma", nav.gate_sigma),
        ] {
            positive!(value, format!("navigation.{path}"));
        }
        require!(
            nav.up_axis.iter().all(|v| v.is_finite())
                && (nav.up_axis.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() < 1e-6,
            "navigation.up_axis",
            "must be a finite unit vector"
        );
        require!(
            nav.lidar_min_range_m < nav.lidar_max_range_m,
            "navigation.lidar_max_range_m",
            "must exceed minimum range"
        );
        require!(
            nav.lidar_offsets_m.len() <= 4,
            "navigation.lidar_offsets_m",
            "at most four LiDARs are supported"
        );
        for (i, v) in nav.lidar_offsets_m.iter().enumerate() {
            require!(
                v.is_finite(),
                format!("navigation.lidar_offsets_m[{i}]"),
                "must be finite"
            );
        }
        require!(
            nav.max_measurement_age_ms > 0,
            "navigation.max_measurement_age_ms",
            "must be positive"
        );
        require!(
            !self.fdir.navigation_required || nav.kind != NavigationKind::Disabled,
            "fdir.navigation_required",
            "navigation cannot be required while disabled"
        );
        require!(
            self.fdir.command_timeout_ms > 0,
            "fdir.command_timeout_ms",
            "must be positive"
        );
        let mut catalog = HashSet::new();
        for (i, id) in self.fdir.fault_catalog.iter().enumerate() {
            require!(
                !id.trim().is_empty() && catalog.insert(id),
                format!("fdir.fault_catalog[{i}]"),
                "fault IDs must be nonempty and unique"
            );
        }
        for (i, rule) in self.fdir.rules.iter().enumerate() {
            require!(
                catalog.contains(&rule.fault_id),
                format!("fdir.rules[{i}].fault_id"),
                "unknown fault catalog reference"
            );
        }
        for (i, axis) in self.tvc.iter().enumerate() {
            let path = format!("tvc[{i}]");
            for (name, value) in [
                ("pivot_radius_m", axis.pivot_radius_m),
                ("structural_distance_m", axis.structural_distance_m),
                ("centered_length_m", axis.centered_length_m),
                ("retracted_length_m", axis.retracted_length_m),
                ("stroke_mm", axis.stroke_mm),
                ("max_angle_rad", axis.max_angle_rad),
                ("max_rate_rad_s", axis.max_rate_rad_s),
            ] {
                positive!(value, format!("{path}.{name}"));
            }
            let (r, d, l) = (
                axis.pivot_radius_m,
                axis.structural_distance_m,
                axis.centered_length_m,
            );
            let triangle = l > (r - d).abs() && l < r + d;
            require!(
                triangle,
                format!("{path}.centered_length_m"),
                "geometry must form a nondegenerate triangle"
            );
            require!(
                axis.max_angle_rad < std::f64::consts::PI,
                format!("{path}.max_angle_rad"),
                "must be below pi radians"
            );
            if triangle {
                let phi = ((r * r + d * d - l * l) / (2.0 * r * d)).acos();
                let fits = phi > axis.max_angle_rad
                    && phi + axis.max_angle_rad < std::f64::consts::PI
                    && [-axis.max_angle_rad, axis.max_angle_rad]
                        .iter()
                        .all(|angle| {
                            let extension = ((r * r + d * d - 2.0 * r * d * (phi + angle).cos())
                                .sqrt()
                                - axis.retracted_length_m)
                                * 1000.0;
                            extension >= self.ec.actuators[i].min_extension_mm
                                && extension <= self.ec.actuators[i].max_extension_mm
                        });
                require!(
                    fits,
                    format!("{path}.max_angle_rad"),
                    "gimbal range exceeds calibrated actuator travel"
                );
            }
        }
        for (i, actuator) in self.ec.actuators.iter().enumerate() {
            let path = format!("ec.actuators[{i}]");
            require!(
                !actuator.model.trim().is_empty(),
                format!("{path}.model"),
                "model is required"
            );
            require!(
                actuator.supply_voltage_v == 12.0,
                format!("{path}.supply_voltage_v"),
                "CTV actuator rail requires the 12 V model"
            );
            require!(
                actuator.uart_port == [7, 5][i],
                format!("{path}.uart_port"),
                "A1 uses UART7; A2 uses UART5"
            );
            require!(
                actuator.device_id <= 253,
                format!("{path}.device_id"),
                "broadcast ID is not permitted"
            );
            require!(
                [9600, 19200, 38600, 57600, 115200].contains(&actuator.baud),
                format!("{path}.baud"),
                "unsupported baud rate"
            );
            require!(
                [1, 2].contains(&actuator.feedback_return_mode),
                format!("{path}.feedback_return_mode"),
                "read feedback must be enabled"
            );
            require!(
                !actuator.calibration_id.trim().is_empty(),
                format!("{path}.calibration_id"),
                "calibration identity is required"
            );
            require!(
                actuator.min_extension_mm.is_finite() && actuator.min_extension_mm >= 0.0,
                format!("{path}.min_extension_mm"),
                "must be finite and nonnegative"
            );
            require!(
                actuator.max_extension_mm.is_finite()
                    && actuator.max_extension_mm > actuator.min_extension_mm
                    && actuator.max_extension_mm <= self.tvc[i].stroke_mm,
                format!("{path}.max_extension_mm"),
                "must exceed minimum and fit configured stroke"
            );
            require!(
                actuator.min_position_counts < actuator.max_position_counts
                    && actuator.max_position_counts <= 4095,
                format!("{path}.max_position_counts"),
                "calibration counts must increase within 0..4095"
            );
            require!(
                actuator.response_timeout_ms > 0,
                format!("{path}.response_timeout_ms"),
                "must be positive"
            );
        }
        let rcs = &self.ec.rcs;
        require!(
            (1..=100_000).contains(&rcs.pwm_hz),
            "ec.rcs.pwm_hz",
            "must be between 1 and 100000 Hz"
        );
        fraction!(rcs.max_duty, "ec.rcs.max_duty");
        positive!(rcs.max_slew_per_s, "ec.rcs.max_slew_per_s");
        require!(
            rcs.command_timeout_ms > 0,
            "ec.rcs.command_timeout_ms",
            "must be positive"
        );
        let mut names = HashSet::new();
        for (i, fan) in rcs.fans.iter().enumerate() {
            let path = format!("ec.rcs.fans[{i}]");
            require!(
                !fan.name.trim().is_empty() && names.insert(&fan.name),
                format!("{path}.name"),
                "fan names must be nonempty and unique"
            );
            require!(
                !fan.location.trim().is_empty() && !fan.calibration_id.trim().is_empty(),
                &path,
                "location and calibration identity are required"
            );
            require!(
                [-1, 1].contains(&fan.torque_direction),
                format!("{path}.torque_direction"),
                "must be -1 or 1"
            );
            fraction!(fan.startup_duty, format!("{path}.startup_duty"));
            fraction!(fan.deadband_duty, format!("{path}.deadband_duty"));
            require!(
                fan.startup_duty <= rcs.max_duty && fan.deadband_duty <= fan.startup_duty,
                &path,
                "deadband <= startup <= maximum duty is required"
            );
            positive!(fan.max_current_a, format!("{path}.max_current_a"));
            for (j, pair) in fan.duty_to_torque_nm.iter().enumerate() {
                require!(
                    pair.iter().all(|v| v.is_finite())
                        && (0.0..=rcs.max_duty).contains(&pair[0])
                        && (j == 0 || pair[0] > fan.duty_to_torque_nm[j - 1][0]),
                    format!("{path}.duty_to_torque_nm[{j}]"),
                    "map requires finite values and strictly increasing duties within limits"
                );
            }
        }
        let esc = &self.ec.esc;
        for (name, text) in [
            ("model", &esc.model),
            ("firmware", &esc.firmware),
            ("calibration_id", &esc.calibration_id),
            ("signal_loss_behavior", &esc.signal_loss_behavior),
        ] {
            require!(
                !text.trim().is_empty(),
                format!("ec.esc.{name}"),
                "must be specified"
            );
        }
        require!(
            esc.pwm_hz > 0
                && esc.pwm_hz <= 100_000
                && esc.min_pulse_us > 0
                && esc.min_pulse_us < esc.max_pulse_us
                && u64::from(esc.max_pulse_us) * u64::from(esc.pwm_hz) < 1_000_000,
            "ec.esc.max_pulse_us",
            "pulse endpoints must increase and fit the PWM period"
        );
        require!(
            esc.disarmed_pulse_us == 0
                || (esc.min_pulse_us..=esc.max_pulse_us).contains(&esc.disarmed_pulse_us),
            "ec.esc.disarmed_pulse_us",
            "must be zero (no pulse) or within endpoints"
        );
        fraction!(esc.max_throttle, "ec.esc.max_throttle");
        positive!(esc.max_slew_per_s, "ec.esc.max_slew_per_s");
        require!(
            esc.command_timeout_ms > 0,
            "ec.esc.command_timeout_ms",
            "must be positive"
        );
        let startup = &self.startup;
        require!(
            startup.logging.buffer_samples > 0 && startup.logging.rotation_bytes > 0,
            "startup.logging",
            "buffer and rotation sizes must be positive"
        );
        require!(
            !startup.logging.enabled
                || (!startup.logging.directories.is_empty()
                    && startup
                        .logging
                        .directories
                        .iter()
                        .all(|p| !p.trim().is_empty())),
            "startup.logging.directories",
            "enabled logging requires output directories"
        );
        for (path, address) in [
            (
                "startup.network.servo_address",
                &startup.network.servo_address,
            ),
            (
                "startup.network.fc_bind_address",
                &startup.network.fc_bind_address,
            ),
        ] {
            require!(
                address.parse::<std::net::SocketAddr>().is_ok(),
                path,
                "must be an IP socket address"
            );
        }
        require!(
            startup.timing.telemetry_interval_ms > 0 && startup.timing.heartbeat_interval_ms > 0,
            "startup.timing",
            "intervals must be positive"
        );
        errors
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn example() -> Bundle {
        Bundle::parse(include_bytes!("../examples/ctv-bench.json")).unwrap()
    }

    #[test]
    fn example_round_trip_and_hash_ignore_input_formatting() {
        let bundle = example();
        let pretty = serde_json::to_vec_pretty(&bundle).unwrap();
        assert_eq!(
            content_hash(&bundle).unwrap(),
            content_hash(&Bundle::parse(&pretty).unwrap()).unwrap()
        );
    }

    #[test]
    fn rejects_unknown_fields_versions_and_trailing_data() {
        let mut value = serde_json::to_value(example()).unwrap();
        value["ec"]["esc"]["rpm"] = 10.into();
        let errors = Bundle::parse(&serde_json::to_vec(&value).unwrap()).unwrap_err();
        assert!(errors[0].path.contains("ec.esc"));
        let mut bundle = example();
        bundle.schema_version = 99;
        assert!(bundle.validate().iter().any(|e| e.path == "schema_version"));
        let mut bytes = serde_json::to_vec(&example()).unwrap();
        bytes.extend_from_slice(b" {}");
        assert!(Bundle::parse(&bytes).is_err());
    }

    #[test]
    fn reports_matrix_grid_finite_and_fault_errors() {
        let mut bundle = example();
        bundle.controller.law = ControlLaw::Lqr {
            tgrid_s: vec![0.0, 0.0],
            xref: vec![vec![0.0; 12]; 2],
            k1flat: vec![vec![0.0; 52]; 2],
            k2grid: vec![vec![0.0; 4]; 2],
            unom: vec![vec![0.0; 4]; 2],
        };
        bundle.ec.rcs.max_duty = f64::NAN;
        bundle.fdir.rules[0].fault_id = "missing".into();
        let errors = bundle.validate();
        for path in [
            "controller.law.tgrid_s[1]",
            "controller.law.xref[0]",
            "ec.rcs.max_duty",
            "fdir.rules[0].fault_id",
        ] {
            assert!(errors.iter().any(|e| e.path == path), "{path}");
        }
    }

    #[test]
    fn rejects_actuator_and_waveform_limits() {
        let mut bundle = example();
        bundle.ec.actuators[0].supply_voltage_v = 7.4;
        bundle.ec.actuators[1].device_id = 254;
        bundle.ec.esc.pwm_hz = 1000;
        bundle.tvc[0].max_angle_rad = 1.0;
        let errors = bundle.validate();
        for path in [
            "ec.actuators[0].supply_voltage_v",
            "ec.actuators[1].device_id",
            "ec.esc.max_pulse_us",
            "tvc[0].max_angle_rad",
        ] {
            assert!(errors.iter().any(|e| e.path == path), "{path}");
        }
    }

    #[test]
    fn lifecycle_requires_device_evidence() {
        use Lifecycle::*;
        assert_eq!(
            Uploaded.transition(Validation, false, false),
            Ok(Validation)
        );
        assert_eq!(Validation.transition(Rejected, false, false), Ok(Rejected));
        assert!(Validated.transition(Staged, true, false).is_err());
        assert!(Validated.transition(Active, true, true).is_err());
        assert_eq!(Validated.transition(Staged, true, true), Ok(Staged));
        assert!(Active.transition(PreviousKnownGood, false, false).is_err());
    }
}
