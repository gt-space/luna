use std::time::Duration;

use nalgebra::SMatrix;
pub use nalgebra::{geometry::Quaternion, Vector3};
use rkyv::Archive;
use serde::{Deserialize, Serialize};

/// Full control state of the CTV, used as input to the control loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlState {
    /// Time since the start of the control loop.
    pub time: Duration,
    /// Position in the local frame.
    pub position: Vector3<f64>,
    /// Velocity in the local frame.
    pub velocity: Vector3<f64>,
    /// Angular rate in the body frame.
    pub body_rate: Vector3<f64>,
    /// Attitude quaternion.
    pub attitude: Quaternion<f64>,
}

impl ControlState {
    /// Converts this state into a 13x1 matrix suitable for the control loop.
    pub fn to_matrix(&self) -> SMatrix<f64, 13, 1> {
        SMatrix::from_column_slice(&[
            self.position.x,
            self.position.y,
            self.position.z,
            self.velocity.x,
            self.velocity.y,
            self.velocity.z,
            self.body_rate.x,
            self.body_rate.y,
            self.body_rate.z,
            // Scalar first (w, i, j, k): the order of hopper's 13-state, and of
            // the golden vectors the controller is tested against
            // (`matches_the_simulink_golden_vectors`).
            self.attitude.w,
            self.attitude.i,
            self.attitude.j,
            self.attitude.k,
        ])
    }
}

/// A control vector sent to the CTV to command thrust, TVC, and RCS.
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Serialize,
    Deserialize,
    Archive,
    rkyv::Serialize,
    rkyv::Deserialize,
)]
pub struct ControlVector {
    /// Commanded thrust.
    pub thrust: f64,
    /// Commanded TVC pitch angle.
    pub tvc_pitch: f64,
    /// Commanded TVC yaw angle.
    pub tvc_yaw: f64,
    /// Commanded RCS torque.
    pub rcs_torque: f64,
}

impl ControlVector {
    /// Constructs a `ControlVector` from a 4x1 matrix.
    pub fn from_matrix(mat: SMatrix<f64, 4, 1>) -> Self {
        ControlVector {
            thrust: mat[0],
            tvc_pitch: mat[1],
            tvc_yaw: mat[2],
            rcs_torque: mat[3],
        }
    }
}

/// Actuator demands in the units the engine controller takes.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ActuatorDemand {
    /// Rod extension of TVC actuators A1 and A2, in mm (0 to the stroke).
    pub tvc_extension_mm: [f64; 2],
    /// Duty of RCS fans M1 to M4, from 0 to 1.
    pub rcs_duty: [f64; 4],
    /// Main ESC throttle, from 0 to 1.
    pub main_throttle: f64,
}

/// A request for the CTV engine controller, from a sequence or operator.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CtvControlMessage {
    /// Thrust, gimbal angles and roll torque, which the FC converts into
    /// actuator demands.
    Control(ControlVector),
    /// Actuator demands as given, for bench testing.
    Actuators(ActuatorDemand),
    /// Gives the EC output authority. Demands must already be flowing, or the
    /// EC safes itself.
    Arm,
    /// Takes output authority away: fans and ESC off, TVC holds.
    Disarm,
    /// Safes the EC immediately and latches it until `ClearSafing`. Not the
    /// same as an FC abort-to-land, which keeps demanding thrust.
    Terminate,
    /// Clears the EC's safing latch. The EC lands in disarmed, not armed.
    ClearSafing,
}

/// The mode of the CTV flight logic, as the operator sees it.
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Default,
    Serialize,
    Deserialize,
    Archive,
    rkyv::Serialize,
    rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub enum CtvMode {
    /// The flight computer was not started with the flight logic
    /// (`--gnc-gains`).
    #[default]
    Off,
    /// On the pad, not armed.
    Idle,
    /// Pad pose taken, waiting for the start command.
    Armed,
    /// Flying the gain schedule.
    Flight,
    /// A fault latched: flying down from the vehicle's altitude.
    AbortLand,
    /// Engine cut at the pad.
    Landed,
    /// Terminated: the EC is safed.
    Safed,
}

/// One pre-flight check and whether it passes.
#[derive(
    Debug,
    Clone,
    PartialEq,
    Default,
    Serialize,
    Deserialize,
    Archive,
    rkyv::Serialize,
    rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct CtvCheck {
    /// What is checked, for example "Navigation estimate".
    pub name: String,
    /// Whether the check passes.
    pub ok: bool,
    /// Why it fails, or the value it passes with.
    pub detail: String,
}

/// How the navigation filter is doing, for the operator's plots: how sure it is
/// and what it did with each sensor's last reading.
#[derive(
    Debug,
    Clone,
    PartialEq,
    Default,
    Serialize,
    Deserialize,
    Archive,
    rkyv::Serialize,
    rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct CtvNav {
    /// The filter's state: "Healthy", "Leveling", "Unhealthy(...)", "Failed(...)".
    pub health: String,
    /// The uncertainties below are valid: the filter has produced an estimate.
    pub has_estimate: bool,
    /// 1 sigma of the position estimate, m, north, east, down.
    pub position_std_m: [f32; 3],
    /// 1 sigma of the velocity estimate, m/s, north, east, down.
    pub velocity_std_mps: [f32; 3],
    /// 1 sigma of the tilt, degrees (the larger of the two tilt axes).
    pub tilt_std_deg: f32,
    /// 1 sigma of the heading about the vertical, degrees.
    pub heading_std_deg: f32,
    /// What the filter did with each sensor's latest reading: 1 taken, 0
    /// rejected, -1 none yet. Barometer, GPS, magnetometer, then the four LiDARs.
    pub sensor_results: [i8; 7],
    /// There has been a GPS update, so `gps_nis` is valid.
    pub has_gps_nis: bool,
    /// The GPS innovation's normalized square, against a gate of about 17; a
    /// reading above it is rejected.
    pub gps_nis: f32,
}

/// One control period of what the plots show. Units are in the names. A group of
/// values that is not available (no estimate; not flying) is zero, and its bit
/// in `available` is clear: NaN would not survive every format the status is
/// serialized to.
#[derive(
    Debug,
    Clone,
    PartialEq,
    Default,
    Serialize,
    Deserialize,
    Archive,
    rkyv::Serialize,
    rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct CtvSample {
    /// Which groups of values below are valid: see the `CtvSample::` constants.
    pub available: u8,
    /// The flight computer's clock, s.
    pub time_s: f64,
    /// Mission time, s; zero before the start.
    pub mission_s: f32,
    /// The estimate's position in the navigation frame (NED from the filter's
    /// origin, the IMU on the pad), m.
    pub position_ned_m: [f32; 3],
    /// The estimate's velocity in the navigation frame, m/s.
    pub velocity_ned_mps: [f32; 3],
    /// The estimate's body rate, deg/s.
    pub body_rate_dps: [f32; 3],
    /// Tilt from vertical, degrees.
    pub tilt_deg: f32,
    /// The position the controller was given, in the pad frame (north, east,
    /// up), m.
    pub pad_position_m: [f32; 3],
    /// The velocity the controller was given, in the pad frame, m/s.
    pub pad_velocity_mps: [f32; 3],
    /// The position the schedule asks for at this moment, pad frame, m.
    pub reference_position_m: [f32; 3],
    /// The velocity the schedule asks for at this moment, pad frame, m/s.
    pub reference_velocity_mps: [f32; 3],
    /// What the controller asked for: thrust N, TVC pitch and yaw degrees, roll
    /// torque N m.
    pub command: [f32; 4],
}

impl CtvSample {
    /// The estimate: `position_ned_m`, `velocity_ned_mps`, `body_rate_dps`, `tilt_deg`.
    pub const ESTIMATE: u8 = 1;
    /// What the controller was given and the schedule's reference: the `pad_*`
    /// and `reference_*` values. Only while flying.
    pub const CONTROLLER: u8 = 2;
    /// What the controller asked for: `command`. Only while flying.
    pub const COMMAND: u8 = 4;
}

/// What the flight computer tells the operator about the CTV, once per
/// telemetry frame. Everything the CTV operations page shows is here.
#[derive(
    Debug,
    Clone,
    PartialEq,
    Default,
    Serialize,
    Deserialize,
    Archive,
    rkyv::Serialize,
    rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct CtvStatus {
    /// The mode of the flight logic.
    pub mode: CtvMode,
    /// Time since the start command, s. Zero before the start.
    pub mission_time_s: f64,
    /// Seconds in the gain schedule, which an abort to land shifts.
    pub schedule_time_s: f64,
    /// The end of the gain schedule, s.
    pub schedule_end_s: f64,
    /// Height above the pad, m, from the estimate in the pad frame. Zero while
    /// not flying.
    pub altitude_m: f64,
    /// Tilt from vertical, degrees.
    pub tilt_deg: f64,
    /// Speed, m/s.
    pub speed_mps: f64,
    /// The EC link: a session is open.
    pub ec_connected: bool,
    /// The EC's state: "Disarmed", "Armed", "Safing", or empty if unknown.
    pub ec_state: String,
    /// Why the EC safed, if it did.
    pub ec_safing_reason: String,
    /// The latched fault and the response it asked for, for example
    /// "NavLost (R4)", or empty.
    pub fault: String,
    /// The latest event line from the flight logic.
    pub last_event: String,
    /// The controller and the hopper commit its gains came from, for example
    /// "lqr_ctv_tuning @ 38b9708".
    pub gains: String,
    /// Control periods missed since the flight computer started.
    pub missed_ticks: u64,
    /// The checks before a start, and whether each passes.
    pub preflight: Vec<CtvCheck>,
    /// Every pre-flight check passes: the vehicle can be started.
    pub can_start: bool,
    /// How the navigation filter is doing.
    pub nav: CtvNav,
    /// The TVC angle the allocation holds the controller to, degrees.
    pub tvc_limit_deg: f32,
    /// The control periods since the previous status, for the plots.
    pub samples: Vec<CtvSample>,
}

/// What the operator can ask of the CTV from the GUI.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CtvOperatorCommand {
    /// Arm the engine controller: it gets output authority.
    ArmEc,
    /// Take output authority away from the engine controller.
    DisarmEc,
    /// Arm the flight logic from the navigation estimate and start the
    /// mission clock. Needs the EC armed and every pre-flight check passing.
    StartFlight,
    /// Land: abort to land in flight, safe the EC on the ground.
    AbortLand,
    /// Safe the EC immediately and latch it.
    Terminate,
    /// Clear the EC's safing latch.
    ClearSafing,
    /// Throw the navigation filter away and level it again, as at start-up:
    /// the way back from an unhealthy estimate. The vehicle has to be still on
    /// the pad; refused in flight.
    ReinitNav,
}
