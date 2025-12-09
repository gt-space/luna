use super::{ahrs, bms, sam, ValveState, VehicleState};
use serde::{Deserialize, Serialize};
use std::{borrow::Cow, collections::HashMap};

/// String that represents the ID of a data board
pub type BoardId = String;

/// A generic data message that can originate from any subsystem to flight.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum DataMessage<'a> {
  /// Represents the inital handshake between the FC and a data board.
  /// When FC recieves this from the data board, it'll reciprocate by
  /// sending one of its own.
  Identity(BoardId),

  /// Flight computer will send this after no response from data board
  /// after extended period of time.
  FlightHeartbeat,

  /// An array of channel data points.
  Sam(BoardId, Cow<'a, Vec<sam::DataPoint>>),

  /// Data originating from the BMS.
  Bms(BoardId, Cow<'a, bms::DataPoint>),

  /// Data originating from Ahrs
  Ahrs(BoardId, Cow<'a, ahrs::DataPoint>),
}

/// Defines how some data coming into the flight computer should be processed
pub trait Ingestible {
  /// Using the data from self, update the vehicle_state
  fn ingest(&self, vehicle_state: &mut VehicleState);
}

#[derive(
  Clone, Copy, Debug, Deserialize, Eq, Hash, PartialEq, Serialize, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize
)]
#[serde(rename_all = "snake_case")]
#[archive_attr(derive(bytecheck::CheckBytes))]
#[cfg_attr(feature = "sequences", pyo3::pyclass)]
/// Information about a specific valve's safe state
pub struct ValveSafeState {
  /// Desired state of a valve 
  pub desired_state: ValveState,

  /// Timer (in milliseconds!!!) that allows us to delay putting a valve in its safe state by some amount of time
  /// Can't use Instant here since Instant does not implement serde::Serialize or deserialize
  pub safing_timer: u32,
}

#[derive(Serialize, Deserialize)]
/// Used for IPC from child Sequence processes and FC process.
pub enum SequenceDomainCommand {
  /// Tells the FC to actuate a valve
  ActuateValve {
    /// The name of the valve to actuate
    valve: String,

    /// The state the valve should be in
    state: ValveState,
  },

  /// Creates an abort stage
  CreateAbortStage {
    /// Name of the abort stage
    stage_name: String,
    /// Condition that, if met, we abort.
    /// Can use the eval() in python to run strings as code
    abort_condition: String, 
    /// Desired states of valves that we want to go to in an abort during this stage
    valve_safe_states: HashMap<String, ValveSafeState>,
  },

  /// Tells FC to set the abort stage to a different stage.
  SetAbortStage {
    /// Name of the stage to change to.
    stage_name: String,
  },

  /// Tells FC to tell sams to abort via the stage's "safe" valve states.
  /// Different from Abort message, which runs the abort sequence
  AbortViaStage,

  /// Tells the FC to run the abort sequence.
  Abort,

  /// Instructs the flight computer to launch the RECO
  RecoLaunch,

  /// Tells the FC to set the voting logic on the RECO board.
  SetRecoVotingLogic {
    /// Whether the MCU 1 is enabled
    mcu_1_enabled: bool,
    /// Whether the MCU 2 is enabled
    mcu_2_enabled: bool,
    /// Whether the MCU 3 is enabled
    mcu_3_enabled: bool,
  },
  /// Tells the FC to arm the detonator for the launch lug
  /// Instruts the flight computer to tell the sam with the passed in hostname
  /// to arm detonator for launch lug
  LaunchLugArm {
    /// The hostname of the SAM board to arm the launch lug for
    sam_hostname: String,
    /// Whether to enable the launch lug arm pin
    should_enable: bool,
  }, 

  /// Instruts the flight computer to tell the sam with the passed in hostname
  /// to detonate launch lug
  LaunchLugDetonate {
    /// The hostname of the SAM board to detonate the launch lug for
    sam_hostname: String,
    /// Whether to enable the launch lug detonate pin
    should_enable: bool,
  }, 
}
