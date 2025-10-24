use super::{ahrs, bms, sam, VehicleState, ValveState};
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
  Ahrs(BoardId, Cow<'a, Vec<ahrs::DataPoint>>),
}

/// Defines how some data coming into the flight computer should be processed
pub trait Ingestible {
  /// Using the data from self, update the vehicle_state
  fn ingest(&self, vehicle_state: &mut VehicleState);
}

#[derive(Serialize, Deserialize)]
/// Information about a specific valve's safe state
pub struct ValveSafeState {
  /// Desired state of a valve 
  pub desired_state: ValveState,

  /// Timer (in seconds!!!) that allows us to delay putting a valve in its safe state by some amount of time
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
    state: ValveState 
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
}