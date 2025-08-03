use super::{ahrs, bms, sam, VehicleState, ValveState};
use serde::{Deserialize, Serialize};
use std::borrow::Cow;

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
/// Used for IPC from child Sequence processes and FC process.
pub enum SequenceDomainCommand {
  /// Tells the FC to actuate a valve
  ActuateValve {
    /// The name of the valve to actuate
    valve: String,
    
    /// The state the vlve should be in
    state: ValveState 
  },

  /// Tells the FC to set the abort stage on a specific SAM.
  SetAbortStage {
    /// The hostname of the SAM we want to change the abort stage on
    sam_hostname: String,

    /// The desired states of the valves for this abort stage. 
    valve_states: [ValveState; 6]
  },

  /// Tells the FC to run the abort sequence.
  Abort,
}