use common::comm::{
  bms, 
  flight::DataMessage, 
  sam::{ChannelType, SamDataPoint, Unit}, CompositeValveState, 
  Measurement, 
  NodeMapping,
  SensorType, 
  ValveState, 
  VehicleState
};
use crate::{Mappings, MMAP_GRACE_PERIOD};
use mmap_sync::locks::LockDisabled;
use mmap_sync::synchronizer::{Synchronizer, SynchronizerError};
use wyhash::WyHash;

// flight-computer PT constants
const FLIGHT_PT_BOARD_ID: &str = "flight";
const FLIGHT_PT_MAPPING_CHANNEL: u32 = 5;

pub(crate) fn sync_sequences(
  sync: &mut Synchronizer<WyHash, LockDisabled, 1024, 500_000>,
  state: &VehicleState,
) -> Result<(usize, bool), SynchronizerError> {
  sync.write(state, MMAP_GRACE_PERIOD)
}

pub(crate) trait Ingestible {
  fn ingest(&self, vehicle_state: &mut VehicleState, mappings: &Mappings);
}

impl<'a> Ingestible for DataMessage<'a> {
  fn ingest(&self, vehicle_state: &mut VehicleState, mappings: &Mappings) {
    match self {
      DataMessage::Sam(id, datapoints) => {
        if !id.starts_with("sam") {
          println!("Detected a SAM data message without a SAM signature.");
        }

        process_sam_data(id, vehicle_state, datapoints.to_vec(), mappings)
      }
      DataMessage::Bms(id, datapoint) => {
        if !id.starts_with("bms") {
          println!("Detected a BMS data message without a BMS signature.");
        }

          process_bms_data(vehicle_state, **datapoint);
      },
      DataMessage::FlightHeartbeat | DataMessage::Identity(_) => {},
    }
  }
}

pub(crate) fn process_bms_data(
  state: &mut VehicleState,
  datapoint: bms::DataPoint,
) {
  state.bms = datapoint.state;
  state.rbf.bms = state.bms.rbf_tag as u8;
}

// TODO: Optimize this function?
pub(crate) fn process_sam_data(board_id: &str, state: &mut VehicleState, datapoints: Vec<SamDataPoint>, mappings: &Mappings) {
  for data_point in datapoints {
    match data_point {
      SamDataPoint::Rbf { value } => {
        state.rbf.sam.insert(board_id.to_string(), value);
      }

      SamDataPoint::Sensor(data_point) => {
        for mapping in mappings {
          let corresponds = data_point.channel == mapping.channel
            && mapping
              .sensor_type
              .channel_types()
              .contains(&data_point.channel_type)
            && board_id == mapping.board_id;

          if !corresponds {
            continue;
          }
          apply_sensor_mapping(
            state,
            mapping,
            data_point.channel_type,
            data_point.value,
          );
        }
      }
    }
  }
}

pub(crate) fn process_flight_pt_data(
  state: &mut VehicleState,
  sample_value: f64,
  mappings: &Mappings,
) {
  for mapping in mappings {
    let corresponds = mapping.channel == FLIGHT_PT_MAPPING_CHANNEL
      && mapping
        .sensor_type
        .channel_types()
        .contains(&ChannelType::CurrentLoop)
      && mapping.board_id == FLIGHT_PT_BOARD_ID;

    if !corresponds {
      continue;
    }

    apply_sensor_mapping(state, mapping, ChannelType::CurrentLoop, sample_value);
  }
}

fn apply_sensor_mapping(
  state: &mut VehicleState,
  mapping: &NodeMapping,
  channel_type: ChannelType,
  sample_value: f64,
) {
  let mut text_id = mapping.text_id.clone();

  let measurement = match mapping.sensor_type {
    SensorType::RailVoltage => Measurement {
      value: sample_value,
      unit: Unit::Volts,
    },
    SensorType::Rtd | SensorType::Tc => Measurement {
      value: sample_value,
      unit: Unit::Kelvin,
    },
    SensorType::RailCurrent => Measurement {
      value: sample_value,
      unit: Unit::Amps,
    },
    SensorType::Pt => {
      let value;
      let unit;

      // apply linear transformations to current loop and differential
      // signal channels if the max and min are supplied by the mappings.
      // otherwise, default back to volts.
      if let (Some(max), Some(min)) = (mapping.max, mapping.min) {
        // formula for converting voltage into psi for our PTs
        // TODO: consider precalculating scale and offset on control server
        value = (sample_value - 0.8) / 3.2 * (max - min) + min
          - mapping.calibrated_offset;
        unit = Unit::Psi;
      } else {
        // if no PT ratings are set, default to displaying raw voltage
        value = sample_value;
        unit = Unit::Volts;
      }

      Measurement { value, unit }
    }
    SensorType::LoadCell => {
      // if no load cell mappings are set, default to these values
      let mut value = sample_value;
      let mut unit = Unit::Volts;

      // apply linear transformations to load cell channel if the max and
      // min are supplied by the mappings. otherwise, default back to volts.
      if let (Some(max), Some(min)) = (mapping.max, mapping.min) {
        // formula for converting voltage into pounds for our load cells
        value = (max - min) / 0.03 * (value + 0.015) + min
          - mapping.calibrated_offset;
        unit = Unit::Pounds;
      }

      Measurement { value, unit }
    }
    SensorType::Valve => {
      let voltage;
      let current;
      let measurement;

      match channel_type {
        ChannelType::ValveVoltage => {
          voltage = sample_value;
          current = state
            .sensor_readings
            .get(&format!("{text_id}_I"))
            .map(|measurement| measurement.value)
            .unwrap_or(0.0);

          measurement = Measurement {
            value: sample_value,
            unit: Unit::Volts,
          };
          text_id = format!("{text_id}_V");
        }
        ChannelType::ValveCurrent => {
          current = sample_value;
          voltage = state
            .sensor_readings
            .get(&format!("{text_id}_V"))
            .map(|measurement| measurement.value)
            .unwrap_or(0.0);

          measurement = Measurement {
            value: sample_value,
            unit: Unit::Amps,
          };
          text_id = format!("{text_id}_I");
        }
        channel_type => {
          eprintln!(
            "Measured channel type of '{channel_type:?}' for valve."
          );
          return;
        }
      };

      let actual_state = estimate_valve_state(
        voltage,
        current,
        mapping.powered_threshold,
        mapping.normally_closed,
      );

      if let Some(existing) = state.valve_states.get_mut(&mapping.text_id) {
        existing.actual = actual_state;
      } else {
        state.valve_states.insert(
          mapping.text_id.clone(),
          CompositeValveState {
            commanded: ValveState::Undetermined,
            actual: actual_state,
          },
        );
      }

      measurement
    }
  };

  // replace item without cloning string if already present
  if let Some(existing) = state.sensor_readings.get_mut(&text_id) {
    *existing = measurement;
  } else {
    state.sensor_readings.insert(text_id, measurement);
  }
}

/// Estimates the state of a valve given its voltage, current, and the current
/// threshold at which it is considered powered.
fn estimate_valve_state(
  voltage: f64,
  current: f64,
  powered_threshold: Option<f64>,
  normally_closed: Option<bool>,
) -> ValveState {
  // calculate the actual state of the valve, assuming that it's normally closed
  let mut estimated = match powered_threshold {
    Some(powered) => {
      if current < powered {
        // valve is unpowered
        if voltage < 4.0 {
          ValveState::Closed
        } else {
          ValveState::Disconnected
        }
      } else {
        // valve is powered
        if voltage < 20.0 {
          ValveState::Fault
        } else {
          ValveState::Open
        }
      }
    }
    None => ValveState::Fault,
  };

  if normally_closed == Some(false) {
    estimated = match estimated {
      ValveState::Open => ValveState::Closed,
      ValveState::Closed => ValveState::Open,
      other => other,
    };
  }

  estimated
}
