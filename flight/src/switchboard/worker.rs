use crate::{handler, state::SharedState};
use common::comm::{
  ahrs,
  bms,
  flight::{BoardId, Ingestible},
  sam::{self, ChannelType, Unit},
  CompositeValveState,
  Measurement,
  NodeMapping,
  SensorType,
  ValveState,
  VehicleState,
};
use jeflog::{fail, warn};
use std::sync::{mpsc::Receiver, Arc, Mutex};

pub enum Gig {
  Sam(Vec<sam::DataPoint>),
  Bms(Vec<bms::DataPoint>),
  Ahrs(Vec<ahrs::DataPoint>),
}

// TODO: I understand, right now this is all very messy. I expect with FC 2.0
// that we get right of all this code bloat and get dynamic traits working
// properly.

/// Deals with all the data processing, only wakes when there's data to be
/// processed.
pub fn worker(
  shared: SharedState,
  gig: Receiver<(BoardId, Gig)>,
) -> impl FnOnce() {
  move || {
    for (board_id, datapoints) in gig {
      match datapoints {
        Gig::Sam(data) => process_sam_data(
          shared.vehicle_state.clone(),
          shared.mappings.clone(),
          board_id,
          data,
        ),
        Gig::Bms(data) => {
          process_ingestible_data(shared.vehicle_state.clone(), data)
        }
        Gig::Ahrs(data) => {
          process_ingestible_data(shared.vehicle_state.clone(), data)
        }
      }
    }

    fail!("Switchboard has unexpectedly closed the gig channel. Aborting.");
    handler::abort(&shared);
  }
}

fn process_ingestible_data<T: Ingestible>(
  vehicle_state: Arc<Mutex<VehicleState>>,
  datapoints: Vec<T>,
) {
  let mut vehicle_state = vehicle_state.lock().unwrap();

  for datapoint in datapoints {
    datapoint.ingest(&mut vehicle_state);
  }
}

fn process_sam_data(
  vehicle_state: Arc<Mutex<VehicleState>>,
  mappings: Arc<Mutex<Vec<NodeMapping>>>,
  board_id: BoardId,
  datapoints: Vec<sam::DataPoint>,
) {
  let mut vehicle_state = vehicle_state.lock().unwrap();

  let mappings = mappings.lock().unwrap();

  for data_point in datapoints {
    for mapping in &*mappings {
      // checks if this mapping corresponds to the data point and, if not,
      // continues. originally, I intended to implement this with a HashMap, but
      // considering how few elements will be there, I suspect that it will
      // actually be faster with a vector and full iteration. I may be wrong; we
      // will have to perf.
      let corresponds = data_point.channel == mapping.channel
        && mapping
          .sensor_type
          .channel_types()
          .contains(&data_point.channel_type)
        && *board_id == mapping.board_id;

      if !corresponds {
        continue;
      }

      println!(
        "DP: Channel: {}, Type: {}, Value: {}",
        data_point.channel, data_point.channel_type, data_point.value
      );

      let mut text_id = mapping.text_id.clone();

      let measurement = match mapping.sensor_type {
        SensorType::RailVoltage => Measurement {
          value: data_point.value,
          unit: Unit::Volts,
        },
        SensorType::Rtd | SensorType::Tc => Measurement {
          value: data_point.value,
          unit: Unit::Kelvin,
        },
        SensorType::RailCurrent => Measurement {
          value: data_point.value,
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
            value = (data_point.value - 0.8) / 3.2 * (max - min) + min
              - mapping.calibrated_offset;
            unit = Unit::Psi;
          } else {
            // if no PT ratings are set, default to displaying raw voltage
            value = data_point.value;
            unit = Unit::Volts;
          }

          Measurement { value, unit }
        }
        SensorType::LoadCell => {
          // if no load cell mappings are set, default to these values
          let mut value = data_point.value;
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

          match data_point.channel_type {
            ChannelType::ValveVoltage => {
              voltage = data_point.value;
              current = vehicle_state
                .sensor_readings
                .get(&format!("{text_id}_I"))
                .map(|measurement| measurement.value)
                .unwrap_or(0.0);

              measurement = Measurement {
                value: data_point.value,
                unit: Unit::Volts,
              };
              text_id = format!("{text_id}_V");
            }
            ChannelType::ValveCurrent => {
              current = data_point.value;
              voltage = vehicle_state
                .sensor_readings
                .get(&format!("{text_id}_V"))
                .map(|measurement| measurement.value)
                .unwrap_or(0.0);

              measurement = Measurement {
                value: data_point.value,
                unit: Unit::Amps,
              };
              text_id = format!("{text_id}_I");
            }
            channel_type => {
              warn!("Measured channel type of '{channel_type:?}' for valve.");
              continue;
            }
          };

          let actual_state = estimate_valve_state(
            voltage,
            current,
            mapping.powered_threshold,
            mapping.normally_closed,
          );

          if let Some(existing) =
            vehicle_state.valve_states.get_mut(&mapping.text_id)
          {
            existing.actual = actual_state;
          } else {
            vehicle_state.valve_states.insert(
              mapping.text_id.clone(),
              CompositeValveState {
                commanded: ValveState::Undetermined,
                actual: actual_state,
              },
            );
          }

          println!(
            "M: Value: {}, Unit: {}",
            measurement.value, measurement.unit
          );
          measurement
        }
      };

      // replace item without cloning string if already present
      if let Some(existing) = vehicle_state.sensor_readings.get_mut(&text_id) {
        *existing = measurement;
      } else {
        vehicle_state.sensor_readings.insert(text_id, measurement);
      }
    }
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
