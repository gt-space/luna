use common::{
  comm::{
    sam::SamControlMessage,
    CompositeValveState,
    NodeMapping,
    ValveState,
    VehicleState,
  },
  sequence::{self, AbortError, DeviceAction},
};
use jeflog::{fail, warn};
use pyo3::{types::PyNone, IntoPy, PyErr, PyObject, Python, ToPyObject};
use std::{sync::Mutex, thread};

use crate::{
  state::SharedState,
  switchboard::commander::Command,
  CommandSender,
};

pub fn create_device_handler(
  shared: SharedState,
  command_tx: CommandSender,
) -> impl Fn(&str, DeviceAction) -> PyObject {
  let tx = command_tx.clone();

  move |device, action| {
    let thread_id = thread::current().id();
    let sequences = shared.sequences.lock().unwrap();

    if sequences.get_by_right(&thread_id).is_none() {
      drop(sequences);

      return Python::with_gil(|py| {
        AbortError::new_err("aborting sequence").restore(py);
        assert!(PyErr::occurred(py));
        drop(PyErr::fetch(py));

        PyNone::get(py).to_object(py)
      });
    }

    drop(sequences);

    match action {
      DeviceAction::ReadSensor => read_sensor(device, &shared.vehicle_state),
      DeviceAction::ReadValveState => {
        read_valve_state(device, &shared.vehicle_state)
      }
      DeviceAction::ActuateValve { state } => {
        actuate_valve(
          device,
          state,
          &shared.mappings,
          &shared.vehicle_state,
          &tx,
        );
        Python::with_gil(|py| PyNone::get(py).to_object(py))
      }
      DeviceAction::Abort => {
        abort(&shared);
        Python::with_gil(|py| PyNone::get(py).to_object(py))
      }
    }
  }
}

fn read_sensor(name: &str, vehicle_state: &Mutex<VehicleState>) -> PyObject {
  let vehicle_state = vehicle_state.lock().unwrap();

  let measurement = vehicle_state.sensor_readings.get(name);

  Python::with_gil(move |py| {
    measurement.map_or(PyNone::get(py).to_object(py), |m| m.clone().into_py(py))
  })
}

fn read_valve_state(
  name: &str,
  vehicle_state: &Mutex<VehicleState>,
) -> PyObject {
  let vehicle_state = vehicle_state.lock().unwrap();

  let state = vehicle_state.valve_states.get(name);

  Python::with_gil(|py| {
    state.map_or(PyNone::get(py).to_object(py), |s| {
      s.actual.to_string().into_py(py)
    })
  })
}

fn actuate_valve(
  name: &str,
  state: ValveState,
  mappings: &Mutex<Vec<NodeMapping>>,
  vehicle_state: &Mutex<VehicleState>,
  command_tx: &CommandSender,
) {
  let mappings = mappings.lock().unwrap();

  let Some(mapping) = mappings.iter().find(|m| m.text_id == name) else {
    fail!("Failed to actuate valve: mapping '{name}' is not defined.");
    return;
  };

  let closed = state == ValveState::Closed;
  let normally_closed = mapping.normally_closed.unwrap_or(true);
  let powered = closed != normally_closed; // True != False

  let message = SamControlMessage::ActuateValve {
    channel: mapping.channel,
    powered,
  };

  if let Err(error) =
    command_tx.send((mapping.board_id.clone(), Command::Sam(message)))
  {
    fail!("Failed to send command: {error}");
  }

  drop(mappings);
  let mut vehicle_state = vehicle_state.lock().unwrap();

  if let Some(existing) = vehicle_state.valve_states.get_mut(name) {
    existing.commanded = state;
  } else {
    vehicle_state.valve_states.insert(
      name.to_owned(),
      CompositeValveState {
        commanded: state,
        actual: ValveState::Undetermined,
      },
    );
  }
}

pub fn abort(shared: &SharedState) {
  let abort_sequence = shared.abort_sequence.lock().unwrap().clone();

  let Some(sequence) = abort_sequence else {
    warn!("Abort was called but no abort sequence is set.");
    return;
  };

  let mut sequences = shared.sequences.lock().unwrap();
  sequences.clear();
  sequences.insert("abort".to_owned(), thread::current().id());
  drop(sequences);

  sequence::run(sequence);
}
