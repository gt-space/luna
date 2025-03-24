use crate::comm::{Measurement, ValveState, flight::SequenceDomainCommand};
use pyo3::{
  pyclass,
  pyclass::CompareOp,
  pymethods,
  IntoPy,
  PyAny,
  PyObject,
  PyResult,
  Python,
};
use rkyv::Deserialize;

use super::{read_vehicle_state, synchronize, PostcardSerializationError, RkyvDeserializationError, SendCommandIpcError, SensorNotFoundError, ValveNotFoundError, SOCKET, SYNCHRONIZER};

/// A Python-exposed class that allows for interacting with a sensor.
#[pyclass]
#[derive(Clone, Debug)]
pub struct Sensor {
  name: String,
}

#[pymethods]
impl Sensor {
  /// Creates a new sensor with the specified text identifier.
  #[new]
  pub fn new(name: String) -> Self {
    Sensor { name }
  }

  /// Reads the latest sensor measurements by indexing into the global vehicle
  /// state.
  pub fn read(&self) -> PyResult<PyObject> {
    let mut sync = synchronize(&SYNCHRONIZER)?;
    // this unwrap() should never fail as synchronize ensures the value is Some.
    let vehicle_state = read_vehicle_state(sync.as_mut().unwrap())?;

    let Some(measurement) = vehicle_state.sensor_readings.get(self.name.as_str()) else {
      return Err(SensorNotFoundError::new_err(format!(
        "Couldn't find the sensor named '{}' in sensor_readings.", self.name
      )));
    };
    
    // TODO: logic can be isolated in a function
    let measurement: Measurement = match measurement.deserialize(&mut rkyv::Infallible) {
      Ok(m) => m,
      Err(e) => return Err(RkyvDeserializationError::new_err(format!(
        "rkyv couldn't deserialize the measurement from '{}': {e}", self.name
      ))),
    };

    // done to ensure we aren't reading during the gil.
    drop(vehicle_state);

    Ok(Python::with_gil(move |py| {
      measurement.into_py(py)
    }))
  }

  fn __richcmp__(&self, other: &PyAny, op: CompareOp) -> PyResult<bool> {
    other.rich_compare(self.read()?, op)?.is_true()
  }
}

/// A Python-exposed class that allows for interacting with a valve.
#[pyclass]
#[derive(Clone, Debug)]
pub struct Valve {
  name: String,
}

impl Valve {
  fn is_state(&self, state: ValveState) -> PyResult<bool> {
    let mut sync = synchronize(&SYNCHRONIZER)?;
    // this unwrap() should never fail as synchronize ensures the value is Some.
    let vehicle_state = read_vehicle_state(sync.as_mut().unwrap())?;

    let Some(valve) = vehicle_state.valve_states.get(self.name.as_str()) else {
      return Err(ValveNotFoundError::new_err(format!(
        "Couldn't find the valve named '{}' in valve_states.", self.name
      )));
    };

    let actual: ValveState = match valve.actual.deserialize(&mut rkyv::Infallible) {
      Ok(a) => a,
      Err(e) => return Err(RkyvDeserializationError::new_err(format!(
        "rkyv couldn't deserialize the state of valve '{}': {e}", self.name
      ))),
    };

    drop(vehicle_state);

    Ok(actual == state)
  }
}

#[pymethods]
impl Valve {
  /// Constructs a new `Valve` with its mapping's text ID.
  #[new]
  pub fn new(name: String) -> Self {
    Valve { name }
  }

  /// Determines if the valve is open.
  pub fn is_open(&self) -> PyResult<bool> {
    self.is_state(ValveState::Open)
  }

  /// Determines if the values is closed.
  pub fn is_closed(&self) -> PyResult<bool> {
    self.is_state(ValveState::Closed)
  }

  /// Instructs the SAM board to open the valve.
  pub fn open(&self) -> PyResult<()> {
    self.actuate(true)
  }

  /// Instructs the SAM board to close the valve.
  pub fn close(&self) -> PyResult<()> {
    self.actuate(false)
  }

  /// Instructs the SAM board to actuate a valve.
  pub fn actuate(&self, open: bool) -> PyResult<()> {
    let mut buf: [u8; 1024] = [0; 1024];

    let state = if open {
      ValveState::Open
    } else {
      ValveState::Closed
    };

    let command = SequenceDomainCommand::ActuateValve {
      valve: self.name.clone(),
      state
    };
    
    match postcard::to_slice(&command, &mut buf) {
      Ok(s) => s,
      Err(e) => return Err(PostcardSerializationError::new_err(format!(
        "Postcard error in serializing an actuate valve command: {e}"
      ))),
    };

    SOCKET.send(&buf).map_or_else(
      |e| Err(SendCommandIpcError::new_err(format!(
        "Error in sending actuate command to FC process via domain socket: {e}"
      ))),
      |_| Ok(())
    )
  }
}
