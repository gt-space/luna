use crate::comm::{Measurement, ValveState, VehicleState};
use jeflog::fail;
use pyo3::{
  pyclass,
  pyclass::CompareOp,
  pymethods,
  types::PyString,
  IntoPy,
  Py,
  PyAny,
  PyObject,
  PyResult,
  Python,
};
use rkyv::Deserialize;

use super::{synchronize, ReadVehicleStateIpcError, RkyvDeserializationError, SensorNotFoundError, SYNCHRONIZER};

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
    // this unwrap() should never fail as synchronize ensures the value is Some.
    let mut sync = synchronize(&SYNCHRONIZER)?;
    let synchronizer = sync.as_mut().unwrap();
    let vs = unsafe { synchronizer.read::<VehicleState>(true) };
    let vehicle_state = match vs {
      Ok(vs) => vs,
      Err(e) => {
        return Err(ReadVehicleStateIpcError::new_err(
          format!("Couldn't read the VehicleState from memory: {e}")
        ));
      },
    };


    let Some(measurement) = vehicle_state.sensor_readings.get(self.name.as_str()) else {
      return Err(SensorNotFoundError::new_err(format!(
        "Couldn't find the sensor named '{}' in sensor_readings.", self.name
      )));
    };
    
    let measurement: Measurement = match measurement.deserialize(&mut rkyv::Infallible) {
      Ok(m) => m,
      Err(e) => return Err(RkyvDeserializationError::new_err(format!(
        "rkyv couldn't deserialize the measurement: {e}"
      ))),
    };

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

#[pymethods]
impl Valve {
  /// Constructs a new `Valve` with its mapping's text ID.
  #[new]
  pub fn new(name: String) -> Self {
    Valve { name }
  }

  /// Determines if the valve is open.
  pub fn is_open(&self) -> Option<bool> {
    let Some(device_handler) = &*DEVICE_HANDLER.lock().unwrap() else {
      fail!("Device handler not set before accessing external device.");
      return None;
    };

    let state = device_handler(&self.name, DeviceAction::ReadValveState);

    Python::with_gil(|py| {
      let open: Py<PyAny> = "open".into_py(py);
      state.into_ref(py).eq(open).ok()
    })
  }

  /// Determines if the values is closed.
  pub fn is_closed(&self) -> Option<bool> {
    let Some(device_handler) = &*DEVICE_HANDLER.lock().unwrap() else {
      fail!("Device handler not set before accessing external device.");
      return None;
    };

    let state = device_handler(&self.name, DeviceAction::ReadValveState);

    Python::with_gil(|py| {
      let closed: Py<PyString> = "closed".into_py(py);
      state.into_ref(py).eq(closed).ok()
    })
  }

  /// Instructs the SAM board to open the valve.
  pub fn open(&self) {
    self.actuate(true);
  }

  /// Instructs the SAM board to close the valve.
  pub fn close(&self) {
    self.actuate(false);
  }

  /// Instructs the SAM board to actuate a valve.
  pub fn actuate(&self, open: bool) {
    let Some(device_handler) = &*DEVICE_HANDLER.lock().unwrap() else {
      fail!("Device handler not set before accessing external device.");
      return;
    };

    let state = if open {
      ValveState::Open
    } else {
      ValveState::Closed
    };
    device_handler(&self.name, DeviceAction::ActuateValve { state });
  }
}
