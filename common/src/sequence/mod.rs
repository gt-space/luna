mod device;
mod func;
mod unit;

pub use device::*;
pub use exceptions::*;
pub use func::*;
pub use unit::*;

use crate::comm::{NodeMapping, SensorType, Sequence, ValveState};
use jeflog::{fail, warn};
use std::{sync::{Arc, Mutex, OnceLock}, thread::{self, ThreadId}};
use bimap::BiHashMap;

use pyo3::{
  pymodule,
  types::PyModule,
  wrap_pyfunction,
  Py,
  PyObject,
  PyResult,
  Python,
};

/// A module containing all exception types declared for sequences.
///
/// This must be a separate module because exceptions can only (reasonably) be
/// created with the create_exception! macro, which obscures the implementation
/// such that the structs it creates cannot be annotated with doc-strings. Thus,
/// we must allow missing docs in this context so as to not get warnings.
#[allow(missing_docs)]
mod exceptions {
  use pyo3::create_exception;

  create_exception!(sequences, AbortError, pyo3::exceptions::PyException);
}

#[pymodule]
fn sequences(py: Python<'_>, module: &PyModule) -> PyResult<()> {
  module.add_class::<Current>()?;
  module.add_class::<Duration>()?;
  module.add_class::<ElectricPotential>()?;
  module.add_class::<Force>()?;
  module.add_class::<Pressure>()?;
  module.add_class::<Temperature>()?;

  module.add("A", Py::new(py, Current::new(1.0))?)?;
  module.add("mA", Py::new(py, Current::new(0.001))?)?;
  module.add("s", Py::new(py, Duration::new(1.0))?)?;
  module.add("ms", Py::new(py, Duration::new(0.001))?)?;
  module.add("us", Py::new(py, Duration::new(0.000001))?)?;
  module.add("V", Py::new(py, ElectricPotential::new(1.0))?)?;
  module.add("mV", Py::new(py, ElectricPotential::new(0.001))?)?;
  module.add("lbf", Py::new(py, Force::new(1.0))?)?;
  module.add("psi", Py::new(py, Pressure::new(1.0))?)?;
  module.add("K", Py::new(py, Temperature::new(1.0))?)?;

  module.add_class::<Sensor>()?;
  module.add_class::<Valve>()?;
  module.add_class::<IntervalIterator>()?;

  module.add_function(wrap_pyfunction!(wait_for, module)?)?;
  module.add_function(wrap_pyfunction!(wait_until, module)?)?;
  module.add_function(wrap_pyfunction!(abort, module)?)?;
  module.add_function(wrap_pyfunction!(interval, module)?)?;

  Ok(())
}

type DeviceHandler = dyn Fn(&str, DeviceAction) -> PyObject + Send;

// let's break this one down:
// Mutex<...> - required because this is a global variable, so needed to
//   implement Sync and be used across threads safely.
//
// Option<...> - before initialization by set_device_handler, this will be None,
//   so necessary for the compiler to be happy.
//
// Box<dyn ...> - wraps the enclosed dynamic type on the heap, because it's
//   exact size and type are unknown at compile-time.
//
// Fn(&str, DeviceAction) -> Option<Measurement> - the trait bound of the type
//   of the closure being stored, with its arguments and return value.
//
// + Send - requires that everything captured in the closure be safe to send
//   across threads.
pub(crate) static DEVICE_HANDLER: Mutex<Option<Box<DeviceHandler>>> =
  Mutex::new(None);

pub(crate) static MAPPINGS: OnceLock<Arc<Mutex<Vec<NodeMapping>>>> =
  OnceLock::new();

pub(crate) static SEQUENCES: OnceLock<Arc<Mutex<BiHashMap<String, ThreadId>>>> = 
  OnceLock::new();

/// Initializes the sequences portion of the library.
pub fn initialize(mappings: Arc<Mutex<Vec<NodeMapping>>>, sequences: Arc<Mutex<BiHashMap<String, ThreadId>>>) {
  if MAPPINGS.set(mappings).is_err() {
    warn!("Sequences library has already been initialized. Ignoring.");
    return;
  }

  if SEQUENCES.set(sequences).is_err() {
    warn!("Cannot set sequences BiHashMap from flight.");
    return;
  }

  pyo3::append_to_inittab!(sequences);
  pyo3::prepare_freethreaded_python();
}

/// Given to the device handler to instruct it to perform a type of action.
pub enum DeviceAction {
  /// Instructs to read and return a sensor value.
  ReadSensor,

  /// Instructs to read the actual estimated valve state (as a string for now).
  ReadValveState,

  /// Instructs to command a valve actuation to match the given state.
  ActuateValve {
    /// The state which the valve should be actuated to match, either `Open` or
    /// `Closed`.
    state: ValveState,
  },

  /// Instructs to abort all sequences and run the saved abort sequence.
  Abort,
}

/// Sets the device handler callback, which interacts with external boards from
/// the flight computer code.
///
/// The first argument of this callback is a `&str` which is the name of the
/// target device (typically a valve or sensor), and the second argument is the
/// action to be performed by the handler. The return value is an
/// `Option<Measurement>` because in the event of a read, a measurement will
/// need to be returned, but a valve actuation requires no return.
pub fn set_device_handler(
  handler: impl Fn(&str, DeviceAction) -> PyObject + Send + 'static,
) {
  let Ok(mut device_handler) = DEVICE_HANDLER.lock() else {
    fail!("Failed to lock global device handler: Mutex is poisoned.");
    return;
  };

  *device_handler = Some(Box::new(handler));
}

// TODO: change the run function to return an error in the event of one instead
//of printing out the error.

/// Runs a sequence. The `initialize` function must be called before this.
pub fn run(sequence: Sequence) {
  let Some(sequences) = SEQUENCES.get() else {
    fail!("Sequences BiHashMap must be initalized before running a sequence");
    return;
  };

  let Ok(mut sequences) = sequences.lock() else {
    fail!("Sequences BiHashMap could not be locked within common::sequence::run().");
    return;
  };

  sequences.insert(sequence.name.clone(), thread::current().id());
  drop(sequences);

  let Some(mappings) = MAPPINGS.get() else {
    fail!("Sequences library must be initialized before running a sequence.");
    return;
  };

  let Ok(mappings) = mappings.lock() else {
    fail!("Mappings could not be locked within common::sequence::run.");
    return;
  };

  Python::with_gil(|py| {
    if let Err(error) = py.run("from sequences import *", None, None) {
      fail!("Failed to import sequences library: {error}");
      return;
    }

    for mapping in &*mappings {
      let definition = match mapping.sensor_type {
        SensorType::Valve => format!("{0} = Valve('{0}')", mapping.text_id),
        _ => format!("{0} = Sensor('{0}')", mapping.text_id),
      };

      if let Err(error) = py.run(&definition, None, None) {
        fail!(
          "Failed to define '{}' as a mapping: {error}",
          mapping.text_id
        );
        return;
      }
    }

    // drop the lock before entering script to prevent deadlock
    drop(mappings);

    if let Err(error) = py.run(&sequence.script, None, None) {
      fail!("Failed to run sequence '{}': {error}", sequence.name);
    }
  });
}
