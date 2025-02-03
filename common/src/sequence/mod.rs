mod device;
mod func;
mod unit;

pub use device::*;
pub use exceptions::*;
pub use func::*;
pub use unit::*;

use jeflog::fail;
use std::{os::unix::net::UnixDatagram, sync::{LazyLock, Mutex, MutexGuard}};
use mmap_sync::{guard::ReadResult, synchronizer::Synchronizer};

use pyo3::{
  pymodule, types::PyModule, wrap_pyfunction, Py, PyResult, Python
};

use crate::comm::VehicleState;

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
  create_exception!(sequences, ReadVehicleStateIpcError, pyo3::exceptions::PyException);
  create_exception!(sequences, SensorNotFoundError, pyo3::exceptions::PyException);
  create_exception!(sequences, ValveNotFoundError, pyo3::exceptions::PyException);
  create_exception!(sequences, SendCommandIpcError, pyo3::exceptions::PyException);
  create_exception!(sequences, PostcardSerializationError, pyo3::exceptions::PyException);
  create_exception!(sequences, RkyvDeserializationError, pyo3::exceptions::PyException);
}

pub const SOCKET_PATH: &str = "";
pub const MMAP_PATH: &str = "";

// let's break this one down:
// Mutex<...> - required because this is a global variable, and a mutable
//   reference must be obtained to modify Synchronizer, so LazyLock can't be
//   used.
//
// Option<...> - before initialization by importing sequences, this will be 
//   None, so necessary for the compiler to be happy.
//
// Synchronizer - the object used to read from shared memory.
pub(crate) static SYNCHRONIZER: Mutex<Option<Synchronizer>> =
  Mutex::new(None);

pub(crate) static SOCKET: LazyLock<UnixDatagram> = LazyLock::new(|| {
  let socket = UnixDatagram::unbound()
    .expect("Can't initialize socket for ");
  socket.connect(SOCKET_PATH)
    .expect("Can't connect to FC for sending commands via IPC.");
  socket
});

fn synchronize(synchronizer: &Mutex<Option<Synchronizer>>) -> PyResult<MutexGuard<'_, Option<Synchronizer>>> {
  let Ok(mut sync) = synchronizer.lock() else {
    fail!("Failed to lock global synchronizer: Mutex is poisoned.");
    return Err(ReadVehicleStateIpcError::new_err(
      "Couldn't read VehicleState from the FC process."
    ));
  };

  if sync.is_none() {
    *sync = Some(Synchronizer::new(MMAP_PATH.as_ref()));
  }
  Ok(sync)
}

fn read_vehicle_state(synchronizer: &mut Synchronizer) -> PyResult<ReadResult<'_, VehicleState>> {
  let vs = unsafe { synchronizer.read::<VehicleState>(true) };
  vs.map_err(|e| ReadVehicleStateIpcError::new_err(
    format!("Couldn't read the VehicleState from memory: {e}")
  ))
}

#[pymodule]
fn sequences(py: Python<'_>, module: &PyModule) -> PyResult<()> {
  // only here to initialize the Synchronizer
  let _initalize = synchronize(&SYNCHRONIZER)?;

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