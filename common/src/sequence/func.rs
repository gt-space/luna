use super::{PostcardSerializationError, SendCommandIpcError, InvalidValveStateError, SOCKET};
use crate::{comm::{flight::SequenceDomainCommand, ValveState}, sequence::{unit::Duration, Valve}};

use pyo3::{pyclass, pyfunction, pymethods, PyAny, PyRef, PyRefMut, PyResult, types::PyDict};
use std::{os::linux::raw, thread, time::Instant, collections::HashMap};

/// A Python-exposed function which waits the thread for the given duration.
#[pyfunction]
pub fn wait_for(duration: Duration) {
  // TODO: considering using a different way to sleep, possibly sleeping only
  // the GIL?
  thread::sleep(duration.into());
}

/// A Python-exposed function which waits until a condition function is true,
/// given an optional timeout and interval between checking.
#[pyfunction]
pub fn wait_until(
  condition: &PyAny,
  timeout: Option<Duration>,
  poll_interval: Option<Duration>,
) -> PyResult<()> {
  let timeout = timeout.map_or(std::time::Duration::MAX, Into::into);

  let interval =
    poll_interval.map_or(std::time::Duration::from_millis(10), Into::into);

  let end_time = Instant::now() + timeout;

  while !condition.call0()?.is_true()? && Instant::now() < end_time {
    thread::sleep(interval);
  }

  Ok(())
}

/// A Python-exposed function which immediately runs the abort sequence.
#[pyfunction]
pub fn abort() -> PyResult<()> {
  let abort_command = match postcard::to_allocvec(&SequenceDomainCommand::Abort) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the Abort command: {e}")
    )),
  };

  match SOCKET.send(&abort_command) {
    Ok(_) => println!("Abort sent successfully."),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the Abort command to the FC process: {e}")
    )),
  }

  Ok(())
}

/// A Python-exposed function that preemptively tells a SAM how to actuate its valves
/// in case of a loss of communications abort
#[pyfunction]
pub fn set_abort_stage(valve_states: &PyDict) -> PyResult<()> {
  // figure out how to return a PyResult error if the runtime somehow allows 
  // a string to be entered or invalid letters are provided

  let mut rust_valve_states: HashMap<String, ValveState> = HashMap::new();

  for (key, value) in valve_states.iter() {
    // convert to rust types
    let valve_name: String = key.extract()?;
    let valve_state: ValveState = value.extract()?;

    // check if valve_name is a valve that actually exists
    let mut sync = synchronize(&SYNCHRONIZER)?;
    // this unwrap() should never fail as synchronize ensures the value is Some.
    let vehicle_state = read_vehicle_state(sync.as_mut().unwrap())?;
    let Some(valve) = vehicle_state.valve_states.get(valve_name.as_str()) else {
      return Err(ValveNotFoundError::new_err(format!(
        "Couldn't find the valve named '{}' in valve_states.", valve_name
      )));
    };

    // we have a valid valve name, so insert the pair into our map
    rust_valve_states.insert(valve_name, valve_state);
  }

  // create command to send to FC
  let command = SequenceDomainCommand::SetAbortStage {
    valve_states: rust_valve_states,
  };

  // serialize command to send to FC
  let command = match postcard::to_allocvec(&command) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the set-abort-stage configuration: {e}")
    )),
  };

  // send command to FC
  match SOCKET.send(&command) {
    Ok(_) => println!("set-abort-stage configuration sent successfully."),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the set-abort-stage configuration to the FC process: {e}")
    ))
  }

  Ok(())
}

/// Iterator which only yields the iteration after waiting for the given period.
#[pyclass]
#[derive(Clone, Debug)]
pub struct IntervalIterator {
  next_tick: Instant,
  period: std::time::Duration,
  iteration: i64,
  total: i64,
}

#[pymethods]
impl IntervalIterator {
  fn __iter__(_self: PyRef<'_, Self>) -> PyRef<'_, Self> {
    _self
  }

  fn __next__(mut _self: PyRefMut<'_, Self>) -> Option<i64> {
    if _self.iteration >= _self.total {
      return None;
    }

    let wait = _self.next_tick - Instant::now();
    thread::sleep(wait);

    let iteration = _self.iteration;
    let next_tick = _self.next_tick + _self.period;

    _self.next_tick = next_tick;
    _self.iteration += 1;

    Some(iteration)
  }
}

/// A Python-exposed function which creates an iterator which yields the
/// iteration after waiting for the period.
#[pyfunction]
pub fn interval(count: i64, period: Duration) -> IntervalIterator {
  IntervalIterator {
    next_tick: Instant::now(),
    period: period.into(),
    iteration: 0,
    total: count,
  }
}
