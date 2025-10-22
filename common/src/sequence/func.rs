use super::{PostcardSerializationError, SendCommandIpcError, SOCKET};
use crate::{comm::{flight::SequenceDomainCommand, ValveState}, sequence::unit::Duration};

use pyo3::{pyclass, pyfunction, pymethods, PyAny, PyRef, PyRefMut, PyResult};
use std::{thread, time::Instant, collections::HashMap};
use rkyv::Deserialize;
use super::{read_vehicle_state, synchronize, RkyvDeserializationError, SensorNotFoundError, ValveNotFoundError, SYNCHRONIZER};

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

/// Python exposed function that lets operators create an abort stage.
#[pyfunction]
pub fn create_abort_stage(stage_name: String, abort_condition: String, safe_valve_states: &PyDict) -> PyResult<()> {
  // will store (valve_name, ValveState) pairs
  let mut rust_valve_states: HashMap<String, ValveState> = HashMap::new();

  // convert to rust types and insert into map
  for (key, value) in valve_states.iter() {
    let valve: PyRef<Valve> = key.extract()?;
    let valve_name: String = valve.get_name();
    let valve_state: ValveState = value.extract()?;

    rust_valve_states.insert(valve_name, valve_state);
  }

  // create command to send to FC
  let command = SequenceDomainCommand::CreateAbortStage {
    stage_name: stage_name,
    abort_condition: abort_condition,
    valve_safe_states: rust_valve_states,
  };

  // serialize command to send to FC
  let command = match postcard::to_allocvec(&command) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the new abort stage creation message for stage {stage_name}: {e}")
    )),
  };

  // send command to FC
  match SOCKET.send(&command) {
    Ok(_) => println!("New abort stage configuration for stage {stage_name} sent successfully."),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the new abort stage configuration for stage {stage_name} to the FC process: {e}")
    ))
  }

  Ok(())
}

/// Python exposed function that lets us set the current abort stage by sending a message to flight to do so.
#[pyfunction]
pub fn set_abort_stage(stage_name: String) -> PyResult<()> {
  // send a message to fc so that it can update the current abort stage
  let command = SequenceDomainCommand::SetAbortStage {
    stage_name: stage_name,
  };

  let command = match postcard::to_allocvec(&command) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the desired abort stage name: {e}")
    )),
  };

  // send command to FC
  match SOCKET.send(&command) {
    Ok(_) => println!("set-abort-stage change sent successfully."),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the set-abort-stage change to the FC process: {e}")
    ))
  }

  Ok(())
}

/// Python exposed function that sends flight a message to abort boards based on current abort stage's safe valve states.
#[pyfunction]
pub fn send_sams_abort() -> PyResult<()> {
  // we need to change vehiclestate.abort_stage.aborted from false to true since we have now aborted (fc side)
  // also make sure to kill all sequences besides the abort stage sequence before we abort. (fc side)
  // in the abort stage seq itself, we don't abort if we are in a "FLIGHT" abort stage.
  let abort_command = match postcard::to_allocvec(&SequenceDomainCommand::AbortViaStage) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the AbortViaStage command: {e}")
    )),
  };

  match SOCKET.send(&abort_command) {
    Ok(_) => println!("AbortViaStage sent successfully."),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the AbortViaStage command to the FC process: {e}")
    )),
  }

  Ok(())
}

// steal default valve states function from abort stages p1 for now until gui is up?

/// Python exposed function that gets the current abort stage's name
#[pyfunction]
pub fn curr_abort_stage() -> PyResult<String> {
  let mut sync = synchronize(&SYNCHRONIZER)?;
    // this unwrap() should never fail as synchronize ensures the value is Some.
    let vehicle_state = read_vehicle_state(sync.as_mut().unwrap())?;

    let stage_name = vehicle_state.abort_stage.name.as_str().to_string();

    drop(vehicle_state);

    Ok(stage_name)
}

/// Python exposed function that gets the current abort stage's abort condition
#[pyfunction]
pub fn curr_abort_condition() -> PyResult<String> {
  let mut sync = synchronize(&SYNCHRONIZER)?;
    // this unwrap() should never fail as synchronize ensures the value is Some.
    let vehicle_state = read_vehicle_state(sync.as_mut().unwrap())?;

    let abort_condition = vehicle_state.abort_stage.abort_condition.as_str().to_string();

    drop(vehicle_state);

    Ok(abort_condition)
}

/// Python exposed function that tells us whether we have already aborted in the current abort stage
#[pyfunction]
pub fn aborted_in_this_stage() -> PyResult<bool> {
  let mut sync = synchronize(&SYNCHRONIZER)?;
    // this unwrap() should never fail as synchronize ensures the value is Some.
    let vehicle_state = read_vehicle_state(sync.as_mut().unwrap())?;

    let abort_condition = vehicle_state.abort_stage.aborted;

    // redundant? doesn't this get dropped at the end of function call?
    drop(vehicle_state);

    Ok(abort_condition)
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
