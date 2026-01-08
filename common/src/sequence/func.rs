use super::{PostcardSerializationError, SendCommandIpcError, SOCKET};
use crate::{comm::{flight::{SequenceDomainCommand, ValveSafeState}, ValveState}, sequence::{unit::Duration, Valve}};

use pyo3::{pyclass, pyfunction, pymethods, Py, PyAny, PyRef, PyRefMut, PyResult, types::PyDict, exceptions::PyValueError, Python, PyObject, IntoPy};
use std::{thread, time::Instant, collections::HashMap};
use rkyv::Deserialize;
use super::{read_vehicle_state, synchronize, RkyvDeserializationError, SensorNotFoundError, ValveNotFoundError, SYNCHRONIZER};
use crate::comm::Measurement;

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
  // will store (valve_name, ValveSafeState) pairs
  let mut rust_valve_states: HashMap<String, ValveSafeState> = HashMap::new();

  // convert to rust types and insert into map
  for (key, value) in safe_valve_states.iter() {
    let valve: PyRef<Valve> = key.extract()?;
    let valve_name: String = valve.get_name();
    let valve_info: (ValveState, u32) = value.extract()?;
    let valve_state: ValveSafeState = ValveSafeState { desired_state: valve_info.0, safing_timer: valve_info.1 };

    rust_valve_states.insert(valve_name, valve_state);
  }

  // create command to send to FC
  let command = SequenceDomainCommand::CreateAbortStage {
    stage_name: stage_name.clone(),
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
    Ok(_) => println!("New abort stage configuration for stage {stage_name} sent successfully to FC for processing."),
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
    stage_name: stage_name.clone(),
  };

  let command = match postcard::to_allocvec(&command) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the desired abort stage name: {e}")
    )),
  };

  // send command to FC
  match SOCKET.send(&command) {
    Ok(_) => println!("Set abort stage to {stage_name} command sent successfully to FC for processing."),
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
    Ok(_) => println!("AbortViaStage sent successfully to FC for processing."),
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

/// A Python-exposed function which runs the abort sequence if we are in the default stage, else the abort via stage.
#[pyfunction]
pub fn abort() -> PyResult<()> {
  let mut abort_command = match postcard::to_allocvec(&SequenceDomainCommand::Abort) {
      Ok(m) => m,
      Err(e) => return Err(PostcardSerializationError::new_err(
        format!("Couldn't serialize the Abort command: {e}")
      )),
    };

  if curr_abort_condition().unwrap() != "DEFAULT" {
    abort_command = match postcard::to_allocvec(&SequenceDomainCommand::AbortViaStage) {
      Ok(m) => m,
      Err(e) => return Err(PostcardSerializationError::new_err(
        format!("Couldn't serialize the AbortViaStage command: {e}")
      )),
    };
  }

  match SOCKET.send(&abort_command) {
    Ok(_) => println!("Abort sent successfully."),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the Abort command to the FC process: {e}")
    )),
  }
  
  Ok(())
}

/// Python exposed function that sends a message to the RECO board that we have launched the rocket.
#[pyfunction]
pub fn send_reco_launch() -> PyResult<()> {
  let command = match postcard::to_allocvec(&SequenceDomainCommand::RecoLaunch) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the RecoLaunch command: {e}")
    )),
  };

  match SOCKET.send(&command) {
    Ok(_) => println!("RecoLaunch sent successfully to FC for processing."),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the RecoLaunch command to the FC process: {e}")
    )),
  }

  Ok(())
}

/// Python exposed function that sends the EKF-initialization message to the RECO board.
#[pyfunction]
pub fn reco_init_ekf() -> PyResult<()> {
  let command = match postcard::to_allocvec(&SequenceDomainCommand::RecoInitEKF) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the RecoInitEKF command: {e}")
    )),
  };

  match SOCKET.send(&command) {
    Ok(_) => println!("RecoInitEKF command sent successfully to FC for processing."),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the RecoInitEKF command to the FC process: {e}")
    )),
  }

  Ok(())
}

/// Python exposed function that reads the umbilical voltage from the BMS.
#[pyfunction]
pub fn read_umbilical_voltage() -> PyResult<PyObject> {
  let mut sync = synchronize(&SYNCHRONIZER)?;
  // this unwrap() should never fail as synchronize ensures the value is Some.
  let vehicle_state = read_vehicle_state(sync.as_mut().unwrap())?;

  let measurement = vehicle_state.bms.umbilical_bus.voltage;

  // done to ensure we aren't reading during the gil.
  drop(vehicle_state);

  Ok(Python::with_gil(move |py| {
    measurement.into_py(py)
  }))
}

/// Python exposed function that reads the reco_recvd_launch.
#[pyfunction]
pub fn reco_recvd_launch() -> PyResult<bool> {
  let mut sync = synchronize(&SYNCHRONIZER)?;
  // this unwrap() should never fail as synchronize ensures the value is Some.
  let vehicle_state = read_vehicle_state(sync.as_mut().unwrap())?;

  let reco_recvd_launch = vehicle_state.reco[0].as_ref().map_or(false, |r| r.reco_recvd_launch) &&
                          vehicle_state.reco[1].as_ref().map_or(false, |r| r.reco_recvd_launch) &&
                          vehicle_state.reco[2].as_ref().map_or(false, |r| r.reco_recvd_launch);

  // done to ensure we aren't reading during the gil.
  drop(vehicle_state);

  Ok(reco_recvd_launch)
}

/// A Python-exposed function which sends a message to the FC to arm the launch lug for the given SAM hostname.
#[pyfunction]
pub fn launch_lug_arm(sam_hostname: String, should_enable: bool) -> PyResult<()> {
  let message = match postcard::to_allocvec(&SequenceDomainCommand::LaunchLugArm {
    sam_hostname: sam_hostname.clone(),
    should_enable: should_enable,
  }) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the LaunchLugArm {} command for {sam_hostname}: {e}", if should_enable { "enable" } else { "disable" })
    )),
  };

  match SOCKET.send(&message) {
    Ok(_) => println!("LaunchLugArm {} command for {sam_hostname} sent successfully to FC for processing.", if should_enable { "enable" } else { "disable" }),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the LaunchLugArm {} command for {sam_hostname} to the FC process: {e}", if should_enable { "enable" } else { "disable" })
    )),
  }

  Ok(())
}

/// A Python-exposed function which sends a message to the FC to detonate the launch lug for the given SAM hostname.
#[pyfunction]
pub fn launch_lug_detonate(sam_hostname: String, should_enable: bool) -> PyResult<()> {
  let message = match postcard::to_allocvec(&SequenceDomainCommand::LaunchLugDetonate {
    sam_hostname: sam_hostname.clone(),
    should_enable: should_enable,
  }) {
    Ok(m) => m,
    Err(e) => return Err(PostcardSerializationError::new_err(
      format!("Couldn't serialize the LaunchLugDetonate {} command for {sam_hostname}: {e}", if should_enable { "enable" } else { "disable" })
    )),
  };

  match SOCKET.send(&message) {
    Ok(_) => println!("LaunchLugDetonate {} command for {sam_hostname} sent successfully to FC for processing.", if should_enable { "enable" } else { "disable" }),
    Err(e) => return Err(SendCommandIpcError::new_err(
      format!("Couldn't send the LaunchLugDetonate {} command for {sam_hostname} to the FC process: {e}", if should_enable { "enable" } else { "disable" })
    )),
  }

  Ok(())
}

/// Python exposed function that tells the FC to ignore servo disconnects if 
/// enabled is false, else to monitor servo disconnects.
#[pyfunction]
pub fn set_servo_disconnect_monitoring(enabled: bool) -> PyResult<()> {
  let command = match postcard::to_allocvec(
    &SequenceDomainCommand::SetServoDisconnectMonitoring { enabled: enabled },
  ) {
    Ok(m) => m,
    Err(e) => {
      return Err(PostcardSerializationError::new_err(
        format!(
          "Couldn't serialize the SetServoDisconnectMonitoring({}) command: {e}", enabled
        ),
      ))
    }
  };

  match SOCKET.send(&command) {
    Ok(_) => println!("SetServoDisconnectMonitoring({}) sent successfully to FC for processing.", enabled),
    Err(e) => {
      return Err(SendCommandIpcError::new_err(
        format!(
          "Couldn't send the SetServoDisconnectMonitoring({}) command to the FC process: {e}", enabled
        ),
      ))
    }
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
