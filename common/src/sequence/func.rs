use super::{PostcardSerializationError, SendCommandIpcError, SOCKET};
use crate::{comm::flight::SequenceDomainCommand, sequence::unit::Duration};

use pyo3::{pyclass, pyfunction, pymethods, PyAny, PyRef, PyRefMut, PyResult};
use std::{thread, time::Instant};

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
