use common::comm::ctv::{ControlState, ControlVector};

pub mod lqr;
pub mod pid;

pub trait Controller {
  /// Perform one step of the control algorithm
  fn step(&mut self, state: ControlState) -> ControlVector;
}
