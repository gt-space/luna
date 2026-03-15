use common::comm::ctv::{ControlState, ControlVector};

pub mod lqr;
pub mod pid;

pub trait Controller {
  /// Configuration parameters
  type Params;

  /// Configure the controller
  fn configure(&mut self, params: Self::Params);

  /// Reset controller state
  fn reset(&mut self);

  /// Perform one step of the control algorithm
  fn step(&mut self, state: ControlState) -> ControlVector;
}
