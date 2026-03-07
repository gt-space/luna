use common::comm::ctv::{ControlState, ControlVector};

use crate::control::Controller;

pub struct LqrController {}

impl LqrController {
  pub fn new() -> LqrController {
    LqrController {}
  }
}

impl Controller for LqrController {
  type Params = ();

  fn configure(&mut self, params: Self::Params) {
    todo!()
  }

  fn reset(&mut self) {
    todo!()
  }

  fn step(&mut self, state: ControlState) -> ControlVector {
    todo!()
  }
}
