use super::{ControlInput, ControlOutput, Controller};

pub struct LqrController {}

impl LqrController {
  pub fn new() -> LqrController {
    LqrController {}
  }
}

impl Controller for LqrController {
  fn step(&mut self, input: ControlInput) -> ControlOutput {
    todo!()
  }
}
