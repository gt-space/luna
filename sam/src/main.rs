pub mod adc;
pub mod communication;
pub mod command;
pub mod state;
pub mod data;
pub mod tc;
pub mod pins;

use communication::get_version;
use pins::config_pin;
use std::sync::LazyLock;

pub static SAM_VERSION: LazyLock<SamVersion> = LazyLock::new(|| get_version());

#[derive(PartialEq, Debug)]
pub enum SamVersion {
  Rev3,
  Rev4Ground,
  Rev4Flight
}

fn main() {
  let mut state = state::State::Init;

  loop {
    state = state.next();
  }
}
