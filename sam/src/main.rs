pub mod adc;
pub mod command;
pub mod communication;
pub mod data;
pub mod pins;
pub mod state;
pub mod tc;

use communication::get_version;
use std::sync::LazyLock;

pub static SAM_VERSION: LazyLock<SamVersion> = LazyLock::new(|| get_version());

#[derive(PartialEq, Debug)]
pub enum SamVersion {
  Rev3,
  Rev4Ground,
  Rev4Flight,
}

fn main() {
  let mut state = state::State::Init;

  loop {
    state = state.next();
  }
}
