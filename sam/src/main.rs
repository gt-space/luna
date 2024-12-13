pub mod adc;
pub mod communication;
pub mod command;
pub mod state;
pub mod data;
pub mod tc;
pub mod pins;

use communication::get_version;
use once_cell::sync::Lazy;

pub static SAM_VERSION: Lazy<SamVersion> = Lazy::new(|| get_version());

#[derive(PartialEq)]
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
