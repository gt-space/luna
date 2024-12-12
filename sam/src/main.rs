pub mod adc;
pub mod communication;
pub mod command;
pub mod state;
pub mod data;
pub mod discovery;
pub mod tc;
pub mod pins;

use once_cell::unsync::Lazy;
use pins::SamVersion;
use communication::get_hostname;

pub static SAM_INFO: Lazy<SamInfo> = Lazy::new(|| get_hostname());

pub struct SamInfo {
  version: SamVersion,
  name: String
}

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
