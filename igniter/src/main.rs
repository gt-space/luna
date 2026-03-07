pub mod adc;
pub mod command;
pub mod communication;
pub mod pins;
pub mod state;

use once_cell::sync::OnceCell;
use clap::{Arg, Command};
use std::{sync::LazyLock, net::SocketAddr};
use communication::get_igniter_id;

pub static FC_ADDR: OnceCell<String> = OnceCell::new();
pub static CACHED_FC_ADDRESS: OnceCell<SocketAddr> = OnceCell::new();
/// ID for the specific compute module we are running on
pub static IGNITER_ID: LazyLock<IgniterId> = LazyLock::new(get_igniter_id);

/// Igniter board has multiple compute modules, which we can differenitate
/// via their IgniterID
#[derive(PartialEq, Debug)]
pub enum IgniterId {
  Igniter1,
  Igniter2,
}

fn main() {
  let matches = Command::new("igniter")
  .about("hostname of flight computer")
  .arg(
    Arg::new("target")
      .long("target")
      .required(false)
  ).get_matches();

  let default_address = "flight".to_owned();
  let target = matches.get_one::<String>("target").cloned().unwrap_or(default_address);
  FC_ADDR.set(target).unwrap();

  let mut state = state::State::Init;

  loop {
    state = state.next();
  }
}
