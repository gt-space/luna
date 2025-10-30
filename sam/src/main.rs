pub mod adc;
pub mod command;
pub mod communication;
pub mod data;
pub mod pins;
pub mod state;
pub mod tc;

use communication::get_version;
use std::{sync::LazyLock, net::SocketAddr};
use once_cell::sync::OnceCell;
use clap::{Arg, Command};

// pub static SAM_VERSION: LazyLock<SamVersion> = LazyLock::new(||
// get_version());
pub static SAM_VERSION: LazyLock<SamVersion> = LazyLock::new(get_version);
pub static FC_ADDR: OnceCell<String> = OnceCell::new();
pub static CACHED_FC_ADDRESS: OnceCell<SocketAddr> = OnceCell::new();

#[derive(PartialEq, Debug)]
pub enum SamVersion {
  Rev3,
  Rev4Ground,
  Rev4Flight,
  Rev4FlightV2,
}
//const DEFAULT_TARGET : &str = "flight";

fn main() {
  let matches = Command::new("sam")
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
