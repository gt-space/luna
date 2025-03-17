pub mod adc;
pub mod command;
pub mod communication;
pub mod pins;
pub mod state;

use once_cell::sync::OnceCell;
use clap::{Arg, Command};

pub static FC_ADDR: OnceCell<String> = OnceCell::new();

fn main() {
  let matches = Command::new("dbms")
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
