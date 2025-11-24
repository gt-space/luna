use clap::{Arg, Command};
use once_cell::sync::OnceCell;

mod adc;
mod communication;
mod driver;
mod pins;
mod state;

pub static FC_ADDR: OnceCell<String> = OnceCell::new();

fn main() {
  let matches = Command::new("ahrs")
    .about("hostname of flight computer")
    .arg(Arg::new("target").long("target").required(false))
    .get_matches();

  let default_address = "flight".to_owned();
  let target = matches
    .get_one::<String>("target")
    .cloned()
    .unwrap_or(default_address);
  FC_ADDR.set(target).unwrap();

  let mut state = state::State::Init;

  loop {
    state = state.next();
  }
}
