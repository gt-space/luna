pub mod sam;
pub mod communication;

use clap::{builder::PossibleValuesParser, Arg, Command};
use std::net::ToSocketAddrs;
use crate::sam::simulate_sam;

fn main() {
  let matches = Command::new("sim")
    .about("simulate a board like sam, bms, ahrs")
    .arg(
      Arg::new("hostname")
        .required(true)
        .help("hostname of board such as sam-01, bms-01")
        .value_parser(PossibleValuesParser::new(["sam-01", "sam-02"])),
    ).get_matches();

    let flight_addr = "localhost:4573"
      .to_socket_addrs()
      .unwrap()
      .find(|addr| addr.is_ipv4())
      .unwrap();

    let hostname = matches.get_one::<String>("hostname").unwrap().as_str();
    match hostname {
      "sam-01" | "sam-02" => {
        println!("Hostname: {}", hostname);
        simulate_sam(hostname.to_string(), flight_addr);
      },

      _ => panic!("Invalid board name given!")
    }
}
