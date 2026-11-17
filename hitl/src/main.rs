//! Runs the HITL service on the CM5.
//!
//! `hitl` opens the board's chips and listens on port 8380.
//! `hitl --mock` runs against a board in memory, for trying the service off
//! the board.

use std::net::Ipv4Addr;

use ad5676r::Reference;
use clap::{value_parser, Arg, ArgAction, Command};
use common::comm::hitl::PORT;
use hitl::{
    hardware::{Board, BoardConfig, Hardware, Mock},
    service::Service,
};
use jeflog::pass;
use tmp112::Address;

fn main() {
    let matches = Command::new("hitl")
        .about("Runs commands from the HITL worker against the HITL test board.")
        .arg(
            Arg::new("port")
                .long("port")
                .value_parser(value_parser!(u16))
                .help("UDP port to listen on [default: 8380]"),
        )
        .arg(
            Arg::new("mock")
                .long("mock")
                .action(ArgAction::SetTrue)
                .help("Use a board in memory instead of the real chips"),
        )
        .arg(
            Arg::new("dac-vref")
                .long("dac-vref")
                .value_parser(value_parser!(f64))
                .help("External DAC reference in volts (default: internal 2.5 V)"),
        )
        .arg(
            Arg::new("tmp-address")
                .long("tmp-address")
                .value_parser(["48", "49"])
                .default_value("48")
                .help("TMP112 I2C address in hex"),
        )
        .get_matches();

    let port = matches.get_one::<u16>("port").copied().unwrap_or(PORT);
    let address = (Ipv4Addr::UNSPECIFIED, port);

    if matches.get_flag("mock") {
        run(address, Mock::default());
    }

    let config = BoardConfig {
        dac_reference: match matches.get_one::<f64>("dac-vref") {
            Some(&volts) => Reference::External(volts),
            None => Reference::Internal,
        },
        tmp_address: match matches.get_one::<String>("tmp-address").unwrap().as_str() {
            "49" => Address::VPlus,
            _ => Address::Gnd,
        },
    };
    run(address, Board::open(config));
}

fn run(address: (Ipv4Addr, u16), hw: impl Hardware) -> ! {
    let mut service = Service::bind(address, hw).expect("could not bind the service socket");
    pass!("Listening on {}:{}.", address.0, address.1);
    service.run()
}
