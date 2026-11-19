//! Runs the HITL service on the CM5.
//!
//! `hitl` opens the board's chips and listens on port 8380.
//! `hitl --mock` runs against a board in memory, for trying the service off
//! the board.
//! `hitl --query <host:port>` asks a running service which build it is and
//! prints it, so a deploy can check it is running the commit under test.

use std::{net::Ipv4Addr, process};

use ad5676r::Reference;
use clap::{value_parser, Arg, ArgAction, Command};
use common::comm::hitl::PORT;
use hitl::{
    client::Client,
    hardware::{Board, BoardConfig, Hardware, Mock},
    service::Service,
    VERSION,
};
use jeflog::{fail, pass};
use tmp112::Address;

fn main() {
    let matches = Command::new("hitl")
        .about("Runs commands from the HITL worker against the HITL test board.")
        .version(VERSION)
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
        .arg(
            Arg::new("query")
                .long("query")
                .value_name("HOST:PORT")
                .conflicts_with_all(["mock", "port", "dac-vref"])
                .help("Print the build a running service reports, then exit"),
        )
        .get_matches();

    if let Some(address) = matches.get_one::<String>("query") {
        query(address);
    }

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

/// Prints the version a running service reports. Exits nonzero if it does not
/// answer.
///
/// Connecting starts the client's heartbeat, which arms the service's
/// watchdog, so the board is safed about a second after this exits. That is
/// harmless right after a deploy, since the board starts in the safe state.
fn query(address: &str) -> ! {
    let version = Client::connect(address).and_then(|mut client| client.version());
    match version {
        Ok(version) => {
            println!("{version}");
            process::exit(0);
        }
        Err(e) => {
            fail!("{address}: {e}");
            process::exit(1);
        }
    }
}
