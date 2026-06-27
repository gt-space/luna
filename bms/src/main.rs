pub mod adc;
pub mod command;
pub mod communication;
pub mod pins;
pub mod state;

use std::{net::SocketAddr, sync::LazyLock};

use clap::{Arg, Command};
use communication::get_version;
use once_cell::sync::OnceCell;

pub static FC_ADDR: OnceCell<String> = OnceCell::new();
pub static CACHED_FC_ADDRESS: OnceCell<SocketAddr> = OnceCell::new();
pub static BMS_VERSION: LazyLock<BmsVersion> = LazyLock::new(get_version);

#[derive(PartialEq, Debug)]
pub enum BmsVersion {
    Rev2,
    Rev3,
    Rev4,
}

fn main() {
    let matches = Command::new("bms")
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
