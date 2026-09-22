use std::fmt::Debug;

use clap::ValueEnum;
use common::sequence::{MMAP_PATH, SOCKET_PATH};

use crate::{
    cli::RuntimeConfig, device::Devices, servo, DEVICE_COMMAND_PORT, FC_SOCKET_ADDRESS,
    FC_TO_SERVO_RADIO_RATE, FC_TO_SERVO_RATE, GOLDFISH_SYSTEM_SAFE_TIMER, LOG_INTERVAL,
    MMAP_GRACE_PERIOD, RADIO_PAYLOAD_MTU, SEND_HEARTBEAT_RATE, SERVO_DATA_PORT,
    SERVO_RECONNECT_RETRY_COUNT, SERVO_RECONNECT_TIMEOUT, SERVO_SOCKET_ADDRESSES,
    SERVO_TO_FC_TIME_TO_LIVE, TIME_TO_LIVE, UMBILICAL_BUS_VOLTAGE_THRESHOLD,
};

/// Select sections of config readout with --show-config flag
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum Section {
    Build,
    Workers,
    Logging,
    Network,
    Timing,
    Safety,
    Controller,
    Navigation,
}

pub fn print(sections: &[Section], config: &RuntimeConfig) {
    let sections = if sections.is_empty() {
        Section::value_variants()
    } else {
        sections
    };

    for section in sections {
        println!("== {section:?} ==");
        match section {
            Section::Build => {
                println!(
                    "package: {} v{}",
                    env!("CARGO_PKG_NAME"),
                    env!("CARGO_PKG_VERSION")
                );
                println!("debug build: {}", cfg!(debug_assertions));
                println!("layout fingerprint: {}", common::LAYOUT_FINGERPRINT);
            }
            Section::Workers => {
                print_fields(&config.worker_config);
                println!("print_gps: {}", config.print_gps);
            }
            Section::Logging => print_fields(&config.logger_config),
            Section::Network => {
                println!("servo addresses: {SERVO_SOCKET_ADDRESSES:?}");
                println!("fc socket: {FC_SOCKET_ADDRESS:?}");
                println!("device command port: {DEVICE_COMMAND_PORT}");
                println!("servo data port: {SERVO_DATA_PORT}");
                println!("radio payload mtu: {RADIO_PAYLOAD_MTU}");
                println!("radio dscp: {:#04X}", servo::RADIO_TELEMETRY_DSCP);
                println!("sequence socket: {SOCKET_PATH}");
                println!("vehicle state mmap: {MMAP_PATH}");
            }
            Section::Timing => {
                println!("fc to servo rate: {FC_TO_SERVO_RATE:?}");
                println!("fc to servo radio rate: {FC_TO_SERVO_RADIO_RATE:?}");
                println!("log interval: {LOG_INTERVAL:?}");
                println!("heartbeat rate: {SEND_HEARTBEAT_RATE:?}");
                println!("board time to live: {TIME_TO_LIVE:?}");
                println!("servo time to live: {SERVO_TO_FC_TIME_TO_LIVE:?}");
                println!("servo keep alive: {:?}", servo::SERVO_KEEP_ALIVE_DELAY);
                println!("servo reconnect tries: {SERVO_RECONNECT_RETRY_COUNT}");
                println!("servo reconnect timeout: {SERVO_RECONNECT_TIMEOUT:?}");
                println!("mmap grace period: {MMAP_GRACE_PERIOD:?}");
            }
            Section::Safety => {
                println!(
                    "abort on servo disconnect: {}",
                    Devices::new().servo_disconnect_abort_enabled()
                );
                println!("goldfish system safe timer: {GOLDFISH_SYSTEM_SAFE_TIMER:?}");
                println!("umbilical bus voltage threshold: {UMBILICAL_BUS_VOLTAGE_THRESHOLD} V");
            }
            // TODO: implement controller config when controller is wired in main
            Section::Controller => println!("not implemented"),
            // TODO: implement state estimation config when controller is wired in main
            Section::Navigation => println!("not implemented"),
        }
    }
}

/// print fields for worker and logger config
fn print_fields(config: &impl Debug) {
    let debug = format!("{config:#?}");
    for field in debug.lines().filter_map(|line| line.strip_prefix("    ")) {
        if field.starts_with(' ') {
            println!("{field}");
        } else {
            println!("{}", field.trim_end_matches(','));
        }
    }
}
