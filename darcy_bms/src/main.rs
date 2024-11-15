// Copied Vespula BMS code from luna/bms directory

pub mod command;
pub mod adc;
pub mod state;
pub mod communication;

use std::{borrow::Cow, net::{SocketAddr, ToSocketAddrs, UdpSocket}, process::exit, thread, time::{Duration, Instant}};
use command::execute;
use common::comm::{ChannelType, DataMessage, DataPoint, Gpio, PinValue::{Low, High}, SamControlMessage, ADCKind, ADCKind::{VBatUmbCharge, SamAnd5V}};
use jeflog::{warn, fail, pass};
use crate::command::{init_gpio, open_controllers};
use ads114s06::ADC;
use adc::{init_adcs, poll_adcs};

const FC_ADDR: &str = "server-01";
const BMS_ID: &str = "bms-01";
const COMMAND_PORT: u16 = 8378;
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(250);

fn main() {
  let mut state = state::State::Init((open_controllers()));
  
  loop {
    state = state.next();
  }
}

  // // begin FC communication
  // let (data_socket, command_socket, fc_address) = establish_flight_computer_connection();
  
  // let mut then = Instant::now();
  // loop {
  //   println!("Checking for commands...");

  //   // check if commands were sent from the FC. if so, execute them
  //   check_and_execute(&gpio_controllers, &command_socket);
  //   println!("Checking heartbeat...");

  //   // check if the FC is sending a heartbeat. if so, reset the timer
  //   then = check_heartbeat(&data_socket, then, &gpio_controllers);
    
  //   let datapoints = poll_adcs(&mut adcs);

  //   println!("Sending data...");

  //   // send the adc data to the FC
  //   send_data(&data_socket, &fc_address, datapoints);
  // }