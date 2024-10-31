// Copied Vespula BMS code from luna/bms directory

pub mod command;
pub mod adc;
pub mod state;
pub mod communication;

use std::{borrow::Cow, net::{SocketAddr, ToSocketAddrs, UdpSocket}, process::exit, thread, time::{Duration, Instant}};
use command::execute;
use common::comm::{ChannelType, DataMessage, DataPoint, Gpio, PinValue::{Low, High}, SamControlMessage, ADCKind, ADCKind::{VBatUmbCharge, SamAnd5V}};
use jeflog::{warn, fail, pass};
use protocol::init_gpio;
use ads114s06::ADC;
use adc::{init_adcs, poll_adcs};

const FC_ADDR: &str = "server-01";
const BMS_ID: &str = "bms-01";
const COMMAND_PORT: u16 = 8378;
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(250);

fn main() {
  let gpio_controllers: Vec<Gpio> = open_controllers();
  init_gpio(&gpio_controllers);
  
  // VBatUmbCharge
  let mut battery_adc: ADC = ADC::new(
    "/dev/spidev0.0",
    gpio_controllers[1].get_pin(28),
    Some(gpio_controllers[0].get_pin(30)),
    VBatUmbCharge
  ).expect("Failed to initialize VBatUmbCharge ADC");

  thread::sleep(Duration::from_millis(100));

  println!("Battery ADC regs (before init)");
  for (reg, reg_value) in battery_adc.spi_read_all_regs().unwrap().into_iter().enumerate() {
    println!("Reg {:x}: {:08b}", reg, reg_value);
  }
  println!("\n");

  let mut adcs: Vec<ADC> = vec![battery_adc];
  init_adcs(&mut adcs);

  let state_machine = state::StateMachine::new(gpio_controllers, adcs);
  
  loop {
    state_machine.next();
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
}