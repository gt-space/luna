use ads114s06::ADC;
use crate::newadc::{init_adcs, poll_adcs};
use common::comm::{ADCKind::{Sam, SamRev4}, SamADC, SamRev4ADC};
use crate::{command::{GPIO_CONTROLLERS, init_gpio}, communication::{check_and_execute, check_heartbeat, establish_flight_computer_connection, send_data}};
use std::{net::{SocketAddr, UdpSocket}, thread, time::{Duration, Instant}};
use jeflog::fail;

pub enum State {
  Init,
  Connect(ConnectData),
  MainLoop(MainLoopData),
  Abort(AbortData)
}

pub struct ConnectData {
  adcs: Vec<ADC>,
}

pub struct MainLoopData {
  adcs: Vec<ADC>,
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr,
  hostname: String,
  then: Instant
}

pub struct AbortData {
  adcs: Vec<ADC>
}

impl State {
  
  pub fn next(self) -> Self {
    match self {
      State::Init => {
        init()
      },

      State::Connect(data) => {
        connect(data)
      },

      State::MainLoop(data) => {
        main_loop(data)
      },

      State::Abort(data) => {
        abort(data)
      }
    }
  }
}

// handle flight for now
fn init() -> State {
  init_gpio();

  // Valve Voltage ADC
  let mut vvalve: ADC = ADC::new(
    "/dev/spidev0.0",
    GPIO_CONTROLLERS[0].get_pin(0),
    Some(GPIO_CONTROLLERS[0].get_pin(0)),
    Sam(SamADC::VValve)
  ).expect("Failed to initialize valve voltage ADC");
}

fn connect(data: ConnectData) -> State {

}

fn main_loop(mut data: MainLoopData) -> State {

}

fn abort(data: AbortData) -> State {

}