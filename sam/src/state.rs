use ads114s06::ADC;
use crate::{adc::{init_adcs, poll_adcs}, SAM_INFO, SamVersion};
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

  // UPDATE ALL CS AND DRDY PINS!

  // Valve Voltage ADC

  let mut adcs = match SAM_INFO.version {
    SamVersion::Rev3 => {

      vec![]
    },

    SamVersion::Rev4Ground => {

      vec![]
    },

    SamVersion::Rev4Flight => {

      vec![]
    }
  };

  init_adcs(&mut adcs);

  State::Connect(
    ConnectData {
      adcs
    }
  )
}

fn connect(data: ConnectData) -> State {
  let (data_socket, command_socket, fc_address, hostname) = establish_flight_computer_connection();

  State::MainLoop(
    MainLoopData {
      adcs: data.adcs,
      my_command_socket: command_socket,
      my_data_socket: data_socket,
      fc_address,
      hostname,
      then: Instant::now()
    }
  )
}

fn main_loop(mut data: MainLoopData) -> State {
  check_and_execute(&data.my_command_socket);
  let (updated_time, abort_status) = check_heartbeat(&data.my_data_socket, data.then);
  data.then = updated_time;

  if abort_status {
    return State::Abort(
      AbortData {
        adcs: data.adcs
      }
    )
  }

  let data_points = poll_adcs(&mut data.adcs);
  send_data(&data.my_data_socket, &data.my_command_socket, datapoints);
  
  State::MainLoop(data)
}

fn abort(data: AbortData) -> State {
  fail!("Aborting goodbye!");
  init_gpio();

  State::Connect(
    ConnectData {
      adcs: data.adcs
    }
  )
}