use std::net::{SocketAddr, UdpSocket};
use std::time::Instant;

use crate::command::{init_drivers, Drivers};
use crate::communication::{
  check_and_execute, check_heartbeat, establish_flight_computer_connection,
};
use crate::pins::config_pins;
use jeflog::fail;

pub enum State {
  Init,
  Connect(ConnectData),
  MainLoop(MainLoopData),
  Abort(AbortData),
}

pub struct ConnectData {
  drivers: Drivers,
}

pub struct MainLoopData {
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr,
  then: Instant,
  drivers: Drivers,
}

pub struct AbortData {}

impl State {
  pub fn next(self) -> Self {
    match self {
      State::Init => init(),

      State::Connect(data) => connect(data),

      State::MainLoop(data) => main_loop(data),

      State::Abort(data) => abort(data),
    }
  }
}

fn init() -> State {
  config_pins(); // through linux calls to 'config-pin' script, change pins to GPIO

  match init_drivers() {
    Ok(drivers) => State::Connect(ConnectData { drivers }),
    Err(e) => {
      fail!("Failed to initialize drivers: {e}");
      State::Abort(AbortData {})
    }
  }
}

fn connect(data: ConnectData) -> State {
  let (data_socket, command_socket, fc_address) =
    establish_flight_computer_connection();

  State::MainLoop(MainLoopData {
    my_data_socket: data_socket,
    my_command_socket: command_socket,
    fc_address,
    then: Instant::now(),
    drivers: data.drivers,
  })
}

fn main_loop(mut data: MainLoopData) -> State {
  check_and_execute(&data.my_command_socket);
  let (updated_time, abort_status) =
    check_heartbeat(&data.my_data_socket, data.then);
  data.then = updated_time;

  if abort_status {
    return State::Abort(AbortData {});
  }

  // let datapoint = poll_adcs(&mut data.adcs);
  // send_data(&data.my_data_socket, &data.fc_address, datapoint);

  State::MainLoop(data)
}

fn abort(data: AbortData) -> State {
  todo!()
}
