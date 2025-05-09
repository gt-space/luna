use crate::{
  adc::{init_adcs, poll_tc_adcs, reset_adcs, start_adcs},
  communication::{
      establish_flight_computer_connection,
      check_heartbeat,
      send_data,
  },
  pins::{config_pins, init_cs, GPIO_CONTROLLERS, SPI_INFO},
};
use ads114s06::ADC;
use jeflog::fail;
use std::{net::{UdpSocket, SocketAddr}, time::Instant};

pub enum State {
  Init,
  Connect(ConnectData),
  MainLoop(MainLoopData),
  Abort(AbortData),
}

pub struct ConnectData {
  adcs: Vec<ADC>,
}

pub struct MainLoopData {
  adcs: Vec<ADC>,
  data_socket: UdpSocket,
  fc_address: SocketAddr,
  then: Instant,
}

pub struct AbortData {
  adcs: Vec<ADC>,
}

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

/// configure pinmux, reset CS lines and ADC registers
fn init() -> State {
  config_pins();        
  init_cs();            

  let mut adcs = Vec::new();
  for (kind, spi_info) in SPI_INFO.iter() {
      let cs_pin = spi_info
          .cs
          .as_ref()
          .map(|info| GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num));

      let adc = ADC::new(
          spi_info.spi_bus,
          None,
          cs_pin,
          *kind,
      )
      .expect("Failed to initialize ADC");

      adcs.push(adc);
  }
  init_adcs(&mut adcs);  

  State::Connect(ConnectData { adcs })
}

/// handshake with flight computer, start ADC conversions
fn connect(data: ConnectData) -> State {
  let (data_socket, _command_socket, fc_address) =
      establish_flight_computer_connection();

  let mut adcs = data.adcs;
  start_adcs(&mut adcs); 

  State::MainLoop(MainLoopData {
      adcs,
      data_socket,
      fc_address,
      then: Instant::now(),
  })
}

/// check heartbeats, poll ADCs, send data
fn main_loop(mut data: MainLoopData) -> State {
  let (updated_time, abort) =
      check_heartbeat(&data.data_socket, data.then);
  data.then = updated_time;
  if abort {
      return State::Abort(AbortData { adcs: data.adcs });
  }

  let datapoint = poll_tc_adcs(&mut data.adcs);

  send_data(&data.data_socket, &data.fc_address, datapoint);

  State::MainLoop(data)
}

/// log error, reset hardware, try connecting again
fn abort(data: AbortData) -> State {
  fail!("Thermocouple module aborting, restarting...");

  init_cs();                 
  reset_adcs(&mut data.adcs); 

  State::Connect(ConnectData { adcs: data.adcs })
}
