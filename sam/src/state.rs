use ads114s06::ADC;
use crate::adc::{init_adcs, poll_adcs};
use crate::{SAM_VERSION, SamVersion};
use crate::pins::{GPIO_CONTROLLERS, SPI_INFO};
use common::comm::ADCKind::{self, SamRev3, SamRev4Gnd, SamRev4Flight};
use crate::{command::init_gpio, communication::{check_and_execute, check_heartbeat, establish_flight_computer_connection, send_data}};
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

  let mut adcs = vec![];

  for (adc_kind, spi_info) in SPI_INFO.iter() {
    let cs_pin = match &spi_info.cs {
      Some(info) => {
        Some(GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num))
      },

      None => None
    };

    let drdy_pin = match &spi_info.drdy {
      Some (info) => {
        Some(GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num))
      },

      None => None
    };

    let adc: ADC = ADC::new(
      spi_info.spi_bus,
      drdy_pin,
      cs_pin,
      *adc_kind // ADCKind implements Copy so I can just deref it
    ).expect("Failed to initialize ADC");

    adcs.push(adc);
  }

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
  send_data(&data.my_data_socket, &data.fc_address, data.hostname.clone(), data_points);
  
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