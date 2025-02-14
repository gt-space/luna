use crate::{
  adc::{init_adcs, poll_adcs, reset_adcs, start_adcs},
  command::init_gpio,
  communication::{
    check_and_execute,
    check_heartbeat,
    establish_flight_computer_connection,
    send_data,
  },
  pins::{config_pins, GPIO_CONTROLLERS, SPI_INFO},
};
use ads114s06::ADC;
use jeflog::fail;
use std::{
  net::{SocketAddr, UdpSocket},
  time::Instant,
};

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
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
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

fn init() -> State {
  config_pins(); // through linux calls to 'config-pin' script, change pins to GPIO
  init_gpio(); // safe system and disable all chip selects

  let mut adcs: Vec<ADC> = vec![];

  for (adc_kind, spi_info) in SPI_INFO.iter() {
    // let cs_pin = match &spi_info.cs {
    //   Some(info) => {
    //     Some(GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num))
    //   }

    //   None => None,
    // };

    // let drdy_pin = match &spi_info.drdy {
    //   Some(info) => {
    //     Some(GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num))
    //   }

    //   None => None,
    // };
    let cs_pin = spi_info
      .cs
      .as_ref()
      .map(|info| GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num));
    let drdy_pin = spi_info
      .drdy
      .as_ref()
      .map(|info| GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num));

    let adc: ADC = ADC::new(
      spi_info.spi_bus,
      drdy_pin,
      cs_pin,
      *adc_kind, // ADCKind implements Copy so I can just deref it
    )
    .expect("Failed to initialize ADC");

    adcs.push(adc);
  }

  init_adcs(&mut adcs);

  State::Connect(ConnectData { adcs })
}

fn connect(mut data: ConnectData) -> State {
  let (data_socket, command_socket, fc_address) =
    establish_flight_computer_connection();
  start_adcs(&mut data.adcs); // tell the ADCs to start collecting data

  State::MainLoop(MainLoopData {
    adcs: data.adcs,
    my_command_socket: command_socket,
    my_data_socket: data_socket,
    fc_address,
    then: Instant::now(),
  })
}

fn main_loop(mut data: MainLoopData) -> State {
  check_and_execute(&data.my_command_socket);
  let (updated_time, abort_status) =
    check_heartbeat(&data.my_data_socket, data.then);
  data.then = updated_time;

  if abort_status {
    return State::Abort(AbortData { adcs: data.adcs });
  }

  let datapoint = poll_adcs(&mut data.adcs);
  send_data(&data.my_data_socket, &data.fc_address, datapoint);

  State::MainLoop(data)
}

fn abort(mut data: AbortData) -> State {
  fail!("Aborting goodbye!");
  init_gpio();
  /* init_gpio turns off all chip selects but reset_adcs makes use of them
  again. However with the ADC driver that reset_adcs uses, each chip select
  will be turned off after the ADC is done being communicated with. init_gpio
  needs to turn off all chip selects at the start so its mainly code reuse
   */

  reset_adcs(&mut data.adcs); // reset ADC pin muxing and stop collecting data
  State::Connect(ConnectData { adcs: data.adcs })
}
