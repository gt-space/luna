use crate::{
  IgniterId, 
  adc::{init_adcs, poll_adcs, reset_adcs, start_adcs}, 
  command::{init_gpio, read_cc_fault, enabling_igniter, arming_igniter, 
            read_rbf,
  }, 
  communication::{
    check_and_execute,
    check_heartbeat,
    establish_flight_computer_connection,
    send_data,
  }, 
  pins::{GPIO_CONTROLLER, SPI_INFO}
};
use ads124s06::ADC as ADC_24_bit;
use common::comm::{
  gpio::GpioPin,
  ADCKind,
  IgniterRev1ADC,
};
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

// info about an abort that occurs
#[derive (Clone, Copy)]
pub struct AbortInfo {
  pub received_abort: bool,     
  pub last_heard_from_fc: Instant,
  pub time_aborted: Option<Instant>,
}

pub struct ConnectData {
  adcs: [ADC_24_bit; 4],
  abort_info: AbortInfo
}

pub struct MainLoopData {
  adcs: [ADC_24_bit; 4],
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr,
  then: Instant,
  abort_info: AbortInfo
}

pub struct AbortData {
  adcs: [ADC_24_bit; 4],
  abort_info: AbortInfo,
}

const IGNITER_ADC_ORDER: [IgniterRev1ADC; 4] = [
  IgniterRev1ADC::Continuity,
  IgniterRev1ADC::ConstantCurrent,
  IgniterRev1ADC::ConstantVoltage,
  IgniterRev1ADC::PowerMonitoring,
];

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
  init_gpio(); 

  let mut adcs: [ADC_24_bit; 4] = IGNITER_ADC_ORDER.map(|adc_kind| {
    let spi_info = SPI_INFO
      .get(&adc_kind)
      .expect("Missing SPI_INFO entry for igniter ADC");

    let cs_pin = spi_info.cs.map(|pin_num| {
      Box::new(GPIO_CONTROLLER.get_pin(pin_num)) as Box<dyn GpioPin>
    });
    let drdy_pin = spi_info.drdy.map(|pin_num| {
      Box::new(GPIO_CONTROLLER.get_pin(pin_num)) as Box<dyn GpioPin>
    });

    ADC_24_bit::new_with_gpio_pins(
      spi_info.spi_bus,
      drdy_pin,
      cs_pin,
      ADCKind::IgniterRev1(adc_kind),
    )
    .expect("Failed to initialize ADC 24 bit")
  });

  init_adcs(&mut adcs);

  State::Connect(ConnectData { adcs, abort_info: AbortInfo { 
    received_abort: false, 
    last_heard_from_fc: Instant::now(), 
    time_aborted: None, 
    }})
}

fn connect(mut data: ConnectData) -> State {
  let fc_connect_info =
    establish_flight_computer_connection(&mut data.abort_info);
  start_adcs(&mut data.adcs); // tell the ADCs to start collecting data

  State::MainLoop(MainLoopData {
    adcs: data.adcs,
    my_command_socket: fc_connect_info.command_socket,
    my_data_socket: fc_connect_info.data_socket,
    fc_address: fc_connect_info.fc_address,
    then: Instant::now(),
    abort_info: data.abort_info,
  })
}

fn main_loop(mut data: MainLoopData) -> State {
  check_and_execute(&data.my_command_socket);
  let (updated_time, abort_status) =
    check_heartbeat(&data.my_data_socket, data.then);
  data.then = updated_time;

  if abort_status {
    return State::Abort(AbortData { 
      adcs: data.adcs, 
      abort_info: AbortInfo { 
      received_abort: true, 
      last_heard_from_fc: data.then, 
      time_aborted: Some(Instant::now()), 
      }
    });
  }

  let mut datapoint = poll_adcs(&mut data.adcs);
  datapoint.state.continuity = read_cc_fault();
  datapoint.state.rbf = read_rbf();

  send_data(&data.my_data_socket, &data.fc_address, datapoint);

  State::MainLoop(data)
}

fn abort(mut data: AbortData) -> State {
  fail!("Aborting goodbye!");
  
  // disable all igniter channels and disarm the igniter
  arming_igniter(false);
  for i in 0..6 {
    enabling_igniter(false, i);
  }
  reset_adcs(&mut data.adcs); // reset ADC pin muxing and stop collecting data
  State::Connect(ConnectData { adcs: data.adcs, abort_info: data.abort_info})
}
