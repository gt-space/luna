use crate::adc::{init_adcs, poll_adcs, reset_adcs, start_adcs};
use crate::pins::{config_pins, GPIO_CONTROLLERS, SPI_INFO};
use crate::{
  command::{init_gpio,
    reset_valve_current_sel_pins,
    safe_valves,
    check_prvnt_abort
  },
  communication::{
    check_and_execute,
    check_heartbeat,
    establish_flight_computer_connection,
    send_data,
  },
};
use crate::{SamVersion, SAM_VERSION};
use ads114s06::ADC;
use common::comm::ValveAction;
use jeflog::fail;
use std::time::Duration;
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

// info about an abort that occurs, including the time of disconnect from fc, (potential) prvnt channel that we have a 10 minute timer on to open,
// and a boolean that tells us if we are still aborted
#[derive (Clone, Copy)]
pub struct AbortInfo {
  pub prvnt_channel: u32, // if prvnt does not exist on this sam board, this value will be 0. else, it will be the channel that prvnt is connected to
  pub aborted: bool,      // whether we have aborted our valves so far
  pub last_heard_from_fc: Instant,
  pub time_aborted: Instant, 
  pub opened_prvnt: bool, // if we have crossed the 10 minute timer and have opened prvnt
}

pub struct ConnectData {
  adcs: Vec<ADC>,
  pub abort_info: AbortInfo,
  abort_valve_states: Vec<ValveAction>, 
}

pub struct MainLoopData {
  adcs: Vec<ADC>,
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr,
  hostname: String,
  then: Instant,
  ambient_temps: Option<Vec<f64>>,
  abort_info: AbortInfo,
  abort_valve_states: Vec<ValveAction>,
}

pub struct AbortData {
  adcs: Vec<ADC>,
  abort_info: AbortInfo,
  abort_valve_states: Vec<ValveAction>,
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
  init_gpio(); // turns off all chip selects and valves

  let mut adcs: Vec<ADC> = vec![];

  for (adc_kind, spi_info) in SPI_INFO.iter() {
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

  // Handles all register settings and initial pin muxing for 1st measurement
  init_adcs(&mut adcs);

  // what to set last_heard_from_fc here since its our first time?
  State::Connect(ConnectData { 
    adcs, 
    abort_info: AbortInfo { 
      prvnt_channel: 0, 
      aborted: false, 
      time_aborted: Instant::now(), // WHENEVER WE USE THIS, MAKE SURE TO ENSURE THAT WE HAVE ABORTED IN THE FIRST PLACE
      last_heard_from_fc: Instant::now(), 
      opened_prvnt: false,
    },
    abort_valve_states: Vec::new(),
  })
}

fn connect(mut data: ConnectData) -> State {
  let (data_socket, command_socket, fc_address, hostname, abort_info) =
    establish_flight_computer_connection(&mut data);
  start_adcs(&mut data.adcs); // tell ADCs to start collecting data

  State::MainLoop(MainLoopData {
    adcs: data.adcs,
    my_command_socket: command_socket,
    my_data_socket: data_socket,
    fc_address,
    hostname,
    then: Instant::now(),
    /*
    Thermocouples (TC) are used on Rev3. A correct TC reading requires
    knowing the ambient temperature of the PCB because the solder is
    an additional junction (hmu if you want to know more about this). The
    ADC can get the temperature of the PCB but this value must be available
    for multiple iterations of the poll_adcs function and the ADC struct
    does not hold any extra data so it is stored in this struct so the values
    can be modified and read. The ambient_temps vector is passed into the
    poll_adcs function to be made available
     */
    ambient_temps: if *SAM_VERSION == SamVersion::Rev3 {
      Some(vec![0.0; 2]) // a TC value needs the ambient temperature
    } else {
      None
    },
    abort_info:  abort_info,
    abort_valve_states: data.abort_valve_states,
  })
}

fn main_loop(mut data: MainLoopData) -> State {
  // check if connection to FC is still exists
  let (updated_time, abort_status) =
    check_heartbeat(&data.my_data_socket, &data.my_command_socket, data.then);
  data.then = updated_time;

  if abort_status {
    return State::Abort(AbortData { 
      adcs: data.adcs, 
      abort_info: AbortInfo { 
        prvnt_channel: data.abort_info.prvnt_channel, 
        aborted: true, 
        time_aborted: data.abort_info.time_aborted, 
        last_heard_from_fc: data.then, 
        opened_prvnt: false
      },
      abort_valve_states: data.abort_valve_states,
    });
  }

  // if there are commands, do them!
  check_and_execute(&data.my_command_socket, &mut data.abort_info.prvnt_channel, &mut data.abort_valve_states, &mut data.abort_info.aborted, &mut data.abort_info.time_aborted);

  // if we need to check prvnt timer
  if data.abort_info.aborted && !data.abort_info.opened_prvnt {
    check_prvnt_abort(&mut data.abort_info.opened_prvnt, data.abort_info.prvnt_channel, data.abort_info.time_aborted);
  }

  let datapoints = poll_adcs(&mut data.adcs, &mut data.ambient_temps);

  send_data(
    &data.my_data_socket,
    &data.fc_address,
    data.hostname.clone(),
    datapoints,
  );

  State::MainLoop(data)
}

fn abort(mut data: AbortData) -> State {
  fail!("Aborting goodbye!");
  // depower all valves
  safe_valves(&data.abort_valve_states, data.abort_info.prvnt_channel, &mut data.abort_info.time_aborted);
  // reset ADC pin muxing
  reset_adcs(&mut data.adcs);
  // reset pins that select which valve currents are measured from valve driver
  reset_valve_current_sel_pins();
  // continiously attempt to reconnect to flight computer
  State::Connect(ConnectData{ 
    adcs: data.adcs,
    abort_info: data.abort_info,
    abort_valve_states: data.abort_valve_states,
  })
}
