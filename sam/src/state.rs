use crate::adc::{init_adcs, poll_adcs, reset_adcs, start_adcs};
use crate::pins::{config_pins, GPIO_CONTROLLERS, ADC_INFORMATION};
use crate::{
  command::{init_gpio,
    reset_valve_current_sel_pins,
    safe_valves
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
use common::comm::{ADCKind, SamRev3ADC, SamRev4GndADC, SamRev4FlightADC};
use jeflog::fail;
use std::collections::VecDeque;
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
  polling_adcs: VecDeque<ADC>,
  waiting_adcs: VecDeque<ADC>
}

pub struct MainLoopData {
  polling_adcs: VecDeque<ADC>,
  waiting_adcs: VecDeque<ADC>,
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr,
  hostname: String,
  then: Instant,
  ambient_temps: Option<Vec<f64>>,
}

pub struct AbortData {
  polling_adcs: VecDeque<ADC>,
  waiting_adcs: VecDeque<ADC>
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

  let mut polling_adcs: VecDeque<ADC> = VecDeque::new();
  let mut waiting_adcs: VecDeque<ADC> = VecDeque::new();

  // polling queue
  for adc_info in ADC_INFORMATION.iter() {
    let cs_pin = adc_info
      .spi_info
      .cs
      .as_ref()
      .map(|info| GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num));

    let drdy_pin = adc_info
      .spi_info
      .drdy
      .as_ref()
      .map(|info| GPIO_CONTROLLERS[info.controller].get_pin(info.pin_num));

    let adc: ADC = ADC::new(
      adc_info.spi_info.spi_bus,
      drdy_pin,
      cs_pin,
      adc_info.kind, // ADCKind implements Copy so I can just deref it
    )
    .expect("Failed to initialize ADC");

    match adc_info.kind {
      ADCKind::SamRev3(SamRev3ADC::CurrentLoopPt) |
      ADCKind::SamRev3(SamRev3ADC::DiffSensors) |
      ADCKind::SamRev4Gnd(SamRev4GndADC::CurrentLoopPt) |
      ADCKind::SamRev4Gnd(SamRev4GndADC::DiffSensors) |
      ADCKind::SamRev4Flight(SamRev4FlightADC::CurrentLoopPt) |
      ADCKind::SamRev4Flight(SamRev4FlightADC::DiffSensors) => {
        polling_adcs.push_back(adc);
      },

      ADCKind::VespulaBms(_) => {
        panic!("Imposter Vespula BMS ADC among us!")
      },

      _ => {
        waiting_adcs.push_back(adc);
      }
    }
  }

  // Handles all register settings and initial pin muxing for 1st measurement
  init_adcs(&mut polling_adcs);
  init_adcs(&mut waiting_adcs);

  State::Connect(ConnectData {
    polling_adcs,
    waiting_adcs
  })
}

fn connect(mut data: ConnectData) -> State {
  let (data_socket, command_socket, fc_address, hostname) =
    establish_flight_computer_connection();
  start_adcs(&mut data.polling_adcs); // tell ADCs to start collecting data
  start_adcs(&mut data.waiting_adcs);

  State::MainLoop(MainLoopData {
    polling_adcs: data.polling_adcs,
    waiting_adcs: data.waiting_adcs,
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
  })
}

fn main_loop(mut data: MainLoopData) -> State {
  // check if connection to FC is still exists
  let (updated_time, abort_status) =
    check_heartbeat(&data.my_data_socket, &data.my_command_socket, data.then);
  data.then = updated_time;

  if abort_status {
    return State::Abort(AbortData { 
      polling_adcs: data.polling_adcs,
      waiting_adcs: data.waiting_adcs
     });
  }

  // if there are commands, do them!
  check_and_execute(&data.my_command_socket);

  let datapoints = poll_adcs(
    &mut data.polling_adcs,
    &mut data.waiting_adcs,
    &mut data.ambient_temps
  );

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
  safe_valves();
  // reset ADC pin muxing
  reset_adcs(&mut data.polling_adcs);
  reset_adcs(&mut data.waiting_adcs);
  // reset pins that select which valve currents are measured from valve driver
  reset_valve_current_sel_pins();
  // continiously attempt to reconnect to flight computer
  State::Connect(ConnectData { 
    polling_adcs: data.polling_adcs,
    waiting_adcs: data.waiting_adcs
  })
}
