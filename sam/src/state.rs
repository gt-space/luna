use crate::adc::{ADCSet, init_adcs, poll_adcs, reset_adcs, start_adcs};
use common::comm::{ADCKind::{SamRev3, SamRev4Gnd, SamRev4Flight}, 
  SamRev3ADC, 
  SamRev4GndADC, 
  SamRev4FlightADC};
use crate::pins::{config_pins, GPIO_CONTROLLERS, ADC_INFORMATION};
use crate::{
  SamVersion,
  SAM_VERSION,
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
  adc_set: ADCSet
}

pub struct MainLoopData {
  iteration: u64,
  adc_set: ADCSet,
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr,
  hostname: String,
  then: Instant,
  ambient_temps: Option<Vec<f64>>,
}

pub struct AbortData {
  adc_set: ADCSet
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

  let mut adc_set = ADCSet::new();

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

    match adc.kind {
      SamRev3(rev3_adc) => match rev3_adc {
        SamRev3ADC::CurrentLoopPt | SamRev3ADC::DiffSensors => {
          adc_set.critical.push(adc)
        },

        SamRev3ADC::Tc1 | SamRev3ADC::Tc2 => {
          adc_set.temperature.push(adc);
        },

        SamRev3ADC::IValve | SamRev3ADC::VValve => {
          adc_set.valves.push(adc);
        },

        SamRev3ADC::IPower | SamRev3ADC::VPower => {
          if let Some(ref mut power) = adc_set.power {
            power.push(adc);
          }
        }
      },

      SamRev4Gnd(rev4_gnd_adc) => match rev4_gnd_adc {
        SamRev4GndADC::CurrentLoopPt | SamRev4GndADC::DiffSensors => {
          adc_set.critical.push(adc);
        },

        SamRev4GndADC::Rtd1 | SamRev4GndADC::Rtd2 | SamRev4GndADC::Rtd3 => {
          adc_set.temperature.push(adc);
        },

        SamRev4GndADC::IValve | SamRev4GndADC::VValve => {
          adc_set.valves.push(adc);
        }
      },

      SamRev4Flight(rev4_flight_adc) => match rev4_flight_adc {
        SamRev4FlightADC::CurrentLoopPt | SamRev4FlightADC::DiffSensors => {
          adc_set.critical.push(adc);
        },

        SamRev4FlightADC::Rtd1 | SamRev4FlightADC::Rtd2 | SamRev4FlightADC::Rtd3 => {
          adc_set.temperature.push(adc);
        },

        SamRev4FlightADC::IValve | SamRev4FlightADC::VValve => {
          adc_set.valves.push(adc);
        }
      },

      _ => unreachable!("Imposter ADC among us!")
    }
  }

  // Handles all register settings and initial pin muxing for 1st measurement
  init_adcs(&mut adc_set);

  State::Connect(ConnectData { adc_set })
}

fn connect(mut data: ConnectData) -> State {
  let (data_socket, command_socket, fc_address, hostname) =
    establish_flight_computer_connection();
  start_adcs(&mut data.adc_set); // tell ADCs to start collecting data

  State::MainLoop(MainLoopData {
    iteration: 0,
    adc_set: data.adc_set,
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
    return State::Abort(AbortData { adc_set: data.adc_set });
  }

  // if there are commands, do them!
  check_and_execute(&data.my_command_socket);

  let datapoints = poll_adcs(
    data.iteration,
    &mut data.adc_set,
    &mut data.ambient_temps
  );

  send_data(
    &data.my_data_socket,
    &data.fc_address,
    data.hostname.clone(),
    datapoints,
  );

  data.iteration += 1;
  
  State::MainLoop(data)
}

fn abort(mut data: AbortData) -> State {
  fail!("Aborting goodbye!");
  // depower all valves
  safe_valves();
  // reset ADC pin muxing
  reset_adcs(&mut data.adc_set);
  // reset pins that select which valve currents are measured from valve driver
  reset_valve_current_sel_pins();
  // continiously attempt to reconnect to flight computer
  State::Connect(ConnectData { adc_set: data.adc_set })
}
