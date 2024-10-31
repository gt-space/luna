use std::{borrow::Cow, net::{SocketAddr, ToSocketAddrs, UdpSocket}, process::exit, thread, time::{Duration, Instant}};
use common::comm::{ChannelType, DataPoint, DataMessage, ADCKind, ADCKind::{VBatUmbCharge, SamAnd5V}, Gpio, PinValue::{Low, High}, PinMode::{Input, Output}};
use ads114s06::ADC;

use crate::communication::{check_and_execute, check_heartbeat, establish_flight_computer_connection, send_data};
use crate::adc::{init_adcs, poll_adcs};
use crate::command::{init_gpio, open_controllers};
use jeflog::{warn, fail, pass};

#[derive(PartialEq, Debug)]
pub enum State {
  EstablishFlightComputerConnection,
  ExecuteCommands,
  CollectSensorData,
  Abort
}

pub struct StateMachine<'a> {
  state: State,
  gpio_controllers: &'a [Gpio],
  adcs: Vec<ADC<'a>>,
  my_data_socket: Option<UdpSocket>,
  my_command_socket: Option<UdpSocket>,
  fc_address: Option<SocketAddr>,
  then: Instant
}

impl<'a> StateMachine<'a> {
  pub fn start(gpio_controllers: &'a [Gpio]) -> Self {
    init_gpio(&gpio_controllers);

    // VBatUmbCharge
    let mut battery_adc: ADC = ADC::new(
      "/dev/spidev0.0",
      gpio_controllers[1].get_pin(17),
      Some(gpio_controllers[0].get_pin(14)),
      VBatUmbCharge
    ).expect("Failed to initialize VBatUmbCharge ADC");

    thread::sleep(Duration::from_millis(100));

    println!("Battery ADC regs (before init)");
    for (reg, reg_value) in battery_adc.spi_read_all_regs().unwrap().into_iter().enumerate() {
      println!("Reg {:x}: {:08b}", reg, reg_value);
    }
    println!("\n");

    let mut adcs: Vec<ADC> = vec![battery_adc];
    init_adcs(&mut adcs);

    StateMachine {
      state: State::EstablishFlightComputerConnection,
      gpio_controllers,
      adcs,
      my_data_socket: None,
      my_command_socket: None,
      fc_address: None,
      then: Instant::now()
    }
  }

  pub fn next(&mut self) {
    match self.state {
      State::EstablishFlightComputerConnection => {
        let (data_socket, command_socket, fc_address) = establish_flight_computer_connection();
        self.my_data_socket = Some(data_socket);
        self.my_command_socket = Some(command_socket);
        self.fc_address = Some(fc_address);

        self.state = State::ExecuteCommands;
      },

      State::ExecuteCommands => {
        check_and_execute(self.gpio_controllers, self.my_command_socket.as_ref().unwrap());
        self.then = Instant::now();
        let (updated_time, abort_status) = check_heartbeat(self.my_data_socket.as_ref().unwrap(), self.then);
        self.then = updated_time;

        if abort_status {
          self.state = State::Abort;
        } else {
          self.state = State::CollectSensorData;
        }
      },

      State::CollectSensorData => {
        let datapoints: Vec<DataPoint> = poll_adcs(&mut self.adcs);
        send_data(self.my_data_socket.as_ref().unwrap(), self.fc_address.as_ref().unwrap(), datapoints);

        self.state = State::ExecuteCommands;
      },

      State::Abort => {
        fail!("Aborting...");
        init_gpio(self.gpio_controllers);
        self.state = State::EstablishFlightComputerConnection;
      }
    }
    
  }
}