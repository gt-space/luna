use std::{borrow::Cow, net::{SocketAddr, ToSocketAddrs, UdpSocket}, process::exit, thread, time::{Duration, Instant}};
use common::comm::{ChannelType, DataPoint, ADCKind, ADCKind::{VBatUmbCharge, SamAnd5V}, Gpio, PinValue::{Low, High}, PinMode::{Input, Output}};
use ads114s06::ADC;

use crate:: communication::{check_and_execute, check_heartbeat, establish_flight_computer_connection, send_data};
use crate::adc::poll_adcs;
use crate::protocol::init_gpio;


#[derive(PartialEq, Debug)]
pub enum State {
  /*
  The init gpio and sensors only happens once and should never be looped
  back to. Excluding it prevents the need for option in StateMachine
   */
  EstablishFlightComputerConnection,
  ExecuteCommands,
  CollectSensorData,
  Abort
}

pub struct StateMachine {
  state: State,
  gpio_controllers: Vec<Gpio>,
  adcs: Vec<ADC>,
  my_data_socket: Option<UdpSocket>,
  my_command_socket: Option<UdpSocket>,
  fc_address: Option<SocketAddr>,
  then: Instant
}

impl StateMachine {
  pub fn new(gpio_controllers: Vec<Gpio>, adcs: Vec<ADC>) -> Self {
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
        check_and_execute(&self.gpio_controllers, &self.command_socket);
        self.then = Instant::now();
        let (updated_time, abort_status) = check_heartbeat(&self.my_data_socket, self.then, gpio_controllers);
        self.then = updated_time;

        if abort_status {
          self.state = State::Abort;
        } else {
          self.state = State::CollectSensorData;
        }
      },

      State::CollectSensorData => {
        let datapoints: Vec<DataPoint> = poll_adcs(&mut self.adcs);
        send_data(&self.my_data_socket, &self.fc_address, datapoints);

        self.state = State::ExecuteCommands;
      },

      State::Abort => {
        fail!("Aborting...");
        init_gpio(&self.gpio_controllers);
        self.state = State::EstablishFlightComputerConnection;
      }
    }

    
  }
}