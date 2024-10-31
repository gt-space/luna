use std::{borrow::Cow, net::{SocketAddr, ToSocketAddrs, UdpSocket}, process::exit, thread, time::{Duration, Instant}};
use common::comm::{ChannelType, DataPoint, ADCKind, ADCKind::{VBatUmbCharge, SamAnd5V}, Gpio, PinValue::{Low, High}, PinMode::{Input, Output}};
use ads114s06::ADC;

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
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr
}

impl State {
  pub fn next(self) -> Self {
    match self.state {
      State::EstablishFlightComputerConnection => {
        State::ExecuteCommands
      },

      State::ExecuteCommands => {
        State::CollectSensorData
      },

      State::CollectSensorData => {
        State::ExecuteCommands
      },

      State::Abort => {
        State::EstablishFlightComputerConnection
      }
    }
  }
}

impl StateMachine {
  pub fn new(&mut self, gpio_controllers: Vec<Gpio>, adcs: Vec<ADC>, my_data_socket: UdpSocket, my_command_socket: UdpSocket, fc_address: SocketAddr) -> Self {
    StateMachine {
      state: State::EstablishFlightComputerConnection,
      gpio_controllers,
      adcs,
      my_data_socket,
      my_command_socket,
      fc_address
    }
  }

  pub fn execute_state(&mut self) {
    match self.state {
      State::EstablishFlightComputerConnection => {

      },

      State::ExecuteCommands => {

      },

      State::CollectSensorData => {

      },

      State::Abort => {

      }
    }
  }
}