use std::{borrow::Cow, net::{SocketAddr, ToSocketAddrs, UdpSocket}, process::exit, thread, time::{Duration, Instant}};
use common::comm::{ChannelType, DataPoint, DataMessage, ADCKind, ADCKind::{VBatUmbCharge, SamAnd5V}, Gpio, PinValue::{Low, High}, PinMode::{Input, Output}};
use ads114s06::ADC;
use std::thread::sleep;

use crate::communication::{check_and_execute, check_heartbeat, establish_flight_computer_connection, send_data};
use crate::adc::{init_adcs, poll_adcs};
use crate::command::{init_gpio, open_controllers};
use jeflog::{warn, fail, pass};

#[derive(PartialEq, Debug)]
pub enum State {
  Init(InitData),
  EstablishFlightComputerConnection(EstablishFlightComputerConnectionData),
  Loop(LoopData),
  Abort(AbortData)
}

pub struct InitData {
  gpio_controllers: Vec<Gpio<'a>>
}

pub struct EstablishFlightComputerConnectionData {
  gpio_controllers: Vec<Gpio<'a>>,
  adcs: Vec<ADC<'a>>,
}

pub struct LoopData {
  gpio_controllers: Vec<Gpio<'a>>,
  adcs: Vec<ADC<'a>>,
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr,
  then: Instant
}

pub struct AbortData {
  gpio_controllers: Vec<Gpio<'a>>,
  adcs: Vec<ADC<'a>>
}

impl<'a> State<'a> {
  pub fn next(&mut self) -> Self {
    match self {
      State::Init(data) => {
        init_gpio(&data.gpio_controllers);

        // VBatUmbCharge
        let mut battery_adc: ADC = ADC::new(
          "/dev/spidev0.0",
          gpio_controllers[1].get_pin(17),
          Some(gpio_controllers[0].get_pin(14)),
          VBatUmbCharge
        ).expect("Failed to initialize VBatUmbCharge ADC");

        println!("Battery ADC regs (before init)");
        for (reg, reg_value) in battery_adc.spi_read_all_regs().unwrap().into_iter().enumerate() {
          println!("Reg {:x}: {:08b}", reg, reg_value);
        }
        println!("\n");
    
        let mut adcs: Vec<ADC> = vec![battery_adc];
        init_adcs(&mut adcs);

        State::EstablishFlightComputerConnection((data.gpio_controllers, adcs))
      }

      State::EstablishFlightComputerConnection(data) => {
        let (data_socket, command_socket, fc_address) = establish_flight_computer_connection();
        State::Loop((data.gpio_controllers, data.adcs, data_socket, command_socket, fc_address, Instant::now()))
      },

      State::Loop(data) => {
        check_and_execute(&data.gpio_controllers, &data.my_command_socket);
        data.then = Instant::now();
        let (updated_time, abort_status) = check_heartbeat(&data.my_data_socket, data.then);
        data.then = updated_time;

        if abort_status {
          return State::Abort((data.gpio_controllers, data.adcs))
        }


      }

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
        println!("Hey im tryna get adc data!");
        let datapoints: Vec<DataPoint> = poll_adcs(&mut self.adcs);
        send_data(self.my_data_socket.as_ref().unwrap(), self.fc_address.as_ref().unwrap(), datapoints);

        self.state = State::ExecuteCommands;
      },

      State::Abort(data) => {
        fail!("Aborting...");
        init_gpio(&data.gpio_controllers);
        State::EstablishFlightComputerConnection((data.gpio_controllers, data.adcs))
      }
    }
    
  }
}