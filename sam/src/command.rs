use common::comm::{gpio::{Gpio, Pin, PinMode::Output, PinValue::{High, Low}}, sam::SamControlMessage, ADCKind::{self, Sam, SamRev4}, SamADC, SamRev4ADC};
use std::{thread, time::Duration};
use std::collections::HashMap;
use once_cell::sync::Lazy;

use crate::pins::{GPIO_CONTROLLERS, VALVE_PINS, GpioInfo};

pub fn init_gpio() {
  // disable all chip selects
  // turn off all valves
  // put valve current sense gpios into low state to sense valves 1, 3, and 5
}

// pub fn begin(gpio_controllers: Vec<Arc<Gpio>>) { // data: 4573
//   let socket = UdpSocket::bind("0.0.0.0:8378").expect("Cannot bind to socket");
//   let mut buf = [0; 65536];
//   loop {
//     let (num_bytes, _src_addr) =
//       socket.recv_from(&mut buf).expect("no data received");
//     println!("{:?}", num_bytes);
//     let deserialized_result =
//       postcard::from_bytes::<SamControlMessage>(&buf[..num_bytes]);
//     println!("{:#?}", deserialized_result);
//     match deserialized_result {
//       Ok(message) => {
//         execute(message, gpio_controllers.clone());
//       }
//       Err(_error) => fail!("Bad command message from flight computer"),
//     };
//   }
// }

fn execute(command: SamControlMessage, gpio_controllers: Vec<Arc<Gpio>>) {
  match command {
    SamControlMessage::SetLed { channel, on } => match on {
      true => match channel {
        0 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr0/brightness")
            .unwrap();
          file.write_all(b"1").expect("Failed to write");
        }
        1 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr1/brightness")
            .unwrap();
          file.write_all(b"1").expect("Failed to write");
        }
        2 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr2/brightness")
            .unwrap();
          file.write_all(b"1").expect("Failed to write");
        }
        3 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr3/brightness")
            .unwrap();
          file.write_all(b"1").expect("Failed to write");
        }
        _ => println!("Error"),
      },
      false => match channel {
        0 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr0/brightness")
            .unwrap();
          file.write_all(b"0").expect("Failed to write");
        }
        1 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr1/brightness")
            .unwrap();
          file.write_all(b"0").expect("Failed to write");
        }
        2 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr2/brightness")
            .unwrap();
          file.write_all(b"0").expect("Failed to write");
        }
        3 => {
          let mut file: File = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .open("/sys/class/leds/beaglebone:green:usr3/brightness")
            .unwrap();
          file.write_all(b"0").expect("Failed to write");
        }
        _ => println!("Error"),
      },
    },

    SamControlMessage::ActuateValve { channel, powered } => {
      if (channel < 1 || channel > 6) {
        fail!("Invalid valve number")
      }

      let info = VALVE_PINS.get(channel).unwrap();
      let pin = GPIO_CONTROLLERS[info.controller].get_pin(info.pin);
      pin.mode(Output);

      match powered {
        true => {
          pin.digital_write(High);
        },

        false => {
          pin.digital_write(Low);
        }
      }
    },
  }
}
