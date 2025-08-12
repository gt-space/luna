use std::{net::SocketAddr, time::Duration};
use std::borrow::Cow;
use common::comm::{flight::DataMessage, 
  sam::{ChannelType, DataPoint, SamControlMessage}
};
use jeflog::{pass, warn};
use crate::communication::establish_flight_computer_connection;


pub fn simulate_sam(hostname: String, flight_addr: SocketAddr) {
  println!("hey im starting the sam sim!");
  let (data_socket, command_socket) = 
    establish_flight_computer_connection(hostname.clone(), &flight_addr);

  let mut data_buffer: [u8; 1024] = [0; 1024];
  let data_points = vec![
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::CurrentLoop,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::RailVoltage,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::RailCurrent,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::Rtd,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::DifferentialSignal,
    },
    DataPoint {
      value: 0.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::Tc,
    },
    DataPoint {
      value: 23.0,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::ValveVoltage,
    },
    DataPoint {
      value: 0.00,
      timestamp: 0.0,
      channel: 1,
      channel_type: ChannelType::ValveCurrent,
    },
  ];

  let message = DataMessage::Sam(hostname.clone(), Cow::Borrowed(&data_points));
  let serialized_message = postcard::to_slice(&message, &mut data_buffer).unwrap();

  let mut command_buffer: [u8; 1024] = [0; 1024];

  loop {
    // send dummy data out
    match data_socket.send(serialized_message) {
      Ok(_) => {},
      Err(_) => warn!("Failed to send dummy data to flight")
    }

    // check for commands
    for _ in 0..10 {
      // check if we got a command from the FC
      let size = match command_socket.recv(&mut command_buffer) {
        Ok(size) => size,
        Err(_) => break, // no data in buffer
      };

      match postcard::from_bytes::<SamControlMessage>(&command_buffer[..size]) {
        Ok(command) => println!("{:?}", command),
        Err(e) => {
          warn!("Command was recieved but could not be deserialized ({e}).");
          break;
        }
      }

      pass!("Executing command...");
    }

    std::thread::sleep(Duration::from_millis(5));
  }

}