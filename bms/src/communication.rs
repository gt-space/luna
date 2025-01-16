use common::comm::{
  bms::{Command, DataPoint},
  flight::DataMessage,
};
use jeflog::{fail, pass, warn};
use std::{
  borrow::Cow,
  net::{SocketAddr, ToSocketAddrs, UdpSocket},
  time::{Duration, Instant},
};

use crate::command::execute;

const FC_ADDR: &str = "server-01";
const BMS_ID: &str = "bms-01";
const COMMAND_PORT: u16 = 8378;
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(250);

// make sure you keep track of these UdpSockets, and pass them into the correct
// functions. Left is data, right is command.
pub fn establish_flight_computer_connection(
) -> (UdpSocket, UdpSocket, SocketAddr) {
  // area in memory where the flight computer handshake response should be
  // stored
  let mut buf: [u8; 1024] = [0; 1024];

  // create the socket where all the data is send from
  let data_socket =
    UdpSocket::bind(("0.0.0.0", 4573)).expect("Could not open data socket.");

  // (new) infinite loop will never go to next iteration if data_socket is
  // blocking
  data_socket
    .set_nonblocking(true)
    .expect("Could not set data socket to nonblocking");

  // create the socket where all the commands are recieved from
  let command_socket = UdpSocket::bind(("0.0.0.0", COMMAND_PORT))
    .expect("Could not open command socket.");

  // make it so the CPU doesn't wait for messages to be recieved
  command_socket
    .set_nonblocking(true)
    .expect("Could not set command socket to nonblocking");

  // look for the flight computer based on it's dynamic IP
  let address = format!("{}.local:4573", FC_ADDR)
    .to_socket_addrs()
    .ok()
    .and_then(|mut addrs| addrs.find(|addr| addr.is_ipv4()));

  let fc_address =
    address.expect("Flight Computer address could not be found!");
  println!("FC Address: {}", fc_address); // this appears to be same every time?

  pass!(
    "Target \x1b[1m{}\x1b[0m located at \x1b[1m{}\x1b[0m.",
    FC_ADDR,
    fc_address.ip()
  );

  // Create the BMS handshake message
  // It lets the flight computer know know what board type and number this
  // device is.
  let identity = DataMessage::Identity(BMS_ID.to_string());

  // Allocate memory to store the BMS handshake message in
  let packet = postcard::to_allocvec(&identity)
    .expect("Could not create identity message send buffer");

  loop {
    // Try to send the BMS handshake to the flight computer.
    // If this panics, it means that the BMS couldn't send the handshake at all.
    let size = data_socket
      .send_to(&packet, fc_address)
      .expect("Could not send Identity message");

    println!("Sent identity of size {size}");

    // Check if the FC has responded with its own handshake message. If so,
    // convert it from raw bytes to a DataMessage enum
    let result = match data_socket.recv_from(&mut buf) {
      Ok((size, _)) => postcard::from_bytes::<DataMessage>(&buf[..size])
        .expect("Could not deserialize recieved message"),
      Err(e) => {
        println!("Failed to recieve FC heartbeat: {e}. Retrying...");
        continue;
      }
    };

    match result {
      // If the Identity message was recieved correctly.
      DataMessage::Identity(id) => {
        println!("Connection established with FC ({id})");

        // data_socket.set_nonblocking(true)
        //   .expect("Could not set data socket to nonblocking");

        return (data_socket, command_socket, fc_address);
      }
      DataMessage::FlightHeartbeat => {
        println!("Recieved heartbeat from FC despite no identity.");
        continue;
      }
      _ => {
        println!("Recieved nonsenical data from FC.");
        continue;
      }
    }
  }
}

pub fn send_data(
  socket: &UdpSocket,
  address: &SocketAddr,
  datapoint: DataPoint,
) {
  // create a buffer to store the data to send in
  let mut buffer: [u8; 2048] = [0; 2048];

  // get the data and store it in the buffer
  let data = DataMessage::Bms(BMS_ID.to_string(), Cow::Owned(datapoint));
  let serialized = match postcard::to_slice(&data, &mut buffer) {
    Ok(slice) => {
      slice
    }
    Err(e) => {
      warn!("Could not serialize buffer ({e}), continuing...");
      return;
    }
  };

  if let Some(e) = socket.send_to(serialized, address).err() {
    warn!("Could not send data ({e}), continuing...");
  }
}

// Make sure you keep track of the timer that is returned, and pass it in on the
// next loop
pub fn check_heartbeat(socket: &UdpSocket, timer: Instant) -> (Instant, bool) {
  // create a location to store the heartbeat recieved from the FC
  let mut buffer: [u8; 256] = [0; 256];

  // check if we have exceeded the heartbeat timer
  let delta = Instant::now() - timer;
  if delta > HEARTBEAT_TIME_LIMIT {
    return (timer, true);
  }

  // get data from the socket and insert into buffer
  let size = match socket.recv_from(&mut buffer) {
    Ok((size, _)) => size,
    Err(_) => {
      return (timer, false);
    }
  };

  // convert the recieved data into a DataMessage
  let message = match postcard::from_bytes::<DataMessage>(&buffer[..size]) {
    Ok(message) => message,
    Err(e) => {
      warn!("Could not deserialize data from FC ({e}), continuing...");
      return (timer, false);
    }
  };

  match message {
    // if the message was a Heartbeat, reset the timer
    DataMessage::FlightHeartbeat => (Instant::now(), false),
    _ => {
      // if not, keep the timer going
      warn!("Expected Flight Heartbeat was not detected.");
      (timer, false)
    }
  }
}

pub fn check_and_execute(command_socket: &UdpSocket) {
  // where to store the command recieved from the FC
  let mut buf: [u8; 1024] = [0; 1024];

  // check if we got a command from the FC
  let size = match command_socket.recv_from(&mut buf) {
    Ok((size, _)) => size,
    Err(_) => return,
  };

  // Convert the recieved data into a SamControlMessage
  let command = match postcard::from_bytes::<Command>(&buf[..size]) {
    Ok(command) => command,
    Err(e) => {
      fail!("Command was recieved but could not be deserialized ({e}).");
      return;
    }
  };

  pass!("Executing command...");

  // execute the command
  execute(command);
}
