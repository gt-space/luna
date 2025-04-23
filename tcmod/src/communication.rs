use common::comm::{
  tcmod::{DataPoint},
  flight::DataMessage,
};
use jeflog::{fail, pass, warn};
use std::{
  borrow::Cow,
  net::{SocketAddr, ToSocketAddrs, UdpSocket},
  time::{Duration, Instant},
};

use crate::{FC_ADDR, command::execute};

//const FC_ADDR: &str = "flight";
const TCMOD_ID: &str = "tcmod-01";
const COMMAND_PORT: u16 = 8378; //what is this ???
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(1000);

// make sure you keep track of these UdpSockets, and pass them into the correct
// functions. Left is data, right is command.
pub fn establish_flight_computer_connection(
) -> (UdpSocket, UdpSocket, SocketAddr) {
  // area in memory where the flight computer handshake response should be
  // stored
  let mut buf: [u8; 1024] = [0; 1024];

  // create socket where all data is sent from
  // make non blocking so that loop to establish FC connection runs many times
  let data_socket = {
    let socket = loop {
      match UdpSocket::bind(("0.0.0.0", 4573)) {
        Ok(x) => break x,
        Err(e) => {
          warn!("Failed to bind data socket: {}", e);
        }
      };
    };

    // should I retry the bind on error from set_nonblocking?
    loop {
      match socket.set_nonblocking(true) {
        Ok(()) => break socket,
        Err(e) => {
          warn!("Failed to set data socket to nonblocking: {}", e);
        }
      };
    }
  };

  // create the socket where all the commands are recieved from
  // make nonblocking so it does not wait for commands to be received
  let command_socket = {
    let socket = loop {
      match UdpSocket::bind(("0.0.0.0", COMMAND_PORT)) {
        Ok(x) => break x,
        Err(e) => {
          warn!("Failed to bind command socket: {}", e);
        }
      };
    };

    loop {
      match socket.set_nonblocking(true) {
        Ok(()) => break socket,
        Err(e) => {
          warn!("Failed to set command socket to nonblocking: {}", e);
        }
      };
    }
  };

  // look for the flight computer based on it's dynamic IP
  // will caches ever result in an incorrect IP address?
  let fc_address = loop {
    let address = format!("{}.local:4573", FC_ADDR.get().unwrap())
      .to_socket_addrs()
      .ok()
      .and_then(|mut addrs| addrs.find(|addr| addr.is_ipv4()));

    if let Some(x) = address {
      break x;
    }
  };

  pass!(
    "Target \x1b[1m{}\x1b[0m located at \x1b[1m{}\x1b[0m.",
    FC_ADDR.get().unwrap(),
    fc_address.ip()
  );

  // Create the TCMOD handshake message
  // It lets the flight computer know know what board type and number this
  // device is.
  let identity = DataMessage::Identity(TCMOD_ID.to_string());

  // Allocate memory to store the TCMOD handshake message in that is sent to FC
  let packet = loop {
    match postcard::to_allocvec(&identity) {
      Ok(bytes) => break bytes,
      /* Possible errors for serialization are WontImplement, NotYetImplemented,
      SerializeBufferFull, SerializeSeqLengthUnknown, and SerdeSerCustom. The
      DataMessage type derives the Serialize trait and does not have a custom
      Serialize functionality. The string provided under the hood is very small
      here. The length of the buffer is known. Thus this should immediately work
       */
      Err(e) => {
        warn!("Could not allocate memory for handshake: {}", e);
      },
    };
  };

  loop {
    // Try to send the TCMOD handshake to the flight computer.
    // If this panics, it means that the TCMOD couldn't send the handshake at all.
    match data_socket.send_to(&packet, fc_address) {
      Ok(_) => {}
      /* Although UDP is a connection-less protocol, the OS still requires
      a valid path for the data to be sent, otherwise the network
      is 'unreachable'. So a std::io::ErrorKind::NetworkUnreachable is returned.
      Until the ethernet connection is present, this will result in an Error.
       */
      Err(e) => {
        warn!("Unable to send packet into the ether: {}", e);
        continue;
      }
    }

    //println!("Sent identity of size {size}");

    // Check if the FC has responded with its own handshake message. If so,
    // convert it from raw bytes to a DataMessage enum
    let result = match data_socket.recv_from(&mut buf) {
      Ok((size, _)) =>
      // disregard the SocketAddr
      {
        match postcard::from_bytes::<DataMessage>(&buf[..size]) {
          Ok(message) => message,
          // failed to deserialize message, try again!
          // todo: match on Error variants to pinpoint issue
          Err(e) => {
            warn!("Failed to deserialize message from FC: {}", e);
            continue
          },
        }
      }
      Err(e) => {
        // failed to receive data from FC, try again!
        warn!("Did not receive data from FC: {}", e);
        continue;
      }
    };

    match result {
      // If the Identity message was recieved correctly.
      DataMessage::Identity(id) => {
        pass!("Connection established with FC ({id})");

        return (data_socket, command_socket, fc_address);
      }
      DataMessage::FlightHeartbeat => {
        warn!("Recieved heartbeat from FC despite no identity.");
        continue;
      }
      _ => {
        warn!("Recieved nonsenical data from FC.");
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
  let data = DataMessage::Tcmod(TCMOD_ID.to_string(), Cow::Owned(datapoint));
  let serialized = match postcard::to_slice(&data, &mut buffer) {
    Ok(slice) => slice,
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