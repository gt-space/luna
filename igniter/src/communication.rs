use crate::{
  state::AbortInfo, 
  command::execute,
  IgniterId, 
  CACHED_FC_ADDRESS, 
  FC_ADDR
};
use common::comm::{
  igniter::{Command, DataPoint},
  flight::DataMessage
};
use hostname::get;
use jeflog::{fail, pass, warn};
use std::{
  borrow::Cow,
  net::{SocketAddr, ToSocketAddrs, UdpSocket},
  time::{Duration, Instant},
};

/// Port for the command socket, where commands that are sent from the 
/// flight computer to the igniter are received
const COMMAND_PORT: u16 = 8378;

/// The amount of time to wait for a heartbeat from the flight computer before 
/// considering the igniter disconnected from the flight computer
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(1000);

/// Information about the connection to the flight computer
pub struct FcConnectionInfo {
  pub data_socket: UdpSocket,
  pub command_socket: UdpSocket,
  pub fc_address: SocketAddr,
  pub device_id: IgniterId,
}

/// Get the hostname of the device we are running on
pub fn get_hostname() -> String {
  loop {
    match get() {
      Ok(hostname) => break hostname.to_string_lossy().to_string(),
      Err(e) => {
        warn!("Error getting hostname: {}", e);
        continue;
      }
    }
  }
}

/// Get the ID of the igniter we are running on
pub fn get_igniter_id() -> IgniterId {
  let name = get_hostname();

  let id: IgniterId = if name == "igniter-01" {
    IgniterId::Igniter1
  } else if name == "igniter-02" {
    IgniterId::Igniter2
  } else {
    panic!("We got an imposter among us!")
  };

  id
}

// make sure you keep track of these UdpSockets, and pass them into the correct
// functions. Left is data, right is command.
pub fn establish_flight_computer_connection(abort_info: &mut AbortInfo) 
  -> FcConnectionInfo {
  // area in memory where the flight computer handshake response will be stored
  let mut buf: [u8; 1024] = [0; 1024];

  // create the socket where data is sent to the flight computer
  let data_socket = {
    let socket = loop {
      match UdpSocket::bind(("0.0.0.0", 4573)) {
        Ok(x) => break x,
        Err(e) => {
          warn!("Failed to bind data socket: {}", e);
        }
      };
    };

    // set the socket to nonblocking so that the loop to establish 
    // FC connection runs many times
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
  let command_socket = {
    let socket = loop {
      match UdpSocket::bind(("0.0.0.0", COMMAND_PORT)) {
        Ok(x) => break x,
        Err(e) => {
          warn!("Failed to bind command socket: {}", e);
        }
      };
    };

    // set the socket to nonblocking so that we don't block the main thread
    // when no commands from FC are available to process
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
  let fc_address = loop {
    if let Some(cached_address) = CACHED_FC_ADDRESS.get() {
      break *cached_address;
    }

    let address = format!("{}.local:4573", FC_ADDR.get().unwrap())
      .to_socket_addrs()
      .ok()
      .and_then(|mut addrs| addrs.find(|addr| addr.is_ipv4()));

    if let Some(x) = address {
      CACHED_FC_ADDRESS.set(x);
      break x;
    } 
  };  

  // Create the Igniter handshake message, that lets FC know what board type
  // and number this device is.
  let hostname: String = get_hostname();
  let identity = DataMessage::Identity(hostname.clone());

  // Allocate memory to store the Igniter handshake message in that is sent to FC
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
    // Try to send the Igniter handshake to the flight computer.
    // If this panics, it means that the BMS couldn't send the handshake at all.
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

    // Check if the FC has responded with its own handshake message
    let result = match data_socket.recv_from(&mut buf) {
      Ok((size, _)) => {
        match postcard::from_bytes::<DataMessage>(&buf[..size]) {
          Ok(message) => message,
          Err(e) => {
            warn!("Failed to deserialize message from FC: {}", e);
            continue
          },
        }
      }
      Err(e) => {
        warn!("Did not receive data from FC: {}", e);
        continue;
      }
    };

    match result {
      DataMessage::Identity(id) => {
        pass!("Connection established with FC ({id})");
        // reset abort info
        abort_info.last_heard_from_fc = Instant::now();
        abort_info.received_abort = false;
        abort_info.time_aborted = None;
        return FcConnectionInfo {
          data_socket,
          command_socket,
          fc_address,
          device_id: get_igniter_id(),
        };
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

/// Send `datapoint` from `socket` to the flight computer at `address`
pub fn send_data(
  socket: &UdpSocket,
  address: &SocketAddr,
  datapoint: DataPoint,
) {
  // create a buffer to store the data to send in
  let mut buffer: [u8; 2048] = [0; 2048];

  // get the data and store it in the buffer
  let data = DataMessage::Igniter(get_hostname(), Cow::Owned(datapoint));
  let serialized = match postcard::to_slice(&data, &mut buffer) {
    Ok(slice) => slice,
    Err(e) => {
      warn!("Could not serialize buffer ({e}), not sending data...");
      return;
    }
  };

  if let Some(e) = socket.send_to(serialized, address).err() {
    warn!("Could not send data ({e}), not sending data...");
  }
}

/// Check if FC has sent a heartbeat to us within the heartbeat time limit
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

/// Check `command_socket` for a command from FC and execute it
pub fn check_and_execute(command_socket: &UdpSocket) {
  // where to store the command recieved from the FC
  let mut buf: [u8; 1024] = [0; 1024];

  // check if we got a command from FC
  let size = match command_socket.recv_from(&mut buf) {
    Ok((size, _)) => size,
    Err(_) => return,
  };

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
