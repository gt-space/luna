use common::comm::{
  flight::DataMessage,
  sam::{DataPoint, SamControlMessage}, ValveAction,
};
use hostname::get;
use jeflog::{pass, warn};
use std::{
  borrow::Cow,
  net::{SocketAddr, ToSocketAddrs, UdpSocket},
  time::{Duration, Instant},
};

use crate::{command::{execute, check_prvnt_abort}, state::{AbortInfo, ConnectData}, SamVersion, FC_ADDR};

// const FC_ADDR: &str = "server-01";
// const FC_ADDR: &str = "flight";
const COMMAND_PORT: u16 = 8378;
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(1000);

pub fn get_hostname() -> String {
  loop {
    // hostname::get()
    match get() {
      Ok(hostname) => break hostname.to_string_lossy().to_string(),
      Err(e) => {
        warn!("Error getting hostname: {}", e);
        continue;
      }
    }
  }
}

pub fn get_version() -> SamVersion {
  let name = get_hostname();

  let version: SamVersion = if name == "sam-01"
    || name == "sam-02"
    || name == "sam-03"
    || name == "sam-04"
    || name == "sam-05"
  {
    SamVersion::Rev3
  } else if name == "sam-11"
    || name == "sam-12"
    || name == "sam-13"
    || name == "sam-14"
  {
    SamVersion::Rev4Ground
  } else if name == "sam-21" || name == "sam-22" || name == "sam-23" {
    SamVersion::Rev4Flight
  } else {
    panic!("We got an imposter among us!")
  };

  version
}

// make sure you keep track of these UdpSockets, and pass them into the correct
// functions. Left is data, right is command.
pub fn establish_flight_computer_connection(data: &mut ConnectData) -> (UdpSocket, UdpSocket, SocketAddr, String, AbortInfo) {
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
        },
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

  // Create the handshake message
  // It lets the flight computer know know what board type and number this
  // device is.
  let hostname: String = get_hostname();
  let identity = DataMessage::Identity(hostname.clone());

  // Allocate memory to store the handshake message in
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
        warn!("Could not allocate memory for handshake: {}", e)
      },
    };
  };

  loop {
    // Check time when we safed valves to see if we should open PRVNT, if it is past the timer + we haven't opened PRVNT yet then open it
    if data.abort_info.prvnt_channel != 0 && data.abort_info.aborted && !data.abort_info.opened_prvnt {
      check_prvnt_abort(data);
    }

    // Try to send the handshake to the flight computer.
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

    // Check if the FC has responded with its own handshake message. If so,
    // convert it from raw bytes to a DataMessage enum
    let result = match data_socket.recv_from(&mut buf) {
      Ok((size, _)) =>
      // disregard the SocketAddr
      {
        match postcard::from_bytes::<DataMessage>(&buf[..size]) {
          Ok(message) => message,
          // failed to deserialize message, try again!
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
        data.abort_info.last_heard_from_fc = Instant::now();
        data.abort_info.aborted = false;
        data.abort_info.opened_prvnt = false;
        pass!("Connection established with FC ({id})");
        return (data_socket, command_socket, fc_address, hostname, data.abort_info);
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
  hostname: String,
  datapoints: Vec<DataPoint>,
) {
  // create a buffer to store the data to send in
  let mut buffer: [u8; 2048] = [0; 2048];

  // get the data and store it in the buffer
  let data = DataMessage::Sam(hostname, Cow::Owned(datapoints));
  let seralized = match postcard::to_slice(&data, &mut buffer) {
    Ok(slice) => {
      //pass!("Sliced data."); // don't need to see this everytime
      slice
    }
    Err(e) => {
      warn!("Could not serialize buffer ({e}), continuing...");
      return;
    }
  };

  // send the data to the FC
  match socket.send_to(seralized, address) {
    Ok(_size) => {
      //pass!("Successfully sent {size} bytes of data...");
    }
    Err(e) => {
      warn!("Could not send data ({e}), continuing...");
    }
  };
}

// Make sure you keep track of the timer that is returned, and pass it in on the
// next loop
pub fn check_heartbeat(data_socket: &UdpSocket, command_socket: &UdpSocket, timer: Instant) -> (Instant, bool) {
  // check if we have exceeded the heartbeat timer
  let delta = Instant::now() - timer;
  if delta > HEARTBEAT_TIME_LIMIT {
    return (timer, true);
  }

  // create a location to store the heartbeat recieved from the FC
  let mut data_buffer: [u8; 256] = [0; 256];
  // create a location to store a command received from the FC
  let mut command_buffer: [u8; 256] = [0; 256];

  // check to see if a Flight Heartbeat was received
  match data_socket.recv_from(&mut data_buffer) {
    Ok((size, _)) => {
      match postcard::from_bytes::<DataMessage>(&data_buffer[..size]) {
        Ok(message) => {
          match message {
            DataMessage::FlightHeartbeat => return (Instant::now(), false),
            _ => warn!("Message was not a Flight Heartbeat")
          }
        },

        Err(e) => warn!("Could not deserialize data from FC ({e}), continuing...")
      }
    },
    
    Err(_e) => {} // did not receive data from FC
  }

  // did not receive anything in data socket, so now checking for command
  // have to peek and not recv so that the command remains in the buffer
  // for when it must be executed later on
  match command_socket.peek_from(&mut command_buffer) {
    Ok((size, _)) => {
      match postcard::from_bytes::<SamControlMessage>(&command_buffer[..size]) {
        Ok(_) => return (Instant::now(), false), // don't care about contents

        Err(e) => warn!("Could not deserialize command from FC ({e}), continuing...")
      }
    },

    Err(_e) => {} // did not receive command from FC
  }

  // At this point a Flight Heartbeat nor a SamControlMessage has been received.
  // We are still under the timeout limit to abort so return the Instant and
  // false to indiciate that we should NOT abort
  (timer, false)
}

pub fn check_and_execute(command_socket: &UdpSocket, prvnt_channel: &mut u32, abort_valve_states: &mut Vec<ValveAction>) {
  // where to store the commands recieved from the FC
  let mut buf: [u8; 1024] = [0; 1024];

  // should I break or return on Err?
  // break would leave the loop and there would be nothing after the loop
  // so it would immediately return
  // returning would be just that
  
  /* If there are always new commands coming in, don't want to infinitely
  stay in this function because the sequences use data feedback to make
  those decisions. Max of 10 commands to be executed is arbitrary
  */

  /* Well since the command_socket is nonblocking then it should simply
  check if there is new data in the buffer, not wait for it to be received. So
  these recv_from calls should be very fast
   */
  for _ in 0..10 {
    // check if we got a command from the FC
    let size = match command_socket.recv_from(&mut buf) {
      Ok((size, _)) => size,
      Err(_) => break, // no data in buffer
    };

    let command = match postcard::from_bytes::<SamControlMessage>(&buf[..size]) {
      Ok(command) => command,
      Err(e) => {
        warn!("Command was recieved but could not be deserialized ({e}).");
        break;
      }
    };

    pass!("Executing command...");
    // execute the command
    execute(command, prvnt_channel, abort_valve_states);
  }
}
