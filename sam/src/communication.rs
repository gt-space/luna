use std::{borrow::Cow, net::{SocketAddr, ToSocketAddrs, UdpSocket}, time::{Duration, Instant}};
use common::comm::{sam::{DataPoint, SamControlMessage}, flight::DataMessage};
use hostname::get;
use jeflog::{warn, fail, pass};
use std::io::ErrorKind;

use crate::{command::execute, SamVersion};

const FC_ADDR: &str = "server-01";
const COMMAND_PORT: u16 = 8378;
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(250);

pub fn get_hostname() -> String {
  match hostname::get() {
    Ok(hostname) => {
      hostname.to_string_lossy().to_string()
    },

    Err(e) => {
      panic!("Error getting hostname: {}", e);
    }
  }
}

pub fn get_version() -> SamVersion {
  let name = get_hostname();
  
  let version: SamVersion = if name == "sam-01"
    || name == "sam-02"
    || name == "sam-03"
    || name == "sam-04"
    || name == "sam-05" {
      SamVersion::Rev3
    } else if name == "gsam-v4-1"
    || name == "gsam-v4-2"
    || name == "gsam-v4-3"
    || name == "gsam-v4-4" {
      SamVersion::Rev4Ground
    } else if name == "fsam-01"
    || name == "fsam-02"
    || name == "fsam-03" {
      SamVersion::Rev4Flight
    } else {
      panic!("We got an imposter among us!")
    };
  
  version
}

// make sure you keep track of these UdpSockets, and pass them into the correct
// functions. Left is data, right is command.
pub fn establish_flight_computer_connection() -> (UdpSocket, UdpSocket, SocketAddr, String) {
  // area in memory where the flight computer handshake response should be stored
  let mut buf: [u8; 1024] = [0; 1024];

  // create socket where all data is sent from
  // make non blocking so that loop to establish FC connection runs many times
  let data_socket = loop {
    let socket = loop {
      match UdpSocket::bind(("0.0.0.0", 4573)) {
        Ok(x) => break x,
        Err(e) => continue
      }
    };

    // should I retry the bind on error from set_nonblocking?
    loop {
      match socket.set_nonblocking(true) {
        Ok(()) => break,
        Err(e) => continue
      }
    }

    break socket
  };

  // create the socket where all the commands are recieved from
  // make nonblocking so it does not wait for commands ot be received
  let command_socket = loop {
    let socket = loop {
      match UdpSocket::bind(("0.0.0.0", COMMAND_PORT)) {
        Ok(x) => break x,
        Err(e) => continue
      }
    };

    loop {
      match socket.set_nonblocking(true) {
        Ok(()) => break,
        Err(e) => continue
      }
    }

    break socket
  };

  // look for the flight computer based on it's dynamic IP
  // will caches ever result in an incorrect IP address?
  let fc_address = loop {
    let address = format!("{}.local:4573", FC_ADDR)
    .to_socket_addrs()
    .ok()
    .and_then(|mut addrs| addrs.find(|addr| addr.is_ipv4()));

    match address {
      Some(x) => break x,
      None => {}
    }
  };

  pass!(
    "Target \x1b[1m{}\x1b[0m located at \x1b[1m{}\x1b[0m.",
    FC_ADDR,
    fc_address.ip()
  );
  
  // Create the handshake message
  // It lets the flight computer know know what board type and number this device is.
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
      Err(e) => continue
    }
  };

  loop {
    // Try to send the handshake to the flight computer.
    match data_socket.send_to(&packet, fc_address) {
      Ok(_) => {},
      /* Although UDP is a connection-less protocol, the OS still requires
      a valid path for the data to be sent, otherwise the network
      is 'unreachable'. So a std::io::ErrorKind::NetworkUnreachable is returned.
      Until the ethernet connection is present, this will result in an Error.
       */
      Err(e) => {
        warn!("Unable to send packet into the ether :(");
        continue;
      }
    }

    //println!("Sent identity of size {size}");
    
    /* Upon a successfull send is this enough time for a successfull
    reception??
     */

    // Check if the FC has responded with its own handshake message. If so,
    // convert it from raw bytes to a DataMessage enum
    let result = match data_socket.recv_from(&mut buf) {
      Ok((size, _)) => // disregard the SocketAddr
        match postcard::from_bytes::<DataMessage>(&buf[..size]) {
          Ok(message) => message,
          // failed to deserialize message, try again!
          // todo: match on Error variants to pinpoint issue
          Err(e) => continue
        }
      Err(e) => {
        // failed to receive data from FC, try again!
        continue;
      }
    };

    match result {
      // If the Identity message was recieved correctly.
      DataMessage::Identity(id) => {
        pass!("Connection established with FC ({id})");
        
        return (data_socket, command_socket, fc_address, hostname)
      },
      DataMessage::FlightHeartbeat => {
        warn!("Recieved heartbeat from FC despite no identity.");
        continue;
      },
      _ => {
        warn!("Recieved nonsenical data from FC.");
        continue;
      }
    }
  }
}

pub fn send_data(socket: &UdpSocket, address: &SocketAddr, hostname: String, datapoints: Vec<DataPoint>) {
  // create a buffer to store the data to send in
  let mut buffer: [u8; 2048] = [0; 2048];

  // get the data and store it in the buffer
  let data = DataMessage::Sam(hostname, Cow::Owned(datapoints));
  let seralized = match postcard::to_slice(&data, &mut buffer) {
    Ok(slice) => {
      //pass!("Sliced data."); // don't need to see this everytime
      slice
    },
    Err(e) => {
      warn!("Could not serialize buffer ({e}), continuing...");
      return;
    }
  };
  
  // send the data to the FC
  match socket.send_to(seralized, address) {
    Ok(size) => {
      //pass!("Successfully sent {size} bytes of data...");
    },
    Err(e) => {
      warn!("Could not send data ({e}), continuing...");
    }
  };
}

// Make sure you keep track of the timer that is returned, and pass it in on the next loop
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
  let command = match postcard::from_bytes::<SamControlMessage>(&buf[..size]) {
    Ok(command) => command,
    Err(e) => {
      fail!("Command was recieved but could not be deserialized ({e}).");
      return;
    }
  };

  //pass!("Executing command...");
  
  // execute the command
  execute(command);
}