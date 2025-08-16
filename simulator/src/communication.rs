use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use common::comm::flight::DataMessage;
use jeflog::{pass, warn};

// make sure you keep track of these UdpSockets, and pass them into the correct
// functions. Left is data, right is command.
pub fn establish_flight_computer_connection(hostname: String, flight_addr: &SocketAddr) -> (UdpSocket, UdpSocket) {
  // area in memory where the flight computer handshake response should be
  // stored
  let mut buf: [u8; 1024] = [0; 1024];

  let (data_socket, command_socket) = generate_data_command_sockets();
  println!("Hey i got my sockets!");
  // data socket only sends data to flight
  data_socket.connect(flight_addr).unwrap();
  // command socket only receives data from flight
  command_socket.connect(flight_addr).unwrap();
  println!("Hey im not a socket whore!");

  // Create the handshake message
  // It lets the flight computer know know what board type and number this
  // device is.
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
    // Try to send the handshake to the flight computer.
    match data_socket.send(&packet) {
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
    let result = match data_socket.recv(&mut buf) {
      Ok(size) =>
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
        pass!("Connection established with FC ({id})");
        return (data_socket, command_socket);
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

fn generate_data_command_sockets() -> (UdpSocket, UdpSocket) {
  for a in 0u8..=255u8 {
    for b in 0u8..=255u8 {
      for c in 0u8..255u8 {
        let data_ip = Ipv4Addr::new(127, a, b, c);
        // 0 should work i guess
        let data_addr = SocketAddrV4::new(data_ip, 0);
        let command_ip = Ipv4Addr::new(127, a, b, c);
        let command_addr = SocketAddrV4::new(command_ip, 8378);
        match (UdpSocket::bind(data_addr), UdpSocket::bind(command_addr)) {
          // d is for data socket, c is for command socket
          (Ok(d), Ok(c)) => {
            match (d.set_nonblocking(true), c.set_nonblocking(true)) {
              (Ok(()), Ok(())) => {
                return (d, c)
              },

              _ => continue
            }
          },

          (Err(_), Err(_)) => {
            println!("Failed on both");
            continue;
          },

          (Err(e), Ok(_)) => {
            println!("Failed on data socket: {}", e);
          },

          (Ok(_), Err(e)) => {
            println!("Failed on command socket: {}", e);
          }

        }
      }
    }
  }

  panic!("Could not find pair of loopback IPv4 addresses")
}