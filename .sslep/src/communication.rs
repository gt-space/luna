use std::{
    borrow::Cow,
    io::ErrorKind,
    net::{SocketAddr, ToSocketAddrs, UdpSocket},
    thread::sleep,
    time::{Duration, Instant},
};

use common::comm::{
  flight::DataMessage,
  sslep::{SslepDataPoint, SslepControlMessage},
};

use jeflog::{pass, warn};

use crate::{
    state::{AbortInfo, ConnectData},
    CACHED_FC_ADDRESS, FC_ADDR,
};

const COMMAND_PORT: u16 = 8378;
const DATA_PORT: u16 = 4573;

pub const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(1000);
const RETRY_DELAY: Duration = Duration::from_millis(100);

fn create_udp_socket(port: u16) -> UdpSocket {
    loop {
        match UdpSocket::bind(("0.0.0.0", port)) {
            Ok(socket) => {
                match socket.set_nonblocking(true) {
                    Ok(()) => return socket,

                    Err(e) => {
                        eprintln!(
                            "Failed to configure socket {port}: {e}"
                        );
                    }
                }
            }

            Err(e) => {
                eprintln!(
                    "Failed to bind socket {port}: {e}"
                );
            }
        }

        sleep(RETRY_DELAY);
    }
}


pub fn establish_flight_computer_connection(
    data: &mut ConnectData,
) -> (UdpSocket, UdpSocket, SocketAddr, String, AbortInfo) {

    let mut buf = [0u8; 1024];

    // 1. Create the data socket.
    let data_socket = loop {
        match UdpSocket::bind(("0.0.0.0", DATA_PORT)) {
            Ok(socket) => {
                if socket.set_nonblocking(true).is_ok() {
                    break socket;
                }
            }
            Err(e) => {
                eprintln!("Failed to bind data socket: {e}");
            }
        }

        sleep(RETRY_DELAY);
    };

    // 2. Create the command socket.
    let command_socket = loop {
        match UdpSocket::bind(("0.0.0.0", COMMAND_PORT)) {
            Ok(socket) => {
                if socket.set_nonblocking(true).is_ok() {
                    break socket;
                }
            }
            Err(e) => {
                eprintln!("Failed to bind command socket: {e}");
            }
        }

        sleep(RETRY_DELAY);
    };

    // 3. Locate the Flight Computer.
    // Reuse its cached address if available.
    let fc_address = loop {
        if let Some(cached_address) = CACHED_FC_ADDRESS.get() {
            break *cached_address;
        }

        let fc_hostname = FC_ADDR
            .get()
            .expect("FC_ADDR has not been initialized");

        // FC_ADDR stores the hostname without ".local".
        let address = format!("{fc_hostname}.local:{DATA_PORT}")
            .to_socket_addrs()
            .ok()
            .and_then(|mut addrs| {
                addrs.find(|addr| addr.is_ipv4())
            });

        if let Some(address) = address {
            // Cache the resolved address for reconnections.
            let _ = CACHED_FC_ADDRESS.set(address);
            break address;
        }

        eprintln!("Could not resolve Flight Computer");
        sleep(RETRY_DELAY);
    };

    println!("Flight Computer located at {fc_address}");

    // 4. Get the SSLEP board's hostname.
    let hostname = loop {
        match hostname::get() {
            Ok(name) => {
                break name.to_string_lossy().to_string();
            }
            Err(e) => {
                eprintln!("Could not get SSLEP hostname: {e}");
                sleep(RETRY_DELAY);
            }
        }
    };

    // 5. Create the identity handshake.
    let identity = DataMessage::Identity(hostname.clone());

    let packet = postcard::to_allocvec(&identity)
        .expect("Could not serialize identity");

    // 6. Send handshake until the FC responds.
    loop {
        if let Err(e) = data_socket.send_to(&packet, fc_address) {
            eprintln!("Failed to send identity: {e}");
            sleep(RETRY_DELAY);
            continue;
        }

        // Process a bounded number of queued responses.
        for _ in 0..16 {
            let (size, sender) =
                match data_socket.recv_from(&mut buf) {
                    Ok(result) => result,

                    Err(e) if e.kind() == ErrorKind::WouldBlock => {
                        break;
                    }

                    Err(e) if e.kind() == ErrorKind::Interrupted => {
                        continue;
                    }

                    Err(e) => {
                        eprintln!("FC receive error: {e}");
                        break;
                    }
                };

            // Ignore other senders.
            if sender.ip() != fc_address.ip() {
                continue;
            }

            // Deserialize the FC's response.
            let message =
                match postcard::from_bytes::<DataMessage>(
                    &buf[..size]
                ) {
                    Ok(message) => message,

                    Err(e) => {
                        eprintln!(
                            "Could not deserialize FC response: {e}"
                        );
                        continue;
                    }
                };

            match message {
                DataMessage::Identity(id) => {
                    println!("Connected to FC: {id}");

                    // 7. Reset connection/heartbeat state.
                    data.abort_info.last_heard_from_fc =
                        Instant::now();

                    data.abort_info.received_abort = false;
                    data.abort_info.time_aborted = None;

                    // 8. Return the five values expected by state.rs.
                    return (
                        data_socket,
                        command_socket,
                        fc_address,
                        hostname,
                        data.abort_info,
                    );
                }

                DataMessage::FlightHeartbeat => {
                    // Wait for FC identity before connecting.
                }

                _ => {
                    eprintln!(
                        "Unexpected message during FC handshake"
                    );
                }
            }
        }

        sleep(RETRY_DELAY);
    }
}

pub fn send_data(
  socket: &UdpSocket,
  address: &SocketAddr,
  hostname: String,
  datapoints: Vec<SslepDataPoint>,
) {
  let mut buffer: [u8; 2048] = [0; 2048];
  let data = DataMessage::Sslep(hostname, Cow::Owned(datapoints));
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

pub fn check_heartbeat(
    my_data_socket: &UdpSocket,
    my_commmand_socket: &UdpSocket,
    last_heard_from_fc: Instant,
) -> (Instant, bool) {
  let delta = Instant::now() - last_heard_from_fc;
    if delta > HEARTBEAT_TIME_LIMIT {
      return (last_heard_from_fc, true);
    }

    let mut data_buffer: [u8; 256] = [0; 256];

    let mut command_buffer: [u8; 256] = [0; 256];

    match data_socket.recv_from(&mut data_buffer) {
        Ok((size, _)) => match postcard::from_bytes::<DataMessage>(&data_buffer[..size]) {
          Ok(message) => match message {
            DataMessage::FlightHeartbeat => return (Instant::now(), false),
            _ => warn!("Message was not a Flight Heartbeat"),
          },

          Err(e) => {
            warn!("Could not deserialize data from FC ({e}), continuing...")
          }
        },

        Err(_e) => {} // did not receive data from FC
    }
      // did not receive anything in data socket, so now checking for command
      // have to peek and not recv so that the command remains in the buffer
      // for when it must be executed later on
      match command_socket.peek_from(&mut command_buffer) {
          Ok((size, _)) => {
              match postcard::from_bytes::<SslepControlMessage>(&command_buffer[..size]) {
                  Ok(_) => return (Instant::now(), false), // don't care about contents

                  Err(e) => {
                      warn!("Could not deserialize command from FC ({e}), continuing...")
                  }
              }
          }

          Err(_e) => {} // did not receive command from FC
      }

      // At this point a Flight Heartbeat nor a SslepControlMessage has been
      // received. We are still under the timeout limit to abort so return the
      // Instant and false to indiciate that we should NOT abort
      (last_heard_from_fc, false)
}