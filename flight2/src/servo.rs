use std::{fmt, io::{self, Read, Write}, net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket}, time::Duration, thread};
use common::comm::{Computer, FlightControlMessage, VehicleState};
use postcard::experimental::max_size::MaxSize;
use socket2::{Socket, Domain, Type, Protocol, TcpKeepalive};

use crate::SERVO_DATA_PORT;

pub const servo_keep_alive_delay: Duration = Duration::from_secs(1);

type Result<T> = std::result::Result<T, ServoError>;

#[derive(Debug)]
pub(crate) enum ServoError {
  ServoDisconnected,
  TransportFailed(io::Error),
  DeserializationFailed(postcard::Error),
}

impl fmt::Display for ServoError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::ServoDisconnected => write!(f, "Servo can't be reached or has disconnected."),
      Self::DeserializationFailed(e) => write!(f, "postcard encountered an error during message deserialization: {e}"),
      Self::TransportFailed(e) => write!(f, "The Servo transport layer raised an error: {e}"),
    }
  }
}

pub(crate) fn establish(servo_addresses: &[impl ToSocketAddrs], prev_connected_servo_addr: Option<&SocketAddr>, chances: u8, timeout: Duration) -> Result<(TcpStream, SocketAddr)> {
  // buffer containing the serialized identity message to be sent to the control server
  let mut identity = [0; Computer::POSTCARD_MAX_SIZE];

  if let Err(error) = postcard::to_slice(&Computer::Flight, &mut identity) {
    eprintln!("Failed to serialize Computer: {error}");
    return Err(ServoError::DeserializationFailed(error));
  }

  let mut prev_addr_exists = false;
  if let Some(a) = prev_connected_servo_addr {
    prev_addr_exists = true;
  }

  let mut fatal_error = io::ErrorKind::ConnectionRefused.into();
  if prev_addr_exists {
    let addr = prev_connected_servo_addr.unwrap();
    println!("Attempting connection with servo at {addr:?}...");
  
        match TcpStream::connect_timeout(addr, timeout) {
          Ok(mut s) => {
            let socket = Socket::from(s);
            socket.set_keepalive(true).map_err(|e| return ServoError::TransportFailed(e))?;
            let keep_alive: TcpKeepalive = TcpKeepalive::new()
              .with_time(servo_keep_alive_delay)
              .with_interval(Duration::from_secs(1))
              .with_retries(1);

            socket.set_tcp_keepalive(&keep_alive).map_err(|e| return ServoError::TransportFailed(e))?;
            let mut stream: std::net::TcpStream = socket.into();
            stream.set_nodelay(true).map_err(|e| ServoError::TransportFailed(e))?;
            stream.set_nonblocking(true).map_err(|e| ServoError::TransportFailed(e))?;

            if let Err(e) = stream.write_all(&identity) {
              return Err(ServoError::TransportFailed(e));
            } else {
              return Ok((stream, *addr));
            }
          },
          Err(e) => fatal_error = e,
        };
  } else {
    let resolved_addresses: Vec<SocketAddr> = servo_addresses.iter().filter_map(|a| a.to_socket_addrs().ok()).flatten().collect();
    for i in 1..=chances {
      for addr in &resolved_addresses {
        if !prev_addr_exists || prev_connected_servo_addr.map_or(false, |prev| addr == prev) {
          println!("[{i}]: Attempting connection with servo at {addr:?}...");
    
          match TcpStream::connect_timeout(addr, timeout) {
            Ok(mut s) => {
              //s.set_nodelay(true).map_err(|e| ServoError::TransportFailed(e))?;
              //s.set_nonblocking(true).map_err(|e| ServoError::TransportFailed(e))?;

              let socket = Socket::from(s);
              socket.set_keepalive(true).map_err(|e| return ServoError::TransportFailed(e))?;
              let keep_alive: TcpKeepalive = TcpKeepalive::new()
                .with_time(servo_keep_alive_delay)
                .with_interval(Duration::from_secs(1))
                .with_retries(1);

              socket.set_tcp_keepalive(&keep_alive).map_err(|e| return ServoError::TransportFailed(e))?;
              let mut stream: std::net::TcpStream = socket.into();
              stream.set_nodelay(true).map_err(|e| ServoError::TransportFailed(e))?;
              stream.set_nonblocking(true).map_err(|e| ServoError::TransportFailed(e))?;

              if let Err(e) = stream.write_all(&identity) {
                return Err(ServoError::TransportFailed(e));
              } else {
                return Ok((stream, *addr));
              }
            },
            Err(e) => fatal_error = e,
          };
        }
      }
    };
  }

  Err(ServoError::TransportFailed(fatal_error))
}

// "pull" new information from servo
pub(crate) fn pull(servo_stream: &mut TcpStream) -> Result<Option<FlightControlMessage>> {
  let mut buffer = vec![0; u16::MAX as usize + 2];
  let mut index: usize = 0;

  while index < 2 {
    index += match servo_stream.read(&mut buffer[index..]) {
      Ok(s) if s == 0 => return Err(ServoError::ServoDisconnected),
      Ok(s) => s,
      Err(ref e) if e.kind() == io::ErrorKind::WouldBlock && index == 0 => return Ok(None),
      Err(e) => return Err(ServoError::TransportFailed(e))
    };
  }
  
  let mut buf: [u8; 2] = [0; 2];
  buf.copy_from_slice(&buffer[0..2]);
  let size = u16::from_be_bytes(buf);

  while index < size as usize + 2 {
    index += match servo_stream.read(&mut buffer[index..]) {
      Ok(s) if s == 0 => return Err(ServoError::ServoDisconnected),
      Ok(s) => s,
      Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => 0,
      Err(e) => return Err(ServoError::TransportFailed(e))
    };
  }

  match postcard::from_bytes::<FlightControlMessage>(&buffer[2..]) {
    Ok(m) => Ok(Some(m)),
    Err(e) => Err(ServoError::DeserializationFailed(e)),
  }
}

// sends new VehicleState to servo. Refactor to use UDP
// Note: File logging is now handled in the GPS worker thread at 200Hz
pub(crate) fn push(
    socket: &UdpSocket, 
    servo_socket: SocketAddr, 
    state: &VehicleState,
) -> Result<usize> {
  
  let message = match postcard::to_allocvec(state) {
    Ok(v) => v,
    Err(e) => return Err(ServoError::DeserializationFailed(e)),
  };

  match socket.send_to(&message, (servo_socket.ip(), SERVO_DATA_PORT)) {
    Ok(s) => Ok(s),
    Err(e) => Err(ServoError::TransportFailed(e)),
  }
}