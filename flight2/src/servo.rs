use std::{fmt, io::{self, Read, Write}, net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket}, time::Duration, thread};
use common::comm::{Computer, FlightControlMessage, VehicleState, VehicleStateCompressionSchema};
use postcard::experimental::max_size::MaxSize;
use socket2::{Socket, Domain, Type, Protocol, TcpKeepalive};

use crate::SERVO_DATA_PORT;

pub const servo_keep_alive_delay: Duration = Duration::from_secs(1);
/// DSCP marker applied to radio telemetry packets so Servo can distinguish
/// them from the uncompressed umbilical telemetry stream on the same UDP port.
pub const RADIO_TELEMETRY_DSCP: u8 = 0x2e;

type Result<T> = std::result::Result<T, ServoError>;

#[derive(Debug)]
pub(crate) enum ServoError {
  ServoDisconnected,
  TransportFailed(io::Error),
  DeserializationFailed(postcard::Error),
  CompressionFailed(&'static str),
  BufferTooSmall,
}

impl fmt::Display for ServoError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::ServoDisconnected => write!(f, "Servo can't be reached or has disconnected."),
      Self::DeserializationFailed(e) => write!(f, "postcard encountered an error during message deserialization: {e}"),
      Self::TransportFailed(e) => write!(f, "The Servo transport layer raised an error: {e}"),
      Self::CompressionFailed(message) => write!(f, "VehicleState compression failed: {message}"),
      Self::BufferTooSmall => write!(f, "The radio telemetry buffer was too small for the compressed payload."),
    }
  }
}

#[derive(Default)]
pub(crate) struct RadioTelemetryEncoder {
  schema: Option<VehicleStateCompressionSchema>,
  valve_keys: Vec<String>,
  sensor_keys: Vec<String>,
}

impl RadioTelemetryEncoder {
  pub(crate) fn encode<'a>(
    &'a mut self,
    state: &VehicleState,
    buffer: &'a mut [u8],
  ) -> Result<&'a [u8]> {
    self.refresh_schema_if_needed(state)?;
    let schema = self
      .schema
      .as_ref()
      .ok_or(ServoError::CompressionFailed("missing radio schema"))?;
    let size = state.compress_with_schema(buffer, schema).map_err(|error| match error {
      common::comm::VehicleStateCompressionError::BufferTooSmall => {
        ServoError::BufferTooSmall
      }
      common::comm::VehicleStateCompressionError::TooManyValves => {
        ServoError::CompressionFailed("too many valves for radio telemetry")
      }
      common::comm::VehicleStateCompressionError::TooManySensors => {
        ServoError::CompressionFailed("too many sensors for radio telemetry")
      }
      common::comm::VehicleStateCompressionError::ValveCountMismatch => {
        ServoError::CompressionFailed("cached valve schema no longer matches the live state")
      }
      common::comm::VehicleStateCompressionError::SensorCountMismatch => {
        ServoError::CompressionFailed("cached sensor schema no longer matches the live state")
      }
      _ => ServoError::CompressionFailed("unexpected compression error"),
    })?;
    Ok(&buffer[..size])
  }

  fn refresh_schema_if_needed(&mut self, state: &VehicleState) -> Result<()> {
    let mut valve_keys: Vec<_> = state.valve_states.keys().cloned().collect();
    valve_keys.sort_unstable();

    let mut sensor_keys: Vec<_> = state
      .sensor_readings
      .keys()
      .filter(|sensor_name| {
        !sensor_name
          .strip_suffix("_V")
          .or_else(|| sensor_name.strip_suffix("_I"))
          .is_some_and(|valve_name| state.valve_states.contains_key(valve_name))
      })
      .cloned()
      .collect();
    sensor_keys.sort_unstable();

    if self.schema.is_none()
      || self.valve_keys != valve_keys
      || self.sensor_keys != sensor_keys
    {
      self.schema = Some(
        VehicleStateCompressionSchema::from_state(state)
          .map_err(|_| ServoError::CompressionFailed("failed to build radio schema"))?,
      );
      self.valve_keys = valve_keys;
      self.sensor_keys = sensor_keys;
    }

    Ok(())
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

pub(crate) fn make_radio_socket() -> Result<UdpSocket> {
  let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))
    .map_err(ServoError::TransportFailed)?;
  socket
    .set_tos(u32::from(RADIO_TELEMETRY_DSCP) << 2)
    .map_err(ServoError::TransportFailed)?;
  socket
    .bind(&SocketAddr::from(([0, 0, 0, 0], 0)).into())
    .map_err(ServoError::TransportFailed)?;
  Ok(socket.into())
}

// sends uncompressed umbilical telemetry to servo over the existing UDP path.
pub(crate) fn push_umbilical(
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

// Sends compressed radio telemetry to servo through the TEL-marked UDP socket.
pub(crate) fn push_radio(
  socket: &UdpSocket,
  servo_socket: SocketAddr,
  state: &VehicleState,
  encoder: &mut RadioTelemetryEncoder,
  buffer: &mut [u8],
) -> Result<usize> {
  let message = encoder.encode(state, buffer)?;
  socket
    .send_to(message, (servo_socket.ip(), SERVO_DATA_PORT))
    .map_err(ServoError::TransportFailed)
}
