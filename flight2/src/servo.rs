use std::{
    fmt,
    io::{self, Read, Write},
    net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket},
    time::Duration,
};

use common::comm::{
    include_in_radio_telemetry, Computer, FlightControlMessage, SensorType, VehicleState,
    VehicleStateCompaqError, VehicleStateCompressionSchema, VehicleStateSchemaError,
};
use postcard::experimental::max_size::MaxSize;
use socket2::{Domain, Protocol, Socket, TcpKeepalive, Type};

use crate::{device::Mappings, SERVO_DATA_PORT};

pub const SERVO_KEEP_ALIVE_DELAY: Duration = Duration::from_secs(1);
/// DSCP marker applied to radio telemetry packets so Servo can distinguish
/// them from the uncompressed umbilical telemetry stream on the same UDP port.
pub const RADIO_TELEMETRY_DSCP: u8 = 0x2E;

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
            Self::ServoDisconnected => {
                write!(f, "Servo can't be reached or has disconnected.")
            }
            Self::DeserializationFailed(e) => write!(
                f,
                "postcard encountered an error during message deserialization: {e}"
            ),
            Self::TransportFailed(e) => {
                write!(f, "The Servo transport layer raised an error: {e}")
            }
            Self::CompressionFailed(message) => {
                write!(f, "VehicleState compression failed: {message}")
            }
            Self::BufferTooSmall => write!(
                f,
                "The radio telemetry buffer was too small for the compressed payload."
            ),
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
        mappings: &Mappings,
        buffer: &'a mut [u8],
    ) -> Result<&'a [u8]> {
        self.refresh_schema_if_needed(state, mappings)?;
        let schema = self
            .schema
            .as_ref()
            .ok_or(ServoError::CompressionFailed("missing radio schema"))?;
        let size =
            state
                .compress_compaq_with_schema(buffer, schema)
                .map_err(|error| match error {
                    VehicleStateCompaqError::Schema(VehicleStateSchemaError::TooManyValves) => {
                        ServoError::CompressionFailed("too many valves for radio telemetry")
                    }
                    VehicleStateCompaqError::Schema(VehicleStateSchemaError::TooManySensors) => {
                        ServoError::CompressionFailed("too many sensors for radio telemetry")
                    }
                    VehicleStateCompaqError::PolicyDesynchronized => ServoError::CompressionFailed(
                        "cached radio schema no longer matches the live state",
                    ),
                    VehicleStateCompaqError::Serialize(postcard::Error::SerializeBufferFull) => {
                        ServoError::BufferTooSmall
                    }
                    VehicleStateCompaqError::Serialize(_) => {
                        ServoError::CompressionFailed("failed to serialize compaq radio telemetry")
                    }
                    VehicleStateCompaqError::Deserialize(_) => ServoError::CompressionFailed(
                        "unexpected compaq deserialize during radio encode",
                    ),
                })?;
        Ok(&buffer[..size])
    }

    fn refresh_schema_if_needed(
        &mut self,
        _state: &VehicleState,
        mappings: &Mappings,
    ) -> Result<()> {
        let mut valve_keys: Vec<_> = mappings
            .iter()
            .filter(|mapping| include_in_radio_telemetry(mapping))
            .filter(|mapping| mapping.sensor_type == SensorType::Valve)
            .map(|mapping| mapping.text_id.clone())
            .collect();
        valve_keys.sort_unstable();

        let mut sensor_keys: Vec<_> = mappings
            .iter()
            .filter(|mapping| include_in_radio_telemetry(mapping))
            .filter(|mapping| mapping.sensor_type != SensorType::Valve)
            .filter(|mapping| {
                !mapping
                    .text_id
                    .strip_suffix("_V")
                    .or_else(|| mapping.text_id.strip_suffix("_I"))
                    .is_some_and(|valve_name| valve_keys.iter().any(|key| key == valve_name))
            })
            .map(|mapping| mapping.text_id.clone())
            .collect();
        sensor_keys.sort_unstable();

        if self.schema.is_none() || self.valve_keys != valve_keys || self.sensor_keys != sensor_keys
        {
            self.schema = Some(
                VehicleStateCompressionSchema::new(
                    valve_keys.iter().cloned(),
                    sensor_keys.iter().cloned(),
                )
                .map_err(|_| ServoError::CompressionFailed("failed to build radio schema"))?,
            );
            self.valve_keys = valve_keys;
            self.sensor_keys = sensor_keys;
        }

        Ok(())
    }
}

pub(crate) fn establish(
    servo_addresses: &[impl ToSocketAddrs],
    prev_connected_servo_addr: Option<&SocketAddr>,
    chances: u8,
    timeout: Duration,
) -> Result<(TcpStream, SocketAddr)> {
    // buffer containing the serialized identity message to be sent to the control
    // server
    let mut identity = [0; Computer::POSTCARD_MAX_SIZE];

    if let Err(error) = postcard::to_slice(&Computer::Flight, &mut identity) {
        eprintln!("Failed to serialize Computer: {error}");
        return Err(ServoError::DeserializationFailed(error));
    }

    let mut prev_addr_exists = false;
    if prev_connected_servo_addr.is_some() {
        prev_addr_exists = true;
    }

    let mut fatal_error = io::ErrorKind::ConnectionRefused.into();
    if prev_addr_exists {
        let addr = prev_connected_servo_addr.unwrap();
        println!("Attempting connection with servo at {addr:?}...");

        match TcpStream::connect_timeout(addr, timeout) {
            Ok(s) => {
                let socket = Socket::from(s);
                socket
                    .set_keepalive(true)
                    .map_err(ServoError::TransportFailed)?;
                let keep_alive: TcpKeepalive = TcpKeepalive::new()
                    .with_time(SERVO_KEEP_ALIVE_DELAY)
                    .with_interval(Duration::from_secs(1))
                    .with_retries(1);

                socket
                    .set_tcp_keepalive(&keep_alive)
                    .map_err(ServoError::TransportFailed)?;
                let mut stream: std::net::TcpStream = socket.into();
                stream
                    .set_nodelay(true)
                    .map_err(ServoError::TransportFailed)?;
                stream
                    .set_nonblocking(true)
                    .map_err(ServoError::TransportFailed)?;

                if let Err(e) = stream.write_all(&identity) {
                    return Err(ServoError::TransportFailed(e));
                } else {
                    return Ok((stream, *addr));
                }
            }
            Err(e) => fatal_error = e,
        };
    } else {
        let resolved_addresses: Vec<SocketAddr> = servo_addresses
            .iter()
            .filter_map(|a| a.to_socket_addrs().ok())
            .flatten()
            .collect();
        for i in 1..=chances {
            for addr in &resolved_addresses {
                if !prev_addr_exists || (prev_connected_servo_addr == Some(addr)) {
                    println!("[{i}]: Attempting connection with servo at {addr:?}...");

                    match TcpStream::connect_timeout(addr, timeout) {
                        Ok(s) => {
                            let socket = Socket::from(s);
                            socket
                                .set_keepalive(true)
                                .map_err(ServoError::TransportFailed)?;
                            let keep_alive: TcpKeepalive = TcpKeepalive::new()
                                .with_time(SERVO_KEEP_ALIVE_DELAY)
                                .with_interval(Duration::from_secs(1))
                                .with_retries(1);

                            socket
                                .set_tcp_keepalive(&keep_alive)
                                .map_err(ServoError::TransportFailed)?;
                            let mut stream: std::net::TcpStream = socket.into();
                            stream
                                .set_nodelay(true)
                                .map_err(ServoError::TransportFailed)?;
                            stream
                                .set_nonblocking(true)
                                .map_err(ServoError::TransportFailed)?;

                            if let Err(e) = stream.write_all(&identity) {
                                return Err(ServoError::TransportFailed(e));
                            } else {
                                return Ok((stream, *addr));
                            }
                        }
                        Err(e) => fatal_error = e,
                    };
                }
            }
        }
    }

    Err(ServoError::TransportFailed(fatal_error))
}

// "pull" new information from servo
pub(crate) fn pull(servo_stream: &mut TcpStream) -> Result<Option<FlightControlMessage>> {
    let mut header = [0; 2];
    match servo_stream.peek(&mut header) {
        Ok(0) => return Err(ServoError::ServoDisconnected),
        Ok(1) => return Ok(None),
        Ok(_) => {}
        Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(None),
        Err(e) => return Err(ServoError::TransportFailed(e)),
    }
    let mut frame = vec![0; usize::from(u16::from_be_bytes(header)) + 2];
    match servo_stream.peek(&mut frame) {
        Ok(size) if size < frame.len() => return Ok(None),
        Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(None),
        Err(e) => return Err(ServoError::TransportFailed(e)),
        _ => {}
    }
    servo_stream
        .read_exact(&mut frame)
        .map_err(ServoError::TransportFailed)?;
    postcard::from_bytes(&frame[2..])
        .map(Some)
        .map_err(ServoError::DeserializationFailed)
}

pub(crate) fn reply_readout(
    stream: &mut TcpStream,
    readout: &common::ctv_config::FcReadout,
) -> io::Result<()> {
    let bytes = serde_json::to_vec(readout)?;
    let size =
        u16::try_from(bytes.len()).map_err(|_| io::Error::other("readout exceeds frame limit"))?;
    let mut frame = size.to_be_bytes().to_vec();
    frame.extend_from_slice(&bytes);
    stream.write_all(&frame)
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
    mappings: &Mappings,
    encoder: &mut RadioTelemetryEncoder,
    buffer: &mut [u8],
) -> Result<usize> {
    let message = encoder.encode(state, mappings, buffer)?;
    socket
        .send_to(message, (servo_socket.ip(), SERVO_DATA_PORT))
        .map_err(ServoError::TransportFailed)
}

#[cfg(test)]
mod ctv_tests {
    use super::*;

    #[test]
    fn fragmented_and_coalesced_frames_preserve_commands() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let mut sender = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let (mut receiver, _) = listener.accept().unwrap();
        receiver.set_nonblocking(true).unwrap();
        let command = FlightControlMessage::CtvReadout(42);
        let payload = postcard::to_allocvec(&command).unwrap();
        let mut frame = (payload.len() as u16).to_be_bytes().to_vec();
        frame.extend_from_slice(&payload);
        sender.write_all(&frame[..1]).unwrap();
        assert!(pull(&mut receiver).unwrap().is_none());
        let mut remaining = frame[1..].to_vec();
        remaining.extend_from_slice(&frame);
        sender.write_all(&remaining).unwrap();
        for _ in 0..2 {
            let deadline = std::time::Instant::now() + Duration::from_secs(1);
            loop {
                if let Some(received) = pull(&mut receiver).unwrap() {
                    assert_eq!(received, command);
                    break;
                }
                assert!(std::time::Instant::now() < deadline);
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        assert!(pull(&mut receiver).unwrap().is_none());
    }
}
