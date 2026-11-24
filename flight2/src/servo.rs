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

/// Reads servo's length-prefixed messages from a non-blocking stream without
/// ever waiting for the rest of one.
///
/// The control loop calls this every iteration, so it must not stay in a read:
/// a message that arrives in pieces (or a servo that stalls mid-write) keeps
/// what has arrived here and returns `Ok(None)`, and the next call carries on.
/// The controller's period, and an abort to land, do not wait on servo.
pub(crate) struct FrameReader {
    /// The bytes of the frame so far: its 2-byte big-endian length, then the body
    buffer: Vec<u8>,
    filled: usize,
}

impl FrameReader {
    pub(crate) fn new() -> Self {
        FrameReader {
            buffer: vec![0; u16::MAX as usize + 2],
            filled: 0,
        }
    }

    /// Drops a partial frame, for a new connection.
    pub(crate) fn reset(&mut self) {
        self.filled = 0;
    }

    /// The next whole message if one has fully arrived, `Ok(None)` if not yet.
    pub(crate) fn pull(
        &mut self,
        servo_stream: &mut TcpStream,
    ) -> Result<Option<FlightControlMessage>> {
        loop {
            // Read no further than the end of this frame, so the next one
            // stays in the socket
            let end = if self.filled < 2 {
                2
            } else {
                2 + usize::from(u16::from_be_bytes([self.buffer[0], self.buffer[1]]))
            };

            if self.filled >= end && self.filled >= 2 {
                let decoded = postcard::from_bytes::<FlightControlMessage>(&self.buffer[2..end]);

                self.filled = 0;

                return match decoded {
                    Ok(message) => Ok(Some(message)),
                    Err(e) => Err(ServoError::DeserializationFailed(e)),
                };
            }

            match servo_stream.read(&mut self.buffer[self.filled..end]) {
                Ok(0) => {
                    self.filled = 0;
                    return Err(ServoError::ServoDisconnected);
                }
                Ok(n) => self.filled += n,
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(None),
                Err(ref e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => {
                    self.filled = 0;
                    return Err(ServoError::TransportFailed(e));
                }
            }
        }
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
mod tests {
    use std::{
        net::TcpListener,
        thread,
        time::{Duration, Instant},
    };

    use super::*;

    /// (the flight computer's side, non-blocking; servo's side)
    fn connected_pair() -> (TcpStream, TcpStream) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let servo = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        let (flight, _) = listener.accept().unwrap();

        flight.set_nonblocking(true).unwrap();
        servo.set_nodelay(true).unwrap();
        (flight, servo)
    }

    fn frame(message: &FlightControlMessage) -> Vec<u8> {
        let body = postcard::to_allocvec(message).unwrap();
        let mut frame = (body.len() as u16).to_be_bytes().to_vec();

        frame.extend(body);
        frame
    }

    /// Pulls until something other than "nothing yet" comes back.
    fn pull_soon(reader: &mut FrameReader, stream: &mut TcpStream) -> Result<FlightControlMessage> {
        let deadline = Instant::now() + Duration::from_secs(2);

        loop {
            match reader.pull(stream) {
                Ok(Some(message)) => return Ok(message),
                Ok(None) if Instant::now() < deadline => thread::sleep(Duration::from_millis(1)),
                Ok(None) => panic!("no message within 2 s"),
                Err(e) => return Err(e),
            }
        }
    }

    #[test]
    fn a_message_in_pieces_never_holds_up_the_caller_and_completes() {
        let (mut flight, mut servo) = connected_pair();
        let mut reader = FrameReader::new();
        let bytes = frame(&FlightControlMessage::StopSequence("abc".to_string()));

        assert!(bytes.len() > 4);
        // Half of the length, then the length and part of the body: every pull
        // returns at once with nothing, and keeps what has arrived
        for piece in [&bytes[..1], &bytes[1..3], &bytes[3..bytes.len() - 1]] {
            servo.write_all(piece).unwrap();
            thread::sleep(Duration::from_millis(30));

            let started = Instant::now();

            assert!(matches!(reader.pull(&mut flight), Ok(None)));
            assert!(started.elapsed() < Duration::from_millis(20));
        }

        servo.write_all(&bytes[bytes.len() - 1..]).unwrap();

        let message = pull_soon(&mut reader, &mut flight).unwrap();

        assert!(matches!(message, FlightControlMessage::StopSequence(name) if name == "abc"));
    }

    #[test]
    fn messages_sent_together_come_out_one_per_pull() {
        let (mut flight, mut servo) = connected_pair();
        let mut reader = FrameReader::new();
        let mut both = frame(&FlightControlMessage::StopSequence("one".to_string()));

        both.extend(frame(&FlightControlMessage::Abort));
        servo.write_all(&both).unwrap();

        let first = pull_soon(&mut reader, &mut flight).unwrap();
        let second = pull_soon(&mut reader, &mut flight).unwrap();

        assert!(matches!(first, FlightControlMessage::StopSequence(name) if name == "one"));
        assert!(matches!(second, FlightControlMessage::Abort));
        assert!(matches!(reader.pull(&mut flight), Ok(None)));
    }

    #[test]
    fn a_closed_connection_is_reported_and_a_reset_forgets_a_partial_frame() {
        let (mut flight, mut servo) = connected_pair();
        let mut reader = FrameReader::new();
        let bytes = frame(&FlightControlMessage::StopSequence("abc".to_string()));

        servo.write_all(&bytes[..3]).unwrap();
        thread::sleep(Duration::from_millis(30));
        assert!(matches!(reader.pull(&mut flight), Ok(None)));

        // A new connection starts a new frame
        reader.reset();
        assert_eq!(reader.filled, 0);

        drop(servo);
        assert!(matches!(
            pull_soon(&mut reader, &mut flight),
            Err(ServoError::ServoDisconnected)
        ));
    }
}
