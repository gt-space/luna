use common::comm::{
  Computer,
  FlightControlMessage,
  NodeMapping,
  Sequence,
  Trigger,
  VehicleState,
  VehicleStateCompressionError,
  AbortStageConfig,
	ValveSafeState,
};
use jeflog::warn;
use postcard::experimental::max_size::MaxSize;
use std::{
  collections::HashMap,
  future::Future,
  io::IoSliceMut,
  mem::{size_of, zeroed},
  net::{IpAddr, SocketAddr as StdSocketAddr, UdpSocket as StdUdpSocket},
  os::fd::AsRawFd,
};
use super::{telemetry::{update_live_telemetry, TelemetrySource}, Database, Shared};
use tokio::{
  io::{self, AsyncReadExt, AsyncWriteExt},
  net::{TcpListener, TcpStream},
};

/// DSCP marker that identifies radio telemetry from the TEL path.
///
/// Servo reads this value from the IP TOS byte on received UDP packets so it
/// can classify otherwise-identical telemetry frames as radio instead of
/// umbilical.
pub const RADIO_TELEMETRY_DSCP: u8 = 0x2e;

/// Struct capable of performing thread-safe operations on a flight computer
/// connection, thus capable of being passed to route handlers.
#[derive(Debug)]
pub struct FlightComputer {
  database: Database,
  stream: TcpStream,
}

impl FlightComputer {
  /// Send a slice of bytes along the TCP connection to the flight computer.
  pub async fn send_bytes(&mut self, bytes: &[u8]) -> io::Result<()> {
    // get length of message, and send that first
		let length = u16::try_from(bytes.len()).map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "message too long"))?;
		// send length of message
		self.stream.write_all(&length.to_be_bytes()).await?;
		// send message
    self.stream.write_all(bytes).await
  }
  /// Get IP of active TCP Connection to Flight Computer
  pub async fn get_ip(&self) -> anyhow::Result<IpAddr> {
    let addr = self.stream.peer_addr()?;
    Ok(addr.ip())
  }
  /// Get Port of active TCP Connection to Flight Computer
  pub async fn get_port(&self) -> anyhow::Result<u16> {
    let addr = self.stream.peer_addr()?;
    Ok(addr.port())
  }

  /// Sends the given set of mappings to the flight computer.
  pub async fn send_mappings(&mut self) -> anyhow::Result<()> {
    let mappings = self
      .database
      .connection
      .lock()
      .await
      .prepare(
        "
				SELECT
					text_id,
					board_id,
					sensor_type,
					channel,
					computer,
					max,
					min,
					calibrated_offset,
					powered_threshold,
					normally_closed
				FROM NodeMappings WHERE active = TRUE
			",
      )?
      .query_and_then([], |row| {
        Ok(NodeMapping {
          text_id: row.get(0)?,
          board_id: row.get(1)?,
          sensor_type: row.get(2)?,
          channel: row.get(3)?,
          computer: row.get(4)?,
          max: row.get(5)?,
          min: row.get(6)?,
          calibrated_offset: row.get(7)?,
          powered_threshold: row.get(8)?,
          normally_closed: row.get(9)?,
        })
      })?
      .collect::<Result<Vec<NodeMapping>, rusqlite::Error>>()?;

    let message = FlightControlMessage::Mappings(mappings);
    let serialized = postcard::to_allocvec(&message)?;

    self.send_bytes(&serialized).await?;

    Ok(())
  }

  /// Sends one abort stage to flight
	pub async fn send_abort_stage_config(&mut self, stage : AbortStageConfig) -> anyhow::Result<()> {
    let message = FlightControlMessage::AbortStageConfig(stage);
    let serialized = postcard::to_allocvec(&message)?;

    self.send_bytes(&serialized).await?;

    Ok(())
	}

	/// Send all abort stages in the SQL database to flight
  pub async fn send_all_abort_configs(&mut self) -> anyhow::Result<()> {

		let stages = self
      .database
      .connection
      .lock()
      .await
			.prepare("SELECT name, condition, config FROM AbortConfigs")?
			.query_map([], |row| {
				let bytes = row.get::<_, Vec<u8>>(2)?;
				let valve_safe_states = postcard::from_bytes::<HashMap<String, ValveSafeState>>(&bytes)
					.map_err(|error| {
						rusqlite::Error::FromSqlConversionFailure(
							1,
							rusqlite::types::Type::Blob,
							Box::new(error),
						)
					})?;

				Ok(AbortStageConfig {
					stage_name: row.get(0)?,
					abort_condition: row.get(1)?,
					valve_safe_states,
				})
			})?
			.collect::<Result<Vec<AbortStageConfig>, rusqlite::Error>>()?;

		for stage in stages {
			self.send_abort_stage_config(stage).await?;
		}

		Ok(())
  }

  /// Sends the given sequence to the flight computer to be executed.
  pub async fn send_sequence(
    &mut self,
    sequence: Sequence,
  ) -> anyhow::Result<()> {
    let message = FlightControlMessage::Sequence(sequence);
    let serialized = postcard::to_allocvec(&message)?;

    self.send_bytes(&serialized).await?;
    Ok(())
  }

  /// Instructs the flight computer to stop a sequence.
  pub async fn stop_sequence(&mut self, name: String) -> anyhow::Result<()> {
    let message = FlightControlMessage::StopSequence(name);
    let serialized = postcard::to_allocvec(&message)?;

    self.send_bytes(&serialized).await?;
    Ok(())
  }

  /// Instructs the flight computer to abort.
  pub async fn abort(&mut self) -> anyhow::Result<()> {
    let message = FlightControlMessage::Abort;
    let serialized = postcard::to_allocvec(&message)?;

    self.send_bytes(&serialized).await?;
    Ok(())
  }

  /// Sends all triggers stored in the database to the flight computer, active
  /// or not.
  pub async fn send_trigger(&mut self, trigger: Trigger) -> anyhow::Result<()> {
    let message = FlightControlMessage::Trigger(trigger);
    let serialized = postcard::to_allocvec(&message)?;

    self.send_bytes(&serialized).await?;
    Ok(())
  }

  /// Checks if the underlying TCP stream has been closed.
  pub fn check_closed(&self) -> bool {
    let mut buffer = [0u8; 1];

    match self.stream.try_read(&mut buffer) {
      // if the flight stream reads zero bytes, it's closed.
      // this indicates that the current flight computer should not be there.
      Ok(size) => size == 0,
      // if the flight stream errors out with WouldBlock, it just means no
      // packet is waiting, otherwise it's a real error
      Err(e) => e.kind() != std::io::ErrorKind::WouldBlock,
    }
  }

  /// Sends a comprehensive update of mappings, triggers, and abort sequence to
  /// flight.
  pub async fn update(&mut self) -> anyhow::Result<()> {
    self.send_mappings().await?;

    // TODO: send triggers and abort sequence automatically

    Ok(())
  }
}

/// A listener function which auto-connects to the flight computer.
///
/// The flight computer is expected to fetch the IP address of the
/// ground computer by hostname resolution, outside the scope of servo.
pub fn auto_connect(server: &Shared) -> impl Future<Output = io::Result<()>> {
  let database = server.database.clone();
  let flight = server.flight.clone();
  let ground = server.ground.clone();

  async move {
    let listener = TcpListener::bind("0.0.0.0:5025").await?;
    let mut buffer = [0; Computer::POSTCARD_MAX_SIZE];

    loop {
      let (mut stream, _) = listener.accept().await?;

      let message_size = match stream.read(&mut buffer).await {
        Ok(size) => size,
        Err(error) => {
          warn!("Failed to read from flight socket: {error}");
          continue;
        }
      };

      let computer = postcard::from_bytes::<Computer>(&buffer[..message_size]);

      let computer = match computer {
        Ok(computer) => computer,
        Err(error) => {
          warn!("Failed to deserialize identity message: {error}");
          continue;
        }
      };

      match computer {
        Computer::Flight => {
          let mut flight = flight.0.lock().await;

          // if there is a flight computer already in there, check if its stream
          // is closed.
          if let Some(existing) = &*flight {
            if existing.check_closed() {
              *flight = None;
            }
          }

          // only replace the flight connection with the new one if there isn't
          // one there already. otherwise, this defaults to gracefully closing
          // the new connection on drop.
          if flight.is_none() {
            let mut new_flight = FlightComputer {
              stream,
              database: database.clone(),
            };

            if let Err(error) = new_flight.update().await {
              warn!("Failed to send update to new flight: {error}");
              continue;
            }

            // send all abort configs by default
            let _ = new_flight.send_all_abort_configs().await;

            *flight = Some(new_flight);
          }
        }
        Computer::Ground => {
          let mut ground = ground.0.lock().await;

          if let Some(existing) = &*ground {
            let mut buffer = [0; 1];

            // if the flight stream reads zero bytes, it's closed. this
            // indicates that the current flight computer should not be there.
            if existing
              .stream
              .try_read(&mut buffer)
              .is_ok_and(|size| size == 0)
            {
              *ground = None;
            }
          }

          if ground.is_none() {
            let mut new_ground = FlightComputer {
              stream,
              database: database.clone(),
            };

            if let Err(error) = new_ground.update().await {
              warn!("Failed to send update to new flight: {error}");
              continue;
            }

            // send all abort configs by default
            let _ = new_ground.send_all_abort_configs().await;
            
            *ground = Some(new_ground);
          }
        }
      };
    }
  }
}

/// Repeatedly receives vehicle state information from the flight computer.
pub fn receive_vehicle_state(
  shared: &Shared,
) -> impl Future<Output = io::Result<()>> {
  let telemetry = shared.telemetry.clone();
  let radio_schema = shared.radio_schema.clone();

  async move {
    let socket = bind_vehicle_state_socket()?;
    let mut frame_buffer = vec![0; 20_000];
    let mut control_buffer = [0u8; 128];

    loop {
      match recv_vehicle_state_packet(
        &socket,
        &mut frame_buffer,
        &mut control_buffer,
      ) {
        Ok((datagram_size, source, _)) => {
          if datagram_size == 0 {
            break;
          } else if datagram_size == frame_buffer.len() {
            frame_buffer.resize(frame_buffer.len() * 2, 0);
            continue;
          }

          let new_state = match source {
            TelemetrySource::Umbilical => {
              postcard::from_bytes::<VehicleState>(&frame_buffer[..datagram_size])
                .map_err(|error| error.to_string())
            }
            TelemetrySource::Radio => {
              let schema_guard = radio_schema.lock().await;
              let Some(schema) = schema_guard.schema() else {
                warn!("Discarding radio telemetry packet because no active radio schema is available.");
                continue;
              };

              VehicleState::decompress_with_schema(
                &frame_buffer[..datagram_size],
                schema,
              )
              .map_err(|error| format_radio_decode_error(error))
            }
          };

          match new_state {
            Ok(state) => {
              update_live_telemetry(telemetry.get(source), state).await;
            }
            Err(error) => warn!(
              "Failed to deserialize {} telemetry: {error}",
              match source {
                TelemetrySource::Umbilical => "umbilical",
                TelemetrySource::Radio => "radio",
              }
            ),
          };
        }
        Err(error) => {
          if error.raw_os_error() == Some(libc::EMSGSIZE) {
            frame_buffer.resize(frame_buffer.len() * 2, 0);
            continue;
          }

          break;
        }
      }
    }

    Ok(())
  }
}

fn bind_vehicle_state_socket() -> io::Result<StdUdpSocket> {
  let socket = StdUdpSocket::bind("0.0.0.0:7201")?;
  socket.set_nonblocking(false)?;

  let enable_tos: libc::c_int = 1;
  unsafe {
    if libc::setsockopt(
      socket.as_raw_fd(),
      libc::IPPROTO_IP,
      libc::IP_RECVTOS,
      (&enable_tos as *const libc::c_int).cast(),
      size_of::<libc::c_int>() as libc::socklen_t,
    ) != 0
    {
      return Err(io::Error::last_os_error());
    }
  }

  Ok(socket)
}

fn recv_vehicle_state_packet(
  socket: &StdUdpSocket,
  frame_buffer: &mut [u8],
  control_buffer: &mut [u8],
) -> io::Result<(usize, TelemetrySource, StdSocketAddr)> {
  let mut payload = [IoSliceMut::new(frame_buffer)];
  let mut source_addr: libc::sockaddr_storage = unsafe { zeroed() };
  let mut source_addr_len = size_of::<libc::sockaddr_storage>() as libc::socklen_t;

  let mut message = libc::msghdr {
    msg_name: (&mut source_addr as *mut libc::sockaddr_storage).cast(),
    msg_namelen: source_addr_len,
    msg_iov: payload.as_mut_ptr().cast(),
    msg_iovlen: payload.len(),
    msg_control: control_buffer.as_mut_ptr().cast(),
    msg_controllen: control_buffer.len(),
    msg_flags: 0,
  };

  let received = unsafe { libc::recvmsg(socket.as_raw_fd(), &mut message, 0) };
  if received < 0 {
    return Err(io::Error::last_os_error());
  }

  source_addr_len = message.msg_namelen;
  let remote = sockaddr_to_socket_addr(&source_addr, source_addr_len)?;
  let source = telemetry_source_from_control(message.msg_control, message.msg_controllen);

  Ok((received as usize, source, remote))
}

fn telemetry_source_from_control(
  control: *mut libc::c_void,
  control_len: usize,
) -> TelemetrySource {
  let mut current = control.cast::<libc::cmsghdr>();
  let end = (control as usize).saturating_add(control_len);

  while !current.is_null()
    && (current as usize).saturating_add(size_of::<libc::cmsghdr>()) <= end
  {
    let header = unsafe { &*current };
    if header.cmsg_level == libc::IPPROTO_IP && header.cmsg_type == libc::IP_TOS {
      let data = unsafe {
        (current as *const u8)
          .add(cmsg_data_offset())
          .cast::<u8>()
      };
      let tos = unsafe { *data };
      let dscp = tos >> 2;
      if dscp == RADIO_TELEMETRY_DSCP {
        return TelemetrySource::Radio;
      }
    }

    let next = (current as usize).saturating_add(cmsg_align(header.cmsg_len as usize));
    if next >= end {
      break;
    }
    current = next as *mut libc::cmsghdr;
  }

  TelemetrySource::Umbilical
}

fn cmsg_align(length: usize) -> usize {
  let align = size_of::<usize>();
  (length + align - 1) & !(align - 1)
}

fn cmsg_data_offset() -> usize {
  cmsg_align(size_of::<libc::cmsghdr>())
}

fn sockaddr_to_socket_addr(
  storage: &libc::sockaddr_storage,
  len: libc::socklen_t,
) -> io::Result<StdSocketAddr> {
  match storage.ss_family as i32 {
    libc::AF_INET => {
      let sockaddr = unsafe {
        *(storage as *const libc::sockaddr_storage as *const libc::sockaddr_in)
      };
      let ip = std::net::Ipv4Addr::from(u32::from_be(sockaddr.sin_addr.s_addr));
      let port = u16::from_be(sockaddr.sin_port);
      Ok(StdSocketAddr::new(ip.into(), port))
    }
    libc::AF_INET6 => {
      let sockaddr = unsafe {
        *(storage as *const libc::sockaddr_storage as *const libc::sockaddr_in6)
      };
      let ip = std::net::Ipv6Addr::from(sockaddr.sin6_addr.s6_addr);
      let port = u16::from_be(sockaddr.sin6_port);
      Ok(StdSocketAddr::new(ip.into(), port))
    }
    _ => Err(io::Error::new(
      io::ErrorKind::InvalidData,
      format!("unsupported UDP address family with length {len}"),
    )),
  }
}

fn format_radio_decode_error(error: VehicleStateCompressionError) -> String {
  match error {
    VehicleStateCompressionError::SensorCountMismatch
    | VehicleStateCompressionError::ValveCountMismatch
    | VehicleStateCompressionError::SensorMetadataLengthMismatch => {
      format!("radio schema mismatch: {error:?}")
    }
    _ => format!("{error:?}"),
  }
}
