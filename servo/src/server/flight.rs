use super::{Database, Shared};

use anyhow::Error;
use axum::extract::ws::close_code::SIZE;
use jeflog::warn;
use postcard::experimental::max_size::MaxSize;
use core::num;
use std::{future::Future, net::{IpAddr, SocketAddr}, ops::DerefMut};
use tokio::time::Instant;
use socket2::{self, Domain, Socket, Type};

use common::comm::{
  Computer,
  FlightControlMessage,
  NodeMapping,
  Sequence,
  Trigger,
  VehicleState,
  AbortStageConfig,
	ValveSafeState,
  flight::{
    FTEL_MTU_TRANSMISSON_LENGTH,
    FTEL_PACKET_PAYLOAD_LENGTH,
    STATE_ID_INDEX,
    PACKET_ID_INDEX,
    TOTAL_INDEX,
    SIZE_RANGE,
  },
};

use std::collections::{HashMap, HashSet};

use tokio::{
  io::{self, AsyncReadExt, AsyncWriteExt},
  net::{TcpListener, TcpStream, UdpSocket},
};

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
    let mut buffer = [0; 1];

    match (self.stream.try_read(&mut buffer)) {
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


struct VehicleStateReconstructor {
  dataholder : Vec<u8>,
  received : Vec<bool>,
  id : u8,
  end_size : usize,
  received_count : usize,
  expected_count : usize,
  pub constructed : bool,
  pub active : bool // whether a reconstructor is "active" (aka if it still holds a valid, even if unfinished, packet)
}

impl Default for VehicleStateReconstructor {
  fn default() -> Self {
    return VehicleStateReconstructor {
      dataholder : Vec::<u8>::new(),
      received : Vec::<bool>::new(),
      id : 0,
      end_size : 0,
      received_count : 0,
      expected_count : 100,
      constructed : false,
      active : false
    }
  }
}

impl VehicleStateReconstructor {
  pub fn start(&mut self, vehicle_state_size : usize, id : u8) {
    // get total size for more convenient math code
    self.end_size = vehicle_state_size;
    self.expected_count = (1 + ((vehicle_state_size + FTEL_PACKET_PAYLOAD_LENGTH - 1) / FTEL_PACKET_PAYLOAD_LENGTH)).into();
    self.received_count = 0;
    self.id = id;

    // housekeeping
    self.constructed = false;
    self.active = true;

    let reserved_size = self.expected_count * FTEL_PACKET_PAYLOAD_LENGTH;

    // isn't there a single function that does this more efficiently?
    self.dataholder.clear();
    self.dataholder.resize(reserved_size, 0);

    self.received.clear();
    self.received.resize(self.expected_count, false);
  }

  /// Checks if the packet id of *index* has already been receieved.
  /// If so, this likely indicates that this is a new vehicle state from the one
  /// currently stored.
  pub fn already_received(&self, index : usize) -> bool {
    return *self.received.get(index).unwrap_or(&true);
  }

  /// insert a packet into the buffer. Only increments
  pub fn insert(&mut self, buf : [u8; FTEL_PACKET_PAYLOAD_LENGTH], index : usize) {
    let start = (index as usize)*FTEL_PACKET_PAYLOAD_LENGTH;
    let end = start + FTEL_PACKET_PAYLOAD_LENGTH;
    self.dataholder[start..end].copy_from_slice(&buf);
    if !self.received[index] {
      self.received_count += 1;
    }
    self.received[index] = true;
  }

  pub fn get_result(&mut self) -> anyhow::Result<VehicleState> {
    if self.received_count < self.expected_count - 1 {
      return Err(anyhow::anyhow!("Not enough packets to construct VehicleState"));
    }

    // Do XOR repair
    if self.received_count == self.expected_count - 1 {
      // get the packet to repair
      let mut repaired_packet = self.expected_count;
      for i in 0..self.expected_count {
        if !self.received[i] {
          repaired_packet = i;
          break;
        }
      }

      for byte_idx in 0..FTEL_PACKET_PAYLOAD_LENGTH {
        let mut byte : u8 = 0;
        for packet_idx in 0..self.expected_count {
          byte = byte ^ self.dataholder[packet_idx * FTEL_PACKET_PAYLOAD_LENGTH + byte_idx as usize];
        }
        self.dataholder[repaired_packet as usize * FTEL_PACKET_PAYLOAD_LENGTH + byte_idx as usize] = byte;
      }
    }

    let state = postcard::from_bytes::<VehicleState>(
      &self.dataholder[..self.end_size],
    )?;

    self.constructed = true;
    Ok(state)
  }

  pub fn is_full(&self) -> bool {
    self.received_count >= self.expected_count
  }

  pub fn can_construct(&self) -> bool {
    self.received_count >= self.expected_count - 1
  }

  pub fn get_state_id(&self) -> u8 {
    return self.id;
  }
}

/// Repeatedly receives vehicle state information from the flight computer.
pub fn receive_vehicle_state(
  shared: &Shared,
) -> impl Future<Output = io::Result<()>> {
  let vehicle_state = shared.vehicle.clone();
  let last_state = shared.last_vehicle_state.clone();
  let last_vehicle_state = shared.last_vehicle_state.clone();
  
  let stats = shared.stats.clone();

  async move {
    //let udp_socket = UdpSocket::bind("0.0.0.0:7201").await.unwrap();
    let mut frame_buffer = vec![0; 20_000];
    
    // use the socket2 wrapper because we want dscp
    //let socket = socket2::SockRef::from(&udp_socket);
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, None)?;

    let address: SocketAddr = "0.0.0.0:7201".parse().expect("If this blows up I do too");
    socket.bind(&address.into())?;
    
    socket.set_nonblocking(true)?;

    // receive TOS so we can check dscp
    socket.set_recv_tos_v4(true)?;

    let udp_socket = UdpSocket::from_std(socket.into())?;

    loop {
      match udp_socket.recv_from(&mut frame_buffer).await {
        Ok((datagram_size, _)) => {        
          if datagram_size == 0 {
            // if the datagram size is zero, the connection has been closed
            break;
          } else if datagram_size == frame_buffer.len() {
            frame_buffer.resize(frame_buffer.len() * 2, 0);
            println!("resized buffer");
            continue;
          }

          let new_state = postcard::from_bytes::<VehicleState>(
            &frame_buffer[..datagram_size],
          );
          match new_state {
            Ok(mut state) => {
              // state that this wasn't from tel
              state.is_tel = false;

              // handle assignement of statistics (switch based on if this
              // is tel or not)

              // handle assignement of statistics (switch based on if this
              // is tel or not)
              let mut stats = stats.0.lock().await;
              let mut last_state_lock = last_state.0.lock().await;

              // increment counts
              stats.packet_count += 1;
              stats.state_count += 1;

              if let Some(ref mut roll_durr) = stats.rolling_duration {
                *roll_durr *= 0.9;
                *roll_durr += (*last_state_lock)
                  .unwrap_or(Instant::now())
                  .elapsed()
                  .as_secs_f64()
                  * 0.1;
              } else {
                stats.rolling_duration = Some(
                  (*last_state_lock)
                    .unwrap_or(Instant::now())
                    .elapsed()
                    .as_secs_f64()
                    * 0.1,
                );
              }


              state.tel_connection_stats.packet_count = stats.tel_packet_count;
              state.tel_connection_stats.rolling_average_update_rate = 1.0 / stats.rolling_tel_duration.unwrap_or(1000000.0);
              state.udp_connection_stats.packet_count = stats.packet_count;
              state.udp_connection_stats.rolling_average_update_rate = 1.0 / stats.rolling_duration.unwrap_or(1000000.0);

              *vehicle_state.0.lock().await = state;
              vehicle_state.1.notify_waiters();

              *last_state_lock = Some(Instant::now()); //current time
              last_vehicle_state.1.notify_waiters();
            }
            Err(error) => warn!("Failed to deserialize vehicle state: {error}"),
          };
        }
        Err(error) => {
          // Windows throws this error when the buffer is not large enough.
          // Unix systems just log whatever they can.
          if error.raw_os_error() == Some(10040) {
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

/// Gets the difference between two state ids, accounting for overflow
fn state_id_difference(a : u8, b : u8) -> i16 {
  // base case
  let a1 : i16 = a.into();
  let b1 : i16 = b.into();
  let c1 : i16 = (a1 - b1).abs();

  // overflow shifted case
  let a2 : i16 = (a1 + 128) % 256;
  let b2 : i16 = (b1 + 128) % 256;
  let c2 : i16 = (a2 - b2).abs();

  // return minimum of the two
  return if c1 < c2 { c1 } else { c2 };
}

const RECONSTRUCTOR_COUNT : usize = 8;
const STATE_ID_DEACTIVATE_THRESHOLD : i16 = 4;

/// Repeatedly receives vehicle state information from the flight computer.
/// Hardcoded to handle custom packet formatting used for TEL.
pub fn receive_vehicle_state_tel(
  shared: &Shared,
) -> impl Future<Output = io::Result<()>> {
  let vehicle_state = shared.vehicle.clone();
  let tel_last_state = shared.last_tel_vehicle_state.clone();
  let stats = shared.stats.clone();

  let last_vehicle_state = shared.last_vehicle_state.clone();

  // hold multiple at once (just in case)
  let mut reconstuctors : [VehicleStateReconstructor; RECONSTRUCTOR_COUNT] = Default::default();
  
  async move {
    // let udp_socket = UdpSocket::bind("0.0.0.0:7201").await.unwrap();

    // it's never going to receive something bigger than this unless something
    // goes wrong
    let mut frame_buffer = vec![0; 2048];
    
    // use the socket2 wrapper because we want dscp
    //let socket = socket2::SockRef::from(&udp_socket);
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, None)?;

    let address: SocketAddr = "0.0.0.0:7202".parse().expect("If this blows up I do too");
    socket.bind(&address.into())?;
    
    socket.set_nonblocking(true)?;

    // receive TOS so we can check dscp
    socket.set_recv_tos_v4(true)?;

    let udp_socket = UdpSocket::from_std(socket.into())?;
    loop {
      // clear framebuffer between receives (needed for xor data recovery)
      // very cheap due to size of frame buffer
      frame_buffer.fill(0);

      // receive next message
      match udp_socket.recv_from(&mut frame_buffer).await {
        Ok((datagram_size, _)) => {
          if datagram_size == 0 {
            // if the datagram size is zero, the connection has been closed
            break;
          } else if datagram_size == frame_buffer.len() {
            frame_buffer.resize(frame_buffer.len() * 2, 0);
            println!("resized buffer");
            continue;
          }

          // hardcoded metadata extraction (See comm::flight documentation)
          let (metadata, buffer) = frame_buffer.split_at(SIZE_RANGE.end);
          let state_id = metadata[STATE_ID_INDEX];
          let packet_id = metadata[PACKET_ID_INDEX];
          let total = metadata[TOTAL_INDEX];
          let size = u16::from_be_bytes(metadata[SIZE_RANGE].try_into().unwrap());
          {
            let mut stats = stats.0.lock().await;
            stats.tel_packet_count += 1;
          }

          // deactivate all reconstructors that are too far from this state_id
          let mut new_failed : usize = 0;
          for mut recon in &mut reconstuctors {
            if state_id_difference(state_id, recon.get_state_id()) >= STATE_ID_DEACTIVATE_THRESHOLD {
              if recon.active == true && recon.constructed == false {
                new_failed += 1;
              }
              recon.active = false;
            }
          }
          if new_failed > 0 {
            let mut stats = stats.0.lock().await;
            stats.tel_failed_count += 1;
          }

          // get reconstructor that would handle this state id
          let mut reconstructor : &mut VehicleStateReconstructor = reconstuctors.get_mut((state_id as usize) % 8).expect("Shouldn't go out of range");

          // Check if this should be considered a new packet
          //println!("{} | {} | {} == {} | {}", reconstructor.active, reconstructor.is_full(), reconstructor.get_state_id(), state_id, reconstructor.already_received(packet_id.into()));
          if (!reconstructor.active || reconstructor.is_full() || reconstructor.get_state_id() != state_id || reconstructor.already_received(packet_id.into())) {
            reconstructor.start(size.into(), state_id);
          }

          // insert packet
          reconstructor.insert(buffer[0..FTEL_PACKET_PAYLOAD_LENGTH].try_into().unwrap(), packet_id.into());

          // if we already constructed, don't bother doing it again
          if !reconstructor.can_construct() || reconstructor.constructed {
            continue;
          }

          let new_state = reconstructor.get_result();

          match new_state {
            Ok(mut state) => {
              // state that this was from tel
              state.is_tel = true;

              // handle assignement of statistics (switch based on if this
              // is tel or not)
              let mut stats = stats.0.lock().await;
              let mut last_state_lock = tel_last_state.0.lock().await;

              // increment state count
              stats.tel_state_count += 1;

              if let Some(ref mut roll_durr) = stats.rolling_tel_duration {
                *roll_durr *= 0.9;
                *roll_durr += (*last_state_lock)
                  .unwrap_or(Instant::now())
                  .elapsed()
                  .as_secs_f64()
                  * 0.1;
              } else {
                stats.rolling_tel_duration = Some(
                  (*last_state_lock)
                    .unwrap_or(Instant::now())
                    .elapsed()
                    .as_secs_f64()
                    * 0.1,
                );
              }

              state.tel_connection_stats.packet_count = stats.tel_packet_count;
              state.tel_connection_stats.rolling_average_update_rate = 1.0 / stats.rolling_tel_duration.unwrap_or(1000000.0);
              state.udp_connection_stats.packet_count = stats.packet_count;
              state.udp_connection_stats.rolling_average_update_rate = 1.0 / stats.rolling_duration.unwrap_or(1000000.0);

              *vehicle_state.0.lock().await = state;
              vehicle_state.1.notify_waiters();

              *last_state_lock = Some(Instant::now()); //current time
              tel_last_state.1.notify_waiters();
            }
            Err(error) => warn!("Failed to deserialize vehicle state: {error}"),
          };
        }
        Err(error) => {
          // Windows throws this error when the buffer is not large enough.
          // Unix systems just log whatever they can.
          if error.raw_os_error() == Some(10040) {
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