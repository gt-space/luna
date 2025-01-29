use super::{Database, Shared};

use jeflog::warn;
use postcard::experimental::max_size::MaxSize;
use std::future::Future;
use tokio::time::Instant;

use common::comm::{
  Computer,
  FlightControlMessage,
  NodeMapping,
  Sequence,
  Trigger,
  VehicleState,
};

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
    self.stream.write_all(bytes).await
  }

  pub async fn get_ip(&self) -> anyhow::Result<String> {
    let addr = self.stream.peer_addr()?;
    Ok(addr.ip().to_string())
  }

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

    // if the flight stream reads zero bytes, it's closed.
    // this indicates that the current flight computer should not be there.
    self
      .stream
      .try_read(&mut buffer)
      .is_ok_and(|size| size == 0)
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
  let vehicle_state = shared.vehicle.clone();
  let roll_durr = shared.rolling_duration.clone();
  let last_state = shared.last_vehicle_state.clone();

  //let last_vehicle_state

  let last_vehicle_state = shared.last_vehicle_state.clone();

  async move {
    let socket = UdpSocket::bind("0.0.0.0:7201").await.unwrap();
    let mut frame_buffer = vec![0; 20_000];

    loop {
      match socket.recv_from(&mut frame_buffer).await {
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
            Ok(state) => {
              let mut last_state_lock = last_state.0.lock().await;
              let mut roll_durr_lock = roll_durr.0.lock().await;

              if let Some(roll_durr) = roll_durr_lock.as_mut() {
                *roll_durr *= 0.9;
                *roll_durr += (*last_state_lock)
                  .unwrap_or(Instant::now())
                  .elapsed()
                  .as_secs_f64()
                  * 0.1;
              } else {
                *roll_durr_lock = Some(
                  (*last_state_lock)
                    .unwrap_or(Instant::now())
                    .elapsed()
                    .as_secs_f64()
                    * 0.1,
                );
              }

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
