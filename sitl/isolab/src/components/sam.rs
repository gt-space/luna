use anyhow::{anyhow, Result};
use common::comm::{
  flight::DataMessage,
  sam::{ChannelType, DataPoint},
};
use std::{
  borrow::Cow,
  net::UdpSocket,
  sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
  },
  thread,
  time::Duration,
};

pub const INTERNAL_ARG: &str = "__internal_sam_emulator";
const FLIGHT_TARGET: &str = "127.0.0.1:4573";

pub fn spawn(stop: Arc<AtomicBool>) -> Result<thread::JoinHandle<Result<()>>> {
  let socket = UdpSocket::bind("0.0.0.0:0")?;
  socket.connect(FLIGHT_TARGET)?;

  Ok(thread::spawn(move || {
    let mut buffer = [0u8; 4096];
    let board_id = "sam-01".to_string();
    let identity = DataMessage::Identity(board_id.clone());
    let handshake = postcard::to_slice(&identity, &mut buffer)?;
    socket.send(handshake)?;

    let mut tick = 0.0f64;
    while !stop.load(Ordering::Relaxed) {
      let mut datapoints = Vec::new();

      for channel in 1..=10u32 {
        datapoints.push(DataPoint {
          value: 24.0 + channel as f64 * 0.1 + tick,
          timestamp: tick,
          channel,
          channel_type: ChannelType::ValveVoltage,
        });
        datapoints.push(DataPoint {
          value: 0.10 + channel as f64 * 0.001,
          timestamp: tick,
          channel,
          channel_type: ChannelType::ValveCurrent,
        });
      }

      for channel in 101..=104u32 {
        datapoints.push(DataPoint {
          value: 1.5 + tick + channel as f64 * 0.001,
          timestamp: tick,
          channel,
          channel_type: ChannelType::CurrentLoop,
        });
      }
      for channel in 105..=106u32 {
        datapoints.push(DataPoint {
          value: 0.005 * (channel as f64 - 100.0) + tick,
          timestamp: tick,
          channel,
          channel_type: ChannelType::DifferentialSignal,
        });
      }
      for channel in 107..=108u32 {
        datapoints.push(DataPoint {
          value: 28.0 + tick + (channel as f64 - 107.0),
          timestamp: tick,
          channel,
          channel_type: ChannelType::RailVoltage,
        });
      }
      for channel in 109..=110u32 {
        datapoints.push(DataPoint {
          value: 285.0 + tick + (channel as f64 - 109.0),
          timestamp: tick,
          channel,
          channel_type: ChannelType::Rtd,
        });
      }
      for channel in 111..=112u32 {
        datapoints.push(DataPoint {
          value: 290.0 + tick + (channel as f64 - 111.0),
          timestamp: tick,
          channel,
          channel_type: ChannelType::Tc,
        });
      }

      let message = DataMessage::Sam(board_id.clone(), Cow::Owned(datapoints));
      let serialized = postcard::to_slice(&message, &mut buffer)?;
      socket.send(serialized)?;

      tick += 0.1;
      thread::sleep(Duration::from_millis(100));
    }

    Ok(())
  }))
}

pub fn run_internal() -> Result<()> {
  let stop = Arc::new(AtomicBool::new(false));
  let handle = spawn(stop)?;
  handle
    .join()
    .map_err(|_| anyhow!("failed to join internal SAM emulator"))??;
  Ok(())
}
