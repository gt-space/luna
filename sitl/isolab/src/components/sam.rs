use anyhow::{anyhow, Result};
use common::comm::{
  flight::DataMessage,
  sam::{ChannelType, SamDataPoint, SensorDataPoint},
};
use std::{
  borrow::Cow,
  net::UdpSocket,
  time::Duration,
};

pub const INTERNAL_ARG: &str = "__internal_sam_emulator";
const FLIGHT_TARGET: &str = "127.0.0.1:4573";

fn run() -> Result<()> {
  let socket = UdpSocket::bind("0.0.0.0:0")?;
  socket.connect(FLIGHT_TARGET)?;

  let mut buffer = [0u8; 4096];
  let board_id = "sam-21".to_string();
  let identity = DataMessage::Identity(board_id.clone());
  let handshake = postcard::to_slice(&identity, &mut buffer)?;
  socket.send(handshake)?;

  let mut tick = 0.0f64;
  loop {
    let mut datapoints = Vec::new();

    for channel in 1..=10u32 {
      datapoints.push(SamDataPoint::Sensor(SensorDataPoint {
        value: 24.0 + channel as f64 * 0.1 + tick,
        timestamp: tick,
        channel,
        channel_type: ChannelType::ValveVoltage,
      }));
      datapoints.push(SamDataPoint::Sensor(SensorDataPoint {
        value: 0.10 + channel as f64 * 0.001,
        timestamp: tick,
        channel,
        channel_type: ChannelType::ValveCurrent,
      }));
    }

    for channel in 101..=104u32 {
      datapoints.push(SamDataPoint::Sensor(SensorDataPoint {
        value: 1.5 + tick + channel as f64 * 0.001,
        timestamp: tick,
        channel,
        channel_type: ChannelType::CurrentLoop,
      }));
    }
    for channel in 105..=106u32 {
      datapoints.push(SamDataPoint::Sensor(SensorDataPoint {
        value: 0.005 * (channel as f64 - 100.0) + tick,
        timestamp: tick,
        channel,
        channel_type: ChannelType::DifferentialSignal,
      }));
    }
    for channel in 107..=108u32 {
      datapoints.push(SamDataPoint::Sensor(SensorDataPoint {
        value: 28.0 + tick + (channel as f64 - 107.0),
        timestamp: tick,
        channel,
        channel_type: ChannelType::RailVoltage,
      }));
    }
    for channel in 109..=110u32 {
      datapoints.push(SamDataPoint::Sensor(SensorDataPoint {
        value: 285.0 + tick + (channel as f64 - 109.0),
        timestamp: tick,
        channel,
        channel_type: ChannelType::Rtd,
      }));
    }
    for channel in 111..=112u32 {
      datapoints.push(SamDataPoint::Sensor(SensorDataPoint {
        value: 290.0 + tick + (channel as f64 - 111.0),
        timestamp: tick,
        channel,
        channel_type: ChannelType::Tc,
      }));
    }

    let message = DataMessage::Sam(board_id.clone(), Cow::Owned(datapoints));
    let serialized = postcard::to_slice(&message, &mut buffer)?;
    socket.send(serialized)?;

    tick += 0.1;
    std::thread::sleep(Duration::from_millis(100));
  }
}

pub fn run_internal() -> Result<()> {
  run().map_err(|error| anyhow!("internal SAM emulator failed: {error}"))
}
