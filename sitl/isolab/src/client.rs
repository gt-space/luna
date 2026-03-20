use anyhow::{anyhow, bail, Result};
use common::comm::{Computer, NodeMapping, SensorType, VehicleState};
use futures_util::StreamExt;
use reqwest::Client;
use serde::Serialize;
use std::{time::{Duration, Instant}};
use tokio::time::{sleep, timeout};
use tokio_tungstenite::{connect_async, MaybeTlsStream, WebSocketStream};

pub const SERVO_HTTP_BASE: &str = "http://172.20.0.6:7200";
pub const SERVO_WS_BASE: &str = "ws://172.20.0.6:7200";

#[derive(Serialize)]
struct SetMappingsRequest {
  configuration_id: String,
  mappings: Vec<NodeMapping>,
}

#[derive(Serialize)]
struct ActiveConfiguration {
  configuration_id: String,
}

pub async fn wait_for_http(client: &Client) -> Result<()> {
  let deadline = Instant::now() + Duration::from_secs(30);
  while Instant::now() < deadline {
    if let Ok(response) = client
      .get(format!("{SERVO_HTTP_BASE}/operator/mappings"))
      .send()
      .await
    {
      if response.status().is_success() {
        return Ok(());
      }
    }
    sleep(Duration::from_millis(200)).await;
  }
  bail!("servo HTTP server did not become ready");
}

pub async fn configure_servo(client: &Client, mappings: &[NodeMapping]) -> Result<()> {
  client
    .post(format!("{SERVO_HTTP_BASE}/operator/mappings"))
    .json(&SetMappingsRequest {
      configuration_id: "sitl".to_string(),
      mappings: mappings.to_vec(),
    })
    .send()
    .await?
    .error_for_status()?;

  client
    .post(format!("{SERVO_HTTP_BASE}/operator/active-configuration"))
    .json(&ActiveConfiguration {
      configuration_id: "sitl".to_string(),
    })
    .send()
    .await?
    .error_for_status()?;

  Ok(())
}

pub fn build_mappings() -> Vec<NodeMapping> {
  let mut mappings = Vec::new();

  for channel in 1..=10u32 {
    mappings.push(NodeMapping {
      text_id: format!("VLV{channel:02}"),
      board_id: "sam-01".to_string(),
      sensor_type: SensorType::Valve,
      channel,
      computer: Computer::Flight,
      max: None,
      min: None,
      calibrated_offset: 0.0,
      powered_threshold: Some(0.05),
      normally_closed: Some(true),
    });
  }

  for (idx, channel) in (101..=104u32).enumerate() {
    mappings.push(NodeMapping {
      text_id: format!("PT{:02}", idx + 1),
      board_id: "sam-01".to_string(),
      sensor_type: SensorType::Pt,
      channel,
      computer: Computer::Flight,
      max: Some(1000.0),
      min: Some(0.0),
      calibrated_offset: 0.0,
      powered_threshold: None,
      normally_closed: None,
    });
  }

  for (idx, channel) in (105..=106u32).enumerate() {
    mappings.push(NodeMapping {
      text_id: format!("LC{:02}", idx + 1),
      board_id: "sam-01".to_string(),
      sensor_type: SensorType::LoadCell,
      channel,
      computer: Computer::Flight,
      max: Some(500.0),
      min: Some(0.0),
      calibrated_offset: 0.0,
      powered_threshold: None,
      normally_closed: None,
    });
  }

  for (idx, channel) in (107..=108u32).enumerate() {
    mappings.push(NodeMapping {
      text_id: format!("RV{:02}", idx + 1),
      board_id: "sam-01".to_string(),
      sensor_type: SensorType::RailVoltage,
      channel,
      computer: Computer::Flight,
      max: None,
      min: None,
      calibrated_offset: 0.0,
      powered_threshold: None,
      normally_closed: None,
    });
  }

  for (idx, channel) in (109..=110u32).enumerate() {
    mappings.push(NodeMapping {
      text_id: format!("RTD{:02}", idx + 1),
      board_id: "sam-01".to_string(),
      sensor_type: SensorType::Rtd,
      channel,
      computer: Computer::Flight,
      max: None,
      min: None,
      calibrated_offset: 0.0,
      powered_threshold: None,
      normally_closed: None,
    });
  }

  for (idx, channel) in (111..=112u32).enumerate() {
    mappings.push(NodeMapping {
      text_id: format!("TC{:02}", idx + 1),
      board_id: "sam-01".to_string(),
      sensor_type: SensorType::Tc,
      channel,
      computer: Computer::Flight,
      max: None,
      min: None,
      calibrated_offset: 0.0,
      powered_threshold: None,
      normally_closed: None,
    });
  }

  mappings
}

pub type ServoSocket = WebSocketStream<MaybeTlsStream<tokio::net::TcpStream>>;

pub async fn connect(source: Option<&str>) -> Result<ServoSocket> {
  let url = match source {
    Some(source) => format!("{SERVO_WS_BASE}/data/forward?source={source}"),
    None => format!("{SERVO_WS_BASE}/data/forward"),
  };
  let (socket, _) = connect_async(url).await?;
  Ok(socket)
}

pub fn count_standalone_sensors(state: &VehicleState) -> usize {
  state
    .sensor_readings
    .keys()
    .filter(|sensor_name| {
      sensor_name
        .strip_suffix("_V")
        .or_else(|| sensor_name.strip_suffix("_I"))
        .is_none_or(|valve_name| !state.valve_states.contains_key(valve_name))
    })
    .count()
}

pub fn assert_expected_shape(state: &VehicleState) -> Result<()> {
  let sensor_count = count_standalone_sensors(state);
  if state.valve_states.len() != 10 || sensor_count != 12 {
    bail!(
      "expected 10 valves and 12 standalone sensors, got {} valves and {} sensors",
      state.valve_states.len(),
      sensor_count,
    );
  }
  Ok(())
}

pub async fn next_vehicle_state(socket: &mut ServoSocket, deadline: Duration) -> Result<VehicleState> {
  let message = timeout(deadline, async {
    loop {
      let Some(message) = socket.next().await else {
        bail!("websocket closed");
      };
      let message = message?;
      if let tokio_tungstenite::tungstenite::Message::Text(text) = message {
        return Ok(text);
      }
    }
  })
  .await
  .map_err(|_| anyhow!("timed out waiting for websocket frame"))??;

  Ok(serde_json::from_str::<VehicleState>(&message)?)
}

pub async fn wait_for_expected_state(socket: &mut ServoSocket) -> Result<VehicleState> {
  let deadline = Instant::now() + Duration::from_secs(20);
  let mut best_valves = 0usize;
  let mut best_sensors = 0usize;
  while Instant::now() < deadline {
    let state = next_vehicle_state(socket, Duration::from_secs(2)).await?;
    let standalone_sensors = count_standalone_sensors(&state);
    if state.valve_states.len() > best_valves || standalone_sensors > best_sensors {
      best_valves = best_valves.max(state.valve_states.len());
      best_sensors = best_sensors.max(standalone_sensors);
      eprintln!(
        "observed telemetry shape: {} valves, {} standalone sensors",
        best_valves, best_sensors
      );
    }
    if state.valve_states.len() == 10 && standalone_sensors == 12 {
      return Ok(state);
    }
  }
  bail!(
    "timed out waiting for expected 10-valve/12-sensor telemetry; best observed {} valves and {} standalone sensors",
    best_valves,
    best_sensors,
  )
}

pub async fn wait_for_changed_state(
  socket: &mut ServoSocket,
  baseline: &VehicleState,
  deadline: Duration,
) -> Result<VehicleState> {
  let end = Instant::now() + deadline;
  while Instant::now() < end {
    let state = next_vehicle_state(socket, Duration::from_secs(1)).await?;
    if state != *baseline {
      return Ok(state);
    }
  }
  bail!("timed out waiting for telemetry to change");
}

pub async fn wait_for_repeated_state(socket: &mut ServoSocket, deadline: Duration) -> Result<VehicleState> {
  let end = Instant::now() + deadline;
  let mut previous = None;

  while Instant::now() < end {
    let state = next_vehicle_state(socket, Duration::from_secs(1)).await?;
    if previous.as_ref().is_some_and(|prior| prior == &state) {
      return Ok(state);
    }
    previous = Some(state);
  }

  bail!("timed out waiting for a telemetry stream to become stale");
}
