use common::comm::{sam::Unit, NodeMapping, SensorType, VehicleState, VehicleStateDecompressionSchema};
use std::{sync::Arc, time::Duration};
use tokio::{
  sync::{Mutex, Notify},
  time::Instant,
};

/// Distinguishes the two telemetry paths Servo can ingest from flight.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TelemetrySource {
  Umbilical,
  Radio,
}

impl TelemetrySource {
  pub fn query_value(self) -> &'static str {
    match self {
      Self::Umbilical => "umbilical",
      Self::Radio => "tel",
    }
  }

  pub fn snapshot_table(self) -> &'static str {
    match self {
      Self::Umbilical => "VehicleSnapshots",
      Self::Radio => "RadioTelemetry",
    }
  }
}

impl Default for TelemetrySource {
  fn default() -> Self {
    Self::Umbilical
  }
}

impl std::str::FromStr for TelemetrySource {
  type Err = ();

  fn from_str(value: &str) -> Result<Self, Self::Err> {
    match value {
      "umbilical" => Ok(Self::Umbilical),
      "tel" => Ok(Self::Radio),
      _ => Err(()),
    }
  }
}

/// Shared live state for one telemetry source.
#[derive(Clone, Debug)]
pub struct LiveTelemetry {
  pub vehicle: Arc<(Mutex<VehicleState>, Notify)>,
  pub last_vehicle_state: Arc<(Mutex<Option<Instant>>, Notify)>,
  pub rolling_duration: Arc<(Mutex<Option<f64>>, Notify)>,
  pub packet_size: Arc<(Mutex<Option<usize>>, Notify)>,
}

impl LiveTelemetry {
  pub fn new() -> Self {
    Self {
      vehicle: Arc::new((Mutex::new(VehicleState::new()), Notify::new())),
      last_vehicle_state: Arc::new((Mutex::new(None), Notify::new())),
      rolling_duration: Arc::new((Mutex::new(None), Notify::new())),
      packet_size: Arc::new((Mutex::new(None), Notify::new())),
    }
  }
}

/// Shared state for both telemetry sources.
#[derive(Clone, Debug)]
pub struct TelemetryState {
  pub umbilical: LiveTelemetry,
  pub radio: LiveTelemetry,
}

impl TelemetryState {
  pub fn new() -> Self {
    Self {
      umbilical: LiveTelemetry::new(),
      radio: LiveTelemetry::new(),
    }
  }

  pub fn get(&self, source: TelemetrySource) -> &LiveTelemetry {
    match source {
      TelemetrySource::Umbilical => &self.umbilical,
      TelemetrySource::Radio => &self.radio,
    }
  }
}

/// Cached radio decompression schema derived from the active mappings.
#[derive(Clone, Debug, Default)]
pub struct RadioSchemaCache {
  active_mappings: Vec<NodeMapping>,
  schema: Option<VehicleStateDecompressionSchema>,
}

impl RadioSchemaCache {
  pub fn refresh(&mut self, active_mappings: Vec<NodeMapping>) {
    let schema = build_radio_schema(&active_mappings);
    self.active_mappings = active_mappings;
    self.schema = Some(schema);
  }

  pub fn schema(&self) -> Option<&VehicleStateDecompressionSchema> {
    self.schema.as_ref()
  }
}

fn build_radio_schema(
  active_mappings: &[NodeMapping],
) -> VehicleStateDecompressionSchema {
  let valve_keys = active_mappings
    .iter()
    .filter(|mapping| mapping.sensor_type == SensorType::Valve)
    .map(|mapping| mapping.text_id.clone());

  let sensor_metadata = active_mappings
    .iter()
    .filter(|mapping| mapping.sensor_type != SensorType::Valve)
    .map(|mapping| {
      (
        mapping.text_id.clone(),
        unit_for_sensor_type(mapping.sensor_type),
      )
    });

  VehicleStateDecompressionSchema::new(valve_keys, sensor_metadata)
}

fn unit_for_sensor_type(sensor_type: SensorType) -> Unit {
  match sensor_type {
    SensorType::Pt => Unit::Psi,
    SensorType::LoadCell => Unit::Pounds,
    SensorType::RailVoltage => Unit::Volts,
    SensorType::RailCurrent => Unit::Amps,
    SensorType::Tc | SensorType::Rtd => Unit::Kelvin,
    SensorType::Valve => unreachable!("valves are encoded outside sensor_readings"),
  }
}

pub async fn update_live_telemetry(
  telemetry: &LiveTelemetry,
  state: VehicleState,
  packet_size: usize,
) {
  let mut last_state_lock = telemetry.last_vehicle_state.0.lock().await;
  let mut rolling_lock = telemetry.rolling_duration.0.lock().await;
  let mut packet_size_lock = telemetry.packet_size.0.lock().await;

  if let Some(rolling_duration) = rolling_lock.as_mut() {
    *rolling_duration *= 0.9;
    *rolling_duration += (*last_state_lock)
      .unwrap_or(Instant::now())
      .elapsed()
      .as_secs_f64()
      * 0.1;
  } else {
    *rolling_lock = Some(
      (*last_state_lock)
        .unwrap_or(Instant::now())
        .elapsed()
        .as_secs_f64()
        * 0.1,
    );
  }

  *telemetry.vehicle.0.lock().await = state;
  telemetry.vehicle.1.notify_waiters();
  *packet_size_lock = Some(packet_size);
  telemetry.packet_size.1.notify_waiters();

  *last_state_lock = Some(Instant::now());
  telemetry.last_vehicle_state.1.notify_waiters();
}

#[allow(dead_code)]
fn _duration_secs(duration: Duration) -> f64 {
  duration.as_secs_f64()
}
