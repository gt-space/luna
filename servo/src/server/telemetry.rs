use common::comm::{
  NodeMapping,
  SensorType,
  VehicleState,
  VehicleStateDecompressionSchema,
  include_in_radio_telemetry,
  sam::Unit,
};
use std::{
  collections::HashSet,
  fmt::{self, Display, Formatter},
  sync::Arc, time::Duration,
};
use tokio::{sync::{Mutex, Notify}, time::Instant};

/// Distinguishes the two telemetry paths Servo can ingest from flight.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TelemetrySource {
  /// Telemetry received directly over the Ethernet umbilical.
  Umbilical,
  /// Telemetry received over the TEL radio link.
  Radio,
}

impl TelemetrySource {
  /// Returns the HTTP query-string value used by Servo's telemetry routes.
  pub fn query_value(self) -> &'static str {
    match self {
      Self::Umbilical => "umbilical",
      Self::Radio => "tel",
    }
  }

  /// Returns the database table that stores snapshots for this source.
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
      "radio" | "tel" => Ok(Self::Radio),
      _ => Err(()),
    }
  }
}

impl Display for TelemetrySource {
  fn fmt(&self, f: &mut Formatter) -> fmt::Result {
    match self {
      Self::Radio => write!(f, "radio"),
      Self::Umbilical => write!(f, "umbilical"),
    }
  }
}

/// Shared live state for one telemetry source.
#[derive(Clone, Debug)]
pub struct LiveTelemetry {
  /// The most recent decoded `VehicleState` for this telemetry source.
  pub vehicle: Arc<(Mutex<VehicleState>, Notify)>,
  /// The arrival time of the latest packet for this source.
  pub last_vehicle_state: Arc<(Mutex<Option<Instant>>, Notify)>,
  /// Exponentially smoothed inter-arrival time, in seconds.
  pub rolling_duration: Arc<(Mutex<Option<f64>>, Notify)>,
  /// Size in bytes of the most recently received UDP payload.
  pub packet_size: Arc<(Mutex<Option<usize>>, Notify)>,
}

impl LiveTelemetry {
  /// Creates the live state container for one telemetry source.
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
  /// Live state for umbilical telemetry.
  pub umbilical: LiveTelemetry,
  /// Live state for radio telemetry.
  pub radio: LiveTelemetry,
}

impl TelemetryState {
  /// Creates live state for both telemetry sources.
  pub fn new() -> Self {
    Self {
      umbilical: LiveTelemetry::new(),
      radio: LiveTelemetry::new(),
    }
  }

  /// Returns the live state container for the requested telemetry source.
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
  /// Rebuilds the cached schema from the current active mapping set.
  pub fn refresh(&mut self, active_mappings: Vec<NodeMapping>) {
    let schema = build_radio_schema(&active_mappings);
    self.active_mappings = active_mappings;
    self.schema = Some(schema);
  }

  /// Returns the active cached schema, if one has been built yet.
  pub fn schema(&self) -> Option<&VehicleStateDecompressionSchema> {
    self.schema.as_ref()
  }
}

fn build_radio_schema(
  active_mappings: &[NodeMapping],
) -> VehicleStateDecompressionSchema {
  let flight_mappings: Vec<_> = active_mappings
    .iter()
    .filter(|mapping| include_in_radio_telemetry(mapping))
    .collect();

  let valve_keys: Vec<_> = flight_mappings
    .iter()
    .filter(|mapping| mapping.sensor_type == SensorType::Valve)
    .map(|mapping| mapping.text_id.clone())
    .collect();

  let valve_key_set: HashSet<_> = valve_keys.iter().cloned().collect();

  let sensor_metadata = flight_mappings
    .iter()
    .filter(|mapping| !should_omit_radio_sensor(&mapping.text_id, &valve_key_set))
    .filter_map(|mapping| {
      Some((mapping.text_id.clone(), mapping.sensor_type.unit()?))
    });

  VehicleStateDecompressionSchema::new(valve_keys, sensor_metadata)
}

fn should_omit_radio_sensor(sensor_name: &str, valve_keys: &HashSet<String>) -> bool {
  sensor_name
    .strip_suffix("_V")
    .or_else(|| sensor_name.strip_suffix("_I"))
    .is_some_and(|valve_name| valve_keys.contains(valve_name))
}

/// Replaces the live state for a telemetry source and updates its arrival
/// timing and packet-size statistics.
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

#[cfg(test)]
mod tests {
  use super::*;

  fn mapping(
    text_id: &str,
    board_id: &str,
    sensor_type: SensorType,
  ) -> NodeMapping {
    NodeMapping {
      text_id: text_id.to_string(),
      board_id: board_id.to_string(),
      sensor_type,
      channel: 1,
      computer: common::comm::Computer::Flight,
      max: None,
      min: None,
      calibrated_offset: 0.0,
      powered_threshold: None,
      normally_closed: None,
    }
  }

  #[test]
  fn radio_schema_uses_only_vehicle_sam_mappings_and_omits_valve_helpers() {
    let schema = build_radio_schema(&[
      mapping("VLV01", "sam-21", SensorType::Valve),
      mapping("VLV01_I", "sam-21", SensorType::RailCurrent),
      mapping("VLV01_V", "sam-21", SensorType::RailVoltage),
      mapping("PT01", "sam-21", SensorType::Pt),
      mapping("GROUND_PT", "sam-01", SensorType::Pt),
      mapping("GROUND_VALVE", "sam-01", SensorType::Valve),
      mapping("GROUND_VALVE_I", "sam-01", SensorType::RailCurrent),
    ]);

    assert_eq!(schema.valve_keys(), ["VLV01"]);
    assert_eq!(schema.sensor_keys(), ["PT01"]);
    assert_eq!(schema.sensor_units(), [Unit::Psi]);
  }
}
