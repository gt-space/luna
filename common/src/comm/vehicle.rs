//! Defines the comprehensive vehicle state.

use super::{
  AbortStage, CompositeValveState, GpsState, Measurement, RecoState,
  Statistics, bms::Bms, fc_sensors::FcSensors, rbf::RbfState, sam,
};
use bytecheck;
use compaq::{Compress, compress};
use rkyv;
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, ops::{Index, IndexMut}};

/// Errors returned while building or validating a `VehicleState` TEL schema.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VehicleStateSchemaError {
  /// The number of valves exceeds what this wire format can represent.
  TooManyValves,
  /// The number of serialized sensors exceeds what this wire format can represent.
  TooManySensors,
}

/// Errors returned by the compaq-based `VehicleState` compression path.
#[derive(Debug)]
pub enum VehicleStateCompaqError {
  /// Building or validating the cached key schema failed.
  Schema(VehicleStateSchemaError),
  /// The ordered-key policy no longer matches the keyed map being encoded.
  PolicyDesynchronized,
  /// Postcard failed while serializing the compressed payload.
  Serialize(postcard::Error),
  /// Postcard failed while deserializing the compressed payload.
  Deserialize(postcard::Error),
}

#[derive(
  Clone,
  Debug,
  Default,
  Deserialize,
  PartialEq,
  Serialize,
  rkyv::Archive,
  rkyv::Serialize,
  rkyv::Deserialize,
)]
#[serde(transparent)]
#[archive_attr(derive(bytecheck::CheckBytes))]
/// Three RECO slots as stored in the live vehicle state.
///
/// The TEL codec intentionally treats this as a lossy aggregate: all present
/// entries are averaged into a single on-wire sample, and the decoded sample is
/// then duplicated back into all three slots.
pub struct RecoTriState(pub [Option<RecoState>; 3]);

impl From<[Option<RecoState>; 3]> for RecoTriState {
  /// Wraps the raw three-entry RECO array in the public telemetry type.
  fn from(value: [Option<RecoState>; 3]) -> Self {
    Self(value)
  }
}

impl Index<usize> for RecoTriState {
  type Output = Option<RecoState>;

  fn index(&self, index: usize) -> &Self::Output {
    &self.0[index]
  }
}

impl IndexMut<usize> for RecoTriState {
  fn index_mut(&mut self, index: usize) -> &mut Self::Output {
    &mut self.0[index]
  }
}

impl Compress for RecoTriState {
  type Compressed = <Option<RecoState> as Compress>::Compressed;

  /// Averages the present RECO samples into the single TEL representation.
  fn compress(&self) -> Self::Compressed {
    average_reco_states(self).compress()
  }

  /// Restores the single decoded TEL sample back into all three RECO slots.
  fn decompress(val: Self::Compressed) -> Self {
    let reco = <Option<RecoState> as Compress>::decompress(val)
      .map(|reco| [Some(reco.clone()), Some(reco.clone()), Some(reco)])
      .unwrap_or([None, None, None]);
    Self(reco)
  }
}

/// A persistent compression schema for `VehicleState`.
///
/// The compressor writes valve and sensor maps keylessly, so callers that send
/// many frames can cache the sorted key order instead of rebuilding it for
/// every packet.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VehicleStateCompressionSchema {
  valve_keys: Vec<String>,
  sensor_keys: Vec<String>,
}

impl VehicleStateCompressionSchema {
  /// Builds a compression schema from explicit valve and sensor key sets.
  pub fn new(
    valve_keys: impl IntoIterator<Item = String>,
    sensor_keys: impl IntoIterator<Item = String>,
  ) -> Result<Self, VehicleStateSchemaError> {
    let mut valve_keys: Vec<_> = valve_keys.into_iter().collect();
    valve_keys.sort_unstable();
    u8::try_from(valve_keys.len())
      .map_err(|_| VehicleStateSchemaError::TooManyValves)?;

    let mut sensor_keys: Vec<_> = sensor_keys.into_iter().collect();
    sensor_keys.sort_unstable();
    u8::try_from(sensor_keys.len())
      .map_err(|_| VehicleStateSchemaError::TooManySensors)?;

    Ok(Self {
      valve_keys,
      sensor_keys,
    })
  }

  /// Builds a compression schema from the current state.
  pub fn from_state(state: &VehicleState) -> Result<Self, VehicleStateSchemaError> {
    let valve_keys = state.valve_states.keys().cloned();
    let sensor_keys = state
      .sensor_readings
      .keys()
      .filter(|sensor_name| !should_omit_sensor(sensor_name, &state.valve_states))
      .cloned();

    Self::new(valve_keys, sensor_keys)
  }

  /// Returns the sorted valve keys used by the compressor.
  pub fn valve_keys(&self) -> &[String] {
    &self.valve_keys
  }

  /// Returns the sorted sensor keys used by the compressor.
  pub fn sensor_keys(&self) -> &[String] {
    &self.sensor_keys
  }
}

/// A persistent decompression schema for `VehicleState`.
///
/// Radio telemetry omits keys on the wire, so Servo can cache this schema from
/// active mappings and reuse it for every received packet.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VehicleStateDecompressionSchema {
  valve_keys: Vec<String>,
  sensor_keys: Vec<String>,
  sensor_units: Vec<sam::Unit>,
}

impl VehicleStateDecompressionSchema {
  /// Builds a decompression schema from unsorted valve and sensor metadata.
  pub fn new(
    valve_keys: impl IntoIterator<Item = String>,
    sensor_metadata: impl IntoIterator<Item = (String, sam::Unit)>,
  ) -> Self {
    let mut valve_keys: Vec<_> = valve_keys.into_iter().collect();
    valve_keys.sort_unstable();

    let mut sensor_metadata: Vec<_> = sensor_metadata.into_iter().collect();
    sensor_metadata.sort_unstable_by(|(left, _), (right, _)| left.cmp(right));

    let (sensor_keys, sensor_units): (Vec<_>, Vec<_>) =
      sensor_metadata.into_iter().unzip();

    Self {
      valve_keys,
      sensor_keys,
      sensor_units,
    }
  }

  /// Returns the sorted valve keys used by the decompressor.
  pub fn valve_keys(&self) -> &[String] {
    &self.valve_keys
  }

  /// Returns the sorted sensor keys used by the decompressor.
  pub fn sensor_keys(&self) -> &[String] {
    &self.sensor_keys
  }

  /// Returns the sorted sensor units used by the decompressor.
  pub fn sensor_units(&self) -> &[sam::Unit] {
    &self.sensor_units
  }
}

/// Holds the state of the SAMs and valves using `HashMap`s which convert a
/// node's name to its state.
#[compress(CompressedVehicleState)]
#[derive(
  Clone,
  Debug,
  Deserialize,
  PartialEq,
  Serialize,
  rkyv::Archive,
  rkyv::Serialize,
  rkyv::Deserialize,
)]
#[archive_attr(derive(bytecheck::CheckBytes))]
pub struct VehicleState {
  /// Holds the actual and commanded states of all valves on the vehicle.
  #[order]
  pub valve_states: HashMap<String, CompositeValveState>,

  /// Holds the state of every device on BMS
  pub bms: Bms,

  /// Holds the state of the flight computer board's sensors
  pub fc_sensors: FcSensors,

  /// Latest GPS state sample, if any.
  pub gps: Option<GpsState>,

  /// Whether the current `gps` sample is fresh for this control-loop
  /// iteration. The flight computer should set this to `true` when it
  /// ingests a new GPS sample, and set it to `false` immediately after
  /// sending telemetry to the server.
  pub gps_valid: bool,

  /// Latest RECO state samples from all three MCUs, if any.
  /// Index 0: MCU A (spidev1.2)
  /// Index 1: MCU B (spidev1.1)
  /// Index 2: MCU C (spidev1.0)
  pub reco: RecoTriState,

  /// Whether the current `reco` samples are fresh for this control-loop
  /// iteration. The flight computer should set this to `true` when it
  /// ingests new RECO samples, and set it to `false` immediately after
  /// sending telemetry to the server.
  pub reco_valid: bool,

  #[exclude]
  /// Aggregated RBF information for BMS, RECO, and SAM boards.
  pub rbf: RbfState,

  /// Holds the latest readings of all sensors on the vehicle.
  #[order]
  pub sensor_readings: HashMap<String, Measurement>,

  /// Holds a HashMap from Board ID to a 2-tuple of the Rolling Average of
  /// obtaining a data packet from the Board ID and the duration between the
  /// last recieved and second-to-last recieved packet of the Board ID.
  #[exclude]
  pub rolling: HashMap<String, Statistics>,

  /// Defines the current abort stage that we are in
  #[exclude]
  pub abort_stage: AbortStage,
}

/// Implements all fields as default except for the AbortStage field whose name becomes "default"
impl Default for VehicleState {
  fn default() -> Self {
    Self {
      valve_states: HashMap::new(),
      bms: Bms::default(),
      fc_sensors: FcSensors::default(),
      gps: None,
      gps_valid: false,
      rbf: RbfState::default(),
      reco: RecoTriState::default(),
      reco_valid: false,
      sensor_readings: HashMap::default(),
      rolling: HashMap::default(),
      abort_stage: AbortStage::default(),
    }
  }
}

fn should_omit_sensor(
  sensor_name: &str,
  valve_states: &HashMap<String, CompositeValveState>,
) -> bool {
  sensor_name
    .strip_suffix("_V")
    .or_else(|| sensor_name.strip_suffix("_I"))
    .is_some_and(|valve_name| valve_states.contains_key(valve_name))
}

// TEL intentionally collapses the three RECO sources to one aggregate sample.
fn average_reco_states(reco: &RecoTriState) -> Option<RecoState> {
  let present: Vec<_> = reco.0.iter().flatten().collect();
  if present.is_empty() {
    return None;
  }

  let mut combined = RecoState::default();

  fn average_array<const N: usize>(
    present: &[&RecoState],
    select: impl Fn(&RecoState) -> [f32; N],
  ) -> [f32; N] {
    let mut sum = [0.0; N];
    for reco in present {
      let values = select(reco);
      for (slot, value) in sum.iter_mut().zip(values) {
        *slot += value;
      }
    }
    for value in &mut sum {
      *value /= present.len() as f32;
    }
    sum
  }

  fn average_scalar(
    present: &[&RecoState],
    select: impl Fn(&RecoState) -> f32,
  ) -> f32 {
    present.iter().map(|reco| select(reco)).sum::<f32>() / present.len() as f32
  }

  combined.quaternion = average_array(&present, |reco| reco.quaternion);
  combined.lla_pos = average_array(&present, |reco| reco.lla_pos);
  combined.velocity = average_array(&present, |reco| reco.velocity);
  combined.g_bias = average_array(&present, |reco| reco.g_bias);
  combined.a_bias = average_array(&present, |reco| reco.a_bias);
  combined.g_sf = average_array(&present, |reco| reco.g_sf);
  combined.a_sf = average_array(&present, |reco| reco.a_sf);
  combined.lin_accel = average_array(&present, |reco| reco.lin_accel);
  combined.angular_rate = average_array(&present, |reco| reco.angular_rate);
  combined.mag_data = average_array(&present, |reco| reco.mag_data);
  combined.temperature = average_scalar(&present, |reco| reco.temperature);
  combined.pressure = average_scalar(&present, |reco| reco.pressure);

  macro_rules! or_reco_flag {
    ($field:ident) => {
      combined.$field = present.iter().any(|reco| reco.$field);
    };
  }

  or_reco_flag!(stage1_enabled);
  or_reco_flag!(stage2_enabled);
  or_reco_flag!(reco_recvd_launch);
  or_reco_flag!(ekf_blown_up);
  or_reco_flag!(drouge_timer_enable);
  or_reco_flag!(main_timer_enable);

  Some(combined)
}

impl VehicleState {
  /// Constructs a new, empty `VehicleState`.
  pub fn new() -> Self {
    VehicleState::default()
  }

  /// Serializes this vehicle state using the compaq-derived TEL representation.
  pub fn compress_compaq(
    &self,
    buf: &mut [u8],
  ) -> Result<usize, VehicleStateCompaqError> {
    let schema = VehicleStateCompressionSchema::from_state(self)
      .map_err(VehicleStateCompaqError::Schema)?;
    self.compress_compaq_with_schema(buf, &schema)
  }

  /// Serializes this vehicle state using a precomputed compaq schema.
  pub fn compress_compaq_with_schema(
    &self,
    buf: &mut [u8],
    schema: &VehicleStateCompressionSchema,
  ) -> Result<usize, VehicleStateCompaqError> {
    let compressed = self
      .deflate(schema.valve_keys.clone(), schema.sensor_keys.clone())
      .map_err(|_| VehicleStateCompaqError::PolicyDesynchronized)?;

    postcard::to_slice(&compressed, buf)
      .map(|bytes| bytes.len())
      .map_err(VehicleStateCompaqError::Serialize)
  }

  /// Reconstructs a `VehicleState` from the compaq-derived TEL representation.
  pub fn decompress_compaq<S, V>(
    bytes: &[u8],
    sensor_keys: &[S],
    sensor_units: &[sam::Unit],
    valve_keys: &[V],
  ) -> Result<Self, VehicleStateCompaqError>
  where
    S: AsRef<str>,
    V: AsRef<str>,
  {
    let schema = VehicleStateDecompressionSchema::new(
      valve_keys.iter().map(|key| key.as_ref().to_string()),
      sensor_keys
        .iter()
        .map(|key| key.as_ref().to_string())
        .zip(sensor_units.iter().copied()),
    );
    Self::decompress_compaq_with_schema(bytes, &schema)
  }

  /// Reconstructs a `VehicleState` using a precomputed compaq schema.
  pub fn decompress_compaq_with_schema(
    bytes: &[u8],
    schema: &VehicleStateDecompressionSchema,
  ) -> Result<Self, VehicleStateCompaqError> {
    let compressed: CompressedVehicleState =
      postcard::from_bytes(bytes).map_err(VehicleStateCompaqError::Deserialize)?;

    let mut state = compressed
      .inflate(schema.valve_keys.clone(), schema.sensor_keys.clone())
      .map_err(|_| VehicleStateCompaqError::PolicyDesynchronized)?;

    for (sensor_name, unit) in schema
      .sensor_keys
      .iter()
      .zip(schema.sensor_units.iter().copied())
    {
      if let Some(measurement) = state.sensor_readings.get_mut(sensor_name) {
        measurement.unit = unit;
      }
    }

    Ok(state)
  }

}

#[cfg(test)]
mod tests {
  use super::{
    super::{ValveState, fc_sensors, sam::Unit},
    *,
  };

  fn vehicle_state_with_counts(
    valve_count: usize,
    sensor_count: usize,
  ) -> VehicleState {
    let valve_states: HashMap<_, _> = (0..valve_count)
      .map(|i| {
        (
          format!("valve_{i:03}"),
          CompositeValveState {
            commanded: if i % 2 == 0 {
              ValveState::Open
            } else {
              ValveState::Closed
            },
            actual: if i % 3 == 0 {
              ValveState::Open
            } else {
              ValveState::Closed
            },
          },
        )
      })
      .collect();

    let standalone_sensor_readings = (0..sensor_count)
      .map(|i| {
        (
          format!("sensor_{i:03}"),
          Measurement {
            value: i as f64 + 0.25,
            unit: Unit::Psi,
          },
        )
      })
      .collect::<HashMap<_, _>>();

    let valve_sensor_readings = valve_states
      .keys()
      .flat_map(|valve_name| {
        [
          (
            format!("{valve_name}_I"),
            Measurement {
              value: 1.25,
              unit: Unit::Amps,
            },
          ),
          (
            format!("{valve_name}_V"),
            Measurement {
              value: 28.0,
              unit: Unit::Volts,
            },
          ),
        ]
      })
      .collect::<HashMap<_, _>>();

    let sensor_readings = standalone_sensor_readings
      .into_iter()
      .chain(valve_sensor_readings)
      .collect();

    VehicleState {
      valve_states,
      sensor_readings,
      gps: Some(GpsState {
        latitude_deg: 32.9903,
        longitude_deg: -106.9756,
        altitude_m: 1401.0,
        north_mps: 1.5,
        east_mps: -0.5,
        down_mps: 0.25,
        timestamp_unix_ms: Some(1_742_313_600_000),
        has_fix: true,
        num_satellites: 12,
      }),
      gps_valid: true,
      reco: RecoTriState([
        Some(RecoState::default()),
        Some(RecoState::default()),
        Some(RecoState::default()),
      ]),
      rolling: Default::default(),
      ..VehicleState::default()
    }
  }

  fn pseudo_randomize_vehicle_state(state: &mut VehicleState) {
    struct Lcg(u64);

    impl Lcg {
      fn new(seed: u64) -> Self {
        Self(seed)
      }

      fn next_u32(&mut self) -> u32 {
        self.0 = self
          .0
          .wrapping_mul(6364136223846793005)
          .wrapping_add(1442695040888963407);
        (self.0 >> 32) as u32
      }

      fn next_bool(&mut self) -> bool {
        self.next_u32() & 1 == 0
      }

      fn next_f64(&mut self, min: f64, max: f64) -> f64 {
        let unit = self.next_u32() as f64 / u32::MAX as f64;
        min + (max - min) * unit
      }

      fn next_f32(&mut self, min: f32, max: f32) -> f32 {
        self.next_f64(min as f64, max as f64) as f32
      }

      fn next_u8(&mut self) -> u8 {
        self.next_u32() as u8
      }
    }

    fn random_valve_state(rng: &mut Lcg) -> ValveState {
      match rng.next_u32() % 5 {
        0 => ValveState::Undetermined,
        1 => ValveState::Disconnected,
        2 => ValveState::Open,
        3 => ValveState::Closed,
        _ => ValveState::Fault,
      }
    }

    fn random_vector(rng: &mut Lcg) -> fc_sensors::Vector {
      fc_sensors::Vector {
        x: rng.next_f64(-100.0, 100.0),
        y: rng.next_f64(-100.0, 100.0),
        z: rng.next_f64(-100.0, 100.0),
      }
    }

    fn random_reco(rng: &mut Lcg) -> RecoState {
      RecoState {
        quaternion: [
          rng.next_f32(-1.0, 1.0),
          rng.next_f32(-1.0, 1.0),
          rng.next_f32(-1.0, 1.0),
          rng.next_f32(-1.0, 1.0),
        ],
        lla_pos: [
          rng.next_f32(-180.0, 180.0),
          rng.next_f32(-90.0, 90.0),
          rng.next_f32(0.0, 5000.0),
        ],
        velocity: [
          rng.next_f32(-500.0, 500.0),
          rng.next_f32(-500.0, 500.0),
          rng.next_f32(-500.0, 500.0),
        ],
        g_bias: [
          rng.next_f32(-5.0, 5.0),
          rng.next_f32(-5.0, 5.0),
          rng.next_f32(-5.0, 5.0),
        ],
        a_bias: [
          rng.next_f32(-5.0, 5.0),
          rng.next_f32(-5.0, 5.0),
          rng.next_f32(-5.0, 5.0),
        ],
        g_sf: [
          rng.next_f32(0.5, 2.0),
          rng.next_f32(0.5, 2.0),
          rng.next_f32(0.5, 2.0),
        ],
        a_sf: [
          rng.next_f32(0.5, 2.0),
          rng.next_f32(0.5, 2.0),
          rng.next_f32(0.5, 2.0),
        ],
        lin_accel: [
          rng.next_f32(-100.0, 100.0),
          rng.next_f32(-100.0, 100.0),
          rng.next_f32(-100.0, 100.0),
        ],
        angular_rate: [
          rng.next_f32(-20.0, 20.0),
          rng.next_f32(-20.0, 20.0),
          rng.next_f32(-20.0, 20.0),
        ],
        mag_data: [
          rng.next_f32(-5.0, 5.0),
          rng.next_f32(-5.0, 5.0),
          rng.next_f32(-5.0, 5.0),
        ],
        temperature: rng.next_f32(200.0, 350.0),
        pressure: rng.next_f32(50_000.0, 150_000.0),
        vref_ch1_dr1: rng.next_f32(0.0, 5.0),
        vref_ch1_dr2: rng.next_f32(0.0, 5.0),
        vref_ch2_dr1: rng.next_f32(0.0, 5.0),
        vref_ch2_dr2: rng.next_f32(0.0, 5.0),
        sns1_current: rng.next_f32(-5.0, 5.0),
        sns2_current: rng.next_f32(-5.0, 5.0),
        v_rail_24v: rng.next_f32(20.0, 28.0),
        v_rail_3v3: rng.next_f32(3.0, 3.6),
        fading_memory_baro: rng.next_f32(50_000.0, 150_000.0),
        fading_memory_gps: rng.next_f32(50_000.0, 150_000.0),
        stage1_enabled: rng.next_bool(),
        stage2_enabled: rng.next_bool(),
        reco_recvd_launch: rng.next_bool(),
        reco_driver_faults: [rng.next_u8(); 10],
        ekf_blown_up: rng.next_bool(),
        drouge_timer_enable: rng.next_bool(),
        main_timer_enable: rng.next_bool(),
        rbf_enabled: rng.next_bool(),
      }
    }

    let mut rng = Lcg::new(0x5eed_fade_cafe_beef);

    for valve_state in state.valve_states.values_mut() {
      valve_state.commanded = random_valve_state(&mut rng);
      valve_state.actual = random_valve_state(&mut rng);
    }

    state.bms.battery_bus.voltage = rng.next_f64(10.0, 30.0);
    state.bms.battery_bus.current = rng.next_f64(-20.0, 20.0);
    state.bms.umbilical_bus.voltage = rng.next_f64(10.0, 30.0);
    state.bms.umbilical_bus.current = rng.next_f64(-20.0, 20.0);
    state.bms.sam_power_bus.voltage = rng.next_f64(10.0, 30.0);
    state.bms.sam_power_bus.current = rng.next_f64(-20.0, 20.0);
    state.bms.five_volt_rail.voltage = rng.next_f64(4.0, 6.0);
    state.bms.five_volt_rail.current = rng.next_f64(0.0, 10.0);
    state.bms.charger = rng.next_f64(0.0, 15.0);
    state.bms.chassis = rng.next_f64(0.0, 15.0);
    state.bms.e_stop = rng.next_f64(0.0, 15.0);
    state.bms.rbf_tag = rng.next_f64(0.0, 15.0);

    state.fc_sensors.adc.rail_3v3.voltage = rng.next_f64(3.0, 3.6);
    state.fc_sensors.adc.rail_3v3.current = rng.next_f64(0.0, 5.0);
    state.fc_sensors.adc.rail_5v.voltage = rng.next_f64(4.5, 5.5);
    state.fc_sensors.adc.rail_5v.current = rng.next_f64(0.0, 5.0);
    state.fc_sensors.imu.accelerometer = random_vector(&mut rng);
    state.fc_sensors.imu.gyroscope = random_vector(&mut rng);
    state.fc_sensors.magnetometer = random_vector(&mut rng);
    state.fc_sensors.barometer.temperature = rng.next_f64(200.0, 350.0);
    state.fc_sensors.barometer.pressure = rng.next_f64(50_000.0, 150_000.0);

    state.gps = Some(GpsState {
      latitude_deg: rng.next_f64(-90.0, 90.0),
      longitude_deg: rng.next_f64(-180.0, 180.0),
      altitude_m: rng.next_f64(0.0, 5000.0),
      north_mps: rng.next_f64(-1000.0, 1000.0),
      east_mps: rng.next_f64(-1000.0, 1000.0),
      down_mps: rng.next_f64(-1000.0, 1000.0),
      timestamp_unix_ms: Some(rng.next_u32() as i64 * 10_000),
      has_fix: rng.next_bool(),
      num_satellites: (rng.next_u32() % 32) as u8,
    });
    state.gps_valid = true;

    state.reco = RecoTriState([
      Some(random_reco(&mut rng)),
      Some(random_reco(&mut rng)),
      Some(random_reco(&mut rng)),
    ]);
    state.reco_valid = true;

    for measurement in state.sensor_readings.values_mut() {
      measurement.value = rng.next_f64(-5000.0, 5000.0);
    }
  }

  fn decompression_schema(state: &VehicleState) -> VehicleStateDecompressionSchema {
    VehicleStateDecompressionSchema::new(
      state.valve_states.keys().cloned(),
      state
        .sensor_readings
        .iter()
        .filter(|(sensor_name, _)| !should_omit_sensor(sensor_name, &state.valve_states))
        .map(|(sensor_name, measurement)| (sensor_name.clone(), measurement.unit)),
    )
  }

  fn half_roundtrip_f64(value: f64) -> f64 {
    half::f16::from_f64(value).to_f64()
  }

  fn half_roundtrip_f32(value: f32) -> f32 {
    half::f16::from_f32(value).to_f32()
  }

  fn quantize_reco(mut reco: RecoState) -> RecoState {
    for values in [
      &mut reco.quaternion[..],
      &mut reco.lla_pos[..],
      &mut reco.velocity[..],
      &mut reco.g_bias[..],
      &mut reco.a_bias[..],
      &mut reco.g_sf[..],
      &mut reco.a_sf[..],
      &mut reco.lin_accel[..],
      &mut reco.angular_rate[..],
      &mut reco.mag_data[..],
    ] {
      for value in values {
        *value = half_roundtrip_f32(*value);
      }
    }
    reco.temperature = half_roundtrip_f32(reco.temperature);
    reco.pressure = half_roundtrip_f32(reco.pressure);
    reco
  }

  #[test]
  fn compaq_compress_vehicle_state_returns_written_size() {
    let state = vehicle_state_with_counts(10, 12);
    let mut buf = [0u8; 2048];

    let size = state
      .compress_compaq(&mut buf)
      .expect("compaq vehicle state compression should succeed");

    assert!(size > 0, "compaq compression should write some bytes");
    assert!(
      size <= buf.len(),
      "reported size should fit within the provided buffer"
    );
  }

  #[test]
  fn compaq_compress_vehicle_state_with_cached_schema_matches_default_path() {
    let state = vehicle_state_with_counts(10, 12);
    let schema = VehicleStateCompressionSchema::from_state(&state)
      .expect("schema creation should succeed");
    let decompression_schema = decompression_schema(&state);
    let mut default_buf = [0u8; 2048];
    let mut cached_buf = [0u8; 2048];

    let default_size = state
      .compress_compaq(&mut default_buf)
      .expect("default compaq compression should succeed");
    let cached_size = state
      .compress_compaq_with_schema(&mut cached_buf, &schema)
      .expect("cached compaq compression should succeed");

    assert_eq!(default_size, cached_size);
    assert_eq!(&default_buf[..default_size], &cached_buf[..cached_size]);

    let decoded = VehicleState::decompress_compaq_with_schema(
      &cached_buf[..cached_size],
      &decompression_schema,
    )
    .expect("cached compaq decompression should succeed");

    assert_eq!(
      decoded.valve_states.len(),
      state.valve_states.len(),
      "decoded state should restore all valve keys"
    );
  }

  #[test]
  /// For the use case of a test, it is much easier to manually assign values to the 
  /// fields of a struct (VehicleState) rather than using the `..Default::default()` syntax.
  #[allow(clippy::field_reassign_with_default)]
  fn compaq_compress_vehicle_state_reconstructs_expected_lossy_state() {
    let mut state = vehicle_state_with_counts(10, 12);
    state.bms.battery_bus.voltage = 13.37;
    state.bms.battery_bus.current = 42.5;
    state.bms.umbilical_bus.voltage = 27.8;
    state.bms.umbilical_bus.current = 6.25;
    state.bms.sam_power_bus.voltage = 23.4;
    state.bms.sam_power_bus.current = 4.5;
    state.bms.five_volt_rail.voltage = 5.1;
    state.bms.five_volt_rail.current = 1.8;
    state.bms.charger = 9.75;
    state.bms.chassis = 12.4;
    state.bms.e_stop = 11.9;
    state.bms.rbf_tag = 0.4;
    state.fc_sensors.adc.rail_3v3.voltage = 3.31;
    state.fc_sensors.adc.rail_3v3.current = 0.8;
    state.fc_sensors.adc.rail_5v.voltage = 5.02;
    state.fc_sensors.adc.rail_5v.current = 1.1;
    state.fc_sensors.imu.accelerometer = fc_sensors::Vector {
      x: 1.1,
      y: -2.2,
      z: 3.3,
    };
    state.fc_sensors.imu.gyroscope = fc_sensors::Vector {
      x: -0.1,
      y: 0.2,
      z: -0.3,
    };
    state.fc_sensors.magnetometer = fc_sensors::Vector {
      x: 0.4,
      y: 0.5,
      z: 0.6,
    };
    state.fc_sensors.barometer.temperature = 289.4;
    state.fc_sensors.barometer.pressure = 101_325.0;
    state.gps = Some(GpsState {
      latitude_deg: 32.9903,
      longitude_deg: -106.9756,
      altitude_m: 1401.0,
      north_mps: 1.5,
      east_mps: -0.5,
      down_mps: 0.25,
      timestamp_unix_ms: Some(1_742_313_600_000),
      has_fix: true,
      num_satellites: 12,
    });
    state.reco_valid = true;
    state.reco = RecoTriState([
      Some(RecoState {
        quaternion: [1.0, 0.1, 0.2, 0.3],
        lla_pos: [1.0, 2.0, 3.0],
        velocity: [4.0, 5.0, 6.0],
        g_bias: [0.1, 0.2, 0.3],
        a_bias: [0.4, 0.5, 0.6],
        g_sf: [1.1, 1.2, 1.3],
        a_sf: [0.9, 0.8, 0.7],
        lin_accel: [7.0, 8.0, 9.0],
        angular_rate: [0.7, 0.8, 0.9],
        mag_data: [0.3, 0.2, 0.1],
        temperature: 280.0,
        pressure: 90_000.0,
        stage1_enabled: true,
        ..RecoState::default()
      }),
      Some(RecoState {
        quaternion: [0.5, 0.4, 0.3, 0.2],
        lla_pos: [2.0, 4.0, 6.0],
        velocity: [1.0, 3.0, 5.0],
        g_bias: [0.3, 0.2, 0.1],
        a_bias: [0.6, 0.5, 0.4],
        g_sf: [1.4, 1.5, 1.6],
        a_sf: [0.6, 0.5, 0.4],
        lin_accel: [6.0, 5.0, 4.0],
        angular_rate: [0.1, 0.2, 0.3],
        mag_data: [0.9, 0.8, 0.7],
        temperature: 281.0,
        pressure: 91_000.0,
        stage2_enabled: true,
        ..RecoState::default()
      }),
      Some(RecoState {
        quaternion: [0.25, 0.5, 0.75, 1.0],
        lla_pos: [3.0, 6.0, 9.0],
        velocity: [2.0, 4.0, 6.0],
        g_bias: [0.9, 0.8, 0.7],
        a_bias: [0.1, 0.3, 0.5],
        g_sf: [0.7, 0.8, 0.9],
        a_sf: [1.7, 1.8, 1.9],
        lin_accel: [3.0, 2.0, 1.0],
        angular_rate: [0.4, 0.5, 0.6],
        mag_data: [0.6, 0.4, 0.2],
        temperature: 282.0,
        pressure: 92_000.0,
        reco_recvd_launch: true,
        ekf_blown_up: true,
        ..RecoState::default()
      }),
    ]);

    let mut buf = [0u8; 2048];
    let size = state
      .compress_compaq(&mut buf)
      .expect("compaq vehicle state compression should succeed");

    let schema = decompression_schema(&state);
    let decoded = VehicleState::decompress_compaq_with_schema(&buf[..size], &schema)
      .expect("compaq vehicle state decompression should succeed");

    let mut expected = VehicleState::default();
    expected.valve_states = state.valve_states.clone();
    expected.sensor_readings = schema
      .sensor_keys()
      .iter()
      .map(|sensor_name| {
        (
          sensor_name.clone(),
          Measurement {
            value: half_roundtrip_f64(
              state.sensor_readings.get(sensor_name).unwrap().value,
            ),
            unit: *schema
              .sensor_units()
              .iter()
              .zip(schema.sensor_keys().iter())
              .find_map(|(unit, key)| (key == sensor_name).then_some(unit))
              .unwrap(),
          },
        )
      })
      .collect();
    expected.bms.battery_bus.voltage =
      half_roundtrip_f64(state.bms.battery_bus.voltage);
    expected.bms.battery_bus.current =
      half_roundtrip_f64(state.bms.battery_bus.current);
    expected.bms.sam_power_bus.voltage =
      half_roundtrip_f64(state.bms.sam_power_bus.voltage);
    expected.bms.sam_power_bus.current =
      half_roundtrip_f64(state.bms.sam_power_bus.current);
    expected.bms.five_volt_rail.voltage =
      half_roundtrip_f64(state.bms.five_volt_rail.voltage);
    expected.bms.five_volt_rail.current =
      half_roundtrip_f64(state.bms.five_volt_rail.current);
    expected.bms.chassis = half_roundtrip_f64(state.bms.chassis);
    expected.bms.e_stop = half_roundtrip_f64(state.bms.e_stop);
    expected.bms.rbf_tag = half_roundtrip_f64(state.bms.rbf_tag);
    expected.fc_sensors.adc.rail_3v3.voltage =
      half_roundtrip_f64(state.fc_sensors.adc.rail_3v3.voltage);
    expected.fc_sensors.adc.rail_3v3.current =
      half_roundtrip_f64(state.fc_sensors.adc.rail_3v3.current);
    expected.fc_sensors.adc.rail_5v.voltage =
      half_roundtrip_f64(state.fc_sensors.adc.rail_5v.voltage);
    expected.fc_sensors.adc.rail_5v.current =
      half_roundtrip_f64(state.fc_sensors.adc.rail_5v.current);
    expected.fc_sensors.imu.accelerometer = fc_sensors::Vector {
      x: half_roundtrip_f64(state.fc_sensors.imu.accelerometer.x),
      y: half_roundtrip_f64(state.fc_sensors.imu.accelerometer.y),
      z: half_roundtrip_f64(state.fc_sensors.imu.accelerometer.z),
    };
    expected.fc_sensors.imu.gyroscope = fc_sensors::Vector {
      x: half_roundtrip_f64(state.fc_sensors.imu.gyroscope.x),
      y: half_roundtrip_f64(state.fc_sensors.imu.gyroscope.y),
      z: half_roundtrip_f64(state.fc_sensors.imu.gyroscope.z),
    };
    expected.fc_sensors.magnetometer = fc_sensors::Vector {
      x: half_roundtrip_f64(state.fc_sensors.magnetometer.x),
      y: half_roundtrip_f64(state.fc_sensors.magnetometer.y),
      z: half_roundtrip_f64(state.fc_sensors.magnetometer.z),
    };
    expected.fc_sensors.barometer.temperature =
      half_roundtrip_f64(state.fc_sensors.barometer.temperature);
    expected.fc_sensors.barometer.pressure =
      half_roundtrip_f64(state.fc_sensors.barometer.pressure);
    expected.gps = Some(GpsState {
      latitude_deg: half_roundtrip_f64(
        state.gps.as_ref().unwrap().latitude_deg,
      ),
      longitude_deg: half_roundtrip_f64(
        state.gps.as_ref().unwrap().longitude_deg,
      ),
      altitude_m: half_roundtrip_f64(state.gps.as_ref().unwrap().altitude_m),
      north_mps: half_roundtrip_f64(state.gps.as_ref().unwrap().north_mps),
      east_mps: half_roundtrip_f64(state.gps.as_ref().unwrap().east_mps),
      down_mps: half_roundtrip_f64(state.gps.as_ref().unwrap().down_mps),
      timestamp_unix_ms: state.gps.as_ref().unwrap().timestamp_unix_ms,
      has_fix: state.gps.as_ref().unwrap().has_fix,
      num_satellites: state.gps.as_ref().unwrap().num_satellites,
    });
    expected.gps_valid = state.gps_valid;
    expected.reco_valid = state.reco_valid;
    let aggregated_reco = quantize_reco(average_reco_states(&state.reco).unwrap());
    expected.reco = RecoTriState([
      Some(aggregated_reco.clone()),
      Some(aggregated_reco.clone()),
      Some(aggregated_reco),
    ]);

    assert_eq!(decoded, expected);
  }

  #[test]
  fn compaq_compress_vehicle_state_errors_on_small_buffer() {
    let state = vehicle_state_with_counts(10, 12);
    let mut buf = [0u8; 1];

    assert!(
      matches!(
        state.compress_compaq(&mut buf),
        Err(VehicleStateCompaqError::Serialize(_))
      ),
      "compaq compression should fail when the buffer is too small"
    );
  }

  #[test]
  fn compaq_compress_vehicle_state_vespula_shape_fits_radio_payload() {
    let mut state = vehicle_state_with_counts(10, 12);
    pseudo_randomize_vehicle_state(&mut state);
    let schema = VehicleStateCompressionSchema::from_state(&state)
      .expect("schema creation should succeed");
    let mut buf = [0u8; 2048];

    assert_eq!(
      state.sensor_readings.len(),
      32,
      "the Vespula test shape should include 12 standalone sensors plus 20 valve current/voltage sensors"
    );
    assert_eq!(
      schema.sensor_keys().len(),
      12,
      "the compression policy should serialize only the 12 standalone sensors"
    );
    assert!(
      schema
        .sensor_keys()
        .iter()
        .all(|key| !key.ends_with("_I") && !key.ends_with("_V")),
      "valve current/voltage helper sensors should be omitted from the ordered sensor policy"
    );

    let size = state
      .compress_compaq_with_schema(&mut buf, &schema)
      .expect("compaq compression should succeed");

    println!("Vespula TEL payload size: compaq={size} bytes");

    assert!(
      size <= 227,
      "compaq TEL payload for the Vespula 10-valve/12-sensor shape should fit under the current 227-byte radio payload limit, got {size} bytes"
    );
  }

  #[test]
  fn compaq_vespula_size_breakdown() {
    fn print_breakdown(label: &str, state: &VehicleState) {
      let schema = VehicleStateCompressionSchema::from_state(state)
        .expect("schema creation should succeed");
      let compressed = match state.deflate(
        schema.valve_keys.clone(),
        schema.sensor_keys.clone(),
      ) {
        Ok(compressed) => compressed,
        Err(_) => panic!("compaq deflate should succeed"),
      };

      let total = postcard::to_allocvec(&compressed)
        .expect("whole compressed state should serialize")
        .len();
      let valve_states = postcard::to_allocvec(&compressed.valve_states)
        .expect("valve states should serialize")
        .len();
      let bms = postcard::to_allocvec(&compressed.bms)
        .expect("bms should serialize")
        .len();
      let fc_sensors = postcard::to_allocvec(&compressed.fc_sensors)
        .expect("fc_sensors should serialize")
        .len();
      let gps = postcard::to_allocvec(&compressed.gps)
        .expect("gps should serialize")
        .len();
      let gps_valid = postcard::to_allocvec(&compressed.gps_valid)
        .expect("gps_valid should serialize")
        .len();
      let reco = postcard::to_allocvec(&compressed.reco)
        .expect("reco should serialize")
        .len();
      let reco_valid = postcard::to_allocvec(&compressed.reco_valid)
        .expect("reco_valid should serialize")
        .len();
      let sensor_readings = postcard::to_allocvec(&compressed.sensor_readings)
        .expect("sensor_readings should serialize")
        .len();

      println!(
        "{label}: total={total} valve_states={valve_states} bms={bms} fc_sensors={fc_sensors} gps={gps} gps_valid={gps_valid} reco={reco} reco_valid={reco_valid} sensor_readings={sensor_readings}"
      );
    }

    let baseline = vehicle_state_with_counts(10, 12);
    let mut randomized = vehicle_state_with_counts(10, 12);
    pseudo_randomize_vehicle_state(&mut randomized);

    print_breakdown("baseline", &baseline);
    print_breakdown("randomized", &randomized);
  }
}
