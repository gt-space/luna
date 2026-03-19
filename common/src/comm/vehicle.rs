//! Defines the comprehensive vehicle state.

use super::{
  AbortStage, CompositeValveState, GpsState, Measurement, RecoState,
  Statistics, ValveState, ahrs, ahrs::Ahrs, bms, bms::Bms, sam,
};
use bytecheck;
use rkyv;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Errors returned by manual `VehicleState` compression and decompression.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VehicleStateCompressionError {
  /// The provided output buffer cannot hold the encoded payload.
  BufferTooSmall,
  /// The number of valves exceeds what this wire format can represent.
  TooManyValves,
  /// The number of serialized sensors exceeds what this wire format can represent.
  TooManySensors,
  /// The provided sensor key and unit metadata slices differ in length.
  SensorMetadataLengthMismatch,
  /// The encoded valve count does not match the provided valve key list.
  ValveCountMismatch,
  /// The encoded sensor count does not match the provided sensor metadata.
  SensorCountMismatch,
  /// A valve state byte contained an unsupported discriminant.
  InvalidValveStateEncoding(u8),
  /// The encoded payload ended before all expected fields were read.
  UnexpectedEof,
  /// The caller provided extra bytes beyond the encoded payload.
  TrailingBytes,
}

/// Holds the state of the SAMs and valves using `HashMap`s which convert a
/// node's name to its state.
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
  pub valve_states: HashMap<String, CompositeValveState>,

  /// Holds the state of every device on BMS
  pub bms: Bms,

  /// Holds the state of every device on AHRS
  pub ahrs: Ahrs,

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
  pub reco: [Option<RecoState>; 3],

  /// Whether the current `reco` samples are fresh for this control-loop
  /// iteration. The flight computer should set this to `true` when it
  /// ingests new RECO samples, and set it to `false` immediately after
  /// sending telemetry to the server.
  pub reco_valid: bool,

  /// Holds the latest readings of all sensors on the vehicle.
  pub sensor_readings: HashMap<String, Measurement>,

  /// Holds a HashMap from Board ID to a 2-tuple of the Rolling Average of
  /// obtaining a data packet from the Board ID and the duration between the
  /// last recieved and second-to-last recieved packet of the Board ID.
  pub rolling: HashMap<String, Statistics>,

  /// Defines the current abort stage that we are in
  pub abort_stage: AbortStage,
}

/// Implements all fields as default except for the AbortStage field whose name becomes "default"
impl Default for VehicleState {
  fn default() -> Self {
    Self {
      valve_states: HashMap::new(),
      bms: Bms::default(),
      ahrs: Ahrs::default(),
      gps: None,
      gps_valid: false,
      reco: [None, None, None],
      reco_valid: false,
      sensor_readings: HashMap::default(),
      rolling: HashMap::default(),
      abort_stage: AbortStage {
        name: "default".to_string(),
        abort_condition: String::new(),
        aborted: false,
        valve_safe_states: HashMap::new(),
      },
    }
  }
}

// All floating-point telemetry is quantized through f16 on the wire. These
// helpers centralize that policy so the encoder and tests use the same rules.
fn write_f16_from_f64(
  buf: &mut [u8],
  cursor: &mut usize,
  value: f64,
) -> Result<(), VehicleStateCompressionError> {
  write_bytes(
    buf,
    cursor,
    &half::f16::from_f64(value).to_bits().to_le_bytes(),
  )
}

fn write_f16_from_f32(
  buf: &mut [u8],
  cursor: &mut usize,
  value: f32,
) -> Result<(), VehicleStateCompressionError> {
  write_bytes(
    buf,
    cursor,
    &half::f16::from_f32(value).to_bits().to_le_bytes(),
  )
}

fn write_f32_slice_as_f16(
  buf: &mut [u8],
  cursor: &mut usize,
  values: &[f32],
) -> Result<(), VehicleStateCompressionError> {
  for &value in values {
    write_f16_from_f32(buf, cursor, value)?;
  }
  Ok(())
}

// Decoder-side mirrors of the f16 helpers above.
fn read_f16_to_f64(
  bytes: &[u8],
  cursor: &mut usize,
) -> Result<f64, VehicleStateCompressionError> {
  let raw = read_exact(bytes, cursor, 2)?;
  let mut array = [0u8; 2];
  array.copy_from_slice(raw);
  Ok(half::f16::from_bits(u16::from_le_bytes(array)).to_f64())
}

fn read_f16_to_f32(
  bytes: &[u8],
  cursor: &mut usize,
) -> Result<f32, VehicleStateCompressionError> {
  let raw = read_exact(bytes, cursor, 2)?;
  let mut array = [0u8; 2];
  array.copy_from_slice(raw);
  Ok(half::f16::from_bits(u16::from_le_bytes(array)).to_f32())
}

fn read_f32_array<const N: usize>(
  bytes: &[u8],
  cursor: &mut usize,
) -> Result<[f32; N], VehicleStateCompressionError> {
  let mut values = [0.0; N];
  for value in &mut values {
    *value = read_f16_to_f32(bytes, cursor)?;
  }
  Ok(values)
}

// Tests use this to build the expected lossy round-trip value without
// re-implementing the f16 conversion logic by hand for each array.
fn quantize_f32_slice(values: &mut [f32]) {
  for value in values {
    *value = half::f16::from_f32(*value).to_f32();
  }
}

// These helpers implement the byte cursor discipline for the manual wire
// format. Any attempt to read or write past the provided slice is surfaced as
// a concrete compression error immediately.
fn write_bytes(
  buf: &mut [u8],
  cursor: &mut usize,
  bytes: &[u8],
) -> Result<(), VehicleStateCompressionError> {
  ensure_capacity(*cursor, bytes.len(), buf.len())?;
  let end = *cursor + bytes.len();
  buf[*cursor..end].copy_from_slice(bytes);
  *cursor = end;
  Ok(())
}

fn write_u8(
  buf: &mut [u8],
  cursor: &mut usize,
  value: u8,
) -> Result<(), VehicleStateCompressionError> {
  write_bytes(buf, cursor, &[value])
}

fn write_i64_le(
  buf: &mut [u8],
  cursor: &mut usize,
  value: i64,
) -> Result<(), VehicleStateCompressionError> {
  write_bytes(buf, cursor, &value.to_le_bytes())
}

fn read_exact<'a>(
  bytes: &'a [u8],
  cursor: &mut usize,
  len: usize,
) -> Result<&'a [u8], VehicleStateCompressionError> {
  let end = cursor
    .checked_add(len)
    .ok_or(VehicleStateCompressionError::UnexpectedEof)?;
  if end > bytes.len() {
    return Err(VehicleStateCompressionError::UnexpectedEof);
  }
  let slice = &bytes[*cursor..end];
  *cursor = end;
  Ok(slice)
}

fn read_u8(
  bytes: &[u8],
  cursor: &mut usize,
) -> Result<u8, VehicleStateCompressionError> {
  Ok(read_exact(bytes, cursor, 1)?[0])
}

fn read_i64_le(
  bytes: &[u8],
  cursor: &mut usize,
) -> Result<i64, VehicleStateCompressionError> {
  let raw = read_exact(bytes, cursor, 8)?;
  let mut array = [0u8; 8];
  array.copy_from_slice(raw);
  Ok(i64::from_le_bytes(array))
}

fn ensure_capacity(
  cursor: usize,
  additional: usize,
  buf_len: usize,
) -> Result<(), VehicleStateCompressionError> {
  match cursor.checked_add(additional) {
    Some(end) if end <= buf_len => Ok(()),
    _ => Err(VehicleStateCompressionError::BufferTooSmall),
  }
}

impl VehicleState {
  /// Constructs a new, empty `VehicleState`.
  pub fn new() -> Self {
    VehicleState::default()
  }

  /// Serializes this vehicle state into a compact binary representation.
  ///
  /// This format is intentionally hand-packed for telemetry use. It omits the
  /// `rolling` and `abort_stage` fields, writes fixed-size sections before the
  /// variable-sized maps, and assumes the decoder already knows the sorted key
  /// lists for `valve_states` and `sensor_readings`.
  pub fn compress(
    &self,
    buf: &mut [u8],
  ) -> Result<usize, VehicleStateCompressionError> {
    fn encode_valve_state(state: ValveState) -> u8 {
      match state {
        ValveState::Undetermined => 0,
        ValveState::Disconnected => 1,
        ValveState::Open => 2,
        ValveState::Closed => 3,
        ValveState::Fault => 4,
      }
    }

    fn write_bus(
      buf: &mut [u8],
      cursor: &mut usize,
      bus: &bms::Bus,
    ) -> Result<(), VehicleStateCompressionError> {
      write_f16_from_f64(buf, cursor, bus.voltage)?;
      write_f16_from_f64(buf, cursor, bus.current)
    }

    fn write_vector(
      buf: &mut [u8],
      cursor: &mut usize,
      vector: &ahrs::Vector,
    ) -> Result<(), VehicleStateCompressionError> {
      write_f16_from_f64(buf, cursor, vector.x)?;
      write_f16_from_f64(buf, cursor, vector.y)?;
      write_f16_from_f64(buf, cursor, vector.z)
    }

    fn write_reco_flags(
      buf: &mut [u8],
      cursor: &mut usize,
      reco: &RecoState,
    ) -> Result<(), VehicleStateCompressionError> {
      let mut flags = [0u8; 3];

      for (byte_index, bit_index, value) in [
        (0, 0, reco.stage1_enabled),
        (0, 1, reco.stage2_enabled),
        (0, 2, reco.vref_a_stage1),
        (0, 3, reco.vref_a_stage2),
        (0, 4, reco.vref_b_stage1),
        (0, 5, reco.vref_b_stage2),
        (0, 6, reco.vref_c_stage1),
        (0, 7, reco.vref_c_stage2),
        (1, 0, reco.vref_d_stage1),
        (1, 1, reco.vref_d_stage2),
        (1, 2, reco.vref_e_stage1_1),
        (1, 3, reco.vref_e_stage1_2),
        (1, 4, reco.reco_recvd_launch),
        (1, 5, reco.fault_driver_a),
        (1, 6, reco.fault_driver_b),
        (1, 7, reco.fault_driver_c),
        (2, 0, reco.fault_driver_d),
        (2, 1, reco.fault_driver_e),
        (2, 2, reco.ekf_blown_up),
      ] {
        if value {
          flags[byte_index] |= 1 << bit_index;
        }
      }

      write_bytes(buf, cursor, &flags)
    }

    fn write_reco_state(
      buf: &mut [u8],
      cursor: &mut usize,
      reco: &RecoState,
    ) -> Result<(), VehicleStateCompressionError> {
      // RECO is the largest structured payload in the frame, so it is written
      // as a simple ordered sequence of f16 arrays followed by packed flags.
      for values in [
        &reco.quaternion[..],
        &reco.lla_pos[..],
        &reco.velocity[..],
        &reco.g_bias[..],
        &reco.a_bias[..],
        &reco.g_sf[..],
        &reco.a_sf[..],
        &reco.lin_accel[..],
        &reco.angular_rate[..],
        &reco.mag_data[..],
      ] {
        write_f32_slice_as_f16(buf, cursor, values)?;
      }

      write_f16_from_f32(buf, cursor, reco.temperature)?;
      write_f16_from_f32(buf, cursor, reco.pressure)?;
      write_reco_flags(buf, cursor, reco)
    }

    fn average_reco_states(reco: &[Option<RecoState>; 3]) -> Option<RecoState> {
      let present: Vec<_> = reco.iter().flatten().collect();
      if present.is_empty() {
        return None;
      }

      let mut combined = RecoState::default();

      // The on-wire format stores only one RECO payload. Float fields are
      // averaged so the aggregate still represents all present MCUs.
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
        present.iter().map(|reco| select(reco)).sum::<f32>()
          / present.len() as f32
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

      // Boolean state is treated as "any MCU observed this".
      macro_rules! or_reco_flag {
        ($field:ident) => {
          combined.$field = present.iter().any(|reco| reco.$field);
        };
      }

      or_reco_flag!(stage1_enabled);
      or_reco_flag!(stage2_enabled);
      or_reco_flag!(vref_a_stage1);
      or_reco_flag!(vref_a_stage2);
      or_reco_flag!(vref_b_stage1);
      or_reco_flag!(vref_b_stage2);
      or_reco_flag!(vref_c_stage1);
      or_reco_flag!(vref_c_stage2);
      or_reco_flag!(vref_d_stage1);
      or_reco_flag!(vref_d_stage2);
      or_reco_flag!(vref_e_stage1_1);
      or_reco_flag!(vref_e_stage1_2);
      or_reco_flag!(reco_recvd_launch);
      or_reco_flag!(fault_driver_a);
      or_reco_flag!(fault_driver_b);
      or_reco_flag!(fault_driver_c);
      or_reco_flag!(fault_driver_d);
      or_reco_flag!(fault_driver_e);
      or_reco_flag!(ekf_blown_up);
      Some(combined)
    }

    fn should_omit_sensor(
      sensor_name: &str,
      valve_states: &HashMap<String, CompositeValveState>,
    ) -> bool {
      // Valve current and voltage sensors can be reconstructed from the known
      // schema, so values named `<valve>_I` and `<valve>_V` are omitted when a
      // matching valve is present.
      sensor_name
        .strip_suffix("_V")
        .or_else(|| sensor_name.strip_suffix("_I"))
        .is_some_and(|valve_name| valve_states.contains_key(valve_name))
    }

    let valve_count = u8::try_from(self.valve_states.len())
      .map_err(|_| VehicleStateCompressionError::TooManyValves)?;
    let mut ordered_sensors: Vec<_> = self
      .sensor_readings
      .iter()
      .filter(|(sensor_name, _)| {
        !should_omit_sensor(sensor_name, &self.valve_states)
      })
      .collect();
    ordered_sensors.sort_unstable_by(|(left, _), (right, _)| left.cmp(right));
    let sensor_count = u8::try_from(ordered_sensors.len())
      .map_err(|_| VehicleStateCompressionError::TooManySensors)?;

    let mut cursor = 0;

    // Pack the always-present BMS and AHRS telemetry first so the fixed-size
    // portion of the frame stays contiguous and easy to decode. Umbilical and
    // charger telemetry are intentionally omitted from this compact format.
    write_bus(buf, &mut cursor, &self.bms.battery_bus)?;
    write_bus(buf, &mut cursor, &self.bms.sam_power_bus)?;
    write_bus(buf, &mut cursor, &self.bms.five_volt_rail)?;
    write_f16_from_f64(buf, &mut cursor, self.bms.chassis)?;
    write_f16_from_f64(buf, &mut cursor, self.bms.e_stop)?;
    write_f16_from_f64(buf, &mut cursor, self.bms.rbf_tag)?;

    write_bus(buf, &mut cursor, &self.ahrs.rail_3v3)?;
    write_bus(buf, &mut cursor, &self.ahrs.rail_5v)?;
    write_vector(buf, &mut cursor, &self.ahrs.imu.accelerometer)?;
    write_vector(buf, &mut cursor, &self.ahrs.imu.gyroscope)?;
    write_vector(buf, &mut cursor, &self.ahrs.magnetometer)?;
    write_f16_from_f64(buf, &mut cursor, self.ahrs.barometer.temperature)?;
    write_f16_from_f64(buf, &mut cursor, self.ahrs.barometer.pressure)?;

    // Encode GPS presence and freshness together, then write the optional GPS
    // payload only when a fix sample exists.
    let mut gps_flags = 0u8;
    if self.gps.is_some() {
      gps_flags |= 1 << 0;
    }
    if self.gps_valid {
      gps_flags |= 1 << 1;
    }
    if self.gps.as_ref().is_some_and(|gps| gps.has_fix) {
      gps_flags |= 1 << 2;
    }
    if self
      .gps
      .as_ref()
      .is_some_and(|gps| gps.timestamp_unix_ms.is_some())
    {
      gps_flags |= 1 << 3;
    }
    write_u8(buf, &mut cursor, gps_flags)?;

    if let Some(gps) = &self.gps {
      write_f16_from_f64(buf, &mut cursor, gps.latitude_deg)?;
      write_f16_from_f64(buf, &mut cursor, gps.longitude_deg)?;
      write_f16_from_f64(buf, &mut cursor, gps.altitude_m)?;
      write_f16_from_f64(buf, &mut cursor, gps.north_mps)?;
      write_f16_from_f64(buf, &mut cursor, gps.east_mps)?;
      write_f16_from_f64(buf, &mut cursor, gps.down_mps)?;
      if let Some(timestamp_unix_ms) = gps.timestamp_unix_ms {
        write_i64_le(buf, &mut cursor, timestamp_unix_ms)?;
      }
      write_u8(buf, &mut cursor, gps.num_satellites)?;
    }

    // Collapse the three RECO samples into one aggregate payload. Float fields
    // are averaged across present samples and bools are OR'd together. A future
    // decoder can duplicate this aggregate back into all three slots.
    let mut reco_flags = 0u8;
    if self.reco_valid {
      reco_flags |= 1 << 0;
    }
    let aggregated_reco = average_reco_states(&self.reco);
    if aggregated_reco.is_some() {
      reco_flags |= 1 << 1;
    }
    write_u8(buf, &mut cursor, reco_flags)?;

    if let Some(reco) = aggregated_reco.as_ref() {
      write_reco_state(buf, &mut cursor, reco)?;
    }

    // Store the map lengths explicitly so a future decoder can validate the
    // provided key lists before reconstructing the omitted key/value maps.
    write_u8(buf, &mut cursor, valve_count)?;
    write_u8(buf, &mut cursor, sensor_count)?;

    // Valve states are ordered by key and encoded keylessly as one byte per
    // valve: three bits for commanded and three bits for actual.
    let mut ordered_valves: Vec<_> = self.valve_states.iter().collect();
    ordered_valves.sort_unstable_by(|(left, _), (right, _)| left.cmp(right));
    for (_, valve_state) in ordered_valves {
      let packed = encode_valve_state(valve_state.commanded)
        | (encode_valve_state(valve_state.actual) << 3);
      write_u8(buf, &mut cursor, packed)?;
    }

    // Sensor readings are likewise ordered by key and emitted keylessly. The
    // decoder is assumed to know each sensor's unit, so only the numeric value
    // is carried here as an exact two-byte `f16`.
    for (_, measurement) in ordered_sensors {
      write_f16_from_f64(buf, &mut cursor, measurement.value)?;
    }

    Ok(cursor)
  }

  /// Reconstructs a `VehicleState` from the exact byte slice emitted by
  /// [`VehicleState::compress`].
  ///
  /// `bytes` must be sliced to exactly the encoded payload length; trailing or
  /// missing bytes are treated as an error.
  ///
  /// `sensor_keys` and `sensor_units` must describe only the serialized sensor
  /// readings, in any order. Sensors omitted by the compressor because their
  /// names matched `<valve>_I` or `<valve>_V` are not recoverable from the
  /// current format and therefore must not be listed here.
  ///
  /// `valve_keys` must describe the serialized valves, in any order.
  pub fn decompress<S, V>(
    bytes: &[u8],
    sensor_keys: &[S],
    sensor_units: &[sam::Unit],
    valve_keys: &[V],
  ) -> Result<Self, VehicleStateCompressionError>
  where
    S: AsRef<str>,
    V: AsRef<str>,
  {
    fn read_bus(
      bytes: &[u8],
      cursor: &mut usize,
    ) -> Result<bms::Bus, VehicleStateCompressionError> {
      Ok(bms::Bus {
        voltage: read_f16_to_f64(bytes, cursor)?,
        current: read_f16_to_f64(bytes, cursor)?,
      })
    }

    fn read_vector(
      bytes: &[u8],
      cursor: &mut usize,
    ) -> Result<ahrs::Vector, VehicleStateCompressionError> {
      Ok(ahrs::Vector {
        x: read_f16_to_f64(bytes, cursor)?,
        y: read_f16_to_f64(bytes, cursor)?,
        z: read_f16_to_f64(bytes, cursor)?,
      })
    }

    fn decode_valve_state(
      value: u8,
    ) -> Result<ValveState, VehicleStateCompressionError> {
      match value {
        0 => Ok(ValveState::Undetermined),
        1 => Ok(ValveState::Disconnected),
        2 => Ok(ValveState::Open),
        3 => Ok(ValveState::Closed),
        4 => Ok(ValveState::Fault),
        _ => Err(VehicleStateCompressionError::InvalidValveStateEncoding(value)),
      }
    }

    fn read_reco_state(
      bytes: &[u8],
      cursor: &mut usize,
    ) -> Result<RecoState, VehicleStateCompressionError> {
      // Decode the RECO payload in the same field order used by the encoder:
      // ten float arrays, two scalar floats, then three packed-flag bytes.
      let quaternion = read_f32_array(bytes, cursor)?;
      let lla_pos = read_f32_array(bytes, cursor)?;
      let velocity = read_f32_array(bytes, cursor)?;
      let g_bias = read_f32_array(bytes, cursor)?;
      let a_bias = read_f32_array(bytes, cursor)?;
      let g_sf = read_f32_array(bytes, cursor)?;
      let a_sf = read_f32_array(bytes, cursor)?;
      let lin_accel = read_f32_array(bytes, cursor)?;
      let angular_rate = read_f32_array(bytes, cursor)?;
      let mag_data = read_f32_array(bytes, cursor)?;
      let temperature = read_f16_to_f32(bytes, cursor)?;
      let pressure = read_f16_to_f32(bytes, cursor)?;

      let flags = [
        read_u8(bytes, cursor)?,
        read_u8(bytes, cursor)?,
        read_u8(bytes, cursor)?,
      ];
      let bit =
        |byte: usize, shift: u8| -> bool { flags[byte] & (1 << shift) != 0 };

      Ok(RecoState {
        quaternion,
        lla_pos,
        velocity,
        g_bias,
        a_bias,
        g_sf,
        a_sf,
        lin_accel,
        angular_rate,
        mag_data,
        temperature,
        pressure,
        stage1_enabled: bit(0, 0),
        stage2_enabled: bit(0, 1),
        vref_a_stage1: bit(0, 2),
        vref_a_stage2: bit(0, 3),
        vref_b_stage1: bit(0, 4),
        vref_b_stage2: bit(0, 5),
        vref_c_stage1: bit(0, 6),
        vref_c_stage2: bit(0, 7),
        vref_d_stage1: bit(1, 0),
        vref_d_stage2: bit(1, 1),
        vref_e_stage1_1: bit(1, 2),
        vref_e_stage1_2: bit(1, 3),
        reco_recvd_launch: bit(1, 4),
        fault_driver_a: bit(1, 5),
        fault_driver_b: bit(1, 6),
        fault_driver_c: bit(1, 7),
        fault_driver_d: bit(2, 0),
        fault_driver_e: bit(2, 1),
        ekf_blown_up: bit(2, 2),
      })
    }

    if sensor_keys.len() != sensor_units.len() {
      return Err(VehicleStateCompressionError::SensorMetadataLengthMismatch);
    }

    // The wire format stores valve and sensor values without keys, so the
    // caller provides the schema and we sort it to match the encoder order.
    let mut sorted_valve_keys: Vec<&str> =
      valve_keys.iter().map(AsRef::as_ref).collect();
    sorted_valve_keys.sort_unstable();

    let mut sorted_sensor_metadata: Vec<(&str, sam::Unit)> = sensor_keys
      .iter()
      .map(AsRef::as_ref)
      .zip(sensor_units.iter().copied())
      .collect();
    sorted_sensor_metadata
      .sort_unstable_by(|(left, _), (right, _)| left.cmp(right));

    let mut cursor = 0;

    // Read the fixed-size BMS and AHRS sections first. These are always
    // present and occupy the front of the frame.
    let battery_bus = read_bus(bytes, &mut cursor)?;
    let sam_power_bus = read_bus(bytes, &mut cursor)?;
    let five_volt_rail = read_bus(bytes, &mut cursor)?;
    let chassis = read_f16_to_f64(bytes, &mut cursor)?;
    let e_stop = read_f16_to_f64(bytes, &mut cursor)?;
    let rbf_tag = read_f16_to_f64(bytes, &mut cursor)?;

    let rail_3v3 = read_bus(bytes, &mut cursor)?;
    let rail_5v = read_bus(bytes, &mut cursor)?;
    let accelerometer = read_vector(bytes, &mut cursor)?;
    let gyroscope = read_vector(bytes, &mut cursor)?;
    let magnetometer = read_vector(bytes, &mut cursor)?;
    let barometer_temperature = read_f16_to_f64(bytes, &mut cursor)?;
    let barometer_pressure = read_f16_to_f64(bytes, &mut cursor)?;

    // GPS is guarded by a compact flags byte that carries both presence and a
    // few optional subfields.
    let gps_flags = read_u8(bytes, &mut cursor)?;
    let gps_valid = gps_flags & (1 << 1) != 0;
    let gps = if gps_flags & (1 << 0) != 0 {
      let latitude_deg = read_f16_to_f64(bytes, &mut cursor)?;
      let longitude_deg = read_f16_to_f64(bytes, &mut cursor)?;
      let altitude_m = read_f16_to_f64(bytes, &mut cursor)?;
      let north_mps = read_f16_to_f64(bytes, &mut cursor)?;
      let east_mps = read_f16_to_f64(bytes, &mut cursor)?;
      let down_mps = read_f16_to_f64(bytes, &mut cursor)?;
      let timestamp_unix_ms = if gps_flags & (1 << 3) != 0 {
        Some(read_i64_le(bytes, &mut cursor)?)
      } else {
        None
      };
      let num_satellites = read_u8(bytes, &mut cursor)?;

      Some(GpsState {
        latitude_deg,
        longitude_deg,
        altitude_m,
        north_mps,
        east_mps,
        down_mps,
        timestamp_unix_ms,
        has_fix: gps_flags & (1 << 2) != 0,
        num_satellites,
      })
    } else {
      None
    };

    // RECO follows the same pattern: a flags byte and then, if present, one
    // aggregated payload that is duplicated back into all three slots.
    let reco_flags = read_u8(bytes, &mut cursor)?;
    let reco_valid = reco_flags & (1 << 0) != 0;
    let reco = if reco_flags & (1 << 1) != 0 {
      let reco = read_reco_state(bytes, &mut cursor)?;
      [Some(reco.clone()), Some(reco.clone()), Some(reco)]
    } else {
      [None, None, None]
    };

    let valve_count = usize::from(read_u8(bytes, &mut cursor)?);
    let sensor_count = usize::from(read_u8(bytes, &mut cursor)?);

    // The explicit counts must match the caller-provided schema exactly.
    if valve_count != sorted_valve_keys.len()
      || sensor_count != sorted_sensor_metadata.len()
    {
      if valve_count != sorted_valve_keys.len() {
        return Err(VehicleStateCompressionError::ValveCountMismatch);
      }
      return Err(VehicleStateCompressionError::SensorCountMismatch);
    }

    // Rehydrate both keyless maps in the same sorted order used during
    // compression.
    let mut valve_states = HashMap::with_capacity(valve_count);
    for valve_name in sorted_valve_keys {
      let packed = read_u8(bytes, &mut cursor)?;
      let commanded = decode_valve_state(packed & 0b111)?;
      let actual = decode_valve_state((packed >> 3) & 0b111)?;
      valve_states.insert(
        valve_name.to_string(),
        CompositeValveState { commanded, actual },
      );
    }

    let mut sensor_readings = HashMap::with_capacity(sensor_count);
    for (sensor_name, unit) in sorted_sensor_metadata {
      sensor_readings.insert(
        sensor_name.to_string(),
        Measurement {
          value: read_f16_to_f64(bytes, &mut cursor)?,
          unit,
        },
      );
    }

    // Any leftover bytes indicate the caller did not pass the exact payload
    // slice produced by `compress`.
    if cursor != bytes.len() {
      return Err(VehicleStateCompressionError::TrailingBytes);
    }

    // Fields omitted from the manual format are restored with their documented
    // defaults so the returned `VehicleState` remains fully populated.
    Ok(VehicleState {
      valve_states,
      bms: Bms {
        battery_bus,
        umbilical_bus: bms::Bus::default(),
        sam_power_bus,
        five_volt_rail,
        charger: 0.0,
        chassis,
        e_stop,
        rbf_tag,
      },
      ahrs: Ahrs {
        rail_3v3,
        rail_5v,
        imu: ahrs::Imu {
          accelerometer,
          gyroscope,
        },
        magnetometer,
        barometer: ahrs::Barometer {
          temperature: barometer_temperature,
          pressure: barometer_pressure,
        },
      },
      gps,
      gps_valid,
      reco,
      reco_valid,
      sensor_readings,
      rolling: HashMap::new(),
      abort_stage: AbortStage {
        name: "default".to_string(),
        abort_condition: String::new(),
        aborted: false,
        valve_safe_states: HashMap::new(),
      },
    })
  }
}

#[cfg(test)]
mod tests {
  use super::{super::sam::Unit, *};

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
      reco: [
        Some(RecoState::default()),
        Some(RecoState::default()),
        Some(RecoState::default()),
      ],
      rolling: Default::default(),
      ..VehicleState::default()
    }
  }

  #[test]
  fn manual_compress_vehicle_state_returns_written_size() {
    let state = vehicle_state_with_counts(10, 12);
    let mut buf = [0u8; 2048];

    let size = state
      .compress(&mut buf)
      .expect("manual vehicle state compression should succeed");

    assert!(size > 0, "manual compression should write some bytes");
    assert!(
      size <= buf.len(),
      "reported size should fit within the provided buffer"
    );
  }

  #[test]
  fn manual_compress_vehicle_state_reconstructs_expected_lossy_state() {
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
        quantize_f32_slice(values);
      }
      reco.temperature = half_roundtrip_f32(reco.temperature);
      reco.pressure = half_roundtrip_f32(reco.pressure);
      reco
    }

    fn average_reco_states_for_test(
      reco: &[Option<RecoState>; 3],
    ) -> Option<RecoState> {
      let present: Vec<_> = reco.iter().flatten().collect();
      if present.is_empty() {
        return None;
      }

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
        present.iter().map(|reco| select(reco)).sum::<f32>()
          / present.len() as f32
      }

      Some(RecoState {
        quaternion: average_array(&present, |reco| reco.quaternion),
        lla_pos: average_array(&present, |reco| reco.lla_pos),
        velocity: average_array(&present, |reco| reco.velocity),
        g_bias: average_array(&present, |reco| reco.g_bias),
        a_bias: average_array(&present, |reco| reco.a_bias),
        g_sf: average_array(&present, |reco| reco.g_sf),
        a_sf: average_array(&present, |reco| reco.a_sf),
        lin_accel: average_array(&present, |reco| reco.lin_accel),
        angular_rate: average_array(&present, |reco| reco.angular_rate),
        mag_data: average_array(&present, |reco| reco.mag_data),
        temperature: average_scalar(&present, |reco| reco.temperature),
        pressure: average_scalar(&present, |reco| reco.pressure),
        stage1_enabled: present.iter().any(|reco| reco.stage1_enabled),
        stage2_enabled: present.iter().any(|reco| reco.stage2_enabled),
        vref_a_stage1: present.iter().any(|reco| reco.vref_a_stage1),
        vref_a_stage2: present.iter().any(|reco| reco.vref_a_stage2),
        vref_b_stage1: present.iter().any(|reco| reco.vref_b_stage1),
        vref_b_stage2: present.iter().any(|reco| reco.vref_b_stage2),
        vref_c_stage1: present.iter().any(|reco| reco.vref_c_stage1),
        vref_c_stage2: present.iter().any(|reco| reco.vref_c_stage2),
        vref_d_stage1: present.iter().any(|reco| reco.vref_d_stage1),
        vref_d_stage2: present.iter().any(|reco| reco.vref_d_stage2),
        vref_e_stage1_1: present.iter().any(|reco| reco.vref_e_stage1_1),
        vref_e_stage1_2: present.iter().any(|reco| reco.vref_e_stage1_2),
        reco_recvd_launch: present.iter().any(|reco| reco.reco_recvd_launch),
        fault_driver_a: present.iter().any(|reco| reco.fault_driver_a),
        fault_driver_b: present.iter().any(|reco| reco.fault_driver_b),
        fault_driver_c: present.iter().any(|reco| reco.fault_driver_c),
        fault_driver_d: present.iter().any(|reco| reco.fault_driver_d),
        fault_driver_e: present.iter().any(|reco| reco.fault_driver_e),
        ekf_blown_up: present.iter().any(|reco| reco.ekf_blown_up),
      })
    }

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
    state.ahrs.rail_3v3.voltage = 3.31;
    state.ahrs.rail_3v3.current = 0.8;
    state.ahrs.rail_5v.voltage = 5.02;
    state.ahrs.rail_5v.current = 1.1;
    state.ahrs.imu.accelerometer = super::ahrs::Vector {
      x: 1.1,
      y: -2.2,
      z: 3.3,
    };
    state.ahrs.imu.gyroscope = super::ahrs::Vector {
      x: -0.1,
      y: 0.2,
      z: -0.3,
    };
    state.ahrs.magnetometer = super::ahrs::Vector {
      x: 0.4,
      y: 0.5,
      z: 0.6,
    };
    state.ahrs.barometer.temperature = 289.4;
    state.ahrs.barometer.pressure = 101_325.0;
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
    state.reco = [
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
        vref_a_stage1: true,
        fault_driver_c: true,
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
        vref_b_stage2: true,
        fault_driver_d: true,
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
        vref_e_stage1_2: true,
        reco_recvd_launch: true,
        ekf_blown_up: true,
        ..RecoState::default()
      }),
    ];

    let mut buf = [0u8; 2048];
    let size = state
      .compress(&mut buf)
      .expect("manual vehicle state compression should succeed");

    let mut valve_keys: Vec<_> = state.valve_states.keys().cloned().collect();
    valve_keys.sort();

    let mut serialized_sensor_metadata: Vec<_> = state
      .sensor_readings
      .iter()
      .filter(|(sensor_name, _)| {
        sensor_name
          .strip_suffix("_V")
          .or_else(|| sensor_name.strip_suffix("_I"))
          .is_none_or(|valve_name| !state.valve_states.contains_key(valve_name))
      })
      .map(|(sensor_name, measurement)| (sensor_name.clone(), measurement.unit))
      .collect();
    serialized_sensor_metadata
      .sort_unstable_by(|(left, _), (right, _)| left.cmp(right));

    let sensor_keys: Vec<_> = serialized_sensor_metadata
      .iter()
      .map(|(sensor_name, _)| sensor_name.clone())
      .collect();
    let sensor_units: Vec<_> = serialized_sensor_metadata
      .iter()
      .map(|(_, unit)| *unit)
      .collect();

    let decoded = VehicleState::decompress(
      &buf[..size],
      &sensor_keys,
      &sensor_units,
      &valve_keys,
    )
    .expect("manual vehicle state decompression should succeed");

    let mut expected = VehicleState::default();
    expected.valve_states = state.valve_states.clone();
    expected.sensor_readings = sensor_keys
      .iter()
      .map(|sensor_name| {
        (
          sensor_name.clone(),
          Measurement {
            value: half_roundtrip_f64(
              state.sensor_readings.get(sensor_name).unwrap().value,
            ),
            unit: *sensor_units
              .iter()
              .zip(sensor_keys.iter())
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
    expected.ahrs.rail_3v3.voltage =
      half_roundtrip_f64(state.ahrs.rail_3v3.voltage);
    expected.ahrs.rail_3v3.current =
      half_roundtrip_f64(state.ahrs.rail_3v3.current);
    expected.ahrs.rail_5v.voltage =
      half_roundtrip_f64(state.ahrs.rail_5v.voltage);
    expected.ahrs.rail_5v.current =
      half_roundtrip_f64(state.ahrs.rail_5v.current);
    expected.ahrs.imu.accelerometer = super::ahrs::Vector {
      x: half_roundtrip_f64(state.ahrs.imu.accelerometer.x),
      y: half_roundtrip_f64(state.ahrs.imu.accelerometer.y),
      z: half_roundtrip_f64(state.ahrs.imu.accelerometer.z),
    };
    expected.ahrs.imu.gyroscope = super::ahrs::Vector {
      x: half_roundtrip_f64(state.ahrs.imu.gyroscope.x),
      y: half_roundtrip_f64(state.ahrs.imu.gyroscope.y),
      z: half_roundtrip_f64(state.ahrs.imu.gyroscope.z),
    };
    expected.ahrs.magnetometer = super::ahrs::Vector {
      x: half_roundtrip_f64(state.ahrs.magnetometer.x),
      y: half_roundtrip_f64(state.ahrs.magnetometer.y),
      z: half_roundtrip_f64(state.ahrs.magnetometer.z),
    };
    expected.ahrs.barometer.temperature =
      half_roundtrip_f64(state.ahrs.barometer.temperature);
    expected.ahrs.barometer.pressure =
      half_roundtrip_f64(state.ahrs.barometer.pressure);
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
    let aggregated_reco =
      quantize_reco(average_reco_states_for_test(&state.reco).unwrap());
    expected.reco = [
      Some(aggregated_reco.clone()),
      Some(aggregated_reco.clone()),
      Some(aggregated_reco),
    ];

    assert_eq!(decoded, expected);
  }

  #[test]
  fn manual_compress_vehicle_state_errors_on_small_buffer() {
    let state = vehicle_state_with_counts(10, 12);
    let mut buf = [0u8; 1];

    assert_eq!(
      state.compress(&mut buf),
      Err(VehicleStateCompressionError::BufferTooSmall)
    );
  }
}
