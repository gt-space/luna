//! Messages between the flight computer (FC) and the engine controller (EC).
//!
//! Byte-for-byte the same format as the C version
//! (`engine-controller/ec_logic/inc/ec_protocol.h`) and the FC side
//! (`common/src/comm/ec.rs`): fixed little-endian layout with a CRC-16 at the
//! end. All three test suites check the same byte vectors, so whichever EC
//! language the team picks, the FC code does not change.
//!
//! Demand (40 bytes): magic, version, type, mode, seq, `tvc_mm[2]`,
//! `rcs_duty[4]`, throttle, CRC. Status (148 bytes): see
//! [`EcStatus::encode`].
//!
//! The EC keeps four versions of every output separate, because they are not
//! the same thing: **requested** (FC asked), **applied** (EC output after
//! limits), **measured** (a sensor said), **inferred** (guessed health).
//! The PA-12 accepting a position command does not prove it got there.

use crate::config;

/// `"EC"` in little-endian.
pub const MAGIC: u16 = 0x4345;
/// Bump whenever either message changes, here, in C and in `common`.
pub const PROTOCOL_VERSION: u16 = 1;
const MSG_DEMAND: u8 = 1;
const MSG_STATUS: u8 = 2;
/// Encoded size of an [`EcDemand`].
pub const DEMAND_LEN: usize = 40;
/// Encoded size of an [`EcStatus`].
pub const STATUS_LEN: usize = 148;

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// What the FC wants the EC to be doing overall.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum RequestedMode {
    /// Everything off. Also how the FC acknowledges a fault so the EC may
    /// leave its safed state.
    Disarm = 0,
    /// Outputs live: fans, ESC and actuators follow the demand.
    Arm = 1,
    /// Abort: safe everything right now.
    Safe = 2,
}

/// FC -> EC, sent every control cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EcDemand {
    /// Increases by one every message. Lets the EC drop duplicates and
    /// out-of-order packets.
    pub seq: u32,
    /// Overall mode.
    pub mode: RequestedMode,
    /// TVC actuator rod extension in mm, `[A1, A2]`.
    pub tvc_extension_mm: [f32; 2],
    /// Fan on-time fraction `0..=1`, `[M1, M2, M3, M4]`. This is NOT speed.
    pub rcs_duty: [f32; 4],
    /// Main propeller throttle `0..=1`. One value: both ESCs share one wire.
    pub main_throttle: f32,
}

/// Why a demand was rejected. A rejected demand changes nothing and does not
/// count as a sign of life from the FC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DemandError {
    /// Wrong length, magic, message type or CRC.
    Malformed,
    /// Sent by FC code built against a different protocol version.
    WrongVersion,
    /// The sequence number did not increase: a duplicate or a stale packet.
    StaleSequence,
    /// A value was NaN or infinite.
    NotFinite,
    /// A value was outside its allowed range.
    OutOfRange,
}

/// A demand that passed every check.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValidDemand(EcDemand);

impl ValidDemand {
    pub fn get(&self) -> &EcDemand {
        &self.0
    }
}

impl EcDemand {
    /// Range and finite checks. Framing and version are checked by
    /// [`EcDemand::decode`]; sequence order by the supervisor.
    pub fn validate(self) -> Result<ValidDemand, DemandError> {
        let values = self
            .tvc_extension_mm
            .iter()
            .chain(self.rcs_duty.iter())
            .chain(core::iter::once(&self.main_throttle));
        for value in values {
            if !value.is_finite() {
                return Err(DemandError::NotFinite);
            }
        }

        let tvc_ok = self
            .tvc_extension_mm
            .iter()
            .all(|mm| (0.0..=config::pa12::STROKE_MM).contains(mm));
        let rcs_ok = self.rcs_duty.iter().all(|d| (0.0..=1.0).contains(d));
        let throttle_ok = (0.0..=1.0).contains(&self.main_throttle);
        if !(tvc_ok && rcs_ok && throttle_ok) {
            return Err(DemandError::OutOfRange);
        }

        Ok(ValidDemand(self))
    }

    /// Decodes and checks framing (length, magic, type, CRC) and version.
    /// Call [`EcDemand::validate`] for value ranges.
    pub fn decode(bytes: &[u8]) -> Result<Self, DemandError> {
        let mut r = Reader::new(bytes, DEMAND_LEN, MSG_DEMAND)?;
        let mode = match r.u8() {
            0 => RequestedMode::Disarm,
            1 => RequestedMode::Arm,
            2 => RequestedMode::Safe,
            _ => return Err(DemandError::OutOfRange),
        };
        Ok(Self {
            mode,
            seq: r.u32(),
            tvc_extension_mm: [r.f32(), r.f32()],
            rcs_duty: [r.f32(), r.f32(), r.f32(), r.f32()],
            main_throttle: r.f32(),
        })
    }

    /// Encodes the exact bytes the FC sends (used in tests and simulation).
    pub fn encode(&self) -> [u8; DEMAND_LEN] {
        let mut w = Writer::<DEMAND_LEN>::new();
        w.u16(MAGIC);
        w.u16(PROTOCOL_VERSION);
        w.u8(MSG_DEMAND);
        w.u8(self.mode as u8);
        w.u32(self.seq);
        self.tvc_extension_mm.iter().for_each(|v| w.f32(*v));
        self.rcs_duty.iter().for_each(|v| w.f32(*v));
        w.f32(self.main_throttle);
        w.finish()
    }
}

/// The EC's overall state, reported back to the FC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EcState {
    /// Just powered on. Outputs held safe while the ESC sees minimum
    /// throttle.
    BootSafe = 0,
    /// Ready, outputs off.
    Disarmed = 1,
    /// ESC is in its minimum-throttle arming hold.
    Arming = 2,
    /// Outputs follow the FC.
    Armed = 3,
    /// Outputs forced safe until the FC sends `RequestedMode::Disarm`.
    Safed = 4,
}

/// Bit flags explaining why the EC is safed or degraded.
pub mod fault {
    /// No valid demand within the timeout.
    pub const LINK_TIMEOUT: u32 = 1 << 0;
    /// FC asked for `RequestedMode::Safe`.
    pub const FC_COMMANDED_SAFE: u32 = 1 << 1;
    /// A fan's current went over the limit. See `RcsReport::faulted`.
    pub const RCS_OVERCURRENT: u32 = 1 << 2;
    /// An actuator stopped answering on its UART.
    pub const TVC_COMM_LOST: u32 = 1 << 3;
    /// An actuator reported its overload bit (latched until restart).
    pub const TVC_OVERLOAD: u32 = 1 << 4;
    /// An actuator reported its input-voltage error bit.
    pub const TVC_VOLTAGE: u32 = 1 << 5;
}

/// Fan block of [`EcStatus`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RcsReport {
    pub requested: [f32; 4],
    pub applied: [f32; 4],
    /// Supply-path current per fan, amps, cycle-averaged.
    pub current_a: [f32; 4],
    /// Bit `n`: fan `n`'s reading is pegged at the sensor's maximum.
    pub current_saturated: u8,
    /// Bit `n`: fan `n` was shut off for overcurrent.
    pub faulted: u8,
}

/// ESC block of [`EcStatus`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EscReport {
    pub requested_throttle: f32,
    pub applied_throttle: f32,
    /// Pulse width on the wire. 0 means no pulses are being sent.
    pub pulse_us: u16,
}

/// One actuator's block of [`EcStatus`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TvcReport {
    pub requested_mm: f32,
    /// Goal position last sent, in the actuator's own counts.
    pub commanded_counts: u16,
    /// Last position the actuator reported, in mm. NaN if none yet.
    pub measured_mm: f32,
    /// Raw error byte from the actuator's last reply.
    pub error_bits: u8,
    pub link_ok: bool,
}

/// One power rail's block of [`EcStatus`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RailReport {
    pub voltage_v: f32,
    pub current_a: f32,
}

/// EC -> FC, sent after each demand.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EcStatus {
    /// Last demand sequence number the EC accepted.
    pub last_seq: u32,
    /// Milliseconds since the EC booted (wraps after ~49 days).
    pub uptime_ms: u32,
    pub state: EcState,
    /// OR of [`fault`] flags.
    pub faults: u32,
    pub rcs: RcsReport,
    pub esc: EscReport,
    /// `[A1, A2]`.
    pub tvc: [TvcReport; 2],
    /// `[22.2 V, 12 V, 4.4 V, 3.3 V]`.
    pub rails: [RailReport; 4],
    /// Shared fan supply voltage.
    pub motor_rail_v: f32,
    /// Board temperatures from the two TMP112s, in degrees C.
    pub board_temp_c: [f32; 2],
}

impl EcStatus {
    /// Encodes the 148-byte status message. Layout (little-endian):
    ///
    /// | offset | size | field |
    /// |---|---|---|
    /// | 0 | 6 | magic, version, type (2), state |
    /// | 6 | 12 | last_seq, uptime_ms, faults (u32 each) |
    /// | 18 | 48 | rcs requested, applied, current (4 x f32 each) |
    /// | 66 | 2 | rcs saturated, faulted |
    /// | 68 | 10 | esc requested, applied (f32), pulse_us (u16) |
    /// | 78 | 24 | tvc x2: requested f32, counts u16, measured f32, error u8, link u8 |
    /// | 102 | 32 | all rail voltages (4 x f32), then all rail currents |
    /// | 134 | 12 | motor_rail_v, board_temp_c[2] (f32) |
    /// | 146 | 2 | CRC-16 |
    pub fn encode(&self) -> [u8; STATUS_LEN] {
        let mut w = Writer::<STATUS_LEN>::new();
        w.u16(MAGIC);
        w.u16(PROTOCOL_VERSION);
        w.u8(MSG_STATUS);
        w.u8(self.state as u8);
        w.u32(self.last_seq);
        w.u32(self.uptime_ms);
        w.u32(self.faults);
        self.rcs.requested.iter().for_each(|v| w.f32(*v));
        self.rcs.applied.iter().for_each(|v| w.f32(*v));
        self.rcs.current_a.iter().for_each(|v| w.f32(*v));
        w.u8(self.rcs.current_saturated);
        w.u8(self.rcs.faulted);
        w.f32(self.esc.requested_throttle);
        w.f32(self.esc.applied_throttle);
        w.u16(self.esc.pulse_us);
        for t in &self.tvc {
            w.f32(t.requested_mm);
            w.u16(t.commanded_counts);
            w.f32(t.measured_mm);
            w.u8(t.error_bits);
            w.u8(t.link_ok as u8);
        }
        self.rails.iter().for_each(|r| w.f32(r.voltage_v));
        self.rails.iter().for_each(|r| w.f32(r.current_a));
        w.f32(self.motor_rail_v);
        self.board_temp_c.iter().for_each(|v| w.f32(*v));
        w.finish()
    }
}

struct Writer<const N: usize> {
    buf: [u8; N],
    at: usize,
}

impl<const N: usize> Writer<N> {
    fn new() -> Self {
        Self { buf: [0; N], at: 0 }
    }

    fn bytes(&mut self, b: &[u8]) {
        self.buf[self.at..self.at + b.len()].copy_from_slice(b);
        self.at += b.len();
    }

    fn u8(&mut self, v: u8) {
        self.bytes(&[v]);
    }

    fn u16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }

    fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }

    fn f32(&mut self, v: f32) {
        self.bytes(&v.to_le_bytes());
    }

    fn finish(mut self) -> [u8; N] {
        let crc = crc16(&self.buf[..self.at]);
        self.u16(crc);
        debug_assert_eq!(self.at, N);
        self.buf
    }
}

struct Reader<'a> {
    buf: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    fn new(buf: &'a [u8], len: usize, msg_type: u8) -> Result<Self, DemandError> {
        if buf.len() != len {
            return Err(DemandError::Malformed);
        }
        let mut r = Self { buf, at: 0 };
        if r.u16() != MAGIC {
            return Err(DemandError::Malformed);
        }
        let version = r.u16();
        let crc = u16::from_le_bytes([buf[len - 2], buf[len - 1]]);
        if r.u8() != msg_type || crc16(&buf[..len - 2]) != crc {
            return Err(DemandError::Malformed);
        }
        if version != PROTOCOL_VERSION {
            return Err(DemandError::WrongVersion);
        }
        Ok(r)
    }

    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0u8; N];
        out.copy_from_slice(&self.buf[self.at..self.at + N]);
        self.at += N;
        out
    }

    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }

    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }

    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }

    fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.take())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Same bytes as engine-controller/test/test_main.c and
    // common/src/comm/ec.rs.
    const DEMAND_VECTOR: [u8; DEMAND_LEN] = [
        0x45, 0x43, 0x01, 0x00, 0x01, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x41, 0x00,
        0x00, 0x48, 0x41, 0x9A, 0x99, 0x19, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x99, 0x19, 0x3F,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xDE, 0x54,
    ];

    const STATUS_VECTOR: [u8; STATUS_LEN] = [
        0x45, 0x43, 0x01, 0x00, 0x02, 0x03, 0x07, 0x00, 0x00, 0x00, 0x40, 0xE2, 0x01, 0x00, 0x08,
        0x00, 0x00, 0x00, 0x9A, 0x99, 0x19, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x3F, 0xCD, 0xCC, 0xCC,
        0x3E, 0x78, 0x05, 0x00, 0x00, 0x20, 0x41, 0x59, 0x05, 0x00, 0x00, 0x18, 0x41, 0x20, 0x01,
        0x00, 0x00, 0x48, 0x41, 0xAF, 0x06, 0x00, 0x00, 0x40, 0x41, 0x00, 0x00, 0x9A, 0x99, 0xB1,
        0x41, 0x00, 0x00, 0x40, 0x41, 0xCD, 0xCC, 0x8C, 0x40, 0x33, 0x33, 0x53, 0x40, 0x00, 0x00,
        0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A,
        0x99, 0x89, 0x40, 0x00, 0x00, 0xF0, 0x41, 0x00, 0x00, 0xFC, 0x41, 0xFD, 0xD4,
    ];

    pub(crate) fn demand() -> EcDemand {
        EcDemand {
            seq: 7,
            mode: RequestedMode::Arm,
            tvc_extension_mm: [10.0, 12.5],
            rcs_duty: [0.6, 0.0, 0.6, 0.0],
            main_throttle: 0.5,
        }
    }

    fn vector_status() -> EcStatus {
        let rail = |voltage_v, current_a| RailReport {
            voltage_v,
            current_a,
        };
        EcStatus {
            last_seq: 7,
            uptime_ms: 123_456,
            state: EcState::Armed,
            faults: fault::TVC_COMM_LOST,
            rcs: RcsReport {
                requested: [0.6, 0.0, 0.0, 0.0],
                applied: [0.5, 0.0, 0.0, 0.0],
                current_a: [0.25, 0.0, 0.0, 0.0],
                current_saturated: 0,
                faulted: 2,
            },
            esc: EscReport {
                requested_throttle: 0.5,
                applied_throttle: 0.4,
                pulse_us: 1400,
            },
            tvc: [
                TvcReport {
                    requested_mm: 10.0,
                    commanded_counts: 1369,
                    measured_mm: 9.5,
                    error_bits: 0x20,
                    link_ok: true,
                },
                TvcReport {
                    requested_mm: 12.5,
                    commanded_counts: 1711,
                    measured_mm: 12.0,
                    error_bits: 0,
                    link_ok: false,
                },
            ],
            rails: [
                rail(22.2, 1.5),
                rail(12.0, 0.0),
                rail(4.4, 0.0),
                rail(3.3, 0.0),
            ],
            motor_rail_v: 4.3,
            board_temp_c: [30.0, 31.5],
        }
    }

    #[test]
    fn crc_check_value() {
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn demand_matches_shared_bytes() {
        assert_eq!(demand().encode(), DEMAND_VECTOR);
        assert_eq!(EcDemand::decode(&DEMAND_VECTOR), Ok(demand()));
    }

    #[test]
    fn status_matches_shared_bytes() {
        assert_eq!(vector_status().encode(), STATUS_VECTOR);
    }

    #[test]
    fn good_demand_passes() {
        assert!(demand().validate().is_ok());
    }

    #[test]
    fn framing_errors() {
        assert_eq!(
            EcDemand::decode(&DEMAND_VECTOR[..39]),
            Err(DemandError::Malformed)
        );
        let mut bad = DEMAND_VECTOR;
        bad[20] ^= 1;
        assert_eq!(EcDemand::decode(&bad), Err(DemandError::Malformed));

        let mut v2 = DEMAND_VECTOR;
        v2[2] = 2;
        let crc = crc16(&v2[..DEMAND_LEN - 2]).to_le_bytes();
        v2[DEMAND_LEN - 2..].copy_from_slice(&crc);
        assert_eq!(EcDemand::decode(&v2), Err(DemandError::WrongVersion));
    }

    #[test]
    fn nan_rejected() {
        let mut d = demand();
        d.rcs_duty[2] = f32::NAN;
        assert_eq!(d.validate(), Err(DemandError::NotFinite));
    }

    #[test]
    fn out_of_range_rejected() {
        let mut d = demand();
        d.main_throttle = 1.01;
        assert_eq!(d.validate(), Err(DemandError::OutOfRange));

        let mut d = demand();
        d.rcs_duty[0] = -0.01;
        assert_eq!(d.validate(), Err(DemandError::OutOfRange));

        let mut d = demand();
        d.tvc_extension_mm[1] = config::pa12::STROKE_MM + 0.1;
        assert_eq!(d.validate(), Err(DemandError::OutOfRange));
    }
}
