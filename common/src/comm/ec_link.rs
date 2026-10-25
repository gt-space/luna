//! Every datagram between the CTV flight computer (FC) and engine controller
//! (EC) is a 48-byte [`Header`], a fixed-size payload chosen by the message
//! [`Kind`], and a CRC-32 over both.
//!
//! All fields are little-endian with no padding and are written one at a time
//! (never a struct copied as raw memory), so the EC firmware, which is C, can
//! encode and decode the exact same bytes. Any layout change here has to be
//! made on the EC too.

use std::{error::Error, fmt, net::Ipv4Addr};

use crate::comm::{Measurement, MeasurementQuality, VehicleState, sam::Unit};

use crc::{Crc, CRC_32_ISO_HDLC};

/// UDP port the EC listens on for FC messages.
pub const EC_PORT: u16 = 8378;

/// UDP port the FC listens on for EC messages. Separate from the SAM socket so
/// EC datagrams never reach the postcard decoder.
pub const FC_PORT: u16 = 4574;

/// Static IP address of the EC. Placeholder until the EC firmware has one.
pub const EC_ADDRESS: Ipv4Addr = Ipv4Addr::new(192, 168, 1, 50);

/// First two bytes of every datagram.
pub const MAGIC: [u8; 2] = *b"EC";

/// Protocol version carried in every header.
pub const VERSION: u8 = 4;

/// Length of the header that starts every datagram.
pub const HEADER_LEN: usize = 48;

/// Length of the CRC-32 that ends every datagram.
pub const CRC_LEN: usize = 4;

/// Largest datagram either side sends or accepts.
pub const MAX_DATAGRAM_LEN: usize = 512;

/// Full stroke of the 1.06" PA-12-T, in mm. TVC demands outside `0..=` this
/// are out of physical range.
pub const TVC_STROKE_MM: f32 = 26.9;

/// CRC-32/ISO-HDLC, the zlib/Ethernet CRC-32. Check value for `"123456789"`
/// is `0xCBF43926`.
const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

/// Bits of the channel masks used in demands, status and capabilities.
pub mod channel {
    /// TVC actuator A1.
    pub const TVC_A1: u16 = 1 << 0;
    /// TVC actuator A2.
    pub const TVC_A2: u16 = 1 << 1;
    /// RCS fans M1 to M4, in order.
    pub const RCS: [u16; 4] = [1 << 2, 1 << 3, 1 << 4, 1 << 5];
    /// The shared main ESC signal.
    pub const MAIN_ESC: u16 = 1 << 6;
    /// Every channel the protocol defines.
    pub const ALL: u16 = 0x7F;
}

/// Returns whether sequence number or transaction ID `a` comes after `b`,
/// treating the `u32` as wrapping.
pub fn is_newer(a: u32, b: u32) -> bool {
    (a.wrapping_sub(b) as i32) > 0
}

macro_rules! wire_enum {
    (
        $(#[$meta:meta])*
        pub enum $name:ident: $repr:ty {
            $($(#[$vmeta:meta])* $variant:ident = $value:literal,)+
        }
    ) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
        #[repr($repr)]
        pub enum $name {
            $($(#[$vmeta])* $variant = $value,)+
        }

        impl TryFrom<$repr> for $name {
            type Error = DecodeError;

            fn try_from(value: $repr) -> Result<Self, Self::Error> {
                match value {
                    $($value => Ok(Self::$variant),)+
                    _ => Err(DecodeError::InvalidEnum {
                        field: stringify!($name),
                        value: value.into(),
                    }),
                }
            }
        }
    };
}

wire_enum! {
    /// Message kind. Kinds below `0x80` go FC -> EC, the rest EC -> FC.
    pub enum Kind: u8 {
        /// Opens a session.
        Hello = 0x01,
        /// Gives the EC output authority.
        Arm = 0x02,
        /// Takes output authority away.
        Disarm = 0x03,
        /// Safes the EC immediately and latches it.
        Terminate = 0x04,
        /// Clears the safing latch.
        ClearSafing = 0x05,
        /// Periodic actuator demand. Only the newest one matters.
        Demand = 0x10,
        /// Keeps the link alive without demanding anything.
        Heartbeat = 0x11,
        /// The EC's answer to Arm, Disarm, Terminate or ClearSafing.
        TxnResult = 0x80,
        /// The EC's answer to Hello.
        HelloResult = 0x81,
        /// Periodic EC status and measurements.
        Status = 0x90,
    }
}

impl Kind {
    /// Length of this kind's payload, in bytes.
    pub const fn payload_len(self) -> usize {
        match self {
            Self::Hello => Hello::LEN,
            Self::Arm => Arm::LEN,
            Self::Disarm | Self::ClearSafing | Self::Heartbeat => 0,
            Self::Terminate => Terminate::LEN,
            Self::Demand => Demand::LEN,
            Self::TxnResult => TxnResult::LEN,
            Self::HelloResult => HelloResult::LEN,
            Self::Status => Status::LEN,
        }
    }

    /// The side that sends this kind.
    pub const fn source(self) -> Source {
        if (self as u8) < 0x80 {
            Source::Fc
        } else {
            Source::Ec
        }
    }
}

wire_enum! {
    /// Which side sent a datagram.
    pub enum Source: u8 {
        /// The flight computer.
        Fc = 1,
        /// The engine controller.
        Ec = 2,
    }
}

wire_enum! {
    /// EC output state.
    #[derive(Default)]
    pub enum EcState: u8 {
        /// Just booted; no session and no output authority.
        #[default]
        BootSafe = 0,
        /// Outputs held safe; demands are stored but not applied.
        Disarmed = 1,
        /// Demands drive the outputs.
        Armed = 2,
        /// Safed and latched until ClearSafing.
        Safing = 3,
    }
}

wire_enum! {
    /// Outcome of a transaction.
    pub enum TxnOutcome: u8 {
        /// The EC did what was asked.
        Accepted = 0,
        /// The EC refused; the reason says why.
        Rejected = 1,
    }
}

wire_enum! {
    /// Why something was rejected, or why the EC safed.
    #[derive(Default)]
    pub enum Reason: u16 {
        /// No reason; nothing went wrong.
        #[default]
        None = 0,
        /// The session ID doesn't match the EC's current session.
        BadSession = 1,
        /// The boot ID is from an earlier EC boot.
        BadBootId = 2,
        /// Not allowed in the EC's current state.
        WrongState = 3,
        /// FC and EC config bundles don't match.
        ConfigMismatch = 4,
        /// An arming interlock isn't met.
        NotReady = 5,
        /// The EC is latched in safing.
        SafingLatched = 6,
        /// The protocol version isn't supported.
        UnsupportedVersion = 7,
        /// A channel the EC can't drive was enabled.
        UnsupportedChannel = 8,
        /// The payload length is wrong for the kind.
        BadLength = 9,
        /// A value is outside its physical range.
        OutOfRange = 10,
        /// A value is NaN or infinite.
        NonFinite = 11,
        /// The message is too old.
        Stale = 12,
        /// The transaction queue is full.
        Busy = 13,
        /// No fresh demand arrived in time while armed.
        DemandTimeout = 14,
        /// Nothing valid arrived from the FC in time.
        LinkTimeout = 15,
        /// The EC detected a fault of its own.
        LocalFault = 16,
        /// The FC sent Terminate.
        FcTerminate = 17,
    }
}

/// Header fields the sender chooses. Magic, version, kind, payload length and
/// source are filled in from the message when encoding.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Header {
    /// On a [`TxnResult`]: this repeats the outcome of a duplicate
    /// transaction, which wasn't acted on again.
    pub duplicate: bool,
    /// Random ID the EC picks at every boot. The FC copies the latest one it
    /// has seen, or 0 before it has seen one.
    pub ec_boot_id: u32,
    /// Session picked by the FC in Hello. 0 means no session.
    pub session_id: u32,
    /// Per sender and session, +1 on every message sent, starting at 1.
    pub seq: u32,
    /// Transaction ID for transactions and their results, else 0.
    pub txn_id: u32,
    /// First 4 bytes of the config bundle SHA-256, read as little-endian. For
    /// logs only; never used to decide anything.
    pub bundle_tag: u32,
    /// FC -> EC: the message expires this long after `ec_time_ref_us`. 0 on
    /// EC -> FC.
    pub valid_for_ms: u16,
    /// Sender's own monotonic clock, in microseconds.
    pub sent_time_us: u64,
    /// FC -> EC: EC `sent_time_us` of the newest Status the FC had when it
    /// built this message, which bounds how old the message is. 0 on EC ->
    /// FC.
    pub ec_time_ref_us: u64,
}

/// Outputs for every channel, in physical units.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Outputs {
    /// Rod extension of TVC actuators A1 and A2, in mm.
    pub tvc_extension_mm: [f32; 2],
    /// Duty of RCS fans M1 to M4, from 0 to 1. Duty, not speed.
    pub rcs_duty: [f32; 4],
    /// Main ESC throttle, from 0 to 1. Both ESCs share this signal.
    pub main_throttle: f32,
}

impl Outputs {
    const LEN: usize = 28;

    /// Checks that every value is finite and within its physical range. Each
    /// side may still clamp to tighter limits of its own.
    pub fn check(&self) -> Result<(), Reason> {
        let [a1, a2] = self.tvc_extension_mm;
        let [m1, m2, m3, m4] = self.rcs_duty;
        let throttle = self.main_throttle;

        if ![a1, a2, m1, m2, m3, m4, throttle]
            .iter()
            .all(|v| v.is_finite())
        {
            return Err(Reason::NonFinite);
        }

        let tvc_ok = [a1, a2].iter().all(|v| (0.0..=TVC_STROKE_MM).contains(v));
        let fractions_ok = [m1, m2, m3, m4, throttle]
            .iter()
            .all(|v| (0.0..=1.0).contains(v));

        if tvc_ok && fractions_ok {
            Ok(())
        } else {
            Err(Reason::OutOfRange)
        }
    }

    fn write(&self, w: &mut Writer) {
        w.f32s(&self.tvc_extension_mm);
        w.f32s(&self.rcs_duty);
        w.f32(self.main_throttle);
    }

    fn read(r: &mut Reader) -> Self {
        Self {
            tvc_extension_mm: r.f32s(),
            rcs_duty: r.f32s(),
            main_throttle: r.f32(),
        }
    }
}

/// Opens a session (FC -> EC).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Hello {
    /// Random ID the FC picks at startup.
    pub fc_boot_id: u32,
    /// ID of the current run.
    pub run_id: u32,
    /// How often the FC plans to send demands.
    pub demand_period_ms: u16,
    /// How often the FC plans to send heartbeats.
    pub heartbeat_period_ms: u16,
    /// SHA-256 of the config bundle the FC expects. All zeros = no bundle.
    pub expected_bundle_sha256: [u8; 32],
}

impl Hello {
    const LEN: usize = 48;

    fn write(&self, w: &mut Writer) {
        w.u32(self.fc_boot_id);
        w.u32(self.run_id);
        w.u16(self.demand_period_ms);
        w.u16(self.heartbeat_period_ms);
        w.u32(0);
        w.bytes(&self.expected_bundle_sha256);
    }

    fn read(r: &mut Reader) -> Self {
        let fc_boot_id = r.u32();
        let run_id = r.u32();
        let demand_period_ms = r.u16();
        let heartbeat_period_ms = r.u16();
        r.skip(4);

        Self {
            fc_boot_id,
            run_id,
            demand_period_ms,
            heartbeat_period_ms,
            expected_bundle_sha256: r.take(),
        }
    }
}

/// The EC's answer to [`Hello`] (EC -> FC).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HelloResult {
    /// Whether the session was opened.
    pub outcome: TxnOutcome,
    /// EC state after the Hello.
    pub ec_state: EcState,
    /// Why it was rejected, if it was.
    pub reason: Reason,
    /// Channels the EC can drive (see [`channel`]).
    pub capabilities: u16,
    /// Oldest demand the EC accepts.
    pub max_demand_age_ms: u16,
    /// EC firmware version.
    pub firmware_version: u32,
    /// ID of the EC's active actuator calibration.
    pub calibration_id: u32,
    /// How long the EC waits without a valid FC message before safing.
    pub link_timeout_ms: u16,
    /// SHA-256 of the EC's active config bundle. All zeros = no bundle.
    pub active_bundle_sha256: [u8; 32],
}

impl HelloResult {
    const LEN: usize = 52;

    fn write(&self, w: &mut Writer) {
        w.u8(self.outcome as u8);
        w.u8(self.ec_state as u8);
        w.u16(self.reason as u16);
        w.u16(self.capabilities);
        w.u16(self.max_demand_age_ms);
        w.u32(self.firmware_version);
        w.u32(self.calibration_id);
        w.u16(self.link_timeout_ms);
        w.u16(0);
        w.bytes(&self.active_bundle_sha256);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        let outcome = TxnOutcome::try_from(r.u8())?;
        let ec_state = EcState::try_from(r.u8())?;
        let reason = Reason::try_from(r.u16())?;
        let capabilities = r.u16();
        let max_demand_age_ms = r.u16();
        let firmware_version = r.u32();
        let calibration_id = r.u32();
        let link_timeout_ms = r.u16();
        r.skip(2);

        Ok(Self {
            outcome,
            ec_state,
            reason,
            capabilities,
            max_demand_age_ms,
            firmware_version,
            calibration_id,
            link_timeout_ms,
            active_bundle_sha256: r.take(),
        })
    }
}

/// Gives the EC output authority (FC -> EC).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Arm {
    /// ID of the current run.
    pub run_id: u32,
    /// SHA-256 of the config bundle the FC expects. All zeros = no bundle.
    pub expected_bundle_sha256: [u8; 32],
}

impl Arm {
    const LEN: usize = 36;

    fn write(&self, w: &mut Writer) {
        w.u32(self.run_id);
        w.bytes(&self.expected_bundle_sha256);
    }

    fn read(r: &mut Reader) -> Self {
        Self {
            run_id: r.u32(),
            expected_bundle_sha256: r.take(),
        }
    }
}

/// Safes the EC immediately and latches it (FC -> EC).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Terminate {
    /// Why the FC is terminating.
    pub reason: Reason,
}

impl Terminate {
    const LEN: usize = 4;

    fn write(&self, w: &mut Writer) {
        w.u16(self.reason as u16);
        w.u16(0);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        let reason = Reason::try_from(r.u16())?;
        r.skip(2);
        Ok(Self { reason })
    }
}

/// Periodic actuator demand (FC -> EC). Its command ID is the header `seq`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Demand {
    /// Requested outputs.
    pub outputs: Outputs,
    /// Channels this demand drives (see [`channel`]).
    pub enable_mask: u16,
}

impl Demand {
    const LEN: usize = Outputs::LEN + 4;

    /// Checks the demand the way the EC does before accepting it: finite, in
    /// physical range, and only enabling channels in `capabilities`.
    pub fn check(&self, capabilities: u16) -> Result<(), Reason> {
        self.outputs.check()?;

        if self.enable_mask & !capabilities != 0 {
            return Err(Reason::UnsupportedChannel);
        }

        Ok(())
    }

    fn write(&self, w: &mut Writer) {
        self.outputs.write(w);
        w.u16(self.enable_mask);
        w.u16(0);
    }

    fn read(r: &mut Reader) -> Self {
        let outputs = Outputs::read(r);
        let enable_mask = r.u16();
        r.skip(2);
        Self {
            outputs,
            enable_mask,
        }
    }
}

/// The EC's answer to Arm, Disarm, Terminate or ClearSafing (EC -> FC).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxnResult {
    /// Kind of the request this answers.
    pub request_kind: Kind,
    /// Whether the EC did it.
    pub outcome: TxnOutcome,
    /// Why it was rejected, if it was.
    pub reason: Reason,
    /// EC state after the request.
    pub ec_state: EcState,
}

impl TxnResult {
    const LEN: usize = 8;

    fn write(&self, w: &mut Writer) {
        w.u8(self.request_kind as u8);
        w.u8(self.outcome as u8);
        w.u16(self.reason as u16);
        w.u8(self.ec_state as u8);
        w.bytes(&[0; 3]);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        let request_kind = Kind::try_from(r.u8())?;
        let outcome = TxnOutcome::try_from(r.u8())?;
        let reason = Reason::try_from(r.u16())?;
        let ec_state = EcState::try_from(r.u8())?;
        r.skip(3);

        Ok(Self {
            request_kind,
            outcome,
            reason,
            ec_state,
        })
    }
}

/// Link health counters reported in [`Status`]. Each wraps at `u16::MAX`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    /// Datagrams dropped for a bad magic, version, length or CRC.
    pub crc_errors: u16,
    /// Messages rejected after decoding.
    pub rejected: u16,
    /// Messages rejected as too old.
    pub stale: u16,
    /// Demands dropped as older than or equal to the last accepted one.
    pub old_or_duplicate: u16,
    /// Transactions refused because the queue was full.
    pub txn_queue_overflow: u16,
    /// Diagnostics dropped because their buffer was full.
    pub diag_overflow: u16,
    /// EC loop iterations that missed their deadline.
    pub deadline_miss: u16,
    /// Motion goals replaced by newer goals before transmission.
    pub motion_coalesced: u16,
    /// Ordinary motion refused by the EC safety/validation authority.
    pub motion_rejected: u16,
}

impl Counters {
    fn write(&self, w: &mut Writer) {
        w.u16s(&[
            self.crc_errors,
            self.rejected,
            self.stale,
            self.old_or_duplicate,
            self.txn_queue_overflow,
            self.diag_overflow,
            self.deadline_miss,
            self.motion_coalesced,
            self.motion_rejected,
        ]);
    }

    fn read(r: &mut Reader) -> Self {
        let [crc_errors, rejected, stale, old_or_duplicate, txn_queue_overflow, diag_overflow, deadline_miss, motion_coalesced, motion_rejected] =
            r.u16s();

        Self {
            crc_errors,
            rejected,
            stale,
            old_or_duplicate,
            txn_queue_overflow,
            diag_overflow,
            deadline_miss,
            motion_coalesced,
            motion_rejected,
        }
    }
}

/// EC measurements reported in [`Status`]. A value only means something if
/// its bit in `validity_mask` is set.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Measurements {
    /// Increments with every new set of samples.
    pub sample_id: u32,
    /// EC time the samples were taken, in microseconds.
    pub sample_time_us: u64,
    /// Bit per value: 0-1 TVC position, 2-5 rail voltage, 6-9 rail current,
    /// 10-13 RCS current, 14 motor rail voltage, 15-16 board temperature.
    pub validity_mask: u32,
    /// Acquisition timestamp for each value in the documented 17-value order.
    pub measurement_sample_time_us: [u64; 17],
    /// Per-value quality code from `MeasurementQuality` wire values.
    pub measurement_quality: [u8; 17],
    /// Raw ADC codes in board channel order (vin, iin, v12, i12, v4v4, i4v4,
    /// v3v3, i3v3, im1..im4, vmotor).
    pub adc_raw_code: [u16; 13],
    /// TVC actuator positions from PA-12 feedback, in mm.
    pub tvc_position_mm: [f32; 2],
    /// Raw PA-12 position counts retained for actuator diagnosis.
    pub tvc_position_counts: [u16; 2],
    /// Age of each TVC position reading, in ms.
    pub tvc_position_age_ms: [u16; 2],
    /// PA-12 hardware error flags for A1 and A2.
    pub pa12_fault_flags: [u16; 2],
    /// Host request IDs of the latest position commands sent to A1 and A2.
    pub pa12_command_request_id: [u32; 2],
    /// Host request IDs of the readback requests associated with positions.
    pub pa12_feedback_request_id: [u32; 2],
    /// Per-actuator command status: 0 none, 1 queued, 2 sent without ACK,
    /// 3 transmit failed.
    pub pa12_command_state: [u8; 2],
    /// Per-actuator readback state: 0 none, 1 pending, 2 valid reply,
    /// 3 valid reply with fault flags, 4 timeout, 5 invalid reply,
    /// 6 transport error.
    pub pa12_feedback_state: [u8; 2],
    /// Input, 12 V, 4.4 V and 3.3 V rail voltages, in V.
    pub rail_voltage_v: [f32; 4],
    /// Currents of the same rails, in A.
    pub rail_current_a: [f32; 4],
    /// Low-side current of RCS fans M1 to M4, in A.
    pub rcs_current_a: [f32; 4],
    /// Shared motor rail voltage, in V.
    pub motor_rail_voltage_v: f32,
    /// Board temperatures from U3 and U4, in degrees C.
    pub board_temp_c: [f32; 2],
    /// TMP112 status flags: bit 0/1 U3/U4 read failure, bit 2 bus failure,
    /// bit 3 shared ALERT active, bit 4/5 U3/U4 overtemperature,
    /// bit 6 ALERT GPIO read failure.
    pub temperature_status: u8,
}

/// Wire values for `Measurements::pa12_command_state`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Pa12CommandState {
    /// No command has been issued in this session.
    #[default]
    None = 0,
    /// Command is queued locally and has not started transmission.
    Queued = 1,
    /// UART write completed; the default PA-12 mode gives no goal ACK.
    SentUnconfirmed = 2,
    /// UART transmission failed after the bounded retry policy.
    TxFailed = 3,
}

/// Wire values for `Measurements::pa12_feedback_state`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Pa12FeedbackState {
    /// No readback request has been issued in this session.
    #[default]
    None = 0,
    /// Readback is in flight.
    Pending = 1,
    /// Valid device feedback was decoded.
    ValidReply = 2,
    /// Valid feedback was decoded and its hardware fault byte is nonzero.
    ValidReplyWithFault = 3,
    /// Reply deadline expired.
    Timeout = 4,
    /// A reply arrived but failed packet/ID/data validation.
    InvalidReply = 5,
    /// UART/transport failed.
    TransportError = 6,
}

impl Measurements {
    fn write(&self, w: &mut Writer) {
        w.u32(self.sample_id);
        w.u64(self.sample_time_us);
        w.u32(self.validity_mask);
        w.u64s(&self.measurement_sample_time_us);
        w.u8s(&self.measurement_quality);
        w.u16s(&self.adc_raw_code);
        w.f32s(&self.tvc_position_mm);
        w.u16s(&self.tvc_position_counts);
        w.u16s(&self.tvc_position_age_ms);
        w.u16s(&self.pa12_fault_flags);
        w.u32s(&self.pa12_command_request_id);
        w.u32s(&self.pa12_feedback_request_id);
        w.u8s(&self.pa12_command_state);
        w.u8s(&self.pa12_feedback_state);
        w.f32s(&self.rail_voltage_v);
        w.f32s(&self.rail_current_a);
        w.f32s(&self.rcs_current_a);
        w.f32(self.motor_rail_voltage_v);
        w.f32s(&self.board_temp_c);
        w.u8(self.temperature_status);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        let sample_id = r.u32();
        let sample_time_us = r.u64();
        let validity_mask = r.u32();
        let measurement_sample_time_us = r.u64s();
        let measurement_quality = r.u8s();
        if let Some(&value) = measurement_quality.iter().find(|&&value| value > 11) {
            return Err(DecodeError::InvalidEnum {
                field: "MeasurementQuality",
                value: u32::from(value),
            });
        }
        Ok(Self {
            sample_id,
            sample_time_us,
            validity_mask,
            measurement_sample_time_us,
            measurement_quality,
            adc_raw_code: r.u16s(),
            tvc_position_mm: r.f32s(),
            tvc_position_counts: r.u16s(),
            tvc_position_age_ms: r.u16s(),
            pa12_fault_flags: r.u16s(),
            pa12_command_request_id: r.u32s(),
            pa12_feedback_request_id: r.u32s(),
            pa12_command_state: r.u8s(),
            pa12_feedback_state: r.u8s(),
            rail_voltage_v: r.f32s(),
            rail_current_a: r.f32s(),
            rcs_current_a: r.f32s(),
            motor_rail_voltage_v: r.f32(),
            board_temp_c: r.f32s(),
            temperature_status: r.u8(),
        })
    }
}

impl Measurements {
    /// Merges EC measurements into the existing named `VehicleState` sensor
    /// map. EC metadata stays nested in each `Measurement`, so no additional
    /// top-level vehicle fields are introduced.
    pub fn merge_into_vehicle_state(&self, state: &mut VehicleState) {
        let values = [
            ("ec.tvc.a1.position", self.tvc_position_mm[0] as f64, Unit::Millimeters, 0usize, Some(self.tvc_position_counts[0])),
            ("ec.tvc.a2.position", self.tvc_position_mm[1] as f64, Unit::Millimeters, 1, Some(self.tvc_position_counts[1])),
            ("ec.vin", self.rail_voltage_v[0] as f64, Unit::Volts, 2, Some(self.adc_raw_code[0])),
            ("ec.v12", self.rail_voltage_v[1] as f64, Unit::Volts, 3, Some(self.adc_raw_code[2])),
            ("ec.v4v4", self.rail_voltage_v[2] as f64, Unit::Volts, 4, Some(self.adc_raw_code[4])),
            ("ec.v3v3", self.rail_voltage_v[3] as f64, Unit::Volts, 5, Some(self.adc_raw_code[6])),
            ("ec.vmotor", self.motor_rail_voltage_v as f64, Unit::Volts, 6, Some(self.adc_raw_code[12])),
            ("ec.iin", self.rail_current_a[0] as f64, Unit::Amps, 7, Some(self.adc_raw_code[1])),
            ("ec.i12", self.rail_current_a[1] as f64, Unit::Amps, 8, Some(self.adc_raw_code[3])),
            ("ec.i4v4", self.rail_current_a[2] as f64, Unit::Amps, 9, Some(self.adc_raw_code[5])),
            ("ec.i3v3", self.rail_current_a[3] as f64, Unit::Amps, 10, Some(self.adc_raw_code[7])),
            ("ec.im1", self.rcs_current_a[0] as f64, Unit::Amps, 11, Some(self.adc_raw_code[8])),
            ("ec.im2", self.rcs_current_a[1] as f64, Unit::Amps, 12, Some(self.adc_raw_code[9])),
            ("ec.im3", self.rcs_current_a[2] as f64, Unit::Amps, 13, Some(self.adc_raw_code[10])),
            ("ec.im4", self.rcs_current_a[3] as f64, Unit::Amps, 14, Some(self.adc_raw_code[11])),
            ("ec.t_u3", self.board_temp_c[0] as f64, Unit::Celsius, 15, None),
            ("ec.t_u4", self.board_temp_c[1] as f64, Unit::Celsius, 16, None),
        ];

        for (name, value, unit, index, raw_count) in values {
            let valid_bit = match index {
                0..=1 => 1u32 << index,
                2..=5 => 1u32 << index,
                6 => 1u32 << 14,
                7..=10 => 1u32 << (index - 1),
                11..=14 => 1u32 << (index - 1),
                15..=16 => 1u32 << index,
                _ => 0,
            };
            let mut quality = measurement_quality(self.measurement_quality[index]);
            if self.validity_mask & valid_bit == 0 && self.measurement_quality[index] == 0 {
                quality = MeasurementQuality::Unconfigured;
            }
            state.sensor_readings.insert(
                name.to_string(),
                Measurement {
                    value,
                    unit,
                    sample_id: self.sample_id,
                    sample_time_us: self.measurement_sample_time_us[index],
                    quality,
                    raw_count: raw_count.map(u32::from),
                },
            );
        }
    }
}

fn measurement_quality(value: u8) -> MeasurementQuality {
    match value {
        1 => MeasurementQuality::Ratiometric,
        2 => MeasurementQuality::Stale,
        3 => MeasurementQuality::SaturatedHigh,
        4 => MeasurementQuality::AtFloor,
        5 => MeasurementQuality::OutOfRange,
        6 => MeasurementQuality::Unsettled,
        7 => MeasurementQuality::NotSampled,
        8 => MeasurementQuality::AdcFault,
        9 => MeasurementQuality::Unconfigured,
        10 => MeasurementQuality::DeviceFault,
        11 => MeasurementQuality::TransportFault,
        _ => MeasurementQuality::Ok,
    }
}

/// Periodic EC status (EC -> FC).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Status {
    /// Current EC state.
    pub ec_state: EcState,
    /// Whether demands are currently driving the outputs.
    pub output_authority: bool,
    /// Whether the EC has a live session with the FC.
    pub session_valid: bool,
    /// Whether the EC is latched in safing.
    pub safing_latched: bool,
    /// Whether the last accepted demand is still fresh.
    pub demand_fresh: bool,
    /// Why the EC safed, if it did.
    pub safing_reason: Reason,
    /// `seq` of the newest valid FC message of any kind.
    pub last_received_seq: u32,
    /// `seq` of the newest demand that passed validation.
    pub last_accepted_demand_seq: u32,
    /// `seq` of the newest demand written to the outputs. Written, not
    /// reached: this says nothing about where the actuators are.
    pub last_applied_demand_seq: u32,
    /// EC time the last applied demand was written, in microseconds.
    pub last_applied_time_us: u64,
    /// Why the last rejected message was rejected.
    pub last_reject_reason: Reason,
    /// Channels clamped by EC limits in the last applied demand.
    pub limited_mask: u16,
    /// Link health counters.
    pub counters: Counters,
    /// Cause of the last EC reset, as the EC reports it.
    pub reset_cause: u8,
    /// EC firmware version.
    pub firmware_version: u32,
    /// Active actuator/measurement calibration configuration identity.
    pub active_config_id: u32,
    /// Driver fault bitmap maintained by EC board support.
    pub driver_faults: u32,
    /// Watchdog state/fault bitmap maintained by EC board support.
    pub watchdog_status: u32,
    /// Outputs actually applied, after EC limits.
    pub applied: Outputs,
    /// EC measurements.
    pub measurements: Measurements,
}

impl Status {
    const LEN: usize = 388;

    const OUTPUT_AUTHORITY: u8 = 1 << 0;
    const SESSION_VALID: u8 = 1 << 1;
    const SAFING_LATCHED: u8 = 1 << 2;
    const DEMAND_FRESH: u8 = 1 << 3;

    fn flags(&self) -> u8 {
        let bits = [
            (self.output_authority, Self::OUTPUT_AUTHORITY),
            (self.session_valid, Self::SESSION_VALID),
            (self.safing_latched, Self::SAFING_LATCHED),
            (self.demand_fresh, Self::DEMAND_FRESH),
        ];

        bits.iter()
            .filter(|(set, _)| *set)
            .fold(0, |flags, (_, bit)| flags | bit)
    }

    fn write(&self, w: &mut Writer) {
        w.u8(self.ec_state as u8);
        w.u8(self.flags());
        w.u16(self.safing_reason as u16);
        w.u32(self.last_received_seq);
        w.u32(self.last_accepted_demand_seq);
        w.u32(self.last_applied_demand_seq);
        w.u64(self.last_applied_time_us);
        w.u16(self.last_reject_reason as u16);
        w.u16(self.limited_mask);
        self.counters.write(w);
        w.u8(self.reset_cause);
        w.u8(0);
        w.u32(self.firmware_version);
        w.u32(self.active_config_id);
        w.u32(self.driver_faults);
        w.u32(self.watchdog_status);
        self.applied.write(w);
        self.measurements.write(w);
    }

    fn read(r: &mut Reader) -> Result<Self, DecodeError> {
        let ec_state = EcState::try_from(r.u8())?;
        let flags = r.u8();
        let safing_reason = Reason::try_from(r.u16())?;
        let last_received_seq = r.u32();
        let last_accepted_demand_seq = r.u32();
        let last_applied_demand_seq = r.u32();
        let last_applied_time_us = r.u64();
        let last_reject_reason = Reason::try_from(r.u16())?;
        let limited_mask = r.u16();
        let counters = Counters::read(r);
        let reset_cause = r.u8();
        r.skip(1);
        let firmware_version = r.u32();
        let active_config_id = r.u32();
        let driver_faults = r.u32();
        let watchdog_status = r.u32();

        Ok(Self {
            ec_state,
            output_authority: flags & Self::OUTPUT_AUTHORITY != 0,
            session_valid: flags & Self::SESSION_VALID != 0,
            safing_latched: flags & Self::SAFING_LATCHED != 0,
            demand_fresh: flags & Self::DEMAND_FRESH != 0,
            safing_reason,
            last_received_seq,
            last_accepted_demand_seq,
            last_applied_demand_seq,
            last_applied_time_us,
            last_reject_reason,
            limited_mask,
            counters,
            reset_cause,
            firmware_version,
            active_config_id,
            driver_faults,
            watchdog_status,
            applied: Outputs::read(r),
            measurements: Measurements::read(r)?,
        })
    }

    /// Merges the EC's named measurements and safety/driver diagnostics into
    /// the existing `VehicleState.sensor_readings` map for the FC DAQ path.
    pub fn merge_into_vehicle_state(&self, state: &mut VehicleState) {
        self.measurements.merge_into_vehicle_state(state);
        let sample_id = self.measurements.sample_id;
        let sample_time_us = self.measurements.sample_time_us;
        let mut put = |name: &str, value: f64| {
            state.sensor_readings.insert(
                name.to_string(),
                Measurement {
                    value,
                    unit: Unit::Counts,
                    sample_id,
                    sample_time_us,
                    quality: MeasurementQuality::Ok,
                    raw_count: None,
                },
            );
        };

        put("ec.status.output_authority", f64::from(self.output_authority));
        put("ec.status.safing_latched", f64::from(self.safing_latched));
        put("ec.status.demand_fresh", f64::from(self.demand_fresh));
        put("ec.status.state", self.ec_state as u8 as f64);
        put("ec.status.safing_reason", self.safing_reason as u16 as f64);
        put("ec.status.last_reject_reason", self.last_reject_reason as u16 as f64);
        put("ec.status.limited_mask", self.limited_mask as f64);
        put("ec.status.active_config_id", self.active_config_id as f64);
        put("ec.status.driver_faults", self.driver_faults as f64);
        put("ec.status.watchdog_status", self.watchdog_status as f64);
        put("ec.status.reset_cause", self.reset_cause as f64);
        put("ec.status.counter.crc_errors", self.counters.crc_errors as f64);
        put("ec.status.counter.rejected", self.counters.rejected as f64);
        put("ec.status.counter.stale", self.counters.stale as f64);
        put("ec.status.counter.old_or_duplicate", self.counters.old_or_duplicate as f64);
        put("ec.status.counter.txn_queue_overflow", self.counters.txn_queue_overflow as f64);
        put("ec.status.counter.diag_overflow", self.counters.diag_overflow as f64);
        put("ec.status.counter.deadline_miss", self.counters.deadline_miss as f64);
        put("ec.status.counter.motion_coalesced", self.counters.motion_coalesced as f64);
        put("ec.status.counter.motion_rejected", self.counters.motion_rejected as f64);
    }
}

/// The payload of a datagram, which also decides its [`Kind`].
// Status is much bigger than the other variants. Boxing it would add an
// allocation per datagram and lose `Copy`; copying a few hundred bytes at
// 100 Hz costs nothing.
#[allow(clippy::large_enum_variant)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Message {
    /// See [`Hello`].
    Hello(Hello),
    /// See [`Arm`].
    Arm(Arm),
    /// Takes output authority away.
    Disarm,
    /// See [`Terminate`].
    Terminate(Terminate),
    /// Clears the safing latch.
    ClearSafing,
    /// See [`Demand`].
    Demand(Demand),
    /// Keeps the link alive.
    Heartbeat,
    /// See [`TxnResult`].
    TxnResult(TxnResult),
    /// See [`HelloResult`].
    HelloResult(HelloResult),
    /// See [`Status`].
    Status(Status),
}

impl Message {
    /// The kind this message is sent as.
    pub fn kind(&self) -> Kind {
        match self {
            Self::Hello(_) => Kind::Hello,
            Self::Arm(_) => Kind::Arm,
            Self::Disarm => Kind::Disarm,
            Self::Terminate(_) => Kind::Terminate,
            Self::ClearSafing => Kind::ClearSafing,
            Self::Demand(_) => Kind::Demand,
            Self::Heartbeat => Kind::Heartbeat,
            Self::TxnResult(_) => Kind::TxnResult,
            Self::HelloResult(_) => Kind::HelloResult,
            Self::Status(_) => Kind::Status,
        }
    }

    fn write(&self, w: &mut Writer) {
        match self {
            Self::Hello(hello) => hello.write(w),
            Self::Arm(arm) => arm.write(w),
            Self::Terminate(terminate) => terminate.write(w),
            Self::Demand(demand) => demand.write(w),
            Self::TxnResult(result) => result.write(w),
            Self::HelloResult(result) => result.write(w),
            Self::Status(status) => status.write(w),
            Self::Disarm | Self::ClearSafing | Self::Heartbeat => {}
        }
    }

    fn read(kind: Kind, r: &mut Reader) -> Result<Self, DecodeError> {
        Ok(match kind {
            Kind::Hello => Self::Hello(Hello::read(r)),
            Kind::Arm => Self::Arm(Arm::read(r)),
            Kind::Disarm => Self::Disarm,
            Kind::Terminate => Self::Terminate(Terminate::read(r)?),
            Kind::ClearSafing => Self::ClearSafing,
            Kind::Demand => Self::Demand(Demand::read(r)),
            Kind::Heartbeat => Self::Heartbeat,
            Kind::TxnResult => Self::TxnResult(TxnResult::read(r)?),
            Kind::HelloResult => Self::HelloResult(HelloResult::read(r)?),
            Kind::Status => Self::Status(Status::read(r)?),
        })
    }
}

/// One datagram: a header and a message.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Packet {
    /// Header fields chosen by the sender.
    pub header: Header,
    /// The payload.
    pub message: Message,
}

impl Packet {
    /// Length of this packet once encoded, in bytes.
    pub fn encoded_len(&self) -> usize {
        HEADER_LEN + self.message.kind().payload_len() + CRC_LEN
    }

    /// Encodes this packet into the start of `buf` and returns how many bytes
    /// it used.
    pub fn encode(&self, buf: &mut [u8]) -> Result<usize, EncodeError> {
        let len = self.encoded_len();
        let available = buf.len();
        let Some(buf) = buf.get_mut(..len) else {
            return Err(EncodeError::BufferTooSmall {
                needed: len,
                available,
            });
        };

        let kind = self.message.kind();
        let header = &self.header;
        let body_len = len - CRC_LEN;
        let mut w = Writer::new(&mut buf[..body_len]);

        w.bytes(&MAGIC);
        w.u8(VERSION);
        w.u8(kind as u8);
        w.u16(kind.payload_len() as u16);
        w.u8(kind.source() as u8);
        w.u8(u8::from(header.duplicate));
        w.u32(header.ec_boot_id);
        w.u32(header.session_id);
        w.u32(header.seq);
        w.u32(header.txn_id);
        w.u32(header.bundle_tag);
        w.u16(header.valid_for_ms);
        w.u16(0);
        w.u64(header.sent_time_us);
        w.u64(header.ec_time_ref_us);
        self.message.write(&mut w);
        debug_assert_eq!(w.pos, body_len, "{kind:?} payload is the wrong length");

        let crc = CRC32.checksum(&buf[..body_len]);
        buf[body_len..].copy_from_slice(&crc.to_le_bytes());

        Ok(len)
    }

    /// Decodes one datagram. Only checks the bytes are well formed; checking
    /// the values (sessions, freshness, ranges) is up to the receiver.
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() < HEADER_LEN + CRC_LEN {
            return Err(DecodeError::TooShort { len: bytes.len() });
        }

        if bytes.len() > MAX_DATAGRAM_LEN {
            return Err(DecodeError::TooLong { len: bytes.len() });
        }

        if bytes[..2] != MAGIC {
            return Err(DecodeError::BadMagic);
        }

        if bytes[2] != VERSION {
            return Err(DecodeError::UnsupportedVersion(bytes[2]));
        }

        let (body, crc) = bytes.split_at(bytes.len() - CRC_LEN);
        if crc != CRC32.checksum(body).to_le_bytes() {
            return Err(DecodeError::BadCrc);
        }

        let mut r = Reader::new(body);
        r.skip(3);
        let kind = Kind::try_from(r.u8())?;
        let declared = r.u16();
        let actual = body.len() - HEADER_LEN;

        if usize::from(declared) != kind.payload_len() || actual != kind.payload_len() {
            return Err(DecodeError::BadLength {
                kind,
                declared,
                actual,
            });
        }

        let source = r.u8();
        if source != kind.source() as u8 {
            return Err(DecodeError::WrongSource { kind, source });
        }

        let duplicate = r.u8() & 1 != 0;
        let ec_boot_id = r.u32();
        let session_id = r.u32();
        let seq = r.u32();
        let txn_id = r.u32();
        let bundle_tag = r.u32();
        let valid_for_ms = r.u16();
        r.skip(2);
        let sent_time_us = r.u64();
        let ec_time_ref_us = r.u64();

        let header = Header {
            duplicate,
            ec_boot_id,
            session_id,
            seq,
            txn_id,
            bundle_tag,
            valid_for_ms,
            sent_time_us,
            ec_time_ref_us,
        };

        Ok(Self {
            header,
            message: Message::read(kind, &mut r)?,
        })
    }
}

/// Why a packet couldn't be encoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// The output buffer is shorter than the packet.
    BufferTooSmall {
        /// Bytes the packet needs.
        needed: usize,
        /// Bytes the buffer has.
        available: usize,
    },
}

impl fmt::Display for EncodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferTooSmall { needed, available } => {
                write!(
                    f,
                    "packet needs {needed} bytes but the buffer has {available}"
                )
            }
        }
    }
}

impl Error for EncodeError {}

/// Why a datagram couldn't be decoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// Shorter than a header plus CRC.
    TooShort {
        /// Length of the datagram.
        len: usize,
    },
    /// Longer than [`MAX_DATAGRAM_LEN`].
    TooLong {
        /// Length of the datagram.
        len: usize,
    },
    /// Doesn't start with [`MAGIC`].
    BadMagic,
    /// Carries a version other than [`VERSION`].
    UnsupportedVersion(u8),
    /// The CRC doesn't match the contents.
    BadCrc,
    /// An enum field holds a value with no meaning.
    InvalidEnum {
        /// Name of the enum.
        field: &'static str,
        /// The value received.
        value: u32,
    },
    /// The payload length is wrong for the kind.
    BadLength {
        /// Kind of the datagram.
        kind: Kind,
        /// Payload length the header claims.
        declared: u16,
        /// Payload length actually received.
        actual: usize,
    },
    /// The source byte doesn't match the side that sends this kind.
    WrongSource {
        /// Kind of the datagram.
        kind: Kind,
        /// The source byte received.
        source: u8,
    },
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooShort { len } => write!(f, "datagram is only {len} bytes"),
            Self::TooLong { len } => write!(f, "datagram is {len} bytes, over the limit"),
            Self::BadMagic => write!(f, "datagram doesn't start with the EC link magic"),
            Self::UnsupportedVersion(version) => {
                write!(f, "unsupported protocol version {version}")
            }
            Self::BadCrc => write!(f, "CRC doesn't match"),
            Self::InvalidEnum { field, value } => write!(f, "{value} isn't a valid {field}"),
            Self::BadLength {
                kind,
                declared,
                actual,
            } => write!(
                f,
                "{kind:?} payload should be {} bytes, header says {declared}, got {actual}",
                kind.payload_len()
            ),
            Self::WrongSource { kind, source } => {
                write!(f, "{kind:?} can't come from source {source}")
            }
        }
    }
}

impl Error for DecodeError {}

/// Writes little-endian fields into a buffer that callers have sized exactly.
struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn bytes(&mut self, bytes: &[u8]) {
        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
    }

    fn u8(&mut self, value: u8) {
        self.bytes(&[value]);
    }

    fn u16(&mut self, value: u16) {
        self.bytes(&value.to_le_bytes());
    }

    fn u32(&mut self, value: u32) {
        self.bytes(&value.to_le_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.bytes(&value.to_le_bytes());
    }

    fn f32(&mut self, value: f32) {
        self.bytes(&value.to_le_bytes());
    }

    fn u16s(&mut self, values: &[u16]) {
        for &value in values {
            self.u16(value);
        }
    }

    fn u32s(&mut self, values: &[u32]) {
        for &value in values {
            self.u32(value);
        }
    }

    fn u64s(&mut self, values: &[u64]) {
        for &value in values {
            self.u64(value);
        }
    }

    fn u8s(&mut self, values: &[u8]) {
        self.bytes(values);
    }

    fn f32s(&mut self, values: &[f32]) {
        for &value in values {
            self.f32(value);
        }
    }
}

/// Reads little-endian fields. Callers check the length before reading, so
/// every read is in bounds.
struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn take<const N: usize>(&mut self) -> [u8; N] {
        let mut out = [0; N];
        out.copy_from_slice(&self.buf[self.pos..self.pos + N]);
        self.pos += N;
        out
    }

    fn skip(&mut self, n: usize) {
        self.pos += n;
    }

    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }

    fn u16(&mut self) -> u16 {
        u16::from_le_bytes(self.take())
    }

    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }

    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take())
    }

    fn f32(&mut self) -> f32 {
        f32::from_le_bytes(self.take())
    }

    fn u16s<const N: usize>(&mut self) -> [u16; N] {
        std::array::from_fn(|_| self.u16())
    }

    fn u32s<const N: usize>(&mut self) -> [u32; N] {
        std::array::from_fn(|_| self.u32())
    }

    fn u64s<const N: usize>(&mut self) -> [u64; N] {
        std::array::from_fn(|_| self.u64())
    }

    fn u8s<const N: usize>(&mut self) -> [u8; N] {
        self.take()
    }

    fn f32s<const N: usize>(&mut self) -> [f32; N] {
        std::array::from_fn(|_| self.f32())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Header used by the spec's test packets.
    fn spec_header(seq: u32) -> Header {
        Header {
            duplicate: false,
            ec_boot_id: 0x1122_3344,
            session_id: 0x0000_BEEF,
            seq,
            txn_id: 0,
            bundle_tag: 7,
            valid_for_ms: 50,
            sent_time_us: 1_000_000,
            ec_time_ref_us: 2_000_000,
        }
    }

    /// Spec test packet: Heartbeat, seq 1.
    const SPEC_HEARTBEAT: [u8; 52] = [
        0x45, 0x43, 0x04, 0x11, 0x00, 0x00, 0x01, 0x00, 0x44, 0x33, 0x22, 0x11, 0xEF, 0xBE, 0x00,
        0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x32, 0x00,
        0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x84, 0x1E, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x65, 0x34, 0x5B, 0x8C,
    ];

    /// Spec test packet: Demand, seq 2.
    const SPEC_DEMAND: [u8; 84] = [
        0x45, 0x43, 0x04, 0x10, 0x20, 0x00, 0x01, 0x00, 0x44, 0x33, 0x22, 0x11, 0xEF, 0xBE, 0x00,
        0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x32, 0x00,
        0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x84, 0x1E, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x33, 0x33, 0x57, 0x41, 0x33, 0x33, 0x57, 0x41, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x80, 0x3F, 0x9A, 0x99, 0x19,
        0x3F, 0x7F, 0x00, 0x00, 0x00, 0x69, 0xC3, 0xC6, 0x5D,
    ];

    fn spec_demand() -> Demand {
        Demand {
            outputs: Outputs {
                tvc_extension_mm: [13.45, 13.45],
                rcs_duty: [0.0, 0.25, 0.5, 1.0],
                main_throttle: 0.6,
            },
            enable_mask: channel::ALL,
        }
    }

    fn encode(packet: &Packet) -> Vec<u8> {
        let mut buf = [0; MAX_DATAGRAM_LEN];
        let len = packet.encode(&mut buf).unwrap();
        buf[..len].to_vec()
    }

    /// Recomputes the CRC after a test edits the body.
    fn reseal(bytes: &mut [u8]) {
        let body_len = bytes.len() - CRC_LEN;
        let crc = CRC32.checksum(&bytes[..body_len]);
        bytes[body_len..].copy_from_slice(&crc.to_le_bytes());
    }

    fn sample_status() -> Status {
        Status {
            ec_state: EcState::Armed,
            output_authority: true,
            session_valid: true,
            safing_latched: false,
            demand_fresh: true,
            safing_reason: Reason::None,
            last_received_seq: 41,
            last_accepted_demand_seq: 40,
            last_applied_demand_seq: 39,
            last_applied_time_us: 123_456_789,
            last_reject_reason: Reason::Stale,
            limited_mask: channel::TVC_A2,
            counters: Counters {
                crc_errors: 1,
                rejected: 2,
                stale: 3,
                old_or_duplicate: 4,
                txn_queue_overflow: 5,
                diag_overflow: 6,
                deadline_miss: 7,
                motion_coalesced: 8,
                motion_rejected: 9,
            },
            reset_cause: 3,
            firmware_version: 0x0001_0002,
            active_config_id: 0x0001_0003,
            driver_faults: 0x12,
            watchdog_status: 0x34,
            applied: spec_demand().outputs,
            measurements: Measurements {
                sample_id: 9,
                sample_time_us: 123_450_000,
                validity_mask: 0x1_FFFF,
                measurement_sample_time_us: [123_450_000; 17],
                measurement_quality: [0; 17],
                adc_raw_code: [1234; 13],
                tvc_position_mm: [13.4, 13.5],
                tvc_position_counts: [2047, 2050],
                tvc_position_age_ms: [8, 9],
                pa12_fault_flags: [0, 0x20],
                pa12_command_request_id: [17, 22],
                pa12_feedback_request_id: [18, 23],
                pa12_command_state: [2, 2],
                pa12_feedback_state: [3, 3],
                rail_voltage_v: [22.2, 12.0, 4.4, 3.3],
                rail_current_a: [1.5, 0.8, 2.0, 0.2],
                rcs_current_a: [0.1, 0.2, 0.3, 0.4],
                motor_rail_voltage_v: 4.38,
                board_temp_c: [31.5, 33.0],
                temperature_status: 0,
            },
        }
    }

    #[test]
    fn crc_is_zlib_crc32() {
        assert_eq!(CRC32.checksum(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn heartbeat_matches_spec_bytes() {
        let packet = Packet {
            header: spec_header(1),
            message: Message::Heartbeat,
        };

        assert_eq!(encode(&packet), SPEC_HEARTBEAT);
        assert_eq!(Packet::decode(&SPEC_HEARTBEAT), Ok(packet));
    }

    #[test]
    fn demand_matches_spec_bytes() {
        let packet = Packet {
            header: spec_header(2),
            message: Message::Demand(spec_demand()),
        };

        assert_eq!(encode(&packet), SPEC_DEMAND);
        assert_eq!(Packet::decode(&SPEC_DEMAND), Ok(packet));
    }

    #[test]
    fn status_matches_spec_bytes() {
        let packet = Packet {
            header: Header {
                valid_for_ms: 0,
                ec_time_ref_us: 0,
                ..spec_header(3)
            },
            message: Message::Status(sample_status()),
        };

        let bytes = encode(&packet);
        assert_eq!(bytes.len(), HEADER_LEN + 388 + CRC_LEN);
        assert_eq!(Packet::decode(&bytes), Ok(packet));
    }

    #[test]
    fn every_kind_round_trips_at_spec_length() {
        let bundle = [0xAB; 32];
        let messages = [
            (
                Message::Hello(Hello {
                    fc_boot_id: 5,
                    run_id: 6,
                    demand_period_ms: 10,
                    heartbeat_period_ms: 50,
                    expected_bundle_sha256: bundle,
                }),
                48,
            ),
            (
                Message::Arm(Arm {
                    run_id: 6,
                    expected_bundle_sha256: bundle,
                }),
                36,
            ),
            (Message::Disarm, 0),
            (
                Message::Terminate(Terminate {
                    reason: Reason::FcTerminate,
                }),
                4,
            ),
            (Message::ClearSafing, 0),
            (Message::Demand(spec_demand()), 32),
            (Message::Heartbeat, 0),
            (
                Message::TxnResult(TxnResult {
                    request_kind: Kind::Arm,
                    outcome: TxnOutcome::Rejected,
                    reason: Reason::NotReady,
                    ec_state: EcState::Disarmed,
                }),
                8,
            ),
            (
                Message::HelloResult(HelloResult {
                    outcome: TxnOutcome::Accepted,
                    ec_state: EcState::Disarmed,
                    reason: Reason::None,
                    capabilities: channel::ALL,
                    max_demand_age_ms: 50,
                    firmware_version: 3,
                    calibration_id: 4,
                    link_timeout_ms: 350,
                    active_bundle_sha256: bundle,
                }),
                52,
            ),
            (Message::Status(sample_status()), 388),
        ];

        for (message, payload_len) in messages {
            let packet = Packet {
                header: Header {
                    duplicate: true,
                    ..spec_header(77)
                },
                message,
            };
            let bytes = encode(&packet);

            assert_eq!(
                bytes.len(),
                HEADER_LEN + payload_len + CRC_LEN,
                "{message:?}"
            );
            assert_eq!(bytes[3], message.kind() as u8);
            assert_eq!(bytes[6], message.kind().source() as u8);
            assert_eq!(Packet::decode(&bytes), Ok(packet));
        }
    }

    #[test]
    fn status_flags_land_on_their_bits() {
        let packet = Packet {
            header: Header::default(),
            message: Message::Status(Status {
                session_valid: true,
                demand_fresh: true,
                ..Status::default()
            }),
        };

        assert_eq!(encode(&packet)[HEADER_LEN + 1], 0b1010);
    }

    #[test]
    fn rejects_malformed_datagrams() {
        let mut bad_crc = SPEC_DEMAND;
        bad_crc[60] ^= 0x01;
        assert_eq!(Packet::decode(&bad_crc), Err(DecodeError::BadCrc));

        let mut bad_magic = SPEC_HEARTBEAT;
        bad_magic[0] = b'X';
        assert_eq!(Packet::decode(&bad_magic), Err(DecodeError::BadMagic));

        let mut bad_version = SPEC_HEARTBEAT;
        bad_version[2] = 2;
        assert_eq!(
            Packet::decode(&bad_version),
            Err(DecodeError::UnsupportedVersion(2))
        );

        assert_eq!(
            Packet::decode(&SPEC_HEARTBEAT[..40]),
            Err(DecodeError::TooShort { len: 40 })
        );
        assert_eq!(
            Packet::decode(&[0; MAX_DATAGRAM_LEN + 1]),
            Err(DecodeError::TooLong {
                len: MAX_DATAGRAM_LEN + 1
            })
        );
    }

    #[test]
    fn rejects_unknown_kind_and_wrong_length() {
        let mut unknown = SPEC_HEARTBEAT;
        unknown[3] = 0x42;
        reseal(&mut unknown);
        assert_eq!(
            Packet::decode(&unknown),
            Err(DecodeError::InvalidEnum {
                field: "Kind",
                value: 0x42
            })
        );

        // A heartbeat that claims to carry a demand-sized payload.
        let mut wrong_length = SPEC_HEARTBEAT;
        wrong_length[4] = 32;
        reseal(&mut wrong_length);
        assert_eq!(
            Packet::decode(&wrong_length),
            Err(DecodeError::BadLength {
                kind: Kind::Heartbeat,
                declared: 32,
                actual: 0
            })
        );

        // A demand cut short: the declared length is right, the datagram isn't.
        let mut short = SPEC_DEMAND[..76].to_vec();
        short.extend_from_slice(&[0; CRC_LEN]);
        reseal(&mut short);
        assert_eq!(
            Packet::decode(&short),
            Err(DecodeError::BadLength {
                kind: Kind::Demand,
                declared: 32,
                actual: 28
            })
        );
    }

    #[test]
    fn rejects_wrong_source_and_bad_enums() {
        let mut from_ec = SPEC_DEMAND;
        from_ec[6] = Source::Ec as u8;
        reseal(&mut from_ec);
        assert_eq!(
            Packet::decode(&from_ec),
            Err(DecodeError::WrongSource {
                kind: Kind::Demand,
                source: 2
            })
        );

        let packet = Packet {
            header: Header::default(),
            message: Message::Status(Status::default()),
        };
        let mut bad_state = encode(&packet);
        bad_state[HEADER_LEN] = 9;
        reseal(&mut bad_state);
        assert_eq!(
            Packet::decode(&bad_state),
            Err(DecodeError::InvalidEnum {
                field: "EcState",
                value: 9
            })
        );
    }

    #[test]
    fn sequence_comparison_wraps() {
        assert!(is_newer(1, 0));
        assert!(!is_newer(0, 1));
        assert!(!is_newer(5, 5));
        assert!(is_newer(0, u32::MAX));
        assert!(is_newer(3, u32::MAX - 3));
        assert!(!is_newer(u32::MAX, 0));
    }

    #[test]
    fn demand_check_matches_the_spec() {
        assert_eq!(spec_demand().check(channel::ALL), Ok(()));

        let mut nan = spec_demand();
        nan.outputs.rcs_duty[2] = f32::NAN;
        assert_eq!(nan.check(channel::ALL), Err(Reason::NonFinite));

        let mut infinite = spec_demand();
        infinite.outputs.tvc_extension_mm[0] = f32::INFINITY;
        assert_eq!(infinite.check(channel::ALL), Err(Reason::NonFinite));

        let mut past_stroke = spec_demand();
        past_stroke.outputs.tvc_extension_mm[1] = TVC_STROKE_MM + 0.1;
        assert_eq!(past_stroke.check(channel::ALL), Err(Reason::OutOfRange));

        let mut negative_duty = spec_demand();
        negative_duty.outputs.rcs_duty[0] = -0.01;
        assert_eq!(negative_duty.check(channel::ALL), Err(Reason::OutOfRange));

        let mut over_throttle = spec_demand();
        over_throttle.outputs.main_throttle = 1.01;
        assert_eq!(over_throttle.check(channel::ALL), Err(Reason::OutOfRange));

        let no_esc = channel::ALL & !channel::MAIN_ESC;
        assert_eq!(spec_demand().check(no_esc), Err(Reason::UnsupportedChannel));
    }

    #[test]
    fn encode_reports_a_short_buffer() {
        let packet = Packet {
            header: Header::default(),
            message: Message::Status(Status::default()),
        };

        assert_eq!(
            packet.encode(&mut [0; 100]),
            Err(EncodeError::BufferTooSmall {
                needed: HEADER_LEN + Status::LEN + CRC_LEN,
                available: 100
            })
        );
    }
}
