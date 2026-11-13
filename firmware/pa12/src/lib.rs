//! Packet encoding and decoding for the Progressive Automations PA-12
//! linear actuator (PA-12-T over TTL, PA-12-R over RS-485).
//!
//! The PA-12 runs its own position PID, so the controller only sends goal
//! positions and reads back status. This crate builds and parses the bytes;
//! sending them over a UART is left to the caller so it can run on any board.
//!
//! Documentation:
//! - Datasheet: <https://f.hubspotusercontent40.net/hubfs/7717445/PDF%20Manuals/Actuator%20datasheets/PA-12%20datasheet.pdf>
//! - Protocol: <https://7717445.fs1.hubspotusercontent-na1.net/hubfs/7717445/PDF%20Manuals/Actuator%20datasheets/Position%20Control%20Communication%20Protocol.pdf>
//!
//! # Link Notes
//!
//! - Half-duplex UART, 57600 baud by default, 8 data bits, no parity, 1 stop
//!   bit.
//! - The manual recommends waiting 5 ms after a write and 10 ms after a read
//!   before the next packet to avoid collisions on the shared line.
//! - Every actuator on a daisy chain needs a unique ID (factory default is 0).
//!   Packets sent to [`BROADCAST_ID`] reach all actuators and get no reply.
//! - By default (Feedback Return Mode 1) only [`Instruction::LoadData`] packets
//!   get a reply.

#![no_std]
#![warn(missing_docs)]

use core::fmt;

/// Bytes that start every command and feedback packet.
pub const HEADER: [u8; 3] = [0xFF, 0xFF, 0xFF];

/// ID that addresses every actuator at once. Broadcast packets get no reply.
pub const BROADCAST_ID: u8 = 0xFE;

/// Highest goal or present position count. With the factory long stroke
/// limit, a 1.06" model tops out at 3686.
pub const MAX_POSITION: u16 = 4095;

/// Errors returned when a millimeter command cannot be safely mapped to a
/// PA-12 protocol position count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionError {
    /// A calibration point or requested position is NaN or infinite.
    NonFinite,
    /// Calibration points or configured hard count limits are inconsistent.
    InvalidCalibration,
    /// Requested millimeters lie outside the measured hard-limit range.
    OutOfRange,
}

/// Measured linear mapping between actuator extension and PA-12 counts.
///
/// The two measurement points define the conversion slope and offset. The
/// hard count limits must come from the configured actuator stroke limits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionCalibration {
    mm_a: f32,
    count_a: u16,
    mm_b: f32,
    count_b: u16,
    hard_min_count: u16,
    hard_max_count: u16,
    hard_min_mm: f32,
    hard_max_mm: f32,
}

impl PositionCalibration {
    /// Builds a mapping from two measured `(millimeters, counts)` points and
    /// the actuator's configured hard count limits.
    pub fn from_two_points(
        mm_a: f32,
        count_a: u16,
        mm_b: f32,
        count_b: u16,
        hard_min_count: u16,
        hard_max_count: u16,
    ) -> Result<Self, PositionError> {
        if !mm_a.is_finite() || !mm_b.is_finite() {
            return Err(PositionError::NonFinite);
        }
        if mm_a == mm_b
            || count_a == count_b
            || hard_min_count >= hard_max_count
            || hard_max_count > MAX_POSITION
            || count_a < hard_min_count
            || count_a > hard_max_count
            || count_b < hard_min_count
            || count_b > hard_max_count
        {
            return Err(PositionError::InvalidCalibration);
        }

        let counts_per_mm = (f32::from(count_b) - f32::from(count_a)) / (mm_b - mm_a);
        if !counts_per_mm.is_finite() || counts_per_mm == 0.0 {
            return Err(PositionError::InvalidCalibration);
        }
        let mm_at_count =
            |count: u16| mm_a + (f32::from(count) - f32::from(count_a)) / counts_per_mm;
        let hard_mm_a = mm_at_count(hard_min_count);
        let hard_mm_b = mm_at_count(hard_max_count);
        if !hard_mm_a.is_finite() || !hard_mm_b.is_finite() || hard_mm_a == hard_mm_b {
            return Err(PositionError::InvalidCalibration);
        }

        Ok(Self {
            mm_a,
            count_a,
            mm_b,
            count_b,
            hard_min_count,
            hard_max_count,
            hard_min_mm: hard_mm_a.min(hard_mm_b),
            hard_max_mm: hard_mm_a.max(hard_mm_b),
        })
    }

    /// Physical millimeter limits implied by the calibration and configured
    /// actuator hard count limits.
    pub const fn hard_mm_limits(&self) -> (f32, f32) {
        (self.hard_min_mm, self.hard_max_mm)
    }

    /// Configured hard count limits enforced after conversion and rounding.
    pub const fn hard_count_limits(&self) -> (u16, u16) {
        (self.hard_min_count, self.hard_max_count)
    }

    /// Converts a finite, in-range physical extension into a count and clamps
    /// rounding at either endpoint to the configured hard count limits.
    pub fn counts_for_mm(&self, mm: f32) -> Result<u16, PositionError> {
        if !mm.is_finite() {
            return Err(PositionError::NonFinite);
        }
        if mm < self.hard_min_mm || mm > self.hard_max_mm {
            return Err(PositionError::OutOfRange);
        }
        let counts_per_mm =
            (f32::from(self.count_b) - f32::from(self.count_a)) / (self.mm_b - self.mm_a);
        let count = f32::from(self.count_a) + (mm - self.mm_a) * counts_per_mm;
        if count <= f32::from(self.hard_min_count) {
            Ok(self.hard_min_count)
        } else if count >= f32::from(self.hard_max_count) {
            Ok(self.hard_max_count)
        } else {
            /* Counts are inside nonnegative hard limits; adding 0.5 rounds
             * half up, and the cast truncates the remaining fraction. */
            Ok((count + 0.5) as u16)
        }
    }
}

/// Builds a motion command from calibrated extension in millimeters.
pub fn set_position_mm(
    id: u8,
    position_mm: f32,
    calibration: &PositionCalibration,
) -> Result<Packet, PositionError> {
    Ok(goal_position(id, calibration.counts_for_mm(position_mm)?))
}

/// Builds a validated safing goal. Queue the returned packet with the
/// scheduler's [`scheduler::Scheduler::queue_safing`] method.
pub fn apply_safing(
    id: u8,
    safe_position_mm: f32,
    calibration: &PositionCalibration,
) -> Result<Packet, PositionError> {
    set_position_mm(id, safe_position_mm, calibration)
}

/// Most actuators [`goal_positions`] can address in one packet.
pub const MAX_SYNC_ACTUATORS: usize = 8;

/// Length of the longest packet this crate builds.
pub const MAX_PACKET_LEN: usize = 7 + 2 + 3 * MAX_SYNC_ACTUATORS;

/// Length of the longest feedback packet this crate accepts. Longer size
/// bytes are treated as corruption so a bad size can't stall the receiver.
pub const MAX_FEEDBACK_LEN: usize = 64;

/// Instruction codes for command packets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Instruction {
    /// Ask the actuator to reply with its status.
    Echo = 0xF1,
    /// Read bytes starting at an address.
    LoadData = 0xF2,
    /// Write bytes starting at an address.
    StoreData = 0xF3,
    /// Write bytes to temporary storage until [`Instruction::Execution`].
    SendData = 0xF4,
    /// Apply data staged with [`Instruction::SendData`].
    Execution = 0xF5,
    /// Reset parameters to factory defaults.
    FactoryReset = 0xF6,
    /// Reboot the actuator.
    Restart = 0xF8,
    /// Write the same address on several actuators in one packet.
    SymmetricStore = 0x73,
}

/// Register addresses from the protocol data map. Two-byte values are stored
/// low byte first.
pub mod address {
    /// Actuator ID (1 byte, non-volatile).
    pub const ID: u8 = 0x03;
    /// Short stroke limit (2 bytes, non-volatile).
    pub const SHORT_STROKE_LIMIT: u8 = 0x06;
    /// Long stroke limit (2 bytes, non-volatile).
    pub const LONG_STROKE_LIMIT: u8 = 0x08;
    /// Feedback return mode (1 byte, non-volatile).
    pub const FEEDBACK_RETURN_MODE: u8 = 0x10;
    /// Force on/off (1 byte).
    pub const FORCE_ON_OFF: u8 = 0x80;
    /// Goal position (2 bytes).
    pub const GOAL_POSITION: u8 = 0x86;
    /// Present position (2 bytes, read only).
    pub const PRESENT_POSITION: u8 = 0x8C;
    /// Present voltage in 0.1 V (1 byte, read only).
    pub const PRESENT_VOLTAGE: u8 = 0x92;
    /// Moving status, 1 while the motor is running (1 byte, read only).
    pub const MOVING: u8 = 0x96;
}

/// Nonblocking two-port transaction scheduler for independent PA-12 UARTs.
pub mod scheduler;

/// An encoded command packet ready to write to the UART.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Packet {
    buf: [u8; MAX_PACKET_LEN],
    len: usize,
}

impl Packet {
    fn new(id: u8, instruction: Instruction, factors: &[u8]) -> Self {
        let mut buf = [0; MAX_PACKET_LEN];
        let len = 7 + factors.len();

        buf[..3].copy_from_slice(&HEADER);
        buf[3] = id;
        buf[4] = factors.len() as u8 + 2;
        buf[5] = instruction as u8;
        buf[6..len - 1].copy_from_slice(factors);
        buf[len - 1] = checksum(&buf[3..len - 1]);

        Self { buf, len }
    }

    /// The bytes to send.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

/// `0xFF` minus the low byte of the sum of `bytes`.
fn checksum(bytes: &[u8]) -> u8 {
    !bytes.iter().fold(0u8, |sum, &b| sum.wrapping_add(b))
}

/// Commands one actuator to move to `position`, clamped to
/// [`MAX_POSITION`]. The actuator also clamps to its own stroke limits.
pub fn goal_position(id: u8, position: u16) -> Packet {
    let [lo, hi] = position.min(MAX_POSITION).to_le_bytes();

    Packet::new(
        id,
        Instruction::StoreData,
        &[address::GOAL_POSITION, lo, hi],
    )
}

/// Commands several actuators in one broadcast packet so they start moving
/// together. Each entry is `(id, position)`; positions are clamped to
/// [`MAX_POSITION`]. Returns `None` for more than [`MAX_SYNC_ACTUATORS`]
/// entries.
pub fn goal_positions(targets: &[(u8, u16)]) -> Option<Packet> {
    if targets.len() > MAX_SYNC_ACTUATORS {
        return None;
    }

    let mut factors = [0; 2 + 3 * MAX_SYNC_ACTUATORS];
    factors[0] = address::GOAL_POSITION;
    factors[1] = 2;

    for (i, &(id, position)) in targets.iter().enumerate() {
        let [lo, hi] = position.min(MAX_POSITION).to_le_bytes();
        factors[2 + 3 * i..5 + 3 * i].copy_from_slice(&[id, lo, hi]);
    }

    Some(Packet::new(
        BROADCAST_ID,
        Instruction::SymmetricStore,
        &factors[..2 + 3 * targets.len()],
    ))
}

/// Requests `length` bytes starting at `address`. The actuator replies with
/// a [`Feedback`] packet.
pub fn read(id: u8, address: u8, length: u8) -> Packet {
    Packet::new(id, Instruction::LoadData, &[address, length])
}

/// Requests the actuator's present position.
pub fn read_position(id: u8) -> Packet {
    read(id, address::PRESENT_POSITION, 2)
}

/// Requests the moving-status register; every PA-12 feedback frame also
/// carries the actuator hardware-error flags.
pub fn read_faults(id: u8) -> Packet {
    read(id, address::MOVING, 1)
}

/// Turns motor power on or off. With force off the actuator holds position
/// by friction and still listens; the next goal position turns force back
/// on.
pub fn force(id: u8, on: bool) -> Packet {
    Packet::new(
        id,
        Instruction::StoreData,
        &[address::FORCE_ON_OFF, u8::from(on)],
    )
}

/// Asks the actuator to reply with its status.
pub fn echo(id: u8) -> Packet {
    Packet::new(id, Instruction::Echo, &[])
}

/// Hardware error flags reported in every feedback packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HardwareError(pub u8);

impl HardwareError {
    /// The motor could not hold the load at maximum force, or tripped the
    /// duty cycle shutdown. Cleared only by a restart or power cycle.
    pub fn overload(self) -> bool {
        self.0 & (1 << 5) != 0
    }

    /// The input voltage is outside the operating range.
    pub fn input_voltage(self) -> bool {
        self.0 & 1 != 0
    }

    /// No error flags are set.
    pub fn is_ok(self) -> bool {
        self.0 == 0
    }
}

/// A decoded feedback packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Feedback<'a> {
    /// ID of the actuator that replied.
    pub id: u8,
    /// Error flags at the time of the reply.
    pub error: HardwareError,
    /// Bytes read, low byte first for two-byte values.
    pub data: &'a [u8],
}

impl Feedback<'_> {
    /// Interprets the data as a two-byte value such as present position.
    pub fn as_u16(&self) -> Option<u16> {
        match *self.data {
            [lo, hi] => Some(u16::from_le_bytes([lo, hi])),
            _ => None,
        }
    }
}

/// Why a feedback packet could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    /// The packet is shorter than its header and size byte claim.
    Truncated,
    /// The packet does not start with [`HEADER`].
    BadHeader,
    /// The size byte is too small to hold an error byte and checksum, or the
    /// packet would be longer than [`MAX_FEEDBACK_LEN`].
    BadSize(u8),
    /// The checksum does not match the contents.
    BadChecksum {
        /// Checksum computed from the contents.
        expected: u8,
        /// Checksum byte received.
        received: u8,
    },
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseError::Truncated => write!(f, "packet is truncated"),
            ParseError::BadHeader => write!(f, "packet header is not 0xFFFFFF"),
            ParseError::BadSize(size) => write!(f, "invalid packet size {size}"),
            ParseError::BadChecksum { expected, received } => write!(
                f,
                "checksum mismatch: expected {expected:#04X}, received {received:#04X}"
            ),
        }
    }
}

/// Decodes a feedback packet from the start of `bytes`. Trailing bytes are
/// ignored.
pub fn parse_feedback(bytes: &[u8]) -> Result<Feedback<'_>, ParseError> {
    if bytes.len() < 3 {
        return Err(ParseError::Truncated);
    }

    if bytes[..3] != HEADER {
        return Err(ParseError::BadHeader);
    }

    if bytes.len() < 5 {
        return Err(ParseError::Truncated);
    }

    let size = usize::from(bytes[4]);
    let len = 5 + size;

    if size < 2 || len > MAX_FEEDBACK_LEN {
        return Err(ParseError::BadSize(bytes[4]));
    }

    if bytes.len() < len {
        return Err(ParseError::Truncated);
    }

    let expected = checksum(&bytes[3..len - 1]);
    let received = bytes[len - 1];

    if expected != received {
        return Err(ParseError::BadChecksum { expected, received });
    }

    Ok(Feedback {
        id: bytes[3],
        error: HardwareError(bytes[5]),
        data: &bytes[6..len - 1],
    })
}

/// Result of scanning a receive buffer for the next feedback packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scan<'a> {
    /// A valid packet was found. Drop `consumed` bytes from the front of the
    /// buffer, which includes any noise before the packet.
    Packet {
        /// The decoded packet.
        feedback: Feedback<'a>,
        /// Bytes to drop from the front of the buffer.
        consumed: usize,
    },
    /// No complete packet yet. Drop `discard` bytes of noise from the front
    /// of the buffer and wait for more data.
    Incomplete {
        /// Bytes to drop from the front of the buffer.
        discard: usize,
    },
}

/// Finds the next valid feedback packet in a receive buffer, skipping noise
/// and packets with a bad size or checksum so the receiver resynchronizes on
/// the next header.
pub fn scan_feedback(bytes: &[u8]) -> Scan<'_> {
    let mut start = 0;

    while let Some(offset) = bytes[start..]
        .windows(HEADER.len())
        .position(|window| window == HEADER)
    {
        let at = start + offset;

        match parse_feedback(&bytes[at..]) {
            Ok(feedback) => {
                return Scan::Packet {
                    feedback,
                    consumed: at + 5 + usize::from(bytes[at + 4]),
                }
            }
            Err(ParseError::Truncated) => return Scan::Incomplete { discard: at },
            Err(_) => start = at + 1,
        }
    }

    // Keep a partial header at the end in case the rest is still arriving.
    let partial = bytes
        .iter()
        .rev()
        .take(HEADER.len() - 1)
        .take_while(|&&b| b == 0xFF)
        .count();

    Scan::Incomplete {
        discard: bytes.len() - partial,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Expected bytes are the command examples from the protocol manual,
    // pages 24-29.

    #[test]
    fn goal_position_matches_manual() {
        assert_eq!(
            goal_position(0x01, 2047).as_bytes(),
            [0xFF, 0xFF, 0xFF, 0x01, 0x05, 0xF3, 0x86, 0xFF, 0x07, 0x7A]
        );
        assert_eq!(
            goal_position(0x01, 1000).as_bytes(),
            [0xFF, 0xFF, 0xFF, 0x01, 0x05, 0xF3, 0x86, 0xE8, 0x03, 0x95]
        );
    }

    #[test]
    fn goal_positions_matches_manual() {
        let packet = goal_positions(&[(0x01, 1023), (0x02, 2047)]).unwrap();

        assert_eq!(
            packet.as_bytes(),
            [
                0xFF, 0xFF, 0xFF, 0xFE, 0x0A, 0x73, 0x86, 0x02, 0x01, 0xFF, 0x03, 0x02, 0xFF, 0x07,
                0xF1
            ]
        );
    }

    #[test]
    fn goal_positions_rejects_too_many_actuators() {
        let targets = [(0, 0); MAX_SYNC_ACTUATORS + 1];

        assert!(goal_positions(&targets[..MAX_SYNC_ACTUATORS]).is_some());
        assert!(goal_positions(&targets).is_none());
    }

    #[test]
    fn positions_clamp_to_max() {
        assert_eq!(goal_position(1, u16::MAX), goal_position(1, MAX_POSITION));
    }

    #[test]
    fn read_position_matches_manual() {
        assert_eq!(
            read_position(0x00).as_bytes(),
            [0xFF, 0xFF, 0xFF, 0x00, 0x04, 0xF2, 0x8C, 0x02, 0x7B]
        );
    }

    #[test]
    fn force_off_matches_manual() {
        assert_eq!(
            force(0x01, false).as_bytes(),
            [0xFF, 0xFF, 0xFF, 0x01, 0x04, 0xF3, 0x80, 0x00, 0x87]
        );
    }

    #[test]
    fn echo_matches_manual() {
        assert_eq!(
            echo(0x00).as_bytes(),
            [0xFF, 0xFF, 0xFF, 0x00, 0x02, 0xF1, 0x0C]
        );
    }

    #[test]
    fn parses_present_position_feedback() {
        let bytes = [0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0xFF, 0x07, 0xF5];
        let feedback = parse_feedback(&bytes).unwrap();

        assert_eq!(feedback.id, 0);
        assert!(feedback.error.is_ok());
        assert_eq!(feedback.as_u16(), Some(2047));
    }

    #[test]
    fn parses_voltage_feedback() {
        let bytes = [0xFF, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x7B, 0x81];
        let feedback = parse_feedback(&bytes).unwrap();

        assert_eq!(feedback.data, [0x7B]);
        assert_eq!(feedback.as_u16(), None);
    }

    #[test]
    fn parses_status_only_feedback() {
        let bytes = [0xFF, 0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
        let feedback = parse_feedback(&bytes).unwrap();

        assert_eq!(feedback.id, 1);
        assert!(feedback.data.is_empty());
    }

    #[test]
    fn reports_error_flags() {
        let mut bytes = [0xFF, 0xFF, 0xFF, 0x01, 0x02, 0b0010_0001, 0x00];
        bytes[6] = checksum(&bytes[3..6]);
        let error = parse_feedback(&bytes).unwrap().error;

        assert!(error.overload());
        assert!(error.input_voltage());
        assert!(!error.is_ok());
    }

    #[test]
    fn rejects_bad_checksum() {
        let bytes = [0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0xFF, 0x07, 0xF4];

        assert_eq!(
            parse_feedback(&bytes),
            Err(ParseError::BadChecksum {
                expected: 0xF5,
                received: 0xF4
            })
        );
    }

    #[test]
    fn rejects_bad_size() {
        let too_small = [0xFF, 0xFF, 0xFF, 0x00, 0x01, 0x00, 0xFE];
        let too_large = [0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00];

        assert_eq!(parse_feedback(&too_small), Err(ParseError::BadSize(0x01)));
        assert_eq!(parse_feedback(&too_large), Err(ParseError::BadSize(0xFF)));
    }

    const POSITION_REPLY: [u8; 9] = [0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0xFF, 0x07, 0xF5];

    fn scanned_position(scan: Scan<'_>) -> (Option<u16>, usize) {
        match scan {
            Scan::Packet { feedback, consumed } => (feedback.as_u16(), consumed),
            Scan::Incomplete { .. } => panic!("expected a packet, got {scan:?}"),
        }
    }

    #[test]
    fn scan_skips_noise_before_packet() {
        let mut bytes = [0u8; 11];
        bytes[..2].copy_from_slice(&[0x12, 0xFF]);
        bytes[2..].copy_from_slice(&POSITION_REPLY);

        assert_eq!(scanned_position(scan_feedback(&bytes)), (Some(2047), 11));
    }

    #[test]
    fn scan_skips_packet_with_bad_checksum() {
        let mut bytes = [0u8; 18];
        bytes[..9].copy_from_slice(&POSITION_REPLY);
        bytes[8] ^= 0x01;
        bytes[9..].copy_from_slice(&POSITION_REPLY);

        assert_eq!(scanned_position(scan_feedback(&bytes)), (Some(2047), 18));
    }

    #[test]
    fn scan_skips_bad_size_instead_of_waiting() {
        let mut bytes = [0u8; 14];
        bytes[..5].copy_from_slice(&[0xFF, 0xFF, 0xFF, 0x00, 0xFF]);
        bytes[5..].copy_from_slice(&POSITION_REPLY);

        assert_eq!(scanned_position(scan_feedback(&bytes)), (Some(2047), 14));
    }

    #[test]
    fn scan_reads_back_to_back_packets() {
        let mut bytes = [0u8; 18];
        bytes[..9].copy_from_slice(&POSITION_REPLY);
        bytes[9..].copy_from_slice(&POSITION_REPLY);

        let (_, consumed) = scanned_position(scan_feedback(&bytes));
        assert_eq!(consumed, 9);
        assert_eq!(
            scanned_position(scan_feedback(&bytes[consumed..])),
            (Some(2047), 9)
        );
    }

    #[test]
    fn scan_waits_for_rest_of_packet() {
        let mut bytes = [0u8; 7];
        bytes[0] = 0x55;
        bytes[1..].copy_from_slice(&POSITION_REPLY[..6]);

        assert_eq!(scan_feedback(&bytes), Scan::Incomplete { discard: 1 });
    }

    #[test]
    fn scan_keeps_partial_header() {
        assert_eq!(
            scan_feedback(&[0x00, 0x12, 0xFF, 0xFF]),
            Scan::Incomplete { discard: 2 }
        );
        assert_eq!(scan_feedback(&[]), Scan::Incomplete { discard: 0 });
    }

    #[test]
    fn rejects_bad_header_and_truncation() {
        let bytes = [0xFF, 0xFF, 0xFE, 0x00, 0x04, 0x00, 0xFF, 0x07, 0xF5];

        assert_eq!(parse_feedback(&bytes), Err(ParseError::BadHeader));
        assert_eq!(
            parse_feedback(&[0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0xFF]),
            Err(ParseError::Truncated)
        );
    }
}
