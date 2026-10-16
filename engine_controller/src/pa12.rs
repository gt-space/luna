//! PA-12-T TVC actuators over half-duplex serial.
//!
//! # How the hardware works
//! Each actuator has one DATA wire used for both talking and listening
//! (half-duplex, like a walkie-talkie). On the board, a buffer chip
//! (U201 for A1, U202 for A2) switches that wire between the MCU's TX and RX
//! pins using a DIR pin:
//!
//! | Actuator | UART  | TX   | RX  | DIR |
//! |----------|-------|------|-----|-----|
//! | A1       | UART7 | PA15 | PA8 | PB8 |
//! | A2       | UART5 | PB6  | PB5 | PB9 |
//!
//! DIR high = MCU talks, DIR low = MCU listens. DIR has a pull-down, so a
//! resetting MCU leaves the wire in "listen". While DIR is high the RX pin is
//! not driven by anything, so enable the MCU's internal pull-up on RX and
//! ignore bytes received while transmitting.
//!
//! **Sending a packet, in the hardware layer:** set DIR high -> write the
//! bytes -> wait for the UART's *transmission complete* flag (not "TX buffer
//! empty", which fires one byte too early) -> set DIR low.
//!
//! # Protocol (PA-12 "Position Control Communication Protocol" manual)
//! 57600 baud, 8 data bits, no parity, 1 stop bit.
//!
//! ```text
//! command:  FF FF FF | ID | SIZE | CMD   | factors... | CHECKSUM
//! reply:    FF FF FF | ID | SIZE | ERROR | factors... | CHECKSUM
//! SIZE     = number of factor bytes + 2
//! CHECKSUM = 0xFF - ((ID + SIZE + CMD/ERROR + factors) & 0xFF)
//! ```
//!
//! With the factory "feedback return mode" (1), the actuator only replies to
//! read commands, so writes never cause a reply that could collide.
//!
//! # Things to know
//! - The manual recommends 5 ms after a write and 10 ms after a read before
//!   the next transfer. That caps updates at roughly 100-150 Hz per actuator.
//!   The rod's top speed (12 or 36 mm/s by model) limits gimbal rate more.
//! - Overload protection: after ~30 s of near-continuous motion the actuator
//!   cuts its motor and sets error bit 5. That error **latches** until a
//!   restart command or a power cycle. Constant correction against thrust
//!   load during a hop could trigger it, so this module reports it loudly.
//! - Never write configuration (ID, baud, PID, limits) in flight. Those live
//!   in non-volatile memory and saving them briefly stalls the actuator.

use crate::{config::pa12 as cfg, protocol::TvcReport};

/// Command codes.
pub mod cmd {
    pub const ECHO: u8 = 0xF1;
    pub const LOAD_DATA: u8 = 0xF2;
    pub const STORE_DATA: u8 = 0xF3;
    pub const RESTART: u8 = 0xF8;
}

/// Memory addresses used here.
pub mod addr {
    pub const FORCE_ON_OFF: u8 = 0x80;
    pub const GOAL_POSITION: u8 = 0x86;
    pub const PRESENT_POSITION: u8 = 0x8C;
}

/// Bits of the error byte in every reply.
pub mod error_bit {
    pub const INPUT_VOLTAGE: u8 = 1 << 0;
    pub const OVERLOAD: u8 = 1 << 5;
}

const HEADER: u8 = 0xFF;
const MAX_PACKET: usize = 12;

/// What the actuators do when the EC safes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SafedAction {
    /// Keep the last commanded position.
    HoldLast,
    /// Drive both actuators to mid-stroke.
    Center,
}

/// A command ready to send.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Packet {
    buf: [u8; MAX_PACKET],
    len: usize,
}

impl Packet {
    pub fn bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    /// Time on the wire at the configured baud (10 bits per byte), rounded
    /// up to whole milliseconds.
    pub fn tx_ms(&self) -> u64 {
        let bits = self.len as u64 * 10;
        (bits * 1_000).div_ceil(u64::from(cfg::BAUD))
    }
}

/// `0xFF - (sum of bytes & 0xFF)`, over everything after the header.
pub fn checksum(bytes: &[u8]) -> u8 {
    0xFF - bytes.iter().fold(0u8, |acc, b| acc.wrapping_add(*b))
}

fn build(id: u8, command: u8, factors: &[u8]) -> Packet {
    let mut buf = [0u8; MAX_PACKET];
    let size = factors.len() as u8 + 2;
    buf[..3].copy_from_slice(&[HEADER; 3]);
    buf[3] = id;
    buf[4] = size;
    buf[5] = command;
    buf[6..6 + factors.len()].copy_from_slice(factors);
    let end = 6 + factors.len();
    buf[end] = checksum(&buf[3..end]);
    Packet { buf, len: end + 1 }
}

/// Move to `counts` (0-4095; clamped to `MAX_COUNTS`).
pub fn goal_position(id: u8, counts: u16) -> Packet {
    let [lo, hi] = counts.min(cfg::MAX_COUNTS).to_le_bytes();
    build(id, cmd::STORE_DATA, &[addr::GOAL_POSITION, lo, hi])
}

/// Ask for the present position (reply carries 2 bytes).
pub fn read_position(id: u8) -> Packet {
    build(id, cmd::LOAD_DATA, &[addr::PRESENT_POSITION, 2])
}

/// Connection check; the reply carries the error byte only.
pub fn echo(id: u8) -> Packet {
    build(id, cmd::ECHO, &[])
}

/// Reboot the actuator. The only way, short of a power cycle, to clear a
/// latched overload error.
pub fn restart(id: u8) -> Packet {
    build(id, cmd::RESTART, &[])
}

/// Motor power on/off. Off lets the motor rest while gear friction holds
/// the rod; the next goal command turns it back on automatically.
pub fn force(id: u8, on: bool) -> Packet {
    build(id, cmd::STORE_DATA, &[addr::FORCE_ON_OFF, on as u8])
}

/// Rod extension in mm to actuator counts, rounded and clamped to the
/// stroke. NaN maps to 0.
pub fn mm_to_counts(mm: f32) -> u16 {
    if mm.is_nan() || mm <= 0.0 {
        return 0;
    }
    let counts = mm * cfg::COUNTS_PER_MM + 0.5;
    if counts >= f32::from(cfg::MAX_COUNTS) {
        cfg::MAX_COUNTS
    } else {
        counts as u16
    }
}

pub fn counts_to_mm(counts: u16) -> f32 {
    f32::from(counts) / cfg::COUNTS_PER_MM
}

/// A decoded reply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Feedback {
    pub id: u8,
    pub error: u8,
    data: [u8; MAX_PACKET],
    len: usize,
}

impl Feedback {
    /// The factor bytes after the error byte.
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// Interprets a 2-byte little-endian reply, e.g. a position.
    pub fn as_u16(&self) -> Option<u16> {
        match self.data() {
            [lo, hi] => Some(u16::from_le_bytes([*lo, *hi])),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    BadChecksum,
    BadSize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ParseState {
    Header(u8),
    Id,
    Size { id: u8 },
    Body { id: u8, size: u8, got: u8 },
}

/// Builds replies one received byte at a time.
#[derive(Clone, Debug)]
pub struct Parser {
    state: ParseState,
    body: [u8; MAX_PACKET],
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    pub const fn new() -> Self {
        Self {
            state: ParseState::Header(0),
            body: [0; MAX_PACKET],
        }
    }

    pub fn reset(&mut self) {
        self.state = ParseState::Header(0);
    }

    /// Feeds one byte. Returns a result once a whole packet has arrived.
    pub fn push(&mut self, byte: u8) -> Option<Result<Feedback, ParseError>> {
        match self.state {
            ParseState::Header(n) => {
                self.state = if byte == HEADER {
                    if n == 2 {
                        ParseState::Id
                    } else {
                        ParseState::Header(n + 1)
                    }
                } else {
                    ParseState::Header(0)
                };
            }
            ParseState::Id => {
                // IDs never use 0xFF, so an extra header byte is skipped.
                if byte != HEADER {
                    self.state = ParseState::Size { id: byte };
                }
            }
            ParseState::Size { id } => {
                // Smallest reply: error + checksum.
                if !(2..=MAX_PACKET as u8).contains(&byte) {
                    self.reset();
                    return Some(Err(ParseError::BadSize));
                }
                self.state = ParseState::Body {
                    id,
                    size: byte,
                    got: 0,
                };
            }
            ParseState::Body { id, size, got } => {
                self.body[got as usize] = byte;
                let got = got + 1;
                if got < size {
                    self.state = ParseState::Body { id, size, got };
                    return None;
                }
                self.reset();

                let n = size as usize;
                let mut summed = [0u8; MAX_PACKET + 2];
                summed[0] = id;
                summed[1] = size;
                summed[2..2 + n - 1].copy_from_slice(&self.body[..n - 1]);
                if checksum(&summed[..n + 1]) != self.body[n - 1] {
                    return Some(Err(ParseError::BadChecksum));
                }

                let mut data = [0u8; MAX_PACKET];
                let len = n - 2;
                data[..len].copy_from_slice(&self.body[1..n - 1]);
                return Some(Ok(Feedback {
                    id,
                    error: self.body[0],
                    data,
                    len,
                }));
            }
        }
        None
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// Free to send once `now >= ready_at_ms`.
    Idle { ready_at_ms: u64 },
    /// Waiting for a reply to a read.
    AwaitingReply { deadline_ms: u64 },
}

/// Schedules traffic to one actuator and tracks what it reports.
///
/// The hardware layer calls [`Pa12Link::poll`] often (every millisecond or
/// so) and sends whatever packet it returns, then feeds every byte received
/// in listen mode to [`Pa12Link::on_byte`].
#[derive(Clone, Debug)]
pub struct Pa12Link {
    id: u8,
    phase: Phase,
    parser: Parser,
    requested_mm: f32,
    goal_counts: Option<u16>,
    writes_since_read: u8,
    missed_replies: u8,
    restart_pending: bool,
    measured_counts: Option<u16>,
    error_bits: u8,
    link_ok: bool,
}

impl Pa12Link {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            phase: Phase::Idle { ready_at_ms: 0 },
            parser: Parser::new(),
            requested_mm: f32::NAN,
            goal_counts: None,
            writes_since_read: 0,
            missed_replies: 0,
            restart_pending: false,
            measured_counts: None,
            error_bits: 0,
            // Unknown until the first reply.
            link_ok: false,
        }
    }

    /// Sets the target extension. It goes out on the next write slot.
    pub fn set_goal_mm(&mut self, mm: f32) {
        self.requested_mm = mm;
        self.goal_counts = Some(mm_to_counts(mm));
    }

    /// Applies `SafedAction`: hold (do nothing) or center.
    pub fn safe(&mut self) {
        if cfg::SAFED_ACTION == SafedAction::Center {
            self.set_goal_mm(cfg::STROKE_MM / 2.0);
        }
    }

    /// Queues a restart, e.g. to clear a latched overload once the FC has
    /// decided it is safe to.
    pub fn request_restart(&mut self) {
        self.restart_pending = true;
    }

    /// Returns the next packet to send, if it is time to send one.
    pub fn poll(&mut self, now_ms: u64) -> Option<Packet> {
        if let Phase::AwaitingReply { deadline_ms } = self.phase {
            if now_ms < deadline_ms {
                return None;
            }
            self.missed_replies = self.missed_replies.saturating_add(1);
            if self.missed_replies >= cfg::MAX_MISSED_REPLIES {
                self.link_ok = false;
            }
            self.phase = Phase::Idle {
                ready_at_ms: now_ms,
            };
        }

        let Phase::Idle { ready_at_ms } = self.phase else {
            return None;
        };
        if now_ms < ready_at_ms {
            return None;
        }

        if self.restart_pending {
            self.restart_pending = false;
            let packet = restart(self.id);
            // Give it time to reboot before talking again.
            self.phase = Phase::Idle {
                ready_at_ms: now_ms + packet.tx_ms() + 100,
            };
            return Some(packet);
        }

        let want_read =
            self.goal_counts.is_none() || self.writes_since_read >= cfg::READ_EVERY_N_WRITES;
        if want_read {
            let packet = read_position(self.id);
            self.writes_since_read = 0;
            self.parser.reset();
            self.phase = Phase::AwaitingReply {
                deadline_ms: now_ms + packet.tx_ms() + cfg::REPLY_TIMEOUT_MS,
            };
            return Some(packet);
        }

        let counts = self.goal_counts?;
        let packet = goal_position(self.id, counts);
        self.writes_since_read += 1;
        self.phase = Phase::Idle {
            ready_at_ms: now_ms + packet.tx_ms() + cfg::WRITE_GAP_MS,
        };
        Some(packet)
    }

    /// Feeds one byte received while listening.
    pub fn on_byte(&mut self, byte: u8, now_ms: u64) {
        if !matches!(self.phase, Phase::AwaitingReply { .. }) {
            return; // Nothing should be talking now; ignore noise.
        }
        let Some(result) = self.parser.push(byte) else {
            return;
        };
        let Ok(fb) = result else {
            return; // Bad packet: wait for the timeout to count it missed.
        };
        if fb.id != self.id {
            return;
        }
        self.error_bits = fb.error;
        if let Some(counts) = fb.as_u16() {
            self.measured_counts = Some(counts);
        }
        self.missed_replies = 0;
        self.link_ok = true;
        self.phase = Phase::Idle {
            ready_at_ms: now_ms + cfg::READ_GAP_MS,
        };
    }

    pub fn link_ok(&self) -> bool {
        self.link_ok
    }

    pub fn error_bits(&self) -> u8 {
        self.error_bits
    }

    pub fn report(&self) -> TvcReport {
        TvcReport {
            requested_mm: self.requested_mm,
            commanded_counts: self.goal_counts.unwrap_or(0),
            measured_mm: self.measured_counts.map_or(f32::NAN, counts_to_mm),
            error_bits: self.error_bits,
            link_ok: self.link_ok,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Packets below are copied from the examples in the PA-12 manual.

    #[test]
    fn matches_manual_goal_position_example() {
        let p = goal_position(1, 2047);
        assert_eq!(
            p.bytes(),
            &[0xFF, 0xFF, 0xFF, 0x01, 0x05, 0xF3, 0x86, 0xFF, 0x07, 0x7A]
        );
        let p = goal_position(1, 1000);
        assert_eq!(p.bytes()[9], 0x95);
    }

    #[test]
    fn matches_manual_read_position_example() {
        assert_eq!(
            read_position(0).bytes(),
            &[0xFF, 0xFF, 0xFF, 0x00, 0x04, 0xF2, 0x8C, 0x02, 0x7B]
        );
    }

    #[test]
    fn matches_manual_echo_restart_force_examples() {
        assert_eq!(echo(0).bytes(), &[0xFF, 0xFF, 0xFF, 0x00, 0x02, 0xF1, 0x0C]);
        assert_eq!(
            restart(0).bytes(),
            &[0xFF, 0xFF, 0xFF, 0x00, 0x02, 0xF8, 0x05]
        );
        assert_eq!(
            force(1, false).bytes(),
            &[0xFF, 0xFF, 0xFF, 0x01, 0x04, 0xF3, 0x80, 0x00, 0x87]
        );
    }

    #[test]
    fn goal_is_clamped_to_stroke_limit() {
        let p = goal_position(0, 4095);
        let counts = u16::from_le_bytes([p.bytes()[7], p.bytes()[8]]);
        assert_eq!(counts, cfg::MAX_COUNTS);
    }

    #[test]
    fn parses_manual_position_reply() {
        let mut parser = Parser::new();
        let reply = [0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0xFF, 0x07, 0xF5];
        let mut out = None;
        for b in reply {
            out = parser.push(b).or(out);
        }
        let fb = out.unwrap().unwrap();
        assert_eq!(fb.error, 0);
        assert_eq!(fb.as_u16(), Some(2047));
    }

    #[test]
    fn parses_manual_voltage_reply() {
        let mut parser = Parser::new();
        let reply = [0xFF, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x7B, 0x81];
        let fb = reply.iter().find_map(|b| parser.push(*b)).unwrap().unwrap();
        assert_eq!(fb.data(), &[0x7B]); // 12.3 V
    }

    #[test]
    fn rejects_corrupted_reply_and_resyncs() {
        let mut parser = Parser::new();
        let bad = [0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0xFF, 0x07, 0xF4];
        let r = bad.iter().find_map(|b| parser.push(*b));
        assert_eq!(r, Some(Err(ParseError::BadChecksum)));

        // Leading noise and an extra header byte are skipped.
        let good = [
            0x12, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0xFF, 0x07, 0xF5,
        ];
        let r = good.iter().find_map(|b| parser.push(*b));
        assert!(r.unwrap().is_ok());
    }

    #[test]
    fn mm_conversion() {
        assert_eq!(mm_to_counts(0.0), 0);
        assert_eq!(mm_to_counts(-3.0), 0);
        assert_eq!(mm_to_counts(f32::NAN), 0);
        assert_eq!(mm_to_counts(cfg::STROKE_MM), cfg::MAX_COUNTS);
        assert_eq!(mm_to_counts(100.0), cfg::MAX_COUNTS);
        let mm = 10.0;
        assert!((counts_to_mm(mm_to_counts(mm)) - mm).abs() < 0.01);
    }

    #[test]
    fn link_reads_first_then_respects_gaps() {
        let mut link = Pa12Link::new(0);
        // No goal yet: ask where it is.
        let p = link.poll(0).unwrap();
        assert_eq!(p, read_position(0));
        assert_eq!(link.poll(1), None); // Waiting for the reply.

        // Reply arrives at t=3.
        for b in [0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0xE8, 0x03, 0x10] {
            link.on_byte(b, 3);
        }
        assert!(link.link_ok());
        assert_eq!(link.report().measured_mm, counts_to_mm(1000));

        link.set_goal_mm(5.0);
        assert_eq!(link.poll(12), None); // Read gap runs to t=13.
        assert_eq!(link.poll(13), Some(goal_position(0, mm_to_counts(5.0))));
        // Write gap: 10 bytes ~ 2 ms on the wire + 5 ms.
        assert_eq!(link.poll(19), None);
        assert!(link.poll(20).is_some());
        // Two writes done, so the next slot is a read.
        assert_eq!(link.poll(27), Some(read_position(0)));
    }

    #[test]
    fn missed_replies_drop_the_link() {
        let mut link = Pa12Link::new(0);
        let mut now = 0;
        for _ in 0..cfg::MAX_MISSED_REPLIES {
            assert_eq!(link.poll(now), Some(read_position(0)));
            now += 100;
        }
        link.poll(now);
        assert!(!link.link_ok());
    }

    #[test]
    fn overload_bit_is_reported() {
        let mut link = Pa12Link::new(0);
        link.poll(0);
        // Echo-style reply with error bit 5 set.
        let body = [0x00, 0x02, error_bit::OVERLOAD];
        let reply = [0xFF, 0xFF, 0xFF, body[0], body[1], body[2], checksum(&body)];
        for b in reply {
            link.on_byte(b, 2);
        }
        assert_eq!(link.error_bits() & error_bit::OVERLOAD, error_bit::OVERLOAD);
    }

    #[test]
    fn restart_goes_out_first_then_waits() {
        let mut link = Pa12Link::new(0);
        link.set_goal_mm(5.0);
        link.request_restart();
        assert_eq!(link.poll(0), Some(restart(0)));
        assert_eq!(link.poll(50), None);
        assert!(link.poll(102).is_some());
    }

    #[test]
    fn bytes_outside_a_read_are_ignored() {
        let mut link = Pa12Link::new(0);
        for b in [0xFF, 0xFF, 0xFF, 0x00, 0x04, 0x00, 0xE8, 0x03, 0x10] {
            link.on_byte(b, 0);
        }
        assert!(!link.link_ok());
    }
}
