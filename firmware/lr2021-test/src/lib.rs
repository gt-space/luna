#![no_std]
#![warn(missing_docs)]

//! Fixed test frame shared by the standalone radio firmware and a future host.

/// Number of bytes in each point-to-point test frame.
pub const FRAME_LEN: usize = 16;
/// Center frequency used by both test boards.
pub const FREQUENCY_HZ: u32 = 2_440_000_000;
const FILL: [u8; 8] = *b"LUNA2G4!";
const MAGIC: [u8; 4] = *b"LR21";

/// Why a received payload is not one of this test's frames.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum FrameError {
    /// The payload has a different number of bytes than [`FRAME_LEN`].
    Length,
    /// The payload starts with an unexpected protocol marker.
    Magic,
    /// The fixed test pattern differs from the expected bytes.
    Payload,
}

/// Build a test frame containing a sequence number and a fixed pattern.
pub fn encode_frame(sequence: u32) -> [u8; FRAME_LEN] {
    let mut frame = [0u8; FRAME_LEN];
    frame[..4].copy_from_slice(&MAGIC);
    frame[4..8].copy_from_slice(&sequence.to_be_bytes());
    frame[8..].copy_from_slice(&FILL);
    frame
}

/// Read a sequence number after validating a received test frame.
pub fn decode_frame(frame: &[u8]) -> Result<u32, FrameError> {
    if frame.len() != FRAME_LEN {
        return Err(FrameError::Length);
    }
    if frame[..4] != MAGIC {
        return Err(FrameError::Magic);
    }
    if frame[8..] != FILL {
        return Err(FrameError::Payload);
    }
    Ok(u32::from_be_bytes([frame[4], frame[5], frame[6], frame[7]]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_round_trip_for_sequence_edges() {
        for sequence in [0, 1, u32::MAX] {
            let frame = encode_frame(sequence);
            assert_eq!(decode_frame(&frame), Ok(sequence));
        }
    }

    #[test]
    fn rejects_wrong_length() {
        let frame = encode_frame(42);
        assert_eq!(decode_frame(&frame[..15]), Err(FrameError::Length));
    }

    #[test]
    fn rejects_wrong_magic() {
        let mut frame = encode_frame(42);
        frame[0] ^= 1;
        assert_eq!(decode_frame(&frame), Err(FrameError::Magic));
    }

    #[test]
    fn rejects_corrupted_payload() {
        let mut frame = encode_frame(42);
        frame[10] ^= 1;
        assert_eq!(decode_frame(&frame), Err(FrameError::Payload));
    }
}
