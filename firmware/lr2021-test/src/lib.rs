#![no_std]

//! Fixed test frame shared by the standalone radio firmware and a future host.

pub const FREQUENCY_HZ: u32 = 2_440_000_000;
pub const FRAME_LEN: usize = 16;
const MAGIC: [u8; 4] = *b"LR21";
const FILL: [u8; 8] = *b"LUNA2G4!";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    Length,
    Magic,
    Payload,
}

pub fn encode_frame(sequence: u32) -> [u8; FRAME_LEN] {
    let mut frame = [0u8; FRAME_LEN];
    frame[..4].copy_from_slice(&MAGIC);
    frame[4..8].copy_from_slice(&sequence.to_be_bytes());
    frame[8..].copy_from_slice(&FILL);
    frame
}

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
    fn frame_round_trip_and_rejects_corruption() {
        let mut frame = encode_frame(u32::MAX);
        assert_eq!(decode_frame(&frame), Ok(u32::MAX));
        frame[10] ^= 1;
        assert_eq!(decode_frame(&frame), Err(FrameError::Payload));
        assert_eq!(decode_frame(&frame[..15]), Err(FrameError::Length));
    }
}
