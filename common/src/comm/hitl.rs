//! Messages between the HITL worker (the Meerkat) and the service on the HITL
//! test board's CM5.
//!
//! The Meerkat sends a [`Request`] over UDP to [`PORT`] on the CM5. The CM5
//! runs the command against the test board's chips and answers with a
//! [`Response`] carrying the same `id`, sent back to the address the request
//! came from. Both are encoded with postcard, like the SAM messages.
//!
//! The CM5 also watches for silence. If it has heard from a worker and then
//! hears nothing for [`HEARTBEAT_TIME_LIMIT`], it puts the board in its safe
//! state (see [`Command::Safe`]). A worker holding outputs should send
//! [`Command::Ping`] at least every [`HEARTBEAT_INTERVAL`].

use std::time::Duration;

use serde::{Deserialize, Serialize};

/// UDP port the CM5 service listens on.
pub const PORT: u16 = 8380;

/// Silence after which the CM5 puts the board in its safe state. Same limit
/// the SAM uses for the flight computer.
pub const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(1000);

/// How often a worker should ping while it holds outputs.
pub const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(250);

/// A command for the CM5 service, tagged with an id that the response echoes.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Request {
    /// Chosen by the sender. The response carries the same id, so a reply can
    /// be matched to its request.
    pub id: u32,

    /// What to do.
    pub command: Command,
}

/// Everything the CM5 service can do to the test board.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum Command {
    /// Does nothing but keep the connection alive. Answered with
    /// [`Reply::Done`].
    Ping,

    /// Drives one AD5676R DAC channel (0 to 7) to a voltage.
    SetDacVolts {
        /// DAC channel, 0 to 7.
        channel: u8,
        /// Output voltage, from 0 V to the DAC's full scale.
        volts: f64,
    },

    /// Sets one AD8403 digital potentiometer channel (0 to 3) to a wiper code.
    SetDigipot {
        /// Potentiometer channel, 0 (RDAC1) to 3 (RDAC4).
        channel: u8,
        /// Wiper code, 0 to 255.
        code: u8,
    },

    /// Reads the voltage between two ADS1261 inputs. Answered with
    /// [`Reply::Volts`].
    ReadAdcVolts {
        /// Positive input, AIN0 to AIN9 as 0 to 9.
        positive: u8,
        /// Negative input, AIN0 to AIN9 as 0 to 9, or `None` for AINCOM (ground
        /// on the HITL adapter board).
        negative: Option<u8>,
    },

    /// Reads the TMP112 temperature sensor. Answered with [`Reply::Celsius`].
    ReadTemperature,

    /// Puts the board in its safe state: every DAC channel at 0 V and every
    /// digipot channel at midscale, the same as at startup.
    Safe,
}

/// The CM5's answer to a [`Request`].
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Response {
    /// The id of the request this answers. A request that could not be decoded
    /// is answered with id 0.
    pub id: u32,

    /// The outcome.
    pub reply: Reply,
}

/// The outcome of a [`Command`].
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub enum Reply {
    /// The command ran. Used by commands that return nothing.
    Done,

    /// A voltage reading, in volts.
    Volts(f64),

    /// A temperature reading, in degrees Celsius.
    Celsius(f64),

    /// The command failed. The text is the driver's error, or says why the
    /// command could not run.
    Failed(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_round_trips_through_postcard() {
        let request = Request {
            id: 7,
            command: Command::ReadAdcVolts {
                positive: 3,
                negative: None,
            },
        };

        let bytes = postcard::to_allocvec(&request).unwrap();
        assert_eq!(postcard::from_bytes::<Request>(&bytes).unwrap(), request);
    }

    #[test]
    fn response_round_trips_through_postcard() {
        let response = Response {
            id: 7,
            reply: Reply::Failed("invalid channel: 9".to_owned()),
        };

        let bytes = postcard::to_allocvec(&response).unwrap();
        assert_eq!(postcard::from_bytes::<Response>(&bytes).unwrap(), response);
    }
}
