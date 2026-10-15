//! # API related to BPSK TX operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for BPSK (Binary Phase Shift Keying) transmission.
//! BPSK is used for Sigfox communication, supporting differential encoding modes and configurable packet parameters.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for BPSK TX operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::bpsk_tx::{BpskMode, SigfoxMsg, SigfoxRank, DiffModeEn};
//! use lr2021::PulseShape;
//!
//! // Set packet type to BPSK TX
//! lr2021.set_packet_type(PacketType::BpskTx).await.expect("Setting packet type");
//!
//! // Configure BPSK modulation (600 bps, no pulse shaping, differential encoding enabled)
//! lr2021.set_bpsk_modulation(
//!     600,                    // Bitrate: 600 bps
//!     PulseShape::None,      // No pulse shaping
//!     DiffModeEn::On,        // Differential encoding enabled
//!     false,                  // Differential mode initial value
//!     false                   // Differential mode parity
//! ).await.expect("Setting BPSK modulation");
//!
//! // Configure packet parameters
//! lr2021.set_bpsk_packet(
//!     12,                     // Payload length: 12 bytes
//!     BpskMode::Sigfox,      // Sigfox mode
//!     SigfoxMsg::Data,       // Data message type
//!     SigfoxRank::Rank0      // Sigfox rank
//! ).await.expect("Setting packet parameters");
//! ```
//!
//! ## Available Methods
//!
//! ### Core BPSK Methods
//! - [`set_bpsk_modulation`](Lr2021::set_bpsk_modulation) - Configure bitrate, pulse shaping, and differential encoding parameters
//! - [`set_bpsk_packet`](Lr2021::set_bpsk_packet) - Set packet parameters (payload length, BPSK mode, Sigfox message type and rank)
//!
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::PulseShape;

pub use super::cmd::cmd_bpsk::*;
use super::{BusyPin, Lr2021, Lr2021Error};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Modulation parameters: raw bitrate, pulse shaping, Bandwidth and fdev
    pub async fn set_bpsk_modulation(&mut self, bitrate: u32, pulse_shape: PulseShape, diff_mode_en: DiffModeEn, diff_mode_init: bool, diff_mode_parity: bool) -> Result<(), Lr2021Error> {
        let req = set_bpsk_modulation_params_cmd(bitrate, pulse_shape, diff_mode_en, diff_mode_init, diff_mode_parity);
        self.cmd_wr(&req).await
    }

    /// Set FLRC packet parameters: preamble, syncword, header implicit/explicit, CRC and packet length (max 511)
    pub async fn set_bpsk_packet(&mut self, pld_len: u8, bpsk_mode: BpskMode, sigfox_msg: SigfoxMsg, sigfox_rank: SigfoxRank) -> Result<(), Lr2021Error> {
        let req = set_bpsk_packet_params_cmd(pld_len, bpsk_mode, sigfox_msg, sigfox_rank);
        self.cmd_wr(&req).await
    }

}
