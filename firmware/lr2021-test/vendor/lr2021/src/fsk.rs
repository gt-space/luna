//! # API related to FSK operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for FSK (Frequency Shift Keying) modulation.
//! This API is for a generic FSK modulation using a packet structure compatible with previous Semtech chips 
//! (SX126x, SX127x, LR11xx). It supports fixed or dynamic length packets up to 511 bytes, with configurable 
//! CRC, address filtering, and whitening.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for FSK operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::fsk::{PblLenDetect, PldLenUnit, AddrComp, FskPktFormat, Crc, BitOrder};
//! use lr2021::{PulseShape, RxBw};
//!
//! // Set packet type to FSK Legacy (compatible with SX126x/SX127x/LR11xx)
//! lr2021.set_packet_type(PacketType::FskLegacy).await.expect("Setting packet type");
//!
//! // Configure FSK modulation (250kbps, BT=0.5 pulse shaping, 444kHz bandwidth, 62.5kHz deviation)
//! lr2021.set_fsk_modulation(
//!     250_000,                // Bitrate: 250 kbps
//!     PulseShape::Bt0p5,     // Pulse shaping: BT=0.5 Gaussian filter
//!     RxBw::Bw444,           // RX bandwidth: 444 kHz
//!     62500                  // Frequency deviation: 62.5 kHz
//! ).await.expect("Setting FSK modulation");
//!
//! // Configure syncword (32-bit, LSB first)
//! lr2021.set_fsk_syncword(
//!     0xCD05DEAD,            // Syncword value
//!     BitOrder::LsbFirst,    // Bit order: LSB first
//!     32                     // Syncword length: 32 bits
//! ).await.expect("Setting syncword");
//!
//! // Configure packet parameters
//! lr2021.set_fsk_packet(
//!     8,                      // TX preamble length: 8 bits
//!     PblLenDetect::None,     // No specific preamble detection length
//!     false,                  // No long preamble
//!     PldLenUnit::Bytes,      // Payload length unit: bytes
//!     AddrComp::Off,          // No address filtering
//!     FskPktFormat::Variable8bit, // Variable length with 8-bit length field
//!     10,                     // Maximum payload length: 10 bytes
//!     Crc::Crc2Byte,         // 2-byte CRC
//!     true                    // DC-free encoding enabled (whitening)
//! ).await.expect("Setting packet parameters");
//! ```
//!
//! ## Available Methods
//!
//! ### Core Configuration
//! - [`set_fsk_modulation`](Lr2021::set_fsk_modulation) - Configure bitrate, pulse shaping, bandwidth, and frequency deviation
//! - [`set_fsk_packet`](Lr2021::set_fsk_packet) - Set packet parameters (preamble, length format, CRC, addressing, whitening)
//! - [`set_fsk_syncword`](Lr2021::set_fsk_syncword) - Configure synchronization word (value, bit order, length)
//! - [`set_fsk_long_prmb_support`](Lr2021::set_fsk_long_prmb_support) - Enable long preamble support in FSK (more than 2048 symbols)
//!
//! ### Status and Statistics  
//! - [`get_fsk_packet_status`](Lr2021::get_fsk_packet_status) - Get packet status information (length, RSSI, LQI)
//! - [`get_fsk_rx_stats`](Lr2021::get_fsk_rx_stats) - Get reception statistics (packets received, errors, sync failures)

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_fsk::*;
use super::{BusyPin, Lr2021, Lr2021Error};

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Modulation parameters: raw bitrate, pulse shaping, Bandwidth and fdev
    pub async fn set_fsk_modulation(&mut self, bitrate: u32, pulse_shape: PulseShape, rx_bw: RxBw, fdev: u32) -> Result<(), Lr2021Error> {
        let req = set_fsk_modulation_params_cmd(bitrate, pulse_shape, rx_bw, fdev);
        self.cmd_wr(&req).await
    }

    // TODO: add dedicated struct and find a good default set of values
    #[allow(clippy::too_many_arguments)]
    /// Set packet parameters (preamble, length format, CRC, addressing, whitening)
    pub async fn set_fsk_packet(&mut self, pbl_len_tx: u16, pbl_len_detect: PblLenDetect, pbl_long: bool, pld_len_unit: PldLenUnit, addr_comp: AddrComp, fsk_pkt_format: FskPktFormat, pld_len: u16, crc: Crc, dc_free: bool) -> Result<(), Lr2021Error> {
        let req = set_fsk_packet_params_cmd(pbl_len_tx, pbl_len_detect, pbl_long, pld_len_unit, addr_comp, fsk_pkt_format, pld_len, crc, dc_free);
        self.cmd_wr(&req).await
    }

    /// Configure syncword
    pub async fn set_fsk_syncword(&mut self, syncword: u64, bit_order: BitOrder, nb_bits: u8) -> Result<(), Lr2021Error> {
        let req = set_fsk_sync_word_cmd(syncword, bit_order, nb_bits);
        self.cmd_wr(&req).await
    }

    /// Enable long preamble support in FSK (more than 2048 symbols)
    pub async fn set_fsk_long_prmb_support(&mut self, en: bool) -> Result<(), Lr2021Error> {
        self.cmd_wr(&[0x02, 0x04, if en {1} else {0}]).await
    }

    /// Return length of last packet received
    pub async fn get_fsk_packet_status(&mut self) -> Result<FskPacketStatusRsp, Lr2021Error> {
        let req = get_fsk_packet_status_req();
        let mut rsp = FskPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_fsk_rx_stats(&mut self) -> Result<FskRxStatsRsp, Lr2021Error> {
        let req = get_fsk_rx_stats_req();
        let mut rsp = FskRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}