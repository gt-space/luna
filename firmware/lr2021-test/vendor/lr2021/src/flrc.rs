//! # API related to FLRC operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for Fast Long-Range Communication (FLRC).
//! FLRC is a Semtech proprietary protocol using GMSK modulation, offering higher sensitivity compared to BLE.
//! It supports configurable bitrates, coding rates, and filtering on syncwords accepting up to 3 values on 32 bits.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for FLRC operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::flrc::{FlrcBitrate, FlrcCr, FlrcPacketParams, AgcPblLen, SwLen, SwTx, SwMatch, PktFormat, Crc};
//! use lr2021::PulseShape;
//!
//! // Set packet type to FLRC
//! lr2021.set_packet_type(PacketType::Flrc).await.expect("Setting packet type");
//!
//! // Configure FLRC modulation (2.6Mbps, no coding, BT=1.0 pulse shaping)
//! lr2021.set_flrc_modulation(FlrcBitrate::Br2600, FlrcCr::None, PulseShape::Bt1p0)
//!     .await.expect("Setting FLRC modulation");
//!
//! // Configure syncwords (up to 3 can be configured)
//! lr2021.set_flrc_syncword(1, 0xCD05CAFE, true).await.expect("Setting syncword 1");
//! lr2021.set_flrc_syncword(2, 0x12345678, true).await.expect("Setting syncword 2");
//! lr2021.set_flrc_syncword(3, 0x9ABCDEF0, true).await.expect("Setting syncword 3");
//!
//! // Configure packet parameters
//! let flrc_params = FlrcPacketParams::new(
//!     AgcPblLen::Len16Bits,    // 16-bit preamble
//!     SwLen::Sw32b,            // 32-bit syncword length
//!     SwTx::Sw1,               // Use syncword 1 for TX
//!     SwMatch::Match123,       // Match any of the 3 syncwords on RX
//!     PktFormat::Dynamic,      // Dynamic packet length
//!     Crc::Crc24,             // 24-bit CRC
//!     255                      // Max payload length
//! );
//! lr2021.set_flrc_packet(&flrc_params).await.expect("Setting packet parameters");
//! ```
//!
//! ## Available Methods
//!
//! - [`set_flrc_modulation`](Lr2021::set_flrc_modulation) - Configure bitrate, coding rate and pulse shaping
//! - [`set_flrc_packet`](Lr2021::set_flrc_packet) - Set packet parameters (preamble, syncword, CRC, length)
//! - [`set_flrc_syncword`](Lr2021::set_flrc_syncword) - Configure one of the three possible syncwords
//! - [`get_flrc_packet_status`](Lr2021::get_flrc_packet_status) - Get status of last received packet
//! - [`get_flrc_rx_stats`](Lr2021::get_flrc_rx_stats) - Get basic reception statistics

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_flrc::*;
use super::{BusyPin, Lr2021, Lr2021Error, PulseShape};

pub struct FlrcPacketParams {
    pub agc_pbl_len: AgcPblLen,
    pub sw_len: SwLen,
    pub sw_tx: SwTx,
    pub sw_match: SwMatch,
    pub hdr_format: PktFormat,
    pub crc: Crc,
    pub pld_len: u16
}

impl FlrcPacketParams {
    pub fn new(agc_pbl_len: AgcPblLen, sw_len: SwLen, sw_tx: SwTx, sw_match: SwMatch, hdr_format: PktFormat, crc: Crc, pld_len: u16) -> Self {
        Self{agc_pbl_len, sw_len, sw_tx, sw_match, hdr_format, crc, pld_len}
    }
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Modulation parameters: raw bitrate, coding rate and pulse shaping
    pub async fn set_flrc_modulation(&mut self, bitrate: FlrcBitrate, cr: FlrcCr, pulse_shape: PulseShape) -> Result<(), Lr2021Error> {
        let req = set_flrc_modulation_params_cmd(bitrate, cr, pulse_shape);
        self.cmd_wr(&req).await
    }

    /// Set FLRC packet parameters: preamble, syncword, header implicit/explicit, CRC and packet length (max 511)
    pub async fn set_flrc_packet(&mut self, params: &FlrcPacketParams) -> Result<(), Lr2021Error> {
        let req = set_flrc_packet_params_cmd(
            params.agc_pbl_len,
            params.sw_len,
            params.sw_tx,
            params.sw_match,
            params.hdr_format,
            params.crc,
            params.pld_len);
        self.cmd_wr(&req).await
    }

    /// Configure one of the three possible syncword
    pub async fn set_flrc_syncword(&mut self, sw_num: u8, syncword: u32, is_16b: bool) -> Result<(), Lr2021Error> {
        let sw = if is_16b {syncword << 16} else {syncword};
        let req = set_flrc_syncword_cmd(sw_num, sw);
        let req_s = if is_16b {&req[..5]} else {&req};
        self.cmd_wr(req_s).await
    }

    /// Return length of last packet received
    pub async fn get_flrc_packet_status(&mut self) -> Result<FlrcPacketStatusRsp, Lr2021Error> {
        let req = get_flrc_packet_status_req();
        let mut rsp = FlrcPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_flrc_rx_stats(&mut self) -> Result<FlrcRxStatsRsp, Lr2021Error> {
        let req = get_flrc_rx_stats_req();
        let mut rsp = FlrcRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}