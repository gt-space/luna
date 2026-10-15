//! # API related to WiSUN operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for WiSUN communication.
//! WiSUN is metering protocol using ISM bands, uwing various FSK modulation schemes (2/4-FSK with modulation index from 0.5 to 2.0).
//! WiSUN protocol supports two optional convolutional encoding scheme, and the LR2021 supports only
//! the RSC (Recursive Systematic Code) for RX operation (in TX NR-NSC is possible).
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for WiSUN operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::wisun::*;
//!
//! // Set packet type to WiSUN
//! lr2021.set_packet_type(PacketType::Wisun).await.expect("SetPktType");
//! lr2021.set_wisun_modulation(mode, RxBw::BwAuto).await.expect("SetModulation");
//! let params = WisunPacketParams::new_data(12, WisunFec::Nrnsc, WisunFcsLen::Fcs16b);
//! lr2021.set_wisun_packet(params).await.expect("SetPktParams");
//!
//! lr2021.set_rx_continuous().await.expect("SetRX");
//! ```
//!
//! ## Available Methods
//!
//! - [`set_wisun_modulation`](Lr2021::set_wisun_modulation) - Set Wisun packet parameters: preamble, Bandwidth, Payload length, Address filtering
//! - [`set_wisun_packet`](Lr2021::set_wisun_packet) - Set Wisun packet parameters: preamble, Bandwidth, Payload length, Address filtering
//! - [`get_wisun_packet_status`](Lr2021::get_wisun_packet_status) - Return info about last packet received: length, CRC error per block, RSSI, LQI
//! - [`get_wisun_rx_stats`](Lr2021::get_wisun_rx_stats) - Return basic RX stats

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_wisun::*;
use super::{BusyPin, Lr2021, Lr2021Error, RxBw};

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Wisun Packet parameters: TX CRC/FEC/Length
pub struct WisunPacketParams {
    pub tx_crc: WisunFcsLen,
    pub whitening: bool,
    pub crc_hw: bool,
    pub mode_switch_tx: bool,
    pub fec_tx: WisunFec,
    pub frame_len_tx: u16,
    pub pbl_len_tx: u8,
    pub pbl_detect: u8
}

impl WisunPacketParams {
    pub fn new_data(tx_len: u16, tx_fec: WisunFec, tx_crc: WisunFcsLen) -> Self {
        Self {
            tx_crc,
            whitening: true,
            crc_hw: true,
            mode_switch_tx: false,
            fec_tx: tx_fec,
            frame_len_tx: tx_len,
            pbl_len_tx: 32,
            pbl_detect: 255,
        }
    }

    /// Use custom preamble length
    pub fn with_pbl_len(self, pbl_len_tx: u8) -> Self {
        Self {
            pbl_len_tx,
            ..self
        }
    }

    pub fn new_mode_switch() -> Self {
        Self {
            tx_crc: WisunFcsLen::Fcs16b,
            whitening: true,
            crc_hw: true,
            mode_switch_tx: true,
            fec_tx: WisunFec::None,
            frame_len_tx: 0,
            pbl_len_tx: 32,
            pbl_detect: 255,
        }
    }
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Wisun packet parameters: preamble, Bandwidth, Payload length, Address filtering
    pub async fn set_wisun_modulation(&mut self, mode: WisunMode, rx_bw: RxBw) -> Result<(), Lr2021Error> {
        let req = set_wisun_mode_cmd(mode, rx_bw);
        self.cmd_wr(&req).await
    }

    /// Set Wisun packet parameters: preamble, Bandwidth, Payload length, Address filtering
    pub async fn set_wisun_packet(&mut self, params: WisunPacketParams) -> Result<(), Lr2021Error> {
        let req = set_wisun_packet_params_cmd(params.tx_crc, params.whitening, params.crc_hw, params.mode_switch_tx, params.fec_tx, params.frame_len_tx, params.pbl_len_tx, params.pbl_detect);
        self.cmd_wr(&req).await
    }

    /// Return info about last packet received: length, CRC error per block, RSSI, LQI
    pub async fn get_wisun_packet_status(&mut self) -> Result<WisunPacketStatusRsp, Lr2021Error> {
        let req = get_wisun_packet_status_req();
        let mut rsp = WisunPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats (Numer of packet received, Number of error CRC or length)
    pub async fn get_wisun_rx_stats(&mut self) -> Result<WisunRxStatsRsp, Lr2021Error> {
        let req = get_wisun_rx_stats_req();
        let mut rsp = WisunRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}