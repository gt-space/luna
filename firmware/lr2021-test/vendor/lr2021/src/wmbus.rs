//! # API related to WMBus operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for WMBus communication.
//! WMBus is metering protocol using ISM bands, uwing various (G)FSK modulation schemes.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for WMBus operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::wmbus::*;
//!
//! // Set packet type to WMBus
//! let mut mode = WmbusMode::ModeS;
//! let rf = mode.rf(0, WmbusSubBand::A); // Choose channel 0
//! lr2021.set_rf(rf).await.expect("SetRF");
//!
//! lr2021.set_packet_type(PacketType::Wmbus).await.expect("SetPktType");
//! let params = WmbusPacketParams::new(mode, WmbusFormat::FormatA, PLD_SIZE);
//! lr2021.set_wmbus_packet(params).await.expect("SetPktParams");
//!
//! lr2021.set_rx_continuous().await.expect("SetRX");
//! ```
//!
//! ## Available Methods
//!
//! - [`set_wmbus_packet`](Lr2021::set_wmbus_packet) - Set Wmbus packet parameters: preamble, Bandwidth, Payload length, Address filtering
//! - [`set_wmbus_address`](Lr2021::set_wmbus_address) - Configure the node address for address filtering
//! - [`get_wmbus_packet_status`](Lr2021::get_wmbus_packet_status) - Return info about last packet received: length, CRC error per block, RSSI, LQI
//! - [`get_wmbus_rx_stats`](Lr2021::get_wmbus_rx_stats) - Return basic RX stats

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_wmbus::*;
use super::{BusyPin, Lr2021, Lr2021Error, RxBw};


#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// W-MBus Packet parameters
pub struct WmbusPacketParams {
    mode: WmbusMode,
    rx_bw: RxBw,
    pkt_format_tx: WmbusFormat,
    addr_filt_en: bool,
    pld_len: u8,
    pbl_len_tx: u16,
    pbl_len_detect: u8
}

impl WmbusPacketParams {
    /// Create w new packet configruation using shortest preamble, automatic bandwidth and no address filtering
    pub fn new(mode: WmbusMode, tx_format: WmbusFormat, tx_len: u8) -> Self {
        let pbl_len = match mode {
            WmbusMode::ModeS => 30,
            WmbusMode::ModeR2 => 78,
            WmbusMode::ModeF2 => 78,
            // Mode T
            WmbusMode::ModeT1 |
            WmbusMode::ModeT2O2m |
            WmbusMode::ModeT2M2o =>38,
            // Mode C
            WmbusMode::ModeC1 |
            WmbusMode::ModeC2O2m |
            WmbusMode::ModeC2M2o => 32,
            // Mode N
            WmbusMode::ModeN4p8 |
            WmbusMode::ModeN2p4 |
            WmbusMode::ModeN6p4 |
            WmbusMode::ModeN19p2 => 16,
        };
        Self {
            mode,
            rx_bw: RxBw::BwAuto,
            pkt_format_tx: tx_format,
            addr_filt_en: false,
            pld_len: tx_len,
            pbl_len_tx: pbl_len,
            pbl_len_detect: 255, // Automatic mode
        }
    }

    /// Enable address filtering
    pub fn with_addr_filt(self) -> Self {
        Self {
            addr_filt_en: true,
            ..self
        }
    }

    /// Use custom preamble length
    pub fn with_pbl_len(self, pbl_len_tx: u16) -> Self {
        Self {
            pbl_len_tx,
            ..self
        }
    }
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Wmbus packet parameters: preamble, Bandwidth, Payload length, Address filtering
    pub async fn set_wmbus_packet(&mut self, params: WmbusPacketParams) -> Result<(), Lr2021Error> {
        let req = set_wmbus_params_cmd(params.mode, params.rx_bw, params.pkt_format_tx, params.addr_filt_en, params.pld_len, params.pbl_len_tx, params.pbl_len_detect);
        self.cmd_wr(&req).await
    }

    /// Configure the node address for address filtering
    pub async fn set_wmbus_address(&mut self, addr: u64) -> Result<(), Lr2021Error> {
        let req = set_wmbus_address_cmd(addr);
        self.cmd_wr(&req).await
    }

    /// Return info about last packet received: length, CRC error per block, RSSI, LQI
    pub async fn get_wmbus_packet_status(&mut self) -> Result<WmbusPacketStatusRsp, Lr2021Error> {
        let req = get_wmbus_packet_status_req();
        let mut rsp = WmbusPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_wmbus_rx_stats(&mut self) -> Result<WmbusRxStatsRsp, Lr2021Error> {
        let req = get_wmbus_rx_stats_req();
        let mut rsp = WmbusRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}