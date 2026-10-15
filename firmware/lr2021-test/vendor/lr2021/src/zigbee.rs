//! # API related to Zigbee operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for Zigbee communication.
//! Zigbee is a smart-home IoT protocol using the ISM band and 2.4GHz with a data rate up to 250 kb/s.
//! The LR2021 supports only the the 250kb/s rate in the 2.4GHz which uses a direct-sequence spread spectrum
//! with FSK modulation. It supports address filtering except for multi-cast (multi-cast packet are not filtered).
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for Z-Wave operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::zigbee::*;
//!
//! // Configure chip for Zigbee for rate 250kb/s
//! lr2021.set_packet_type(PacketType::Zigbee).await.expect("Setting packet type");
//! let params = ZigbeePacketParams::new(ZigbeeMode::Oqpsk250, 127, false);
//! lr2021.set_zigbee_packet(&params).await.expect("SetPkt");
//! lr2021.set_rx(0xFFFFFFFF, true).await.expect("SetRX");
//! ```
//!
//! ## Available Methods
//!
//! - [`set_zigbee_packet`](Lr2021::set_zigbee_packet) - Set Zigbee packet parameters
//! - [`set_zigbee_packet_len`](Lr2021::set_zigbee_packet_len) - Set only the zigbee packet length for transmission
//! - [`set_zigbee_address`](Lr2021::set_zigbee_address) - Configure the different Zigbee addresses for filtering in RX.
//! - [`get_zigbee_packet_status`](Lr2021::get_zigbee_packet_status) - Return length of last packet received
//! - [`get_zigbee_rx_stats`](Lr2021::get_zigbee_rx_stats) - Return basic RX stats
//!

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_zigbee::*;
use super::{BusyPin, Lr2021, Lr2021Error, RxBw};

#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ZigbeePacketParams {
    pub mode: ZigbeeMode,
    pub rx_bw: RxBw,
    pub pld_len: u8,
    pub pbl_len_tx: u16,
    pub addr_filt_en: bool,
    pub fcs_mode: FcsMode,
}

impl ZigbeePacketParams {
    pub fn new(mode: ZigbeeMode, pld_len: u8, addr_filt_en: bool) -> Self {
        Self {
            mode,
            rx_bw: RxBw::BwAuto,
            pld_len,
            pbl_len_tx: 32,
            addr_filt_en,
            fcs_mode: FcsMode::FcsOn,
        }
    }
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set Zigbee packet parameters: preamble, Bandwidth, Payload length, Address filtering, FCS handling (software/Hardware)
    pub async fn set_zigbee_packet(&mut self, params: &ZigbeePacketParams) -> Result<(), Lr2021Error> {
        let req = set_zigbee_params_cmd(
            params.mode,
            params.rx_bw,
            params.pld_len,
            params.pbl_len_tx,
            params.addr_filt_en,
            params.fcs_mode);
        self.cmd_wr(&req).await
    }

    /// Sets the zigbee packet length without calling set_zigbee_packet which takes longer
    /// The function set_zigbee_packet must have been called once before !
    pub async fn set_zigbee_packet_len(&mut self, pld_len: u8) -> Result<(), Lr2021Error> {
        let req = set_zigbee_packet_len_cmd(pld_len);
        self.cmd_wr(&req).await
    }

    /// Configure the different Zigbee addresses for filtering in RX.
    /// Frames that don't match the addresses raise an address error IRQ and reception is aborted.
    /// When a packet is received, the destination address and PAN ID are both checked.
    /// Multi-cast is not supported or filtered and must be handled by the host
    pub async fn set_zigbee_address(&mut self, long_dest_addr: u64, short_dest_addr: u16, pan_id: u16, trans_id: u8) -> Result<(), Lr2021Error> {
        let req = set_zigbee_address_cmd(long_dest_addr, short_dest_addr, pan_id, trans_id);
        self.cmd_wr(&req).await
    }

    /// Return length of last packet received
    pub async fn get_zigbee_packet_status(&mut self) -> Result<ZigbeePacketStatusRsp, Lr2021Error> {
        let req = get_zigbee_packet_status_req();
        let mut rsp = ZigbeePacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_zigbee_rx_stats(&mut self) -> Result<ZigbeeRxStatsRsp, Lr2021Error> {
        let req = get_zigbee_rx_stats_req();
        let mut rsp = ZigbeeRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}