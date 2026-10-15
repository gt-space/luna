//! # API related to BLE operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for Bluetooth Low Energy (BLE) communication.
//! It includes methods for setting modulation parameters, configuring BLE channels, transmitting packets, and retrieving statistics.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for BLE operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::ble::{BleMode, ChannelType};
//!
//! // Set packet type to BLE
//! lr2021.set_packet_type(PacketType::Ble).await.expect("Setting packet type to BLE");
//!
//! // Configure BLE modulation (1Mb/s)
//! lr2021.set_ble_modulation(BleMode::Le1mb).await.expect("Setting BLE mode (1Mb/s)");
//!
//! // Configure BLE parameters (advertising channel)
//! lr2021.set_ble_params(
//!     false,                    // CRC not in FIFO
//!     ChannelType::Advertiser,  // Channel type
//!     0x53,                     // Whitening init (for channel 37)
//!     0x555555,                 // CRC init
//!     0x8e89bed6               // Sync word/Access code
//! ).await.expect("Set BLE params");
//! ```
//!
//! ## Available Methods
//!
//! ### Core BLE Methods
//! - [`set_ble_modulation`](Lr2021::set_ble_modulation) - Configure BLE modulation mode (1M, 2M, 500k, 125k)
//! - [`set_ble_params`](Lr2021::set_ble_params) - Set BLE channel parameters (whitening, CRC, sync word)
//! - [`set_ble_tx`](Lr2021::set_ble_tx) - Set PDU length and transmit packet
//! - [`set_ble_tx_pdu_len`](Lr2021::set_ble_tx_pdu_len) - Set PDU length for pin-triggered transmission
//! - [`patch_ble_coded`](Lr2021::patch_ble_coded) - Patch some settings when BLE Coded is used
//!
//! ### Status and Statistics
//! - [`get_ble_packet_status`](Lr2021::get_ble_packet_status) - Get status of last received packet
//! - [`get_ble_rx_stats`](Lr2021::get_ble_rx_stats) - Get basic reception statistics
//!
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::constants::*;

pub use super::cmd::cmd_ble::*;
use super::{BusyPin, Lr2021, Lr2021Error};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Sampling period of Angle of Arrival data
pub enum AoaSampling {
    Cte1us = 0, Cte2us = 1
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Constant Tone Extension kind from last received header
pub enum CteKind {
    AoA = 0, AoD1us = 1, AoD2us = 2
}

impl From<u8> for CteKind {
    fn from(value: u8) -> Self {
        match value & 3 {
            2 => CteKind::AoD2us,
            1 => CteKind::AoD1us,
            _ => CteKind::AoA,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Info on constant tone extension from last received packet
pub struct CteInfo {
    /// Number of CTE sample stored
    pub nb_sample: u8,
    /// Kind of CTE (extracted from header)
    pub kind: CteKind,
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set BLE Mode (1M, 2M, 500k, 125k)
    pub async fn set_ble_modulation(&mut self, mode: BleMode) -> Result<(), Lr2021Error> {
        let req = set_ble_modulation_params_cmd(mode);
        self.cmd_wr(&req).await?;
        // Applu some extra configuration to pass certification
        match mode {
            BleMode::Le1mb => {}
            // Fix preamble length for 2Mb/s
            BleMode::Le2mb => {
                self.cmd_wr(&[0x02,0x30,0x01,0x21,0x00,0x07,0x00]).await?;
            },
            // Coded mode
            BleMode::LeCoded500k | BleMode::LeCoded125k => {
                self.patch_ble_coded(None).await?;
            }
        }
        Ok(())
    }

    /// Set BLE parameters: Channel type (advertising, Header16/24b), Whitening & CRC init, SyncWord/AccessCode
    /// Call before `set_ble_modulation` to ensure BLE coded settings are not overwritten
    pub async fn set_ble_params(&mut self, crc_in_fifo: bool, channel_type: ChannelType, whit_init: u8, crc_init: u32, syncword: u32) -> Result<(), Lr2021Error> {
        let req = set_ble_channel_params_cmd(crc_in_fifo, channel_type, whit_init, crc_init, syncword);
        self.cmd_wr(&req).await
    }

    /// Set the PDU length and send the packet
    /// PDU must be ready in FIFO
    pub async fn set_ble_tx(&mut self, len: u8) -> Result<(), Lr2021Error> {
        let req = set_ble_tx_cmd(len);
        self.cmd_wr(&req).await
    }

    /// Set the PDU length
    /// Useful compare to set_ble_tx when using a pin trigger to control the exact transmit time
    pub async fn set_ble_tx_pdu_len(&mut self, len: u8) -> Result<(), Lr2021Error> {
        let req = set_ble_tx_pdu_len_cmd(len);
        self.cmd_wr(&req).await
    }

    /// Return length of last packet received
    pub async fn get_ble_packet_status(&mut self) -> Result<BlePacketStatusRsp, Lr2021Error> {
        let req = get_ble_packet_status_req();
        let mut rsp = BlePacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_ble_rx_stats(&mut self) -> Result<BleRxStatsRsp, Lr2021Error> {
        let req = get_ble_rx_stats_req();
        let mut rsp = BleRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Patch some settings when BLE Coded is used
    /// This fixes some issue related to BLE certification
    /// Automatically called by `set_ble_modulation` (without a retention slot)
    /// Note that two retention slot are used (slot and slot+1)
    pub async fn patch_ble_coded(&mut self, ret_en: Option<u8>) -> Result<(), Lr2021Error> {
        //
        if let Some(slot) = ret_en {
            self.add_register_to_retention(slot,ADDR_CPFSK_DEMOD).await?;
            self.add_register_to_retention(slot+1,ADDR_CPFSK_DETECT).await?;
        }
        // Fix preamble polarity
        self.cmd_wr(&[0x02,0x30,0x01,0x20,0x00,0x09,0x00]).await?;
        // Change tracking to support Dirty TX certification
        self.wr_reg_mask(ADDR_CPFSK_DEMOD, 0x0020, 0x00).await?;
        // Change detection settings to pass blocking certification
        self.wr_reg_mask(ADDR_CPFSK_DETECT, 0x007F, 0x7C).await
    }

}
