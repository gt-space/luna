//! # API related to Z-Wave operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for Z-Wave communication.
//! Z-Wave is a smart-home IoT protocol using ISM radio bands with data rates ranging from 9.6 up to 100 kb/s.
//! It supports multiple modulation schemes (R1/R2/R3/LR1) and advanced features like multi-channel scanning,
//! address filtering, and beam frame processing.
//!
//! Note: CRC generation and check is not available when using the LR1 rate, and must be checked by the host, 
//! with FCS mode set to `FcsMode::Fifo`
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for Z-Wave operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::zwave::{ZwaveScanCfg, ZwaveAddrComp, FcsMode, ZwaveRfRegion};
//!
//! // Set packet type to Z-Wave
//! lr2021.set_packet_type(PacketType::Zwave).await.expect("Setting packet type");
//!
//! // Configure Z-Wave scan for EU region (scans R1, R2, R3 modes)
//! let scan_cfg = ZwaveScanCfg::from_region(
//!     ZwaveAddrComp::Off,    // No address filtering initially
//!     FcsMode::Auto,         // Automatic frame check sequence
//!     ZwaveRfRegion::Eu      // European region frequencies
//! );
//! lr2021.set_zwave_scan_config(&scan_cfg).await.expect("Setting scan config");
//!
//! // Start multi-channel scanning
//! lr2021.start_zwave_scan().await.expect("Starting Z-Wave scan");
//! ```
//!
//! ## Available Methods
//!
//! ### Core Z-Wave Methods
//! - [`set_zwave_packet`](Lr2021::set_zwave_packet) - Configure packet parameters (mode, bandwidth, address filtering)
//! - [`set_zwave_scan_config`](Lr2021::set_zwave_scan_config) - Configure multi-channel scanning parameters
//! - [`start_zwave_scan`](Lr2021::start_zwave_scan) - Start scanning across configured channels
//!
//! ### Address and Filtering
//! - [`set_zwave_home_id`](Lr2021::set_zwave_home_id) - Set home ID for network address filtering
//! - [`set_zwave_beam_filt`](Lr2021::set_zwave_beam_filt) - Configure beam frame filtering parameters
//!
//! ### Status and Statistics
//! - [`get_zwave_packet_status`](Lr2021::get_zwave_packet_status) - Get last packet status information
//! - [`get_zwave_rx_stats`](Lr2021::get_zwave_rx_stats) - Get basic reception statistics

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

pub use super::cmd::cmd_zwave::*;
use super::{BusyPin, Lr2021, Lr2021Error, RxBw};

#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ZwavePacketParams {
    pub mode: ZwaveMode,
    pub rx_bw: RxBw,
    pub addr_comp: ZwaveAddrComp,
    pub pld_len: u8,
    pub pbl_len_tx: u16,
    pub pbl_len_detect: u8,
    pub fcs_mode: FcsMode,
}

#[derive(Debug, Clone, Copy)]
pub enum ZwavePpduKind {
    SingleCast, MultiCast, Beam
}

impl ZwavePacketParams {
    pub fn new(mode: ZwaveMode, rx_bw: RxBw, addr_comp: ZwaveAddrComp, pld_len: u8, pbl_len_tx: u16, pbl_len_detect: u8, fcs_mode: FcsMode) -> Self {
        Self {mode, rx_bw, addr_comp, pld_len, pbl_len_tx, pbl_len_detect, fcs_mode}
    }

    pub fn from_mode(mode: ZwaveMode, kind: ZwavePpduKind, pld_len: u8) -> Self {
        let fcs_mode = if mode==ZwaveMode::Lr1 {FcsMode::Fifo} else {FcsMode::Auto};
        let pbl_len_tx = match (kind, mode) {
            (ZwavePpduKind::Beam      , ZwaveMode::Lr1) => 2,
            (_                        , ZwaveMode::Lr1) => 10,
            (ZwavePpduKind::SingleCast, _             ) => 10,
            (ZwavePpduKind::MultiCast , ZwaveMode::R1 ) => 10,
            (ZwavePpduKind::MultiCast , ZwaveMode::R2 ) => 20,
            (ZwavePpduKind::MultiCast , ZwaveMode::R3 ) => 24,
            (ZwavePpduKind::Beam      , ZwaveMode::R3 ) => 8,
            (ZwavePpduKind::Beam      , _             ) => 20,
        } * 8; // Multiply by 8 to get the number of bits instead of byte as defined in standard
        Self {
            mode,
            rx_bw: RxBw::BwAuto,
            addr_comp: ZwaveAddrComp::Off,
            pld_len,
            pbl_len_tx,
            pbl_len_detect:32,
            fcs_mode
        }
    }

    pub fn with_filt(self, beam_en: bool) -> Self {
        Self {
            mode: self.mode,
            rx_bw: self.rx_bw,
            addr_comp: if beam_en {ZwaveAddrComp::HomeidBeam} else {ZwaveAddrComp::Homeid},
            pld_len: self.pld_len,
            pbl_len_tx: self.pbl_len_tx,
            pbl_len_detect: self.pbl_len_detect,
            fcs_mode: self.fcs_mode,
        }
    }
}

#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ZwaveChanCfg {
    /// Frequency associated with this channel
    pub freq: u32,
    /// ZWave Mode selection (R1, R2, R3, LR1)
    pub mode: ZwaveMode,
    /// Timeout used in scan to switch to the next channel
    pub timeout: u8,
    /// Use CCA as a first step detection in scan
    pub cca_en: bool,
}

impl ZwaveChanCfg {
    /// Create a channel configuration for R1
    pub fn r1(freq: u32) -> Self {
        Self {
            freq,
            mode: ZwaveMode::R1,
            timeout: 19,
            cca_en: false,
        }
    }
    /// Create a channel configuration for R2
    /// Fast flag is used when scanning 4 channel
    pub fn r2(freq: u32, fast: bool) -> Self {
        Self {
            freq,
            mode: ZwaveMode::R2,
            timeout: if fast {30} else {34},
            cca_en: false,
        }
    }
    /// Create a channel configuration for R3
    /// Fast flag is used when scanning 4 channel
    pub fn r3(freq: u32, fast: bool) -> Self {
        Self {
            freq,
            mode: ZwaveMode::R3,
            timeout: if fast {7} else {8},
            cca_en: false,
        }
    }
    /// Create a channel configuration for LR1
    pub fn lr1(freq: u32) -> Self {
        Self {
            freq,
            mode: ZwaveMode::Lr1,
            timeout: 12,
            cca_en: false,
        }
    }
}

#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ZwaveScanCfg {
    pub addr_comp: ZwaveAddrComp,
    pub fcs_mode: FcsMode,
    pub nb_ch: u8,
    pub ch1 : ZwaveChanCfg,
    pub ch2 : ZwaveChanCfg,
    pub ch3 : ZwaveChanCfg,
    pub ch4 : ZwaveChanCfg,
}

pub enum ZwaveRfRegion {Anz, Cn, Eu, EuLr1, EuLr2, Hk, Il, In, Jp, Kr, Ru, Us, UsLr1, UsLr2}

impl ZwaveScanCfg {

    /// Create the scan configuration corresponding to an official region
    pub fn from_region(addr_comp: ZwaveAddrComp, fcs_mode: FcsMode, region: ZwaveRfRegion) -> Self {
        match region {
            // Base Region
            ZwaveRfRegion::Anz   => Self::base_rate(addr_comp, fcs_mode, 921_400_000, 921_400_000, 919_800_000),
            ZwaveRfRegion::Cn    => Self::base_rate(addr_comp, fcs_mode, 868_400_000, 868_400_000, 868_400_000),
            ZwaveRfRegion::Eu    => Self::base_rate(addr_comp, fcs_mode, 868_400_000, 868_400_000, 869_850_000),
            ZwaveRfRegion::Hk    => Self::base_rate(addr_comp, fcs_mode, 919_800_000, 919_800_000, 919_800_000),
            ZwaveRfRegion::Il    => Self::base_rate(addr_comp, fcs_mode, 916_000_000, 916_000_000, 916_000_000),
            ZwaveRfRegion::In    => Self::base_rate(addr_comp, fcs_mode, 865_200_000, 865_200_000, 865_200_000),
            ZwaveRfRegion::Ru    => Self::base_rate(addr_comp, fcs_mode, 869_000_000, 869_000_000, 869_000_000),
            ZwaveRfRegion::Us    => Self::base_rate(addr_comp, fcs_mode, 908_400_000, 908_400_000, 916_000_000),
            // Only R3 on 3 RF
            ZwaveRfRegion::Jp    => Self::all_r3(addr_comp, fcs_mode, 922_500_000, 923_900_000, 926_300_000),
            ZwaveRfRegion::Kr    => Self::all_r3(addr_comp, fcs_mode, 920_900_000, 921_700_000, 923_100_000),
            // Long-Range Region
            ZwaveRfRegion::EuLr1 => Self::all_rate(addr_comp, fcs_mode, 868_400_000, 868_400_000, 869_850_000, 864_400_000),
            ZwaveRfRegion::EuLr2 => Self::all_rate(addr_comp, fcs_mode, 868_400_000, 868_400_000, 869_850_000, 866_400_000),
            ZwaveRfRegion::UsLr1 => Self::all_rate(addr_comp, fcs_mode, 908_400_000, 908_400_000, 916_000_000, 912_000_000),
            ZwaveRfRegion::UsLr2 => Self::all_rate(addr_comp, fcs_mode, 908_400_000, 908_400_000, 916_000_000, 920_000_000),
        }
    }

    /// Create scanning for all base rate only (i.e no LR)
    pub fn base_rate(addr_comp: ZwaveAddrComp, fcs_mode: FcsMode, rf_r1: u32, rf_r2: u32, rf_r3: u32) -> Self {
        Self {addr_comp, fcs_mode, nb_ch: 3,
            ch1: ZwaveChanCfg::r1(rf_r1),
            ch2: ZwaveChanCfg::r2(rf_r2, false),
            ch3: ZwaveChanCfg::r3(rf_r3, false),
            ch4: ZwaveChanCfg::lr1(912_000_000),
        }
    }

    /// Create scanning for all rate including LR
    pub fn all_rate(addr_comp: ZwaveAddrComp, fcs_mode: FcsMode, rf_r1: u32, rf_r2: u32, rf_r3: u32, rf_lr1: u32) -> Self {
        Self {addr_comp, fcs_mode, nb_ch: 4,
            ch1: ZwaveChanCfg::r1(rf_r1),
            ch2: ZwaveChanCfg::r2(rf_r2, true),
            ch3: ZwaveChanCfg::r3(rf_r3, true),
            ch4: ZwaveChanCfg::lr1(rf_lr1),
        }
    }

    /// Create scanning for all rate including LR
    pub fn all_r3(addr_comp: ZwaveAddrComp, fcs_mode: FcsMode, rf_r1: u32, rf_r2: u32, rf_r3: u32) -> Self {
        Self {addr_comp, fcs_mode, nb_ch: 3,
            ch1: ZwaveChanCfg::r3(rf_r1, false),
            ch2: ZwaveChanCfg::r3(rf_r2, false),
            ch3: ZwaveChanCfg::r3(rf_r3, false),
            ch4: ZwaveChanCfg::lr1(912_000_000),
        }
    }

    /// Scan only the two LR channel
    pub fn lr_only(addr_comp: ZwaveAddrComp, fcs_mode: FcsMode, is_us: bool) -> Self {
        if is_us {
            Self {addr_comp, fcs_mode, nb_ch: 2, ch1:ZwaveChanCfg::lr1(912_000_000), ch2: ZwaveChanCfg::lr1(920_000_000), ch3: ZwaveChanCfg::lr1(919_800_000), ch4: ZwaveChanCfg::lr1(919_800_000)}
        } else {
            Self {addr_comp, fcs_mode, nb_ch: 2, ch1:ZwaveChanCfg::lr1(864_400_000), ch2: ZwaveChanCfg::lr1(866_400_000), ch3: ZwaveChanCfg::lr1(919_800_000), ch4: ZwaveChanCfg::lr1(919_800_000)}
        }
    }

    /// Constructone byte of the command containing a mix of number of channel and cca enable
    pub fn cmd_nb_ch(&self) -> u8 {
        ((self.nb_ch&0x0F) << 4) |
        if self.ch1.cca_en {1} else {0} |
        if self.ch2.cca_en {2} else {0} |
        if self.ch3.cca_en {4} else {0} |
        if self.ch4.cca_en {8} else {0}
    }

    /// Constructone byte of the command containing a mix of number of channel and cca enable
    pub fn cmd_mode(&self) -> u8 {
        (self.ch1.mode as u8) |
        ((self.ch2.mode as u8) << 2) |
        ((self.ch3.mode as u8) << 4) |
        ((self.ch4.mode as u8) << 6)
    }
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set ZWave packet parameters: preamble, syncword, header implicit/explicit, CRC and packet length (max 511)
    pub async fn set_zwave_packet(&mut self, params: &ZwavePacketParams) -> Result<(), Lr2021Error> {
        let req = set_zwave_params_cmd(
            params.mode,
            params.rx_bw,
            params.addr_comp,
            params.pld_len,
            params.pbl_len_tx,
            params.pbl_len_detect,
            params.fcs_mode);
        self.cmd_wr(&req).await
    }

    /// Sets the zwave Home ID, used as address filter in RX
    pub async fn set_zwave_home_id(&mut self, id: u32) -> Result<(), Lr2021Error> {
        let req = set_zwave_home_id_filtering_cmd(id);
        self.cmd_wr(&req).await
    }

    /// Sets the zwave Beam frame filtering
    pub async fn set_zwave_beam_filt(&mut self, beam_tag: u8, addr_len: AddrLen, node_id: u16, id_hash: u8) -> Result<(), Lr2021Error> {
        let req = set_zwave_beam_filtering_cmd(beam_tag, addr_len, node_id, id_hash);
        self.cmd_wr(&req).await
    }

    /// Configure scan: number of active channel, their mode and frequency
    pub async fn set_zwave_scan_config(&mut self, cfg: &ZwaveScanCfg) -> Result<(), Lr2021Error> {
        let req = [0x02, 0x9C,
            cfg.cmd_nb_ch(),
            cfg.cmd_mode(),
            cfg.addr_comp as u8,
            cfg.fcs_mode as u8,
            // Channel 1
            ((cfg.ch1.freq >> 24) & 0xFF) as u8,
            ((cfg.ch1.freq >> 16) & 0xFF) as u8,
            ((cfg.ch1.freq >> 8 ) & 0xFF) as u8,
            ( cfg.ch1.freq        & 0xFF) as u8,
            cfg.ch1.timeout,
            // Channel 2
            ((cfg.ch2.freq >> 24) & 0xFF) as u8,
            ((cfg.ch2.freq >> 16) & 0xFF) as u8,
            ((cfg.ch2.freq >> 8 ) & 0xFF) as u8,
            ( cfg.ch2.freq        & 0xFF) as u8,
            cfg.ch2.timeout,
            // Channel 3
            ((cfg.ch3.freq >> 24) & 0xFF) as u8,
            ((cfg.ch3.freq >> 16) & 0xFF) as u8,
            ((cfg.ch3.freq >> 8 ) & 0xFF) as u8,
            ( cfg.ch3.freq        & 0xFF) as u8,
            cfg.ch3.timeout,
            // Channel 4
            ((cfg.ch4.freq >> 24) & 0xFF) as u8,
            ((cfg.ch4.freq >> 16) & 0xFF) as u8,
            ((cfg.ch4.freq >> 8 ) & 0xFF) as u8,
            ( cfg.ch4.freq        & 0xFF) as u8,
            cfg.ch4.timeout,
        ];
        let len = match cfg.nb_ch {
            2 => 16,
            3 => 21,
            _ => 26,
        };
        self.cmd_wr(&req[..len]).await
    }

    /// Start the ZWave Scan: it will alternate between up to 4 channels to find an incoming packet
    pub async fn start_zwave_scan(&mut self) -> Result<(), Lr2021Error> {
        let req = set_zwave_scan_cmd();
        self.cmd_wr(&req).await
    }

    /// Return length of last packet received
    pub async fn get_zwave_packet_status(&mut self) -> Result<ZwavePacketStatusRsp, Lr2021Error> {
        let req = get_zwave_packet_status_req();
        let mut rsp = ZwavePacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return basic RX stats
    pub async fn get_zwave_rx_stats(&mut self) -> Result<ZwaveRxStatsRsp, Lr2021Error> {
        let req = get_zwave_rx_stats_req();
        let mut rsp = ZwaveRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

}