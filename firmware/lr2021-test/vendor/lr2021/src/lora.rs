//! # API related to LoRa operations
//!
//! This module provides an API for configuring and operating the LR2021 chip for LoRa (Long Range) communication.
//! LoRa is a Semtech proprietary modulation using chirp spread-spectrum, providing the highest sensitivity modulation
//! supported by the LR2021, ideal for communication over multiple kilometers at low bit-rate.
//!
//! ## Quick Start
//!
//! Here's a typical sequence to initialize the chip for LoRa operations:
//!
//! ```rust,no_run
//! use lr2021::radio::PacketType;
//! use lr2021::lora::{Sf, LoraBw};
//!
//! // Set packet type to LoRa
//! lr2021.set_packet_type(PacketType::Lora).await.expect("Setting packet type");
//!
//! // Configure LoRa parameters: modulation & packet format (10 bytes with header and CRC)
//! let modulation = LoraModulationParams::basic(Sf::Sf5, LoraBw::Bw1000);
//! let packet_params = LoraPacketParams::basic(10, &modulation);
//!
//! lr2021.set_lora_modulation(&modulation).await.expect("Setting LoRa modulation");
//! lr2021.set_lora_packet(&packet_params).await.expect("Setting packet parameters");
//! ```
//!
//! ## Available Methods
//!
//! ### Core LoRa Methods
//! - [`set_lora_modulation`](Lr2021::set_lora_modulation) - Configure spreading factor, bandwidth, coding rate, and LDRO
//! - [`set_lora_packet`](Lr2021::set_lora_packet) - Set packet parameters (preamble, payload length, header type, CRC)
//! - [`set_lora_syncword`](Lr2021::set_lora_syncword) - Set syncword using legacy 1-byte format
//! - [`set_lora_syncword_ext`](Lr2021::set_lora_syncword_ext) - Set syncword using extended 2-byte format
//! - [`set_lora_synch_timeout`](Lr2021::set_lora_synch_timeout) - Configure synchronization timeout
//! - [`set_lora_address`](Lr2021::set_lora_address) - Set address filtering parameters
//!
//! ### Status and Statistics
//! - [`get_lora_packet_status`](Lr2021::get_lora_packet_status) - Get basic packet status information
//! - [`get_lora_rx_stats`](Lr2021::get_lora_rx_stats) - Get reception statistics
//!
//! ### Channel Activity Detection (CAD)
//! - [`set_lora_cad_params`](Lr2021::set_lora_cad_params) - Configure CAD parameters
//! - [`set_lora_cad`](Lr2021::set_lora_cad) - Start channel activity detection
//!
//! ### Misc Features
//! - [`comp_sx127x_sf6_sw`](Lr2021::comp_sx127x_sf6_sw) - Enable SX127x compatibility for SF6 and syncword format
//! - [`comp_sx127x_hopping`](Lr2021::comp_sx127x_hopping) - Enable compatibility with SX127x for frequency hopping communication
//! - [`set_lora_preamble_modulation`](Lr2021::set_lora_preamble_modulation) - Enable preamble phase modulation
//! - [`set_lora_blanking`](Lr2021::set_lora_blanking) - Configure blanking (algorithm to reduce impact of interferers)
//! - [`set_lora_hopping`](Lr2021::set_lora_hopping) - Configure intra-packet frequency hopping
//! - [`set_lora_freq_range`](Lr2021::set_lora_freq_range) - Configure the frequency error range supported by detection
//!
//! ### Side-Detection (Multi-SF receiver)
//! - [`set_lora_sidedet_cfg`](Lr2021::set_lora_sidedet_cfg) - Configure side-detector for multiple SF detection
//! - [`set_lora_sidedet_syncword`](Lr2021::set_lora_sidedet_syncword) - Configure side-detector syncwords
//!
//! ### Ranging Operations
//! - [`set_ranging_modulation`](Lr2021::set_ranging_modulation) - Set Modulation for ranging operation
//! - [`set_ranging_dev_addr`](Lr2021::set_ranging_dev_addr) - Set device address for ranging
//! - [`set_ranging_req_addr`](Lr2021::set_ranging_req_addr) - Set request address for ranging
//! - [`set_ranging_txrx_delay`](Lr2021::set_ranging_txrx_delay) - Set ranging calibration delay
//! - [`set_ranging_params`](Lr2021::set_ranging_params) - Configure ranging parameters (extended/spy mode)
//! - [`get_ranging_result`](Lr2021::get_ranging_result) - Get basic ranging results
//! - [`get_ranging_ext_result`](Lr2021::get_ranging_ext_result) - Get extended ranging results
//! - [`get_ranging_gain`](Lr2021::get_ranging_gain) - Get ranging gain steps (debug)
//! - [`get_ranging_stats`](Lr2021::get_ranging_stats) - Get ranging statistics
//! - [`get_ranging_rssi_offset`](Lr2021::get_ranging_rssi_offset) - Return a correction offset on ranging RSSI
//! - [`patch_ranging_rf`](Lr2021::patch_ranging_rf) - Patch the RF setting for ranging operation
//!
//! ### Timing Synchronization
//! - [`set_lora_timing_sync`](Lr2021::set_lora_timing_sync) - Configure timing synchronization mode
//! - [`set_lora_timing_sync_pulse`](Lr2021::set_lora_timing_sync_pulse) - Configure timing sync pulse parameters

use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::constants::*;
use crate::system::DioNum;

pub use super::cmd::cmd_lora::*;
pub use super::cmd::cmd_ranging::*;
use super::{BusyPin, Lr2021, Lr2021Error};

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// LoRa Modulation parameters: SF, Bandwidth, Code-rate, LDRO
pub struct LoraModulationParams {
    /// Spreading factor
    pub sf: Sf,
    /// Bandwidth
    pub bw: LoraBw,
    /// Coding Rate
    pub cr: LoraCr,
    /// Low Data-Rate Optimisation
    pub ldro: Ldro,
}

impl LoraModulationParams {
    /// Modulation with default coderate (4/5) and LDRO based on SF/BW
    pub fn basic(sf: Sf, bw: LoraBw) -> Self {
        let ldro_en = (sf==Sf::Sf12 && !matches!(bw,LoraBw::Bw1000|LoraBw::Bw500))
                    || (sf==Sf::Sf11 && !matches!(bw,LoraBw::Bw1000|LoraBw::Bw500|LoraBw::Bw250) );
        Self {
            sf, bw,
            cr: LoraCr::Cr1Ham45Si,
            ldro: if ldro_en {Ldro::On} else {Ldro::Off},
        }
    }

    /// Modulation with default coderate (4/5) and LDRO based on SF/BW
    pub fn new(sf: Sf, bw: LoraBw, cr: LoraCr, ldro: Ldro) -> Self {
        Self {sf, bw, cr, ldro}
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// LoRa Modulation parameters: SF, Bandwidth, Code-rate, LDRO
pub struct LoraPacketParams {
    /// Preamble length (in symbol)
    pub pbl_len: u16,
    /// Payload length (in byte)
    pub payload_len: u8,
    /// Explicit or implicit header
    pub header_type: HeaderType,
    /// CRC Enable
    pub crc_en: bool,
    /// Chirp direction
    pub invert_iq: bool,
}

impl LoraPacketParams {
    /// Default Packet parameters (Explicit header with CRC and standard direction)
    pub fn basic(payload_len: u8, modulation: &LoraModulationParams) -> Self {
        Self {
            pbl_len: if modulation.sf < Sf::Sf7 {12} else {8},
            payload_len,
            header_type: HeaderType::Explicit,
            crc_en: true,
            invert_iq: false
        }
    }

    /// Modulation with default coderate (4/5) and LDRO based on SF/BW
    pub fn new(pbl_len: u16, payload_len: u8, header_type: HeaderType, crc_en: bool, invert_iq: bool) -> Self {
        Self {pbl_len, payload_len, header_type, crc_en, invert_iq}
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// LoRa CAD parameters: SF, Bandwidth, Code-rate, LDRO
pub struct LoraCadParams {
    /// Number of symbols (1 to 15)
    nb_symbols: u8,
    /// Configure CAD to detect only preamble rather than any LoRa symbols
    pub preamble_only: bool ,
    /// Exit Mode: Idle, RX (low power detect on long preamble) or TX (for Listen-Before-Talk)
    pub exit_mode: ExitMode,
    /// timeout for the following RX or TX if exit mode is not CAD_ONLY
    pub timeout: u32,
    /// Detection threshold
    pub thr: u8,
    /// Delta applied on threashold at each symbols to shorten CAD when there is no LoRa symbols
    pub delta: u8,
}

/// Recommended CAD threshold for a given SF and number of symbols
pub fn lora_cad_thr(sf: Sf, nb_symbols: u8) -> u8 {
    let base_symb = match nb_symbols {
        0 | 1 => 60,
        2 => 56,
        3 => 52,
        _ => 51
    };
    let offset_sf = match sf {
        Sf::Sf5 => 0,
        Sf::Sf6 => 1,
        Sf::Sf7 => 2,
        Sf::Sf8 => 3,
        Sf::Sf9 => 5,
        Sf::Sf10 => 6,
        Sf::Sf11 => 8,
        Sf::Sf12 => 10,
    };
    base_symb + offset_sf
}

impl LoraCadParams {

    /// Create CAD parameter for a CAD only operation
    pub fn new_cad_only(sf: Sf, nb_symbols: u8, fast: bool) -> Self {
        let nb_symbols = nb_symbols.clamp(1,15);
        let thr = lora_cad_thr(sf, nb_symbols);
        LoraCadParams {
            nb_symbols,
            preamble_only: false,
            exit_mode: ExitMode::CadOnly,
            timeout: 0,
            delta: if fast {8} else {0},
            thr
        }
    }

    /// Create CAD parameter with automatic detection threshold
    pub fn new_auto(sf: Sf, nb_symbols: u8, exit_mode: ExitMode, timeout: u32, fast: bool) -> Self {
        let nb_symbols = nb_symbols.clamp(1,15);
        let thr = lora_cad_thr(sf, nb_symbols);
        let delta = if fast {8} else {0};
        let preamble_only = exit_mode==ExitMode::CadRx;
        LoraCadParams {nb_symbols, preamble_only, exit_mode, timeout, thr, delta}
    }

    /// Create CAD parameter with manual detection threshold
    pub fn new(nb_symbols: u8, thr: u8, exit_mode: ExitMode, timeout: u32, delta: u8) -> Self {
        let nb_symbols = nb_symbols.clamp(1,15);
        let delta = delta&0xF;
        let preamble_only = exit_mode==ExitMode::CadRx;
        LoraCadParams {nb_symbols, preamble_only, exit_mode, timeout, thr, delta}
    }
}

// Recommneded delay for ranging
// One line per bandwidth: 1000, 812, 500, 406, 250, 203, 125
const RANGING_DELAY : [u32; 56] = [
    21711, 21729, 21733, 21715, 21669, 21577, 21391, 21016,
    25547, 25596, 25599, 25652, 25683, 25765, 25918, 26283,
    20616, 20607, 20567, 20480, 20307, 19959, 19258, 17860,
    24460, 24548, 24547, 24624, 24936, 25264, 26084, 27689,
    20149, 20141, 20100, 20013, 19837, 19486, 18787, 17386,
    23975, 24087, 24089, 24191, 24356, 24806, 25560, 27153,
    19688, 19649, 19560, 19387, 19043, 18350, 16967, 14191,
];

#[derive(Debug, Clone, Copy)]
pub struct SidedetCfg(u8);
impl SidedetCfg {
    pub fn new(sf: Sf, ldro: Ldro, inv: bool) -> Self{
        let b = ((sf as u8) << 4) |
            (ldro as u8) << 2 |
            if inv {1} else {0};
        Self(b)
    }

    pub fn to_byte(&self) -> u8 {
        self.0
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// LoRa Blanking configuration
pub struct BlankingCfg {
    /// Threshold on SNR margin (0.5dB) to enable symbol domain blanking (0-15)
    pub snr_thr     : u8,
    /// Gain (0-3) to adapt threshold based on average SNR
    pub thr_gain: u8,
    /// Symbol domain blanking coefficient (0 to 3, with 0 being hard-blanking)
    pub symb_gain: u8,
    /// Threshold on RSSI (0.5dB) for time domain blanking (0-15)
    pub rssi_thr: u8,
    /// Enable Time domain blanking during detection
    pub detect  : bool,
}

impl BlankingCfg {

    /// Blanking disabled
    pub fn off() -> Self {
        Self {
            thr_gain: 0,
            snr_thr : 0,
            symb_gain: 0,
            detect  : false,
            rssi_thr: 0,
        }
    }

    /// Blanking enabled at symbol domain only
    pub fn symbol() -> Self {
        Self {
            thr_gain: 2,
            snr_thr : 8,
            symb_gain: 2,
            detect  : false,
            rssi_thr: 0,
        }
    }

    /// Blanking enabled at time-Domain & symbol domain
    pub fn td_symb() -> Self {
        Self {
            thr_gain: 2,
            snr_thr : 8,
            symb_gain: 2,
            detect  : false,
            rssi_thr: 7,
        }
    }

    /// Blanking fully enabled including during detection
    pub fn full() -> Self {
        Self {
            thr_gain: 2,
            snr_thr : 8,
            symb_gain: 2,
            detect  : true,
            rssi_thr: 7,
        }
    }
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Frequency estimation during ranging exchange (valid only on responder side)
pub struct RangingFei {
    /// Frequency estimation on first exchange
    pub fei1: i32,
    /// Frequency estimation on second exchange
    pub fei2: i32,
}


#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Define duration of the TimingSync pulse of the responder
pub enum TimingSyncPulseWidth {
    W1 = 0, W5 = 1, W52 = 2, W520 = 3, W5200 = 4, W52k = 5, W260k = 6, W1024k = 7
}

#[derive(Debug, Clone, Copy, Default)]
/// Define Frequency range toelrated by detector
pub enum FreqRange {#[default]
    /// +/- Bandwidth/4
    Narrow = 0,
    /// +/- Bandwidth/3
    Medium = 1,
    /// +/- Bandwidth/2
    Wide = 2,
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set LoRa Modulation parameters
    pub async fn set_lora_modulation(&mut self, params: &LoraModulationParams) -> Result<(), Lr2021Error> {
        let req = set_lora_modulation_params_cmd(params.sf, params.bw, params.cr, params.ldro, LoraFilter::Auto);
        self.cmd_wr(&req).await
    }

    /// Set LoRa Modulation parameters for ranging operation
    pub async fn set_ranging_modulation(&mut self, params: &LoraModulationParams, is_initiator: bool) -> Result<(), Lr2021Error> {
        let filter = match (params.bw.is_fractional(),is_initiator) {
            (true, true) => LoraFilter::Dcc,
            (true, false) => LoraFilter::DccF,
            _ => LoraFilter::Auto,
        };
        let req = set_lora_modulation_params_cmd(params.sf, params.bw, params.cr, params.ldro, filter);
        self.cmd_wr(&req).await
    }

    /// Set LoRa Packet parameters
    pub async fn set_lora_packet(&mut self, params: &LoraPacketParams) -> Result<(), Lr2021Error> {
        let req = set_lora_packet_params_cmd(params.pbl_len, params.payload_len, params.header_type, params.crc_en, params.invert_iq);
        self.cmd_wr(&req).await
    }

    /// Set LoRa Syncword using legacy (SX127x) 1B notation: 0x34 for public network, 0x12 for private
    pub async fn set_lora_syncword(&mut self, syncword: u8) -> Result<(), Lr2021Error> {
        let req = set_lora_syncword_cmd(syncword);
        self.cmd_wr(&req).await
    }

    /// Set LoRa Syncword, using 2B notation (2 values on 5b signed each)
    /// Public network is (6,8) and private network is (2,4)
    pub async fn set_lora_syncword_ext(&mut self, s1: i8, s2: i8) -> Result<(), Lr2021Error> {
        let req = set_lora_syncword_extended_cmd((s1&0x1F) as u8, (s2&0x1F) as u8);
        self.cmd_wr(&req).await
    }

    /// Set synchronisation timeout
    /// Timeout is given in number of symbol: either the direct value or with mantissa/exponent (like SX126x)
    pub async fn set_lora_synch_timeout(&mut self, timeout: u8, format: TimeoutFormat) -> Result<(), Lr2021Error> {
        let req = set_lora_synch_timeout_cmd(timeout, format);
        self.cmd_wr(&req).await
    }

    /// Set address for address filtering
    /// Length is the address length in number of byte 0 (no address filtering, default) up to 8
    /// Pos is the first byte in the payload the address appears
    pub async fn set_lora_address(&mut self, len: AddrLen, pos: u8, addr: u64) -> Result<(), Lr2021Error> {
        let req = set_lora_address_cmd(len, pos, addr);
        let len = 2 + (len as usize);
        self.cmd_wr(&req[..len]).await
    }

    /// Return Information about last packet received
    pub async fn get_lora_packet_status(&mut self) -> Result<LoraPacketStatusRsp, Lr2021Error> {
        let req = get_lora_packet_status_req();
        let mut rsp = LoraPacketStatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return RX statistics: packet received, CRC errors, ...
    pub async fn get_lora_rx_stats(&mut self) -> Result<LoraRxStatsRsp, Lr2021Error> {
        let req = get_lora_rx_stats_req();
        let mut rsp = LoraRxStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Set LoRa Channel Activity Detection parameters
    pub async fn set_lora_cad_params(&mut self, params: &LoraCadParams) -> Result<(), Lr2021Error> {
        let req = set_lora_cad_params_cmd(params.nb_symbols, params.preamble_only, params.delta, params.exit_mode, params.timeout, params.thr);
        self.cmd_wr(&req).await
    }

    /// Start a LoRa Channel Activity Detection (CAD)
    pub async fn set_lora_cad(&mut self) -> Result<(), Lr2021Error> {
        let req = set_lora_cad_cmd();
        self.cmd_wr(&req).await
    }

    /// Enable compatibility with SX127x for SF6 communication and syncword format
    /// When enabled, use `set_lora_syncword_ext` to configure syncword with only even value in the range 0..30
    /// Must be called after each SetLoraModulation
    /// The retention enable allows to define a register slot to save this compatibility mode in retention
    pub async fn comp_sx127x_sf6_sw(&mut self, en: bool, ret_en: Option<u8>) -> Result<(), Lr2021Error> {
        let value = if en {2} else {0};
        self.wr_field(ADDR_LORA_PARAM, value, 18, 2).await?;
        if let Some(slot) = ret_en {
            self.add_register_to_retention(slot,ADDR_LORA_PARAM).await?;
        }
        Ok(())
    }

    /// Enable compatibility with SX127x for frequency hopping communication
    /// The retention enable allows to define a register slot to save this compatibility mode in retention
    pub async fn comp_sx127x_hopping(&mut self, en: bool, ret_en: Option<u8>) -> Result<(), Lr2021Error> {
        let value = if en {1} else {0};
        self.wr_field(ADDR_LORA_TX_CFG1, value, 18, 1).await?;
        if let Some(slot) = ret_en {
            self.add_register_to_retention(slot,ADDR_LORA_TX_CFG1).await?;
        }
        Ok(())
    }

    #[allow(clippy::get_first)]
    /// Configure Side-Detector allowing multiple SF to be detected
    /// Must be called after set_lora_modulation
    /// If cfg is an empty slice, this disabled all side-detector
    pub async fn set_lora_sidedet_cfg(&mut self, cfg: &[SidedetCfg]) -> Result<(), Lr2021Error> {
        let req = [
            0x02, 0x24,
            cfg.get(0).map(|c| c.to_byte()).unwrap_or(0),
            cfg.get(1).map(|c| c.to_byte()).unwrap_or(0),
            cfg.get(2).map(|c| c.to_byte()).unwrap_or(0),
        ];
        let len = cfg.len() + 2;
        self.cmd_wr(&req[..len]).await
    }

    #[allow(clippy::get_first)]
    /// Configure Side-Detector Syncword using basic syncword format
    pub async fn set_lora_sidedet_syncword(&mut self, sw: &[u8]) -> Result<(), Lr2021Error> {
        let req = [
            0x02, 0x25,
            sw.get(0).copied().unwrap_or(0x24),
            sw.get(1).copied().unwrap_or(0x24),
            sw.get(2).copied().unwrap_or(0x24),
        ];
        let len = sw.len() + 2;
        self.cmd_wr(&req[..len]).await
    }

    /// Configure the frequency error range supported by detection
    /// Medium range (+/-BW/3) has only a very minor sensitivity impact while the max range can degrade sensitivity by 2dB
    pub async fn set_lora_freq_range(&mut self, range: FreqRange) -> Result<(), Lr2021Error> {
        self.wr_field(ADDR_LORA_RX_CFG, range as u32, 16, 2).await
    }

    /// Long preamble can be modulated in phase in order to provide information about how many symbols are left
    /// This allows a receiver to go back to sleep if beginning of the frame starts in a long time
    pub async fn set_lora_preamble_modulation(&mut self, en: bool, dram_ret: u8, wakeup_time: u16, min_sleep_time: u32) -> Result<(), Lr2021Error> {
        let req = config_lora_preamble_modulation_cmd(en, dram_ret, wakeup_time, min_sleep_time);
        self.cmd_wr(&req).await
    }

    /// Configure blanking (algorithm to reduce impact of interferers)
    /// Works best when long interleaving is enabled (i.e. any CR > 4)
    pub async fn set_lora_blanking(&mut self, cfg: BlankingCfg) -> Result<(), Lr2021Error> {
        let req = set_lora_blanking_cmd(cfg.thr_gain, cfg.snr_thr, cfg.symb_gain, cfg.detect, cfg.rssi_thr);
        self.cmd_wr(&req).await
    }

    /// Configure intra-packet frequency hopping
    /// Provide an empty slice of hops to disable hopping
    /// Max number of hops if 40
    pub async fn set_lora_hopping(&mut self, period: u16, freq_hops: &[u32]) -> Result<(), Lr2021Error> {
        let buffer = self.buffer.as_mut();
        buffer[0] = 0x02;
        buffer[1] = 0x2C;
        buffer[2] = if freq_hops.is_empty() {0} else {0x40 | ((period>>8) as u8 & 0x1F)};
        buffer[3] = (period & 0xFF) as u8;
        for (i, f) in freq_hops.iter().enumerate() {
            buffer[4+4*i] = ((f >> 24) & 0xFF) as u8;
            buffer[5+4*i] = ((f >> 16) & 0xFF) as u8;
            buffer[6+4*i] = ((f >>  8) & 0xFF) as u8;
            buffer[7+4*i] = ( f        & 0xFF) as u8;
        }
        let len = 3 + 4*freq_hops.len();
        self.cmd_buf_wr(len).await
    }

    /// Patch the RF setting for ranging operation
    /// This ensure the RF channel setting is coherent with PLL configuration
    /// MUST be called after a `set_rf` or `patch_dcdc`
    pub async fn patch_ranging_rf(&mut self) -> Result<(), Lr2021Error> {
        self.wr_reg_mask(ADDR_FREQ_RF, 0x7F, 0).await
    }

    /// Set the device address for ranging operation
    /// The device will answer to ranging request only if the request address matches the device address
    /// The length allows to define how many bytes from the address are checked (starting from LSB)
    pub async fn set_ranging_dev_addr(&mut self, addr: u32, length: Option<CheckLength>) -> Result<(), Lr2021Error> {
         let req = set_ranging_addr_cmd(addr, length.unwrap_or(CheckLength::Addr32b));
        self.cmd_wr(&req).await
   }

    /// Set the request address for ranging operation
    pub async fn set_ranging_req_addr(&mut self, addr: u32) -> Result<(), Lr2021Error> {
         let req = set_ranging_req_addr_cmd(addr);
        self.cmd_wr(&req).await
    }

    /// Set the ranging calibration value
    pub async fn set_ranging_txrx_delay(&mut self, delay: u32) -> Result<(), Lr2021Error> {
         let req = set_ranging_tx_rx_delay_cmd(delay);
        self.cmd_wr(&req).await
   }

    /// Get the base delay for ranging depdending on bandwidth and SF
    /// Delay was calibrated only for bandwidth 125kHz and higher.
    /// For lower bandwidth (not recommended to use in ranging) a crude estimation is provided
    /// Note: the board itself will introduce some offset which should not be dependent on SF
    /// but might vary with bandwidth.
    pub fn get_ranging_base_delay(&self, modulation: &LoraModulationParams) -> u32 {
        let offset = match modulation.bw {
            LoraBw::Bw1000 =>  0,
            LoraBw::Bw812  =>  8,
            LoraBw::Bw500  => 16,
            LoraBw::Bw406  => 24,
            LoraBw::Bw250  => 32,
            LoraBw::Bw203  => 40,
            LoraBw::Bw125  => 48,
            _              => 56
        };
        let idx = offset + (modulation.sf as usize - 5);
        RANGING_DELAY.get(idx).copied().unwrap_or(18000 - (5600 >> (12 - modulation.sf as u32)))
    }

    /// Set the ranging parameters: Extended/Spy and number of symbols
    /// Extended mode initiate a second exchange with an inverted direction to improve accuracy and provide some relative speed indication
    /// Spy mode allows to estimate distance between two device while they are performing a ranging exchange.
    /// Number of symbols should typically be between 8 to 16 symbols, with 12 being close to optimal performances
    pub async fn set_ranging_params(&mut self, extended: bool, spy_mode: bool, nb_symbols: u8) -> Result<(), Lr2021Error> {
         let req = set_ranging_params_cmd(extended, spy_mode, nb_symbols);
        self.cmd_wr(&req).await?;
        // Fix a bad default setting
        if extended {
            self.wr_field(ADDR_LORA_RANGING_EXTRA, 0, 24, 3).await?;
        }
        Ok(())
   }

    /// Return the result of last ranging exchange (round-trip time of flight and RSSI)
    /// The distance is provided
    pub async fn get_ranging_result(&mut self) -> Result<RangingResultRsp, Lr2021Error> {
        let req = get_ranging_result_req(RangingResKind::LatestRaw);
        let mut rsp = RangingResultRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return the result of last extended ranging exchange (round-trip time of flight and RSSI for both exchange)
    /// The round-trip time of flight can be converted in meter with the formula: rng*150/(2^12*Bandwidth)
    pub async fn get_ranging_ext_result(&mut self) -> Result<RangingExtResultRsp, Lr2021Error> {
        let req = get_ranging_result_req(RangingResKind::ExtendedRaw);
        let mut rsp = RangingExtResultRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return the gain step used during ranging (the second gain step is only valid for extended ranging)
    /// This is mainly for debug, since gain can influence very slightly the results
    pub async fn get_ranging_gain(&mut self) -> Result<RangingGainStepRsp, Lr2021Error> {
        let req = get_ranging_result_req(RangingResKind::GainSteps);
        let mut rsp = RangingGainStepRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return statistics about ranging exchanges
    pub async fn get_ranging_stats(&mut self) -> Result<RangingStatsRsp, Lr2021Error> {
        let req = get_ranging_stats_req();
        let mut rsp = RangingStatsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Return a correction offset on ranging RSSI
    /// Read the value after any change to the gain table
    pub async fn get_ranging_rssi_offset(&mut self) -> Result<i16, Lr2021Error> {
        let gmax = (self.rd_reg(0xF301A4).await? & 0x3FF) as i16; // u10.2b
        let pwr_offset = (self.rd_reg(0xF30128).await? >> 6) & 0x3F;
        let pwr_offset = pwr_offset as i16 - if (pwr_offset&0x20) !=0 {64} else {0}; // s6.1b
        let offset = 104 + ((gmax + 2*pwr_offset + 2) >> 2);
        Ok(-offset)
    }

    /// Set Lora in Timing Synchronisation mode
    /// The initiator sends a special frame when the dio is asserted
    /// The responder is in reception mode and will assert the DIO a known delay after reception of the TimingSync packet
    pub async fn set_lora_timing_sync(&mut self, mode: TimingSyncMode, dio_num: DioNum) -> Result<(), Lr2021Error> {
        let req = set_lora_tx_sync_cmd(mode, dio_num);
        self.cmd_wr(&req).await
    }

    /// Configure the LoRa TimingSync Pulse for the initiator (delay and width)
    pub async fn set_lora_timing_sync_pulse(&mut self, delay: u32, width: TimingSyncPulseWidth) -> Result<(), Lr2021Error> {
        let value = ((width as u32) << 29) | (delay & 0x7FF_FFFF) | 0x0800_0000;
        self.wr_reg(ADDR_LORA_TIMING_SYNC, value).await
    }

}
