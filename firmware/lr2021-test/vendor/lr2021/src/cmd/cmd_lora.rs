// Lora commands API

use crate::status::Status;
use super::cmd_system::DioNum;

/// Spreading factor
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sf {
    Sf5 = 5,
    Sf6 = 6,
    Sf7 = 7,
    Sf8 = 8,
    Sf9 = 9,
    Sf10 = 10,
    Sf11 = 11,
    Sf12 = 12,
}

/// Bandwidth selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LoraBw {
    Bw7 = 0,
    Bw15 = 1,
    Bw31 = 2,
    Bw62 = 3,
    Bw125 = 4,
    Bw250 = 5,
    Bw500 = 6,
    Bw1000 = 7,
    Bw10 = 8,
    Bw20 = 9,
    Bw41 = 10,
    Bw83 = 11,
    Bw101 = 12,
    Bw203 = 13,
    Bw406 = 14,
    Bw812 = 15,
}

impl LoraBw {
    /// Return Bandwidth in Hz
    pub fn to_hz(&self) -> u32 {
        match self {
            LoraBw::Bw1000 => 1_000_000,
            LoraBw::Bw812  =>   812_500,
            LoraBw::Bw500  =>   500_000,
            LoraBw::Bw406  =>   406_250,
            LoraBw::Bw250  =>   250_000,
            LoraBw::Bw203  =>   203_125,
            LoraBw::Bw125  =>   125_000,
            LoraBw::Bw101  =>   101_562,
            LoraBw::Bw83   =>    83_333,
            LoraBw::Bw62   =>    62_500,
            LoraBw::Bw41   =>    41_666,
            LoraBw::Bw31   =>    31_250,
            LoraBw::Bw20   =>    20_833,
            LoraBw::Bw15   =>    15_625,
            LoraBw::Bw10   =>    10_416,
            LoraBw::Bw7    =>     7_812,
        }
    }

    /// Flag Fractional bandwidth
    /// Corresponds to band used in SX1280
    pub fn is_fractional(&self) -> bool {
        use LoraBw::*;
        matches!(self, Bw812 | Bw406 | Bw203 | Bw101)
    }
}

impl PartialOrd for LoraBw {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for LoraBw {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.to_hz().cmp(&other.to_hz())
    }
}

/// Coding rate
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LoraCr {
    NoCoding = 0,
    Cr1Ham45Si = 1,
    Cr2Ham23Si = 2,
    Cr3Ham47Si = 3,
    Cr4Ham12Si = 4,
    Cr5Ham45Li = 5,
    Cr6Ham23Li = 6,
    Cr7Ham12Li = 7,
    Cr8Cc23 = 8,
    Cr9Cc12 = 9,
}

impl LoraCr {
    /// Return if Code-rate uses long interleaving
    pub fn is_li(&self) -> bool {
        use LoraCr::*;
        matches!(self, Cr5Ham45Li|Cr6Ham23Li|Cr7Ham12Li|Cr8Cc23|Cr9Cc12)
    }
    /// Return denominator for the coding rate, supposing a numerator equal to 4
    pub fn denominator(&self) -> u8 {
        match self {
            LoraCr::NoCoding   => 4,
            // Code rate 4/5
            LoraCr::Cr1Ham45Si |
            LoraCr::Cr5Ham45Li => 5,
            // Code rate 2/3 -> 4/6
            LoraCr::Cr2Ham23Si |
            LoraCr::Cr6Ham23Li |
            LoraCr::Cr8Cc23    => 6,
            // Code rate 4/7
            LoraCr::Cr3Ham47Si => 7,
            // Code rate 1/2 -> 4/8
            LoraCr::Cr4Ham12Si |
            LoraCr::Cr7Ham12Li |
            LoraCr::Cr9Cc12    => 8,
        }
    }
}

/// Low Data Rate Optimisation. Enable for high Spreading factor to increase tolerance to clock drift.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Ldro {
    Off = 0,
    On = 1,
}

/// Configure extra filtering (for fractional bandwidth)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LoraFilter {
    Auto = 0,
    Chf = 1,
    Dcc = 2,
    DccF = 3,
}

/// Header type selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HeaderType {
    Explicit = 0,
    Implicit = 1,
}

/// Format selection for symbols parameter: either an integer number of symbol or a floating point representation (exponent on 3 MSB bits with mantissa on 5 LSB bits) Exponent has a resolution of 2 with an offset meaning the mantisse is multiplied by 2^(n+1)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimeoutFormat {
    Integer = 0,
    Float = 1,
}

/// Action taken after CAD
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExitMode {
    CadOnly = 0,
    CadRx = 1,
    CadLbt = 16,
}

/// Number of bytes (0..8) used in address filtering. 0=no address filtering
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddrLen {
    AddrNone = 0,
    Addr1B = 1,
    Addr2B = 2,
    Addr3B = 3,
    Addr4B = 4,
    Addr5B = 5,
    Addr6B = 6,
    Addr7B = 7,
    Addr8B = 8,
}

/// TX Sync function
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimingSyncMode {
    Disabled = 0,
    Initiator = 1,
    Responder = 2,
}

/// Sets the LoRa modulation parameters. FW configures respective modem registers. Will return CMD_FAIL in the status of the next command, if the packet type is not LoRa
pub fn set_lora_modulation_params_cmd(sf: Sf, lora_bw: LoraBw, lora_cr: LoraCr, ldro: Ldro, lora_filter: LoraFilter) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x20;

    cmd[2] |= ((sf as u8) & 0xF) << 4;
    cmd[2] |= (lora_bw as u8) & 0xF;
    cmd[3] |= ((lora_cr as u8) & 0xF) << 4;
    cmd[3] |= (ldro as u8) & 0x3;
    cmd[3] |= ((lora_filter as u8) & 0x3) << 2;
    cmd
}

/// Sets the packet parameters for the LoRa packets. FW configures according modem registers
pub fn set_lora_packet_params_cmd(pbl_len: u16, payload_len: u8, header_type: HeaderType, crc_en: bool, invert_iq: bool) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x21;

    cmd[2] |= ((pbl_len >> 8) & 0xFF) as u8;
    cmd[3] |= (pbl_len & 0xFF) as u8;
    cmd[4] |= payload_len;
    cmd[5] |= ((header_type as u8) & 0x1) << 2;
    if crc_en { cmd[5] |= 2; }
    if invert_iq { cmd[5] |= 1; }
    cmd
}

/// Configure LoRa modem to search for a detect for N symbols. N can be given as number, or as mantissa/exponent. SymbolNum 0x00 means no timeout
pub fn set_lora_synch_timeout_cmd(symbols: u8, timeout_format: TimeoutFormat) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x22;

    cmd[2] |= symbols;
    cmd[3] |= (timeout_format as u8) & 0x1;
    cmd
}

/// Sets the LoRa syncword. Default value is 0x12. Examples: Public Network: 0x34, Private Network: 0x12
pub fn set_lora_syncword_cmd(syncword: u8) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x23;

    cmd[2] |= syncword;
    cmd
}

/// Configures the LoRa phase modulation demodulation during preamble feature. If enabled, for TX the preamble will have a phase modulation overlayed to the LoRa modulation
pub fn config_lora_preamble_modulation_cmd(pmod_en: bool, dram_ret: u8, wakeup_time: u16, min_sleep_time: u32) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x02;
    cmd[1] = 0x26;

    if pmod_en { cmd[2] |= 128; }
    cmd[2] |= dram_ret & 0x7;
    cmd[3] |= ((wakeup_time >> 8) & 0xFF) as u8;
    cmd[4] |= (wakeup_time & 0xFF) as u8;
    cmd[5] |= ((min_sleep_time >> 16) & 0xFF) as u8;
    cmd[6] |= ((min_sleep_time >> 8) & 0xFF) as u8;
    cmd[7] |= (min_sleep_time & 0xFF) as u8;
    cmd
}

#[allow(clippy::too_many_arguments)]
/// Configures the LoRa phase modulation demodulation during preamble feature. If enabled, for TX the preamble will have a phase modulation overlayed to the LoRa modulation
pub fn config_lora_preamble_modulation_adv_cmd(pmod_en: bool, dram_ret: u8, wakeup_time: u16, min_sleep_time: u32, err_thr: u8, min_sym: u8, detect_time_sym: u8, start_offset: u8, end_offset: u8) -> [u8; 12] {
    let mut cmd = [0u8; 12];
    cmd[0] = 0x02;
    cmd[1] = 0x26;

    if pmod_en { cmd[2] |= 128; }
    cmd[2] |= dram_ret & 0x7;
    cmd[3] |= ((wakeup_time >> 8) & 0xFF) as u8;
    cmd[4] |= (wakeup_time & 0xFF) as u8;
    cmd[5] |= ((min_sleep_time >> 16) & 0xFF) as u8;
    cmd[6] |= ((min_sleep_time >> 8) & 0xFF) as u8;
    cmd[7] |= (min_sleep_time & 0xFF) as u8;
    cmd[8] |= err_thr & 0x7F;
    cmd[9] |= (min_sym & 0xF) << 4;
    cmd[9] |= detect_time_sym & 0xF;
    cmd[10] |= start_offset;
    cmd[11] |= end_offset;
    cmd
}

/// Configure LoRa CAD mode parameters. Sets up Channel Activity Detection which searches for presence of LoRa preamble symbols
pub fn set_lora_cad_params_cmd(nb_symbols: u8, pbl_any: bool, pnr_delta: u8, exit_mode: ExitMode, timeout: u32, det_peak: u8) -> [u8; 9] {
    let mut cmd = [0u8; 9];
    cmd[0] = 0x02;
    cmd[1] = 0x27;

    cmd[2] |= nb_symbols;
    if pbl_any { cmd[3] |= 16; }
    cmd[3] |= pnr_delta & 0xF;
    cmd[4] |= exit_mode as u8;
    cmd[5] |= ((timeout >> 16) & 0xFF) as u8;
    cmd[6] |= ((timeout >> 8) & 0xFF) as u8;
    cmd[7] |= (timeout & 0xFF) as u8;
    cmd[8] |= det_peak;
    cmd
}

/// Set device into RX CAD mode (LoRa). The Channel Activity Detection searches for the presence of LoRa preamble symbols. Parameters must be previously set using SetLoraCadParams
pub fn set_lora_cad_cmd() -> [u8; 2] {
    [0x02, 0x28]
}

/// Gets the internal statistics of the received packets. Statistics are reset on a POR, sleep without memory retention and the command ResetRxStats
pub fn get_lora_rx_stats_req() -> [u8; 2] {
    [0x02, 0x29]
}

/// Gets the status of the last received packet. Status is updated at the end of a reception (RxDone or CadDone irqs)
pub fn get_lora_packet_status_req() -> [u8; 2] {
    [0x02, 0x2A]
}

/// Sets the address for LoRa RX address filtering
pub fn set_lora_address_cmd(addr_len: AddrLen, addr_pos: u8, addr: u64) -> [u8; 11] {
    let mut cmd = [0u8; 11];
    cmd[0] = 0x02;
    cmd[1] = 0x2B;

    cmd[2] |= ((addr_len as u8) & 0xF) << 4;
    cmd[2] |= addr_pos & 0xF;
    cmd[3] |= (addr & 0xFF) as u8;
    cmd[4] |= ((addr >> 8) & 0xFF) as u8;
    cmd[5] |= ((addr >> 16) & 0xFF) as u8;
    cmd[6] |= ((addr >> 24) & 0xFF) as u8;
    cmd[7] |= ((addr >> 32) & 0xFF) as u8;
    cmd[8] |= ((addr >> 40) & 0xFF) as u8;
    cmd[9] |= ((addr >> 48) & 0xFF) as u8;
    cmd[10] |= ((addr >> 56) & 0xFF) as u8;
    cmd
}

/// Extended version of the SetLoraSyncword command to set all 10bits of the syncword
pub fn set_lora_syncword_extended_cmd(sync1: u8, sync2: u8) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x2D;

    cmd[2] |= sync1 & 0x1F;
    cmd[3] |= sync2 & 0x1F;
    cmd
}

/// Extended version of the SetLoraSideDetSyncword command to set all 10bits of the syncwords
pub fn set_lora_side_det_syncword_extended_cmd(sd1_sw1: u8, sd1_sw2: u8, sd2_sw1: u8, sd2_sw2: u8, sd3_sw1: u8, sd3_sw2: u8) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x02;
    cmd[1] = 0x2E;

    cmd[2] |= sd1_sw1 & 0x1F;
    cmd[3] |= sd1_sw2 & 0x1F;
    cmd[4] |= sd2_sw1 & 0x1F;
    cmd[5] |= sd2_sw2 & 0x1F;
    cmd[6] |= sd3_sw1 & 0x1F;
    cmd[7] |= sd3_sw2 & 0x1F;
    cmd
}

/// Configures the LoRa Tx synchronization using dio
pub fn set_lora_tx_sync_cmd(timing_sync_mode: TimingSyncMode, dio_num: DioNum) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x1D;

    cmd[2] |= ((timing_sync_mode as u8) & 0x3) << 6;
    cmd[2] |= (dio_num as u8) & 0xF;
    cmd
}

/// Configures the LoRa Tx synchronization using dio
pub fn set_lora_blanking_cmd(thr_gain: u8, thr: u8, lvl_gain: u8, detect: bool, rssi_thr: u8) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x1D;

    cmd[2] |= (thr_gain & 0x3) << 6;
    cmd[2] |= (thr & 0xF) << 2;
    cmd[2] |= lvl_gain & 0x3;
    if detect { cmd[3] |= 16; }
    cmd[3] |= rssi_thr & 0xF;
    cmd
}

// Response structs

/// Response for GetLoraRxStats command
#[derive(Default)]
pub struct LoraRxStatsRsp([u8; 12]);

impl LoraRxStatsRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Total number of received packets
    pub fn pkt_rx(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }

    /// Number of received packets with a CRC error
    pub fn crc_error(&self) -> u16 {
        (self.0[5] as u16) |
        ((self.0[4] as u16) << 8)
    }

    /// Number of received packets with a header error
    pub fn header_error(&self) -> u16 {
        (self.0[7] as u16) |
        ((self.0[6] as u16) << 8)
    }

    /// Number of preamble detection
    pub fn detection(&self) -> u16 {
        (self.0[9] as u16) |
        ((self.0[8] as u16) << 8)
    }

    /// Number of false synchronizations
    pub fn false_sync(&self) -> u16 {
        (self.0[11] as u16) |
        ((self.0[10] as u16) << 8)
    }
}

impl AsMut<[u8]> for LoraRxStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetLoraPacketStatus command
#[derive(Default)]
pub struct LoraPacketStatusRsp([u8; 8]);

impl LoraPacketStatusRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// CRC status from header (explicit mode) or configured setting (implicit mode). 1=CRC_ON, 0=CRC_OFF
    pub fn crc(&self) -> bool {
        (self.0[2] >> 4) & 0x1 != 0
    }

    /// Coding rate from header (explicit mode) or configured setting (implicit mode)
    pub fn coding_rate(&self) -> u8 {
        self.0[2] & 0xF
    }

    /// Length of the last packet received
    pub fn pkt_length(&self) -> u8 {
        self.0[3]
    }

    /// Estimation of SNR on last packet received. In two's complement format multiplied by 4. Actual SNR in dB is snr_pkt/4
    pub fn snr_pkt(&self) -> i8 {
        self.0[4] as i8
    }

    /// Average over last packet received of RSSI. Actual signal power is –rssi_pkt/2 (dBm)
    pub fn rssi_pkt(&self) -> u16 {
        (((self.0[7] >> 1) & 0x1) as u16) |
        ((self.0[5] as u16) << 1)
    }

    /// Estimation of RSSI of the LoRa signal (after despreading) on last packet received. Actual value is -rssi_signal_pkt/2 (dBm)
    pub fn rssi_signal_pkt(&self) -> u16 {
        ((self.0[7] & 0x1) as u16) |
        ((self.0[6] as u16) << 1)
    }

    /// Flags which detector(s) received the packet. 0001=main, 0010=side1, 0100=side2, 1000=side3. In normal RX, only one flag is set. In CAD, all detector paths triggered are set
    pub fn detector(&self) -> u8 {
        (self.0[7] >> 2) & 0xF
    }
}

impl AsMut<[u8]> for LoraPacketStatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
