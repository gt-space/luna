// Wmbus commands API

use crate::status::Status;
use super::RxBw;

/// WM-Bus mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WmbusMode {
    ModeS = 0,
    ModeT1 = 1,
    ModeT2O2m = 2,
    ModeT2M2o = 3,
    ModeR2 = 4,
    ModeC1 = 5,
    ModeC2O2m = 6,
    ModeC2M2o = 7,
    ModeN4p8 = 8,
    ModeN2p4 = 9,
    ModeN6p4 = 10,
    ModeN19p2 = 11,
    ModeF2 = 12,
}

/// WM-Bus mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WmbusSubBand {A,B,C,D}

impl WmbusMode {
    /// Return center frequency associated with a mode
    /// The channel index is only used in Mode R2 and N
    /// The sub-band is only used in Mode N
    pub fn rf(&self, channel: u8, subband: WmbusSubBand) -> u32 {
        use {WmbusMode::*, WmbusSubBand::*};
        match self {
            ModeS     => 868_300_000,
            ModeT1    |
            ModeT2M2o => 868_950_000,
            ModeT2O2m => 868_300_000,
            ModeR2    => 868_030_000 + 60_000 * channel as u32,
            ModeC1    |
            ModeC2M2o => 868_950_000,
            ModeC2O2m => 868_525_000,
            ModeF2    => 433_820_000,
            ModeN4p8 |
            ModeN2p4 |
            ModeN6p4 => {
                match subband {
                    A => 169_406_250 + 12_500 * channel as u32,
                    B => 169_481_250 ,
                    C => 169_493_750 + 12_500 * channel as u32,
                    D => 169_593_250 + 12_500 * channel as u32,
                }
            }
            ModeN19p2 => {
                match subband {
                    A => 169_437_500,
                    B => 169_481_250,
                    C => 169_493_750,
                    D => 169_625_000 + 50_000 * channel as u32,
                }
            }
        }
    }
}

/// Packet format (A or B)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WmbusFormat {
    FormatA = 0,
    FormatB = 1,
}

/// Configure the wm-bus mode according to EN13757-4 2019
pub fn set_wmbus_params_cmd(wmbus_mode: WmbusMode, rx_bw: RxBw, wmbus_format: WmbusFormat, addr_filt_en: bool, pld_len: u8, pbl_len_tx: u16, pbl_len_detect: u8) -> [u8; 10] {
    let mut cmd = [0u8; 10];
    cmd[0] = 0x02;
    cmd[1] = 0x6A;

    cmd[2] |= (wmbus_mode as u8) & 0xF;
    cmd[3] |= rx_bw as u8;
    cmd[4] |= (wmbus_format as u8) & 0x1;
    if addr_filt_en { cmd[5] |= 1; }
    cmd[6] |= pld_len;
    cmd[7] |= ((pbl_len_tx >> 8) & 0xFF) as u8;
    cmd[8] |= (pbl_len_tx & 0xFF) as u8;
    cmd[9] |= pbl_len_detect;
    cmd
}

/// Get the Rx statistics for wmbus packets
pub fn get_wmbus_rx_stats_req() -> [u8; 2] {
    [0x02, 0x6C]
}

/// Get Rx packet status informations for wmbus packets
pub fn get_wmbus_packet_status_req() -> [u8; 2] {
    [0x02, 0x6D]
}

/// Set the WM-Bus address for RX filtering (A-field)
pub fn set_wmbus_address_cmd(address: u64) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x02;
    cmd[1] = 0x6E;

    cmd[2] |= ((address >> 40) & 0xFF) as u8;
    cmd[3] |= ((address >> 32) & 0xFF) as u8;
    cmd[4] |= ((address >> 24) & 0xFF) as u8;
    cmd[5] |= ((address >> 16) & 0xFF) as u8;
    cmd[6] |= ((address >> 8) & 0xFF) as u8;
    cmd[7] |= (address & 0xFF) as u8;
    cmd
}

// Response structs

/// Response for GetWmbusRxStats command
#[derive(Default)]
pub struct WmbusRxStatsRsp([u8; 8]);

impl WmbusRxStatsRsp {
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

    /// Number of packets with a length error
    pub fn len_error(&self) -> u16 {
        (self.0[7] as u16) |
        ((self.0[6] as u16) << 8)
    }
}

impl AsMut<[u8]> for WmbusRxStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetWmbusPacketStatus command
#[derive(Default)]
pub struct WmbusPacketStatusRsp([u8; 11]);

impl WmbusPacketStatusRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// The demodulated length field. This can be different from get_rx_packet_length (in mode A)
    pub fn l_field(&self) -> u8 {
        self.0[2]
    }

    /// Length of the last received packet in bytes (including optional data added in the FIFO, crc, ...)
    pub fn pkt_len(&self) -> u16 {
        (self.0[4] as u16) |
        ((self.0[3] as u16) << 8)
    }

    /// Average over last packet received of RSSI. Actual signal power is –rssi_avg/2 (dBm)
    pub fn rssi_avg(&self) -> u16 {
        (((self.0[9] >> 4) & 0x1) as u16) |
        ((self.0[5] as u16) << 1)
    }

    /// Latch RSSI value after syncword detection. Actual signal power is –rssi_sync/2 (dBm)
    pub fn rssi_sync(&self) -> u16 {
        ((self.0[9] & 0x1) as u16) |
        ((self.0[6] as u16) << 1)
    }

    /// Indicates which crc has failed. In A mode, bit0 is the crc of the header, then each bit is the crc of the subsequent chunks
    pub fn crc_err(&self) -> u32 {
        (((self.0[9] >> 6) & 0x1) as u32) |
        ((self.0[8] as u32) << 1) |
        ((self.0[7] as u32) << 9)
    }

    /// Index of detected syncword (0/1)
    pub fn syncword_idx(&self) -> bool {
        (self.0[9] >> 7) & 0x1 != 0
    }

    /// Link quality indicator (0.25dB)
    pub fn lqi(&self) -> u8 {
        self.0[10]
    }
}

impl AsMut<[u8]> for WmbusPacketStatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
