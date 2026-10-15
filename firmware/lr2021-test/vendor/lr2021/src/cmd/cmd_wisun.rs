// Wisun commands API

use crate::status::Status;
use super::RxBw;

/// WISun mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WisunMode {
    Mode1a = 0,
    Mode1b = 1,
    Mode2a = 2,
    Mode2b = 3,
    Mode3 = 4,
    Mode4a = 5,
    Mode4b = 6,
    Mode5 = 7,
}

/// FCS selection for TX
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WisunFcsLen {
    Fcs32b = 0,
    Fcs16b = 1,
}

/// FEC encoding selection for TX packet
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WisunFec {
    None = 0,
    Nrnsc = 1,
    Rsc = 2,
    RscIntlvr = 3,
}

/// Configure the wisun mode (1a, 1b, 2a, 2b, 3, 4a, 4b, 5)
pub fn set_wisun_mode_cmd(wisun_mode: WisunMode, rx_bw: RxBw) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x70;

    cmd[2] |= (wisun_mode as u8) & 0x7;
    cmd[3] |= rx_bw as u8;
    cmd
}

#[allow(clippy::too_many_arguments)]
/// Configure the wisun packet parameters
pub fn set_wisun_packet_params_cmd(wisun_fcs_len: WisunFcsLen, whitening_en: bool, crc_hw: bool, mode_switch_tx: bool, wisun_fec: WisunFec, frame_len_tx: u16, pbl_len_tx: u8, pbl_detect: u8) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x02;
    cmd[1] = 0x71;

    cmd[2] |= ((wisun_fcs_len as u8) & 0x1) << 5;
    if whitening_en { cmd[2] |= 16; }
    if crc_hw { cmd[2] |= 8; }
    if mode_switch_tx { cmd[2] |= 4; }
    cmd[2] |= (wisun_fec as u8) & 0x3;
    cmd[3] |= ((frame_len_tx >> 8) & 0xFF) as u8;
    cmd[4] |= (frame_len_tx & 0xFF) as u8;
    cmd[5] |= pbl_len_tx;
    cmd[6] |= pbl_detect;
    cmd
}

/// Gets the status of the last received packet. Status is updated at the end of a reception (RxDone irq), but syncword_idx and rssi_sync are already updated on SyncWordValid irq
pub fn get_wisun_packet_status_req() -> [u8; 2] {
    [0x02, 0x73]
}

/// Sets length of frame for TX for normal packets, or header value for mode_switch packets
pub fn set_wisun_packet_len_cmd(frame_len_tx: u16) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x74;

    cmd[2] |= ((frame_len_tx >> 8) & 0xFF) as u8;
    cmd[3] |= (frame_len_tx & 0xFF) as u8;
    cmd
}

/// Get the Rx statistics for WiSUN packets
pub fn get_wisun_rx_stats_req() -> [u8; 2] {
    [0x02, 0x6C]
}

// Response structs

/// Response for GetWisunPacketStatus command
#[derive(Default)]
pub struct WisunPacketStatusRsp([u8; 11]);

impl WisunPacketStatusRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Raw 16bit received header
    pub fn header(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }

    /// Length of the last received packet in bytes (including optional data added in the FIFO, crc, ...)
    pub fn pkt_len(&self) -> u16 {
        (self.0[5] as u16) |
        ((self.0[4] as u16) << 8)
    }

    /// Average over last packet received of RSSI. Actual signal power is –rssi_avg/2 (dBm)
    pub fn rssi_avg(&self) -> u16 {
        (((self.0[9] >> 2) & 0x1) as u16) |
        ((self.0[6] as u16) << 1)
    }

    /// Latch RSSI value after syncword detection. Actual signal power is –rssi_sync/2 (dBm)
    pub fn rssi_sync(&self) -> u16 {
        ((self.0[9] & 0x1) as u16) |
        ((self.0[7] as u16) << 1)
    }

    /// Index of detected syncword (0/1)
    pub fn syncword_idx(&self) -> bool {
        (self.0[8] >> 7) & 0x1 != 0
    }

    /// Link quality indicator (0.25dB)
    pub fn lqi(&self) -> u8 {
        self.0[10]
    }
}

impl AsMut<[u8]> for WisunPacketStatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetWisunRxStats command
#[derive(Default)]
pub struct WisunRxStatsRsp([u8; 8]);

impl WisunRxStatsRsp {
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

impl AsMut<[u8]> for WisunRxStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
