// Flrc commands API

use crate::status::Status;
use super::PulseShape;

/// Bitrate and bandwidth combination
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlrcBitrate {
    Br2600 = 0,
    Br2080 = 1,
    Br1300 = 2,
    Br1040 = 3,
    Br0650 = 4,
    Br0520 = 5,
    Br0325 = 6,
    Br0260 = 7,
}

/// Coding rate selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlrcCr {
    Cr12 = 0,
    Cr34 = 1,
    None = 2,
    Cr23 = 3,
}

/// AGC preamble length
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AgcPblLen {
    Len4Bits = 0,
    Len8Bits = 1,
    Len12Bits = 2,
    Len16Bits = 3,
    Len20Bits = 4,
    Len24Bits = 5,
    Len28Bits = 6,
    Len32Bits = 7,
}

/// Length of syncword (unit is 2 bytes: 0/16/32 bits). Must be 0 if sync_match is OFF
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SwLen {
    SwNone = 0,
    Sw16b = 1,
    Sw32b = 2,
}

/// Defines which syncword to use for TX operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SwTx {
    SwNone = 0,
    Sw1 = 1,
    Sw2 = 2,
    Sw3 = 3,
}

/// Match syncword(s) configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SwMatch {
    MatchNone = 0,
    Match1 = 1,
    Match2 = 2,
    Match12 = 3,
    Match3 = 4,
    Match13 = 5,
    Match23 = 6,
    Match123 = 7,
}

/// Packet format selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PktFormat {
    Dynamic = 0,
    Fixed = 1,
}

/// CRC configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Crc {
    CrcOff = 0,
    Crc16 = 1,
    Crc24 = 2,
    Crc32 = 3,
}

/// Sets the modulation parameters for FLRC packets. FW configures respective modem registers. Will return CMD_FAIL in the status of the next command, if the packet type is not FLRC
pub fn set_flrc_modulation_params_cmd(flrc_bitrate: FlrcBitrate, flrc_cr: FlrcCr, pulse_shape: PulseShape) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x48;

    cmd[2] |= (flrc_bitrate as u8) & 0x7;
    cmd[3] |= ((flrc_cr as u8) & 0xF) << 4;
    cmd[3] |= (pulse_shape as u8) & 0xF;
    cmd
}

/// Sets the packet parameters for FLRC packets. FW configures respective modem registers
pub fn set_flrc_packet_params_cmd(agc_pbl_len: AgcPblLen, sw_len: SwLen, sw_tx: SwTx, sw_match: SwMatch, pkt_format: PktFormat, crc: Crc, pld_len: u16) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x49;

    cmd[2] |= ((agc_pbl_len as u8) & 0xF) << 2;
    cmd[2] |= (sw_len as u8) & 0x3;
    cmd[3] |= ((sw_tx as u8) & 0x3) << 6;
    cmd[3] |= ((sw_match as u8) & 0x7) << 3;
    cmd[3] |= ((pkt_format as u8) & 0x1) << 2;
    cmd[3] |= (crc as u8) & 0x3;
    cmd[4] |= ((pld_len >> 8) & 0xFF) as u8;
    cmd[5] |= (pld_len & 0xFF) as u8;
    cmd
}

/// Gets the internal statistics of the received packets. Statistics are reset on a POR, sleep without memory retention and the command ResetRxStats
pub fn get_flrc_rx_stats_req() -> [u8; 2] {
    [0x02, 0x4A]
}

/// Gets the status of the last received packet. Status is updated at the end of a reception (RxDone irq), but rssi_sync is already updated on SyncWordValid irq
pub fn get_flrc_packet_status_req() -> [u8; 2] {
    [0x02, 0x4B]
}

/// Sets the syncword for FLRC packets. 16 or 32 bits can be sent for the syncword. Bytes 5 and 6 are optional if only 16bit syncword is required
pub fn set_flrc_syncword_cmd(sw_num: u8, syncword: u32) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x02;
    cmd[1] = 0x4C;

    cmd[2] |= sw_num;
    cmd[3] |= ((syncword >> 24) & 0xFF) as u8;
    cmd[4] |= ((syncword >> 16) & 0xFF) as u8;
    cmd[5] |= ((syncword >> 8) & 0xFF) as u8;
    cmd[6] |= (syncword & 0xFF) as u8;
    cmd
}

// Response structs

/// Response for GetFlrcRxStats command
#[derive(Default)]
pub struct FlrcRxStatsRsp([u8; 8]);

impl FlrcRxStatsRsp {
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

impl AsMut<[u8]> for FlrcRxStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetFlrcPacketStatus command
#[derive(Default)]
pub struct FlrcPacketStatusRsp([u8; 7]);

impl FlrcPacketStatusRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Length of the last received packet in bytes
    pub fn pkt_len(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }

    /// Average over last packet received of RSSI. Actual signal power is –rssi_avg/2 (dBm)
    pub fn rssi_avg(&self) -> u16 {
        (((self.0[6] >> 2) & 0x1) as u16) |
        ((self.0[4] as u16) << 1)
    }

    /// Latch RSSI value after syncword detection. Actual signal power is –rssi_sync/2 (dBm)
    pub fn rssi_sync(&self) -> u16 {
        ((self.0[6] & 0x1) as u16) |
        ((self.0[5] as u16) << 1)
    }

    /// Detected syncword number of latest received packet
    pub fn sw_num(&self) -> u8 {
        (self.0[6] >> 4) & 0xF
    }
}

impl AsMut<[u8]> for FlrcPacketStatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
