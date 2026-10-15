// Ook commands API

use crate::status::Status;
use super::RxBw;
use super::PulseShape;

/// Magnitude depth
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OokDepth {
    Full = 0,
    Max20Db = 1,
}

/// Address comparison mode (same as for FSK)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddrComp {
    Off = 0,
    Node = 1,
    NodeBcast = 2,
}

/// Packet format
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PktFormat {
    FixedLength = 0,
    Variable8bit = 1,
}

/// CRC configuration (same as FSK)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Crc {
    CrcOff = 0,
    Crc1Byte = 1,
    Crc2Byte = 2,
    Crc3Byte = 3,
    Crc4Byte = 4,
    Crc1ByteInv = 9,
    Crc2ByteInv = 10,
    Crc3ByteInv = 11,
    Crc4ByteInv = 12,
}

/// Encoding configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Encoding {
    None = 0,
    Manchester = 1,
    ManchesterInv = 9,
    BiphaseMark = 2,
    BiphaseMarkInv = 10,
}

/// Bit order for syncword transmission
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitOrder {
    LsbFirst = 0,
    MsbFirst = 1,
}

/// Start of frame delimiter kind. Set to 0 for ADS-B, RTS and INOVA
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SfdKind {
    FallingEdge = 0,
    RisingEdge = 1,
}

/// Sets the OOK modulation parameters
pub fn set_ook_modulation_params_cmd(bitrate: u32, pulse_shape: PulseShape, rx_bw: RxBw) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x02;
    cmd[1] = 0x81;

    cmd[2] |= ((bitrate >> 24) & 0xFF) as u8;
    cmd[3] |= ((bitrate >> 16) & 0xFF) as u8;
    cmd[4] |= ((bitrate >> 8) & 0xFF) as u8;
    cmd[5] |= (bitrate & 0xFF) as u8;
    cmd[6] |= (pulse_shape as u8) & 0xF;
    cmd[7] |= rx_bw as u8;
    cmd
}

/// Sets the OOK modulation parameters
pub fn set_ook_modulation_params_adv_cmd(bitrate: u32, pulse_shape: PulseShape, rx_bw: RxBw, ook_depth: OokDepth) -> [u8; 9] {
    let mut cmd = [0u8; 9];
    cmd[0] = 0x02;
    cmd[1] = 0x81;

    cmd[2] |= ((bitrate >> 24) & 0xFF) as u8;
    cmd[3] |= ((bitrate >> 16) & 0xFF) as u8;
    cmd[4] |= ((bitrate >> 8) & 0xFF) as u8;
    cmd[5] |= (bitrate & 0xFF) as u8;
    cmd[6] |= (pulse_shape as u8) & 0xF;
    cmd[7] |= rx_bw as u8;
    cmd[8] |= (ook_depth as u8) & 0x1;
    cmd
}

/// Sets the OOK packet parameters. It is recommended to have either whitening or manchester encoding enabled for OOK
pub fn set_ook_packet_params_cmd(pre_len_tx: u16, addr_comp: AddrComp, pkt_format: PktFormat, pld_len: u16, crc: Crc, encoding: Encoding) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x02;
    cmd[1] = 0x82;

    cmd[2] |= ((pre_len_tx >> 8) & 0xFF) as u8;
    cmd[3] |= (pre_len_tx & 0xFF) as u8;
    cmd[4] |= ((addr_comp as u8) & 0x3) << 2;
    cmd[4] |= (pkt_format as u8) & 0x3;
    cmd[5] |= ((pld_len >> 8) & 0xFF) as u8;
    cmd[6] |= (pld_len & 0xFF) as u8;
    cmd[7] |= ((crc as u8) & 0xF) << 4;
    cmd[7] |= (encoding as u8) & 0xF;
    cmd
}

/// Sets the OOK CRC parameters (same as for FSK)
pub fn set_ook_crc_params_cmd(polynom: u32, init: u32) -> [u8; 10] {
    let mut cmd = [0u8; 10];
    cmd[0] = 0x02;
    cmd[1] = 0x83;

    cmd[2] |= ((polynom >> 24) & 0xFF) as u8;
    cmd[3] |= ((polynom >> 16) & 0xFF) as u8;
    cmd[4] |= ((polynom >> 8) & 0xFF) as u8;
    cmd[5] |= (polynom & 0xFF) as u8;
    cmd[6] |= ((init >> 24) & 0xFF) as u8;
    cmd[7] |= ((init >> 16) & 0xFF) as u8;
    cmd[8] |= ((init >> 8) & 0xFF) as u8;
    cmd[9] |= (init & 0xFF) as u8;
    cmd
}

/// Sets the OOK syncword. Limited to 32bits max
pub fn set_ook_sync_word_cmd(syncword: u32, bit_order: BitOrder, nb_bits: u8) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x02;
    cmd[1] = 0x84;

    cmd[2] |= ((syncword >> 24) & 0xFF) as u8;
    cmd[3] |= ((syncword >> 16) & 0xFF) as u8;
    cmd[4] |= ((syncword >> 8) & 0xFF) as u8;
    cmd[5] |= (syncword & 0xFF) as u8;
    cmd[6] |= ((bit_order as u8) & 0x1) << 7;
    cmd[6] |= nb_bits & 0x7F;
    cmd
}

/// Sets the OOK address (same as for FSK)
pub fn set_ook_address_cmd(addr_node: u8, addr_bcast: u8) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x85;

    cmd[2] |= addr_node;
    cmd[3] |= addr_bcast;
    cmd
}

/// Gets the internal statistics of the received packets. Statistics are reset on a POR, sleep without memory retention and the command ResetRxStats
pub fn get_ook_rx_stats_req() -> [u8; 2] {
    [0x02, 0x86]
}

/// Gets the status of the last received OOK packet. Status is updated at the end of a reception (RxDone irq), but rssi_sync is already updated on SyncWordValid irq
pub fn get_ook_packet_status_req() -> [u8; 2] {
    [0x02, 0x87]
}

/// Configures the OOK detection. This API is for RX only. Detection pattern for TX has directly to be put into the TX FIFO
pub fn set_ook_detector_cmd(preamble_pattern: u16, pattern_length: u8, pattern_num_repeats: u8, sw_is_raw: bool, sfd_kind: SfdKind, sfd_length: u8) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x02;
    cmd[1] = 0x88;

    cmd[2] |= ((preamble_pattern >> 8) & 0xFF) as u8;
    cmd[3] |= (preamble_pattern & 0xFF) as u8;
    cmd[4] |= pattern_length & 0xF;
    cmd[5] |= pattern_num_repeats & 0x1F;
    if sw_is_raw { cmd[6] |= 32; }
    cmd[6] |= ((sfd_kind as u8) & 0x1) << 4;
    cmd[6] |= sfd_length & 0xF;
    cmd
}

/// Configure the whitening for OOK packets. Setting a polynomial of 0 will disable the whitening. It is recommended to have either whitening or manchester encoding enabled for OOK
pub fn set_ook_whitening_params_cmd(bit_idx: u8, polynom: u16, init: u16) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x89;

    cmd[2] |= (bit_idx & 0xF) << 4;
    cmd[2] |= ((polynom >> 8) & 0xFF) as u8;
    cmd[3] |= (polynom & 0xFF) as u8;
    cmd[4] |= ((init >> 8) & 0xFF) as u8;
    cmd[5] |= (init & 0xFF) as u8;
    cmd
}

// Response structs

/// Response for GetOokRxStats command
#[derive(Default)]
pub struct OokRxStatsRsp([u8; 8]);

impl OokRxStatsRsp {
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

impl AsMut<[u8]> for OokRxStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetOokRxStats command
#[derive(Default)]
pub struct OokRxStatsRspAdv([u8; 16]);

impl OokRxStatsRspAdv {
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

    /// Number of detections
    pub fn pbl_det(&self) -> u16 {
        (self.0[9] as u16) |
        ((self.0[8] as u16) << 8)
    }

    /// Number of good found syncword
    pub fn sync_ok(&self) -> u16 {
        (self.0[11] as u16) |
        ((self.0[10] as u16) << 8)
    }

    /// Number of failed syncword
    pub fn sync_fail(&self) -> u16 {
        (self.0[13] as u16) |
        ((self.0[12] as u16) << 8)
    }

    /// Number of timeouts
    pub fn timeout(&self) -> u16 {
        (self.0[15] as u16) |
        ((self.0[14] as u16) << 8)
    }
}

impl AsMut<[u8]> for OokRxStatsRspAdv {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetOokPacketStatus command
#[derive(Default)]
pub struct OokPacketStatusRsp([u8; 8]);

impl OokPacketStatusRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Length of the last received packet in bytes (including optional data added in the FIFO, crc, ...)
    pub fn pkt_len(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }

    /// Average RSSI over the last packet received. Actual signal power is –rssi_avg/2 (dBm)
    pub fn rssi_avg(&self) -> u16 {
        (((self.0[6] >> 2) & 0x1) as u16) |
        ((self.0[4] as u16) << 1)
    }

    /// RSSI value of high bits. Actual signal power is –rssi_high/2 (dBm)
    pub fn rssi_high(&self) -> u16 {
        ((self.0[6] & 0x1) as u16) |
        ((self.0[5] as u16) << 1)
    }

    /// Broadcast address match indication
    pub fn addr_match_bcast(&self) -> bool {
        (self.0[6] >> 5) & 0x1 != 0
    }

    /// Node address match indication
    pub fn addr_match_node(&self) -> bool {
        (self.0[6] >> 4) & 0x1 != 0
    }

    /// Link quality indicator (0.25dB)
    pub fn lqi(&self) -> u8 {
        self.0[7]
    }
}

impl AsMut<[u8]> for OokPacketStatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
