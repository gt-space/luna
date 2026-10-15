// Fsk commands API

use crate::status::Status;

/// Pulse shaping filter selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PulseShape {
    None = 0,
    Custom = 1,
    Bt0p3 = 4,
    Bt0p5 = 5,
    Bt0p7 = 6,
    Bt1p0 = 7,
    Bt2p0 = 2,
    Rc0p3 = 8,
    Rc0p5 = 9,
    Rc0p7 = 10,
    Rc1p0 = 11,
    Rrc0p3 = 12,
    Rrc0p4 = 3,
    Rrc0p5 = 13,
    Rrc0p7 = 14,
    Rrc1p0 = 15,
}

/// RX bandwidth (same format as in the SetAdvancedModulationParams command)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxBw {
    BwAuto = 255,
    Bw3076 = 0,
    Bw2857 = 64,
    Bw2666 = 128,
    Bw2222 = 192,
    Bw1333 = 136,
    Bw1111 = 200,
    Bw888 = 144,
    Bw769 = 24,
    Bw740 = 208,
    Bw714 = 88,
    Bw666 = 152,
    Bw615 = 32,
    Bw571 = 96,
    Bw555 = 216,
    Bw533 = 160,
    Bw512 = 17,
    Bw476 = 81,
    Bw444 = 224,
    Bw384 = 25,
    Bw370 = 209,
    Bw357 = 89,
    Bw333 = 153,
    Bw307 = 33,
    Bw285 = 97,
    Bw277 = 217,
    Bw266 = 161,
    Bw256 = 18,
    Bw238 = 82,
    Bw222 = 225,
    Bw192 = 26,
    Bw185 = 210,
    Bw178 = 90,
    Bw166 = 154,
    Bw153 = 34,
    Bw142 = 98,
    Bw138 = 218,
    Bw133 = 162,
    Bw128 = 19,
    Bw119 = 83,
    Bw111 = 226,
    Bw96 = 27,
    Bw92 = 211,
    Bw89 = 91,
    Bw83 = 155,
    Bw76 = 35,
    Bw71 = 99,
    Bw69 = 219,
    Bw66 = 163,
    Bw64 = 20,
    Bw59 = 84,
    Bw55 = 227,
    Bw48 = 28,
    Bw46 = 212,
    Bw44 = 92,
    Bw41 = 156,
    Bw38 = 36,
    Bw35 = 100,
    Bw34 = 220,
    Bw33 = 164,
    Bw32 = 21,
    Bw29 = 85,
    Bw27 = 228,
    Bw24 = 29,
    Bw23 = 213,
    Bw22 = 93,
    Bw20 = 157,
    Bw19 = 37,
    Bw17 = 101,
    Bw16 = 165,
    Bw14 = 86,
    Bw13 = 229,
    Bw12 = 30,
    Bw11 = 94,
    Bw10 = 158,
    Bw9p6 = 38,
    Bw8p9 = 102,
    Bw8p7 = 222,
    Bw8p3 = 166,
    Bw8 = 23,
    Bw7p4 = 87,
    Bw6p9 = 230,
    Bw6 = 31,
    Bw5p8 = 215,
    Bw5p6 = 95,
    Bw5p2 = 159,
    Bw4p8 = 39,
    Bw4p5 = 103,
    Bw4p3 = 223,
    Bw4p2 = 167,
    Bw3p5 = 231,
}

/// Preamble detection length. 0=off (detection on syncword), others=length of preamble detection. Enables/disables PreambleDetected IRQ
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PblLenDetect {
    None = 0,
    Len8Bits = 8,
    Len16Bits = 16,
    Len24Bits = 24,
    Len32Bits = 32,
}

/// Payload length unit
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PldLenUnit {
    Bytes = 0,
    Bits = 1,
}

/// Address filtering mode. If address comparison fails, packet reception is aborted and addrErr flag is set
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddrComp {
    Off = 0,
    Node = 1,
    NodeBcast = 2,
}

/// Packet format selection (fixed or variable length)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FskPktFormat {
    FixedLength = 0,
    Variable8bit = 1,
    Variable9bit = 2,
    Variable16bit = 3,
}

/// CRC mode selection
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

/// Whitening type compatibility
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WhitenType {
    Sx126xLr11xx = 0,
    Sx128x = 1,
}

/// Bit order for syncword transmission (over the air). Set MSB first for SX126x, LR11xx, SX1280 compatible value
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitOrder {
    LsbFirst = 0,
    MsbFirst = 1,
}

/// Sets the modulation parameters for FSK packets. FW configures respective modem registers. Will return CMD_FAIL in the status of the next command, if the packet type is not FSK
pub fn set_fsk_modulation_params_cmd(bitrate: u32, pulse_shape: PulseShape, rx_bw: RxBw, fdev: u32) -> [u8; 11] {
    let mut cmd = [0u8; 11];
    cmd[0] = 0x02;
    cmd[1] = 0x40;

    cmd[2] |= ((bitrate >> 24) & 0xFF) as u8;
    cmd[3] |= ((bitrate >> 16) & 0xFF) as u8;
    cmd[4] |= ((bitrate >> 8) & 0xFF) as u8;
    cmd[5] |= (bitrate & 0xFF) as u8;
    cmd[6] |= (pulse_shape as u8) & 0xF;
    cmd[7] |= rx_bw as u8;
    cmd[8] |= ((fdev >> 16) & 0xFF) as u8;
    cmd[9] |= ((fdev >> 8) & 0xFF) as u8;
    cmd[10] |= (fdev & 0xFF) as u8;
    cmd
}

#[allow(clippy::too_many_arguments)]
/// Sets the packet parameters for FSK packets.. Command will fail if packet type is not FSK
pub fn set_fsk_packet_params_cmd(pbl_len_tx: u16, pbl_len_detect: PblLenDetect, pbl_long: bool, pld_len_unit: PldLenUnit, addr_comp: AddrComp, fsk_pkt_format: FskPktFormat, pld_len: u16, crc: Crc, dc_free: bool) -> [u8; 9] {
    let mut cmd = [0u8; 9];
    cmd[0] = 0x02;
    cmd[1] = 0x41;

    cmd[2] |= ((pbl_len_tx >> 8) & 0xFF) as u8;
    cmd[3] |= (pbl_len_tx & 0xFF) as u8;
    cmd[4] |= pbl_len_detect as u8;
    if pbl_long { cmd[5] |= 16; }
    cmd[5] |= ((pld_len_unit as u8) & 0x1) << 4;
    cmd[5] |= ((addr_comp as u8) & 0x3) << 2;
    cmd[5] |= (fsk_pkt_format as u8) & 0x3;
    cmd[6] |= ((pld_len >> 8) & 0xFF) as u8;
    cmd[7] |= (pld_len & 0xFF) as u8;
    cmd[8] |= ((crc as u8) & 0xF) << 4;
    if dc_free { cmd[8] |= 1; }
    cmd
}

/// Configure the whitening params for FSK packets, SX126x/LR11xx or SX128x compatible
pub fn set_fsk_whitening_params_cmd(whiten_type: WhitenType, init: u16) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x42;

    cmd[2] |= ((whiten_type as u8) & 0x1) << 4;
    cmd[2] |= ((init >> 8) & 0xFF) as u8;
    cmd[3] |= (init & 0xFF) as u8;
    cmd
}

/// Configure the CRC params for FSK packets
pub fn set_fsk_crc_params_cmd(polynom: u32, init: u32) -> [u8; 10] {
    let mut cmd = [0u8; 10];
    cmd[0] = 0x02;
    cmd[1] = 0x43;

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

/// Configure the syncword for FSK packets
pub fn set_fsk_sync_word_cmd(syncword: u64, bit_order: BitOrder, nb_bits: u8) -> [u8; 11] {
    let mut cmd = [0u8; 11];
    cmd[0] = 0x02;
    cmd[1] = 0x44;

    cmd[2] |= ((syncword >> 56) & 0xFF) as u8;
    cmd[3] |= ((syncword >> 48) & 0xFF) as u8;
    cmd[4] |= ((syncword >> 40) & 0xFF) as u8;
    cmd[5] |= ((syncword >> 32) & 0xFF) as u8;
    cmd[6] |= ((syncword >> 24) & 0xFF) as u8;
    cmd[7] |= ((syncword >> 16) & 0xFF) as u8;
    cmd[8] |= ((syncword >> 8) & 0xFF) as u8;
    cmd[9] |= (syncword & 0xFF) as u8;
    cmd[10] |= ((bit_order as u8) & 0x1) << 7;
    cmd[10] |= nb_bits & 0x7F;
    cmd
}

/// Configure the addresses for filtering for FSK packets
pub fn set_fsk_address_cmd(addr_node: u8, addr_bcast: u8) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x45;

    cmd[2] |= addr_node;
    cmd[3] |= addr_bcast;
    cmd
}

/// Get FSK RX stats
pub fn get_fsk_rx_stats_req() -> [u8; 2] {
    [0x02, 0x46]
}

/// Gets the status of the last received packet. Status is updated at the end of a reception (RxDone irq), but rssi_sync is already updated on SyncWordValid irq
pub fn get_fsk_packet_status_req() -> [u8; 2] {
    [0x02, 0x47]
}

// Response structs

/// Response for GetFskRxStats command
#[derive(Default)]
pub struct FskRxStatsRsp([u8; 16]);

impl FskRxStatsRsp {
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

    /// Number of RTC timeouts
    pub fn timeout(&self) -> u16 {
        (self.0[15] as u16) |
        ((self.0[14] as u16) << 8)
    }
}

impl AsMut<[u8]> for FskRxStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetFskPacketStatus command
#[derive(Default)]
pub struct FskPacketStatusRsp([u8; 8]);

impl FskPacketStatusRsp {
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

    /// Indicates if the last packet received matched the broadcast address
    pub fn addr_match_bcast(&self) -> bool {
        (self.0[6] >> 5) & 0x1 != 0
    }

    /// Indicates if the last packet received matched the node address
    pub fn addr_match_node(&self) -> bool {
        (self.0[6] >> 4) & 0x1 != 0
    }

    /// Link quality indicator (0.25dB)
    pub fn lqi(&self) -> u8 {
        self.0[7]
    }
}

impl AsMut<[u8]> for FskPacketStatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
