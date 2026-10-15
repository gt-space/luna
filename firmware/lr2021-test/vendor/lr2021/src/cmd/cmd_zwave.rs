// Zwave commands API

use crate::status::Status;
use super::RxBw;

/// The data rate to be used for the RX and the TX
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ZwaveMode {
    Lr1 = 0,
    R1 = 1,
    R2 = 2,
    R3 = 3,
}

impl ZwaveMode {
    pub fn new(val: u8) -> Self{
        match val {
            3 => ZwaveMode::R3,
            2 => ZwaveMode::R2,
            _ => ZwaveMode::R1,
        }
    }
}

/// Enable or disable the filtering of the HomeID
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ZwaveAddrComp {
    Off = 0,
    Homeid = 1,
    HomeidBeam = 2,
}

/// FCS mode: auto to automatically generate FCS in TX and remove it in RX. In fifo mode, the FCS is expected to be part of the payload and is not checked in RX.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FcsMode {
    Auto = 0,
    Fifo = 1,
}

/// Address length selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddrLen {
    Addr8bit = 0,
    Addr12bit = 1,
}

/// Sets the parameters for Z-Wave packets
pub fn set_zwave_params_cmd(zwave_mode: ZwaveMode, rx_bw: RxBw, zwave_addr_comp: ZwaveAddrComp, pld_len: u8, pbl_len_tx: u16, pbl_len_detect: u8, fcs_mode: FcsMode) -> [u8; 10] {
    let mut cmd = [0u8; 10];
    cmd[0] = 0x02;
    cmd[1] = 0x97;

    cmd[2] |= (zwave_mode as u8) & 0x3;
    cmd[3] |= rx_bw as u8;
    cmd[4] |= zwave_addr_comp as u8;
    cmd[5] |= pld_len;
    cmd[6] |= ((pbl_len_tx >> 8) & 0xFF) as u8;
    cmd[7] |= (pbl_len_tx & 0xFF) as u8;
    cmd[8] |= pbl_len_detect;
    cmd[9] |= (fcs_mode as u8) & 0x1;
    cmd
}

/// Sets the HomeID address to use as a filter in Rx. The MSB is the first bit sent/received. Frames that don't match the home_id raise an address error IRQ
pub fn set_zwave_home_id_filtering_cmd(home_id: u32) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x98;

    cmd[2] |= ((home_id >> 24) & 0xFF) as u8;
    cmd[3] |= ((home_id >> 16) & 0xFF) as u8;
    cmd[4] |= ((home_id >> 8) & 0xFF) as u8;
    cmd[5] |= (home_id & 0xFF) as u8;
    cmd
}

/// Gets the status of the last received packet. Status is updated at the end of a reception (RxDone irq), but rssi_sync is already updated on SyncWordValid irq
pub fn get_zwave_packet_status_req() -> [u8; 2] {
    [0x02, 0x9A]
}

/// Settings for filtering incoming beam frames in Rx
pub fn set_zwave_beam_filtering_cmd(beam_tag: u8, addr_len: AddrLen, node_id: u16, id_hash: u8) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x9B;

    cmd[2] |= beam_tag;
    cmd[3] |= ((addr_len as u8) & 0x1) << 7;
    cmd[3] |= ((node_id >> 8) & 0xFF) as u8;
    cmd[4] |= (node_id & 0xFF) as u8;
    cmd[5] |= id_hash;
    cmd
}

/// This command is used to enter the Zwave RX scan mode. The scan mode will alternate between data rates and channels and attempt to detect an incoming packet
pub fn set_zwave_scan_cmd() -> [u8; 2] {
    [0x02, 0x9D]
}

/// Gets the internal statistics of the received packets. Statistics are reset on a POR, sleep without memory retention and the command ResetRxStats
pub fn get_zwave_rx_stats_req() -> [u8; 2] {
    [0x02, 0x99]
}

// Response structs

/// Response for GetZwavePacketStatus command
#[derive(Default)]
pub struct ZwavePacketStatusRsp([u8; 9]);

impl ZwavePacketStatusRsp {
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

    /// Average over last packet received of RSSI. Actual signal power is –rssi_pkt/2 (dBm)
    pub fn rssi_avg(&self) -> u16 {
        (((self.0[7] >> 2) & 0x1) as u16) |
        ((self.0[4] as u16) << 1)
    }

    /// Latch RSSI value after syncword detection. Actual signal power is –rssi_sync/2 (dBm)
    pub fn rssi_sync(&self) -> u16 {
        ((self.0[7] & 0x1) as u16) |
        ((self.0[5] as u16) << 1)
    }

    /// Last detected packet data rate
    pub fn last_detect(&self) -> ZwaveMode {
        ZwaveMode::new(self.0[6] & 0x3)
    }

    /// Link quality indicator (0.25dB)
    pub fn lqi(&self) -> u8 {
        self.0[8]
    }
}

impl AsMut<[u8]> for ZwavePacketStatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetZwaveRxStats command
#[derive(Default)]
pub struct ZwaveRxStatsRsp([u8; 8]);

impl ZwaveRxStatsRsp {
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

impl AsMut<[u8]> for ZwaveRxStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
