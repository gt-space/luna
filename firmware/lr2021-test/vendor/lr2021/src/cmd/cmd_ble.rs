// Ble commands API

use crate::status::Status;
use super::RxBw;

/// BLE PHY mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BleMode {
    Le1mb = 0,
    Le2mb = 1,
    LeCoded500k = 2,
    LeCoded125k = 3,
}

/// BLE channel type selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChannelType {
    Advertiser = 0,
    Data16bitHeader = 1,
    Data24bitHeader = 2,
}

/// Configure the modulation parameters for BLE packets
pub fn set_ble_modulation_params_cmd(ble_mode: BleMode) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x60;

    cmd[2] |= (ble_mode as u8) & 0x3;
    cmd
}

/// Configure the modulation parameters for BLE packets
pub fn set_ble_modulation_params_adv_cmd(ble_mode: BleMode, rx_bw: RxBw) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x60;

    cmd[2] |= (ble_mode as u8) & 0x3;
    cmd[3] |= rx_bw as u8;
    cmd
}

/// Sets the BLE channel/packet dependent parameters
pub fn set_ble_channel_params_cmd(crc_in_fifo: bool, channel_type: ChannelType, whit_init: u8, crc_init: u32, syncword: u32) -> [u8; 11] {
    let mut cmd = [0u8; 11];
    cmd[0] = 0x02;
    cmd[1] = 0x61;

    if crc_in_fifo { cmd[2] |= 16; }
    cmd[2] |= (channel_type as u8) & 0xF;
    cmd[3] |= whit_init;
    cmd[4] |= ((crc_init >> 16) & 0xFF) as u8;
    cmd[5] |= ((crc_init >> 8) & 0xFF) as u8;
    cmd[6] |= (crc_init & 0xFF) as u8;
    cmd[7] |= ((syncword >> 24) & 0xFF) as u8;
    cmd[8] |= ((syncword >> 16) & 0xFF) as u8;
    cmd[9] |= ((syncword >> 8) & 0xFF) as u8;
    cmd[10] |= (syncword & 0xFF) as u8;
    cmd
}

/// Configure PDU length to transmit and send a BLE packet. This command is a concatenation of SetBlePduLen(pld_len) and SetTx(0)
pub fn set_ble_tx_cmd(pld_len: u8) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x62;

    cmd[2] |= pld_len;
    cmd
}

/// Gets the status of the last received packet. Status is updated at the end of a reception (RxDone irq), but rssi_sync is already updated on SyncWordValid irq
pub fn get_ble_packet_status_req() -> [u8; 2] {
    [0x02, 0x65]
}

/// Gets the internal statistics of the received packets. Statistics are reset on a POR, sleep without memory retention and the command ResetRxStats
pub fn get_ble_rx_stats_req() -> [u8; 2] {
    [0x02, 0x64]
}

/// Sets PDU length for TX
pub fn set_ble_tx_pdu_len_cmd(pdu_len: u8) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x66;

    cmd[2] |= pdu_len;
    cmd
}

// Response structs

/// Response for GetBlePacketStatus command
#[derive(Default)]
pub struct BlePacketStatusRsp([u8; 8]);

impl BlePacketStatusRsp {
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

    /// Link quality indicator (0.25dB)
    pub fn lqi(&self) -> u8 {
        self.0[7]
    }
}

impl AsMut<[u8]> for BlePacketStatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetBleRxStats command
#[derive(Default)]
pub struct BleRxStatsRsp([u8; 8]);

impl BleRxStatsRsp {
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

impl AsMut<[u8]> for BleRxStatsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
