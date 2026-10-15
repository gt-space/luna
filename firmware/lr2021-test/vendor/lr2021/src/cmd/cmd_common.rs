// Common commands API

use crate::status::Status;

/// RX path selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxPath {
    LfPath = 0,
    #[cfg(feature = "rf2g4")]
    HfPath = 1,
}

/// RX boost configuration (0..7). Will keep previous value if not sent
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxBoost {
    Off = 0,
    B1 = 1,
    B2 = 2,
    B3 = 3,
    B4 = 4,
    B5 = 5,
    B6 = 6,
    Max = 7,
}

/// Select which PA to use
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PaSel {
    LfPa = 0,
    HfPa = 1,
}

/// PA LF mode (if unused set to 0)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PaLfMode {
    LfPaFsm = 0,
    LfPaFdm = 1,
    LfPaHsmRfo1 = 2,
    LfPaHsmRfo2 = 3,
}

/// PA ramp time selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RampTime {
    Ramp2u = 0,
    Ramp4u = 1,
    Ramp8u = 2,
    Ramp16u = 3,
    Ramp32u = 4,
    Ramp48u = 5,
    Ramp64u = 6,
    Ramp80u = 7,
    Ramp96u = 8,
    Ramp112u = 9,
    Ramp128u = 10,
    Ramp144u = 11,
    Ramp160u = 12,
    Ramp176u = 13,
    Ramp192u = 14,
    Ramp208u = 15,
}

/// Fallback mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FallbackMode {
    StandbyRc = 1,
    StandbyXosc = 2,
    Fs = 3,
}

/// Packet type selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PacketType {
    Lora = 0,
    FskGeneric = 1,
    FskLegacy = 2,
    Ble = 3,
    Ranging = 4,
    Flrc = 5,
    Bpsk = 6,
    LrFhss = 7,
    Wmbus = 8,
    Wisun = 9,
    Ook = 10,
    Raw = 11,
    Zwave = 12,
    Zigbee = 13,
}

/// Test mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TestMode {
    Packet = 0,
    Preamble = 1,
    Tone = 2,
    Prbs9 = 3,
}

/// Auto mode configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AutoTxrxMode {
    Disable = 0,
    Always = 1,
    RxOk = 2,
}

/// Index of the source to configure (0-2)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimestampIndex {
    Ts0 = 0,
    Ts1 = 1,
    Ts2 = 2,
}

/// Event source selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimestampSource {
    None = 0,
    TxDone = 1,
    RxDone = 2,
    Sync = 3,
    Header = 4,
}

/// Action taken after the CAD
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExitMode {
    Fallback = 0,
    Tx = 1,
    Rx = 2,
}

/// Sets the RF frequency for subsequent radio operations. Will not work with the chip in TX mode. All frequency dependent parameters are automatically recomputed by the FW
pub fn set_rf_frequency_cmd(rf_freq: u32) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x00;

    cmd[2] |= ((rf_freq >> 24) & 0xFF) as u8;
    cmd[3] |= ((rf_freq >> 16) & 0xFF) as u8;
    cmd[4] |= ((rf_freq >> 8) & 0xFF) as u8;
    cmd[5] |= (rf_freq & 0xFF) as u8;
    cmd
}

/// Sets the RX path and boost configuration. If rx_boost is changed, the SRC calibration (ADC offset) is run again for G12 and G13 with the updated boost configuration
pub fn set_rx_path_cmd(rx_path: RxPath) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x01;

    cmd[2] |= (rx_path as u8) & 0x1;
    cmd
}

/// Sets the RX path and boost configuration. If rx_boost is changed, the SRC calibration (ADC offset) is run again for G12 and G13 with the updated boost configuration
pub fn set_rx_path_adv_cmd(rx_path: RxPath, rx_boost: RxBoost) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x01;

    cmd[2] |= (rx_path as u8) & 0x1;
    cmd[3] |= (rx_boost as u8) & 0x7;
    cmd
}

/// Chooses which PA to use and sets the parameters of the PA
pub fn set_pa_config_cmd(pa_sel: PaSel, pa_lf_mode: PaLfMode, pa_lf_duty_cycle: u8, pa_lf_slices: u8) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x02;

    cmd[2] |= ((pa_sel as u8) & 0x1) << 7;
    cmd[2] |= (pa_lf_mode as u8) & 0x3;
    cmd[2] |= (pa_lf_duty_cycle & 0xF) << 4;
    cmd[3] |= pa_lf_slices & 0xF;
    cmd
}

/// Chooses which PA to use and sets the parameters of the PA
pub fn set_pa_config_adv_cmd(pa_sel: PaSel, pa_lf_mode: PaLfMode, pa_lf_duty_cycle: u8, pa_lf_slices: u8, pa_hf_duty_cycle: u8) -> [u8; 5] {
    let mut cmd = [0u8; 5];
    cmd[0] = 0x02;
    cmd[1] = 0x02;

    cmd[2] |= ((pa_sel as u8) & 0x1) << 7;
    cmd[2] |= (pa_lf_mode as u8) & 0x3;
    cmd[2] |= (pa_lf_duty_cycle & 0xF) << 4;
    cmd[3] |= pa_lf_slices & 0xF;
    cmd[4] |= pa_hf_duty_cycle & 0x1F;
    cmd
}

/// Sets the TX power and ramp time of the PA. The FW configures the corresponding registers, including OCP/OVP
pub fn set_tx_params_cmd(tx_power: i8, ramp_time: RampTime) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x02;
    cmd[1] = 0x03;

    cmd[2] |= (tx_power) as u8;
    cmd[3] |= ramp_time as u8;
    cmd
}

/// Configures the fallback mode after a RX or TX operation (after transmission/reception or timeout)
pub fn set_rx_tx_fallback_mode_cmd(fallback_mode: FallbackMode) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x06;

    cmd[2] |= (fallback_mode as u8) & 0x3;
    cmd
}

/// Sets the current packet type. This is the first command to be sent when configuring the radio for transceiver operation. Will only work in Standby RC, Standby XOSC or FS mode
pub fn set_packet_type_cmd(packet_type: PacketType) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x07;

    cmd[2] |= packet_type as u8;
    cmd
}

/// Returns the current packet type of the radio
pub fn get_packet_type_req() -> [u8; 2] {
    [0x02, 0x08]
}

/// Defines if the RX timeout should be stopped on Syncword/Header detection or on Preamble detection
pub fn set_stop_timeout_cmd(stop_on_preamble: bool) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x09;

    if stop_on_preamble { cmd[2] |= 1; }
    cmd
}

/// Reset Rx Statistics
pub fn reset_rx_stats_cmd() -> [u8; 2] {
    [0x02, 0x0A]
}

/// Gets the instantaneous RSSI value during reception of the packet. Returned value corresponds to -rssi/2 (dBm)
pub fn get_rssi_inst_req() -> [u8; 2] {
    [0x02, 0x0B]
}

/// Sets the device into RX mode. The RTC is started with the given value. RxTimeout is in 1/32.768kHz steps, allowing a maximum of 512 seconds timeout. If image rejection calibration was not done for current RF frequency, error RXFREQ_NO_CAL_ERR is generated
pub fn set_rx_cmd() -> [u8; 2] {
    [0x02, 0x0C]
}

/// Sets the device into RX mode. The RTC is started with the given value. RxTimeout is in 1/32.768kHz steps, allowing a maximum of 512 seconds timeout. If image rejection calibration was not done for current RF frequency, error RXFREQ_NO_CAL_ERR is generated
pub fn set_rx_adv_cmd(rx_timeout: u32) -> [u8; 5] {
    let mut cmd = [0u8; 5];
    cmd[0] = 0x02;
    cmd[1] = 0x0C;

    cmd[2] |= ((rx_timeout >> 16) & 0xFF) as u8;
    cmd[3] |= ((rx_timeout >> 8) & 0xFF) as u8;
    cmd[4] |= (rx_timeout & 0xFF) as u8;
    cmd
}

/// Sets the device into TX mode. The RTC is started with the given value. TxTimeout is in 1/32.768kHz steps, allowing a maximum of 512 seconds timeout
pub fn set_tx_cmd() -> [u8; 2] {
    [0x02, 0x0D]
}

/// Sets the device into TX mode. The RTC is started with the given value. TxTimeout is in 1/32.768kHz steps, allowing a maximum of 512 seconds timeout
pub fn set_tx_adv_cmd(tx_timeout: u32) -> [u8; 5] {
    let mut cmd = [0u8; 5];
    cmd[0] = 0x02;
    cmd[1] = 0x0D;

    cmd[2] |= ((tx_timeout >> 16) & 0xFF) as u8;
    cmd[3] |= ((tx_timeout >> 8) & 0xFF) as u8;
    cmd[4] |= (tx_timeout & 0xFF) as u8;
    cmd
}

/// Sets the device into TX test mode
pub fn set_tx_test_mode_cmd(test_mode: TestMode) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x0E;

    cmd[2] |= test_mode as u8;
    cmd
}

/// Select which PA to use. Configuration must have been provided beforehand using SetPaConfig. Selection cannot be changed in TX mode
pub fn sel_pa_cmd(pa_sel: PaSel) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x0F;

    cmd[2] |= (pa_sel as u8) & 0x1;
    cmd
}

/// Start reception every cycle_time and listen for rx_max_time
pub fn set_rx_duty_cycle_cmd(rx_max_time: u32, cycle_time: u32, use_lora_cad: bool, dram_ret: u8) -> [u8; 9] {
    let mut cmd = [0u8; 9];
    cmd[0] = 0x02;
    cmd[1] = 0x10;

    cmd[2] |= ((rx_max_time >> 16) & 0xFF) as u8;
    cmd[3] |= ((rx_max_time >> 8) & 0xFF) as u8;
    cmd[4] |= (rx_max_time & 0xFF) as u8;
    cmd[5] |= ((cycle_time >> 16) & 0xFF) as u8;
    cmd[6] |= ((cycle_time >> 8) & 0xFF) as u8;
    cmd[7] |= (cycle_time & 0xFF) as u8;
    if use_lora_cad { cmd[8] |= 16; }
    cmd[8] |= dram_ret & 0x7;
    cmd
}

/// Activate or deactivate the auto TX/auto RX mode. In auto RX mode, chip automatically goes from TX to RX after TxDone. In auto TX mode, chip automatically goes from RX to TX after RxDone
pub fn set_auto_rx_tx_cmd(clear: bool, auto_txrx_mode: AutoTxrxMode, timeout: u32, delay: u32) -> [u8; 10] {
    let mut cmd = [0u8; 10];
    cmd[0] = 0x02;
    cmd[1] = 0x11;

    if clear { cmd[2] |= 128; }
    cmd[2] |= (auto_txrx_mode as u8) & 0x3;
    cmd[3] |= ((timeout >> 16) & 0xFF) as u8;
    cmd[4] |= ((timeout >> 8) & 0xFF) as u8;
    cmd[5] |= (timeout & 0xFF) as u8;
    cmd[6] |= ((delay >> 24) & 0xFF) as u8;
    cmd[7] |= ((delay >> 16) & 0xFF) as u8;
    cmd[8] |= ((delay >> 8) & 0xFF) as u8;
    cmd[9] |= (delay & 0xFF) as u8;
    cmd
}

/// Get the length of the last received packet
pub fn get_rx_pkt_length_req() -> [u8; 2] {
    [0x02, 0x12]
}

/// Set the global value of the power offset
pub fn set_power_offset_cmd(power_offset: u8) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x14;

    cmd[2] |= power_offset & 0x3F;
    cmd
}

/// Sets the default RX and TX timeouts to be used for DIO RX/TX triggers, or if the timeout parameters are not sent in the SetRx and SetTx commands
pub fn set_default_rx_tx_timeout_cmd(rx_timeout: u32, tx_timeout: u32) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x02;
    cmd[1] = 0x15;

    cmd[2] |= ((rx_timeout >> 16) & 0xFF) as u8;
    cmd[3] |= ((rx_timeout >> 8) & 0xFF) as u8;
    cmd[4] |= (rx_timeout & 0xFF) as u8;
    cmd[5] |= ((tx_timeout >> 16) & 0xFF) as u8;
    cmd[6] |= ((tx_timeout >> 8) & 0xFF) as u8;
    cmd[7] |= (tx_timeout & 0xFF) as u8;
    cmd
}

/// Sets the source event for time-stamping different radio events. 3 sources can be configured in parallel
pub fn set_timestamp_source_cmd(timestamp_index: TimestampIndex, timestamp_source: TimestampSource) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x16;

    cmd[2] |= ((timestamp_index as u8) & 0x3) << 4;
    cmd[2] |= (timestamp_source as u8) & 0xF;
    cmd
}

/// Get the delay in HF clk tick between the event and the SPI NSS falling edge of the request. Will not return a correct value if the event occurred before a sleep period
pub fn get_timestamp_value_req(timestamp_index: TimestampIndex) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x17;

    cmd[2] |= (timestamp_index as u8) & 0x3;
    cmd
}

/// Set the radio into RX mode for Clear Channel Assessment measurements. The radio measures the RSSI for the given duration
pub fn set_cca_cmd(duration: u32) -> [u8; 5] {
    let mut cmd = [0u8; 5];
    cmd[0] = 0x02;
    cmd[1] = 0x18;

    cmd[2] |= ((duration >> 16) & 0xFF) as u8;
    cmd[3] |= ((duration >> 8) & 0xFF) as u8;
    cmd[4] |= (duration & 0xFF) as u8;
    cmd
}

/// Set the radio into RX mode for Clear Channel Assessment measurements. The radio measures the RSSI for the given duration
pub fn set_cca_adv_cmd(duration: u32, gain: u8) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x02;
    cmd[1] = 0x18;

    cmd[2] |= ((duration >> 16) & 0xFF) as u8;
    cmd[3] |= ((duration >> 8) & 0xFF) as u8;
    cmd[4] |= (duration & 0xFF) as u8;
    cmd[5] |= gain;
    cmd
}

/// Get the RSSI statistics for the CCA measurement
pub fn get_cca_result_req() -> [u8; 2] {
    [0x02, 0x19]
}

/// Set the manual gain of the AGC. A value of 0 enables the AGC: automatic gain
pub fn set_agc_gain_manual_cmd(gain_step: u8) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x02;
    cmd[1] = 0x1A;

    cmd[2] |= gain_step & 0xF;
    cmd
}

/// Set the CAD parameters for the Channel Activity Detect for packet types other than LoRa. This CAD is based on the measured RSSI
pub fn set_cad_params_cmd(cad_timeout: u32, threshold: u8, exit_mode: ExitMode, trx_timeout: u32) -> [u8; 10] {
    let mut cmd = [0u8; 10];
    cmd[0] = 0x02;
    cmd[1] = 0x1B;

    cmd[2] |= ((cad_timeout >> 16) & 0xFF) as u8;
    cmd[3] |= ((cad_timeout >> 8) & 0xFF) as u8;
    cmd[4] |= (cad_timeout & 0xFF) as u8;
    cmd[5] |= threshold;
    cmd[6] |= (exit_mode as u8) & 0x3;
    cmd[7] |= ((trx_timeout >> 16) & 0xFF) as u8;
    cmd[8] |= ((trx_timeout >> 8) & 0xFF) as u8;
    cmd[9] |= (trx_timeout & 0xFF) as u8;
    cmd
}

/// Set device into RX CAD mode (not LoRa). Parameters must have been previously set using SetCadParams command
pub fn set_cad_cmd() -> [u8; 2] {
    [0x02, 0x1C]
}

// Response structs

/// Response for GetPacketType command
#[derive(Default)]
pub struct PacketTypeRsp([u8; 3]);

impl PacketTypeRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Current packet type (see SetPacketType for values)
    pub fn packet_type(&self) -> u8 {
        self.0[2]
    }
}

impl AsMut<[u8]> for PacketTypeRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetRssiInst command
#[derive(Default)]
pub struct RssiInstRsp([u8; 4]);

impl RssiInstRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Instantaneous RSSI value. Actual signal power is -rssi/2 (dBm). If only 1 dBm resolution is wanted, reading the fractional bit is optional
    pub fn rssi(&self) -> u16 {
        ((self.0[3] & 0x1) as u16) |
        ((self.0[2] as u16) << 1)
    }
}

impl AsMut<[u8]> for RssiInstRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetRxPktLength command
#[derive(Default)]
pub struct RxPktLengthRsp([u8; 4]);

impl RxPktLengthRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Length of the last received packet in bytes
    pub fn pkt_length(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }
}

impl AsMut<[u8]> for RxPktLengthRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetTimestampValue command
#[derive(Default)]
pub struct TimestampValueRsp([u8; 6]);

impl TimestampValueRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Timestamp value in HF clock ticks
    pub fn timestamp(&self) -> u32 {
        (self.0[5] as u32) |
        ((self.0[4] as u32) << 8) |
        ((self.0[3] as u32) << 16) |
        ((self.0[2] as u32) << 24)
    }
}

impl AsMut<[u8]> for TimestampValueRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetCcaResult command
#[derive(Default)]
pub struct CcaResultRsp([u8; 6]);

impl CcaResultRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Minimum RSSI value measured during CCA. Actual value is -rssi_min/2 (dBm)
    pub fn rssi_min(&self) -> u16 {
        (((self.0[5] >> 2) & 0x1) as u16) |
        ((self.0[2] as u16) << 1)
    }

    /// Maximum RSSI value measured during CCA. Actual value is -rssi_max/2 (dBm)
    pub fn rssi_max(&self) -> u16 {
        (((self.0[5] >> 1) & 0x1) as u16) |
        ((self.0[3] as u16) << 1)
    }

    /// Average RSSI value measured during CCA. Actual value is -rssi_avg/2 (dBm)
    pub fn rssi_avg(&self) -> u16 {
        ((self.0[5] & 0x1) as u16) |
        ((self.0[4] as u16) << 1)
    }
}

impl AsMut<[u8]> for CcaResultRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

// Commands with variable length parameters (not implemented):
// - SetRssiCalibration
