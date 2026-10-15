// System commands API

use crate::status::{Status,Intr};

/// DIO number (allowed values are 5-11)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DioNum {
    Dio5 = 5,
    Dio6 = 6,
    Dio7 = 7,
    Dio8 = 8,
    Dio9 = 9,
    Dio10 = 10,
    Dio11 = 11,
}

/// DIO function selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DioFunc {
    None = 0,
    Irq = 1,
    RfSwitch = 2,
    GpioOutputLow = 5,
    GpioOutputHigh = 6,
    HfClkOut = 7,
    LfClkOut = 8,
    TxTrigger = 9,
    RxTrigger = 10,
}

/// Pull-up/down configuration for sleep mode. DIO_PULL_AUTO means if DIO value in Standby was '1', it will be pulled-up, if '0' it will be pulled-down
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PullDrive {
    PullNone = 0,
    PullDown = 1,
    PullUp = 2,
    PullAuto = 3,
}

/// LF clock source selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LfClock {
    Rc = 0,
    Xtal = 1,
    Dio11 = 2,
}

/// 32Mhz clock division factor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClkScaling {
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
    Div16 = 4,
    Div32 = 5,
    Div64 = 6,
    Div128 = 7,
    Div256 = 8,
    Div512 = 9,
    Div1024 = 10,
    Div2048 = 11,
    Div4096 = 12,
    Div8192 = 13,
    Div16384 = 14,
    Div32768 = 15,
}

/// SIMO usage configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SimoUsage {
    Off = 0,
    All = 1,
    Auto = 2,
    Vdcc = 3,
}

/// Ramp time RC to RU resolution
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RampTimeRc2ruUnit {
    Res2u = 0,
    Res4u = 1,
    Res8u = 2,
    Res16u = 3,
}

/// Ramp time TX to RU resolution
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RampTimeTx2ruUnit {
    Res2u = 0,
    Res4u = 1,
    Res8u = 2,
    Res16u = 3,
}

/// Ramp time RU to RC resolution
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RampTimeRu2rcUnit {
    Res2u = 0,
    Res4u = 1,
    Res8u = 2,
    Res16u = 3,
}

/// Ramp down time
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RampTimeRampDownUnit {
    Res2u = 0,
    Res4u = 1,
    Res8u = 2,
    Res16u = 3,
}

/// Format of returned value
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VbatFormat {
    Raw = 0,
    Millivolts = 1,
}

/// ADC resolution for measurement
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AdcRes {
    Res8bit = 0,
    Res9bit = 1,
    Res10bit = 2,
    Res11bit = 3,
    Res12bit = 4,
    Res13bit = 5,
}

/// Temperature sensor source
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TempSrc {
    Vbe = 0,
    Xosc = 1,
    Ntc = 2,
}

/// Standby mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StandbyMode {
    Rc = 0,
    Xosc = 1,
}

/// Threshold voltage for EOL interrupt
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EolTrim {
    Eol1p60 = 0,
    Eol1p67 = 1,
    Eol1p74 = 2,
    Eol1p80 = 3,
    Eol1p88 = 4,
    Eol1p95 = 5,
    Eol2p00 = 6,
    Eol2p10 = 7,
}

/// Control Voltage provided to the TCXO
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TcxoVoltage {
    Tcxo1v6 = 0,
    Tcxo1v7 = 1,
    Tcxo1v8 = 2,
    Tcxo2v2 = 3,
    Tcxo2v4 = 4,
    Tcxo2v7 = 5,
    Tcxo3v0 = 6,
    Tcxo3v3 = 7,
}

/// Temperature compensation mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CompMode {
    Disabled = 0,
    Relative = 1,
    Absolute = 2,
}

/// Returns status of device. If the previous command was not a read command, returns the status register. If previous transaction was a read command, returns the result of the read command. Also clears the reset status returned in the Stat variable
pub fn get_status_req() -> [u8; 2] {
    [0x01, 0x00]
}

/// Returns version of firmware
pub fn get_version_req() -> [u8; 2] {
    [0x01, 0x01]
}

/// Returns current pending errors that occurred since the last ClearErrors() call, or the startup of the circuit. It is possible to generate an Irq on DIO when an error occurs
pub fn get_errors_req() -> [u8; 2] {
    [0x01, 0x10]
}

/// Clears all error flags in the status. The flags cannot be cleared individually. Calling ClearErrors will not clear the Error IRQ - the IRQ has to be cleared explicitly with the ClearIrq command
pub fn clear_errors_cmd() -> [u8; 2] {
    [0x01, 0x11]
}

/// Configure the functionality of the freely configurable DIOs, as well as the pull-up/down configuration for sleep modes. On DIO5, only DIO_SLEEP_PULL_UP is accepted. DIO5/6 have pull-up by default
pub fn set_dio_function_cmd(dio_num: DioNum, dio_func: DioFunc, pull_drive: PullDrive) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x01;
    cmd[1] = 0x12;

    cmd[2] |= (dio_num as u8) & 0xF;
    cmd[3] |= ((dio_func as u8) & 0xF) << 4;
    cmd[3] |= (pull_drive as u8) & 0xF;
    cmd
}

/// Configure the value of the specified DIO pin when configured as RF switch with the SetDioFunction command
pub fn set_dio_rf_switch_config_cmd(dio_num: DioNum, tx_hf: bool, rx_hf: bool, tx_lf: bool, rx_lf: bool, standby: bool) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x01;
    cmd[1] = 0x13;

    cmd[2] |= (dio_num as u8) & 0xF;
    if tx_hf { cmd[3] |= 16; }
    if rx_hf { cmd[3] |= 8; }
    if tx_lf { cmd[3] |= 4; }
    if rx_lf { cmd[3] |= 2; }
    if standby { cmd[3] |= 1; }
    cmd
}

/// Clears specific fifo irq flags
pub fn clear_fifo_irq_flags_cmd(rx_fifo_flags_to_clear: u8, tx_fifo_flags_to_clear: u8) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x01;
    cmd[1] = 0x14;

    cmd[2] |= rx_fifo_flags_to_clear;
    cmd[3] |= tx_fifo_flags_to_clear;
    cmd
}

/// Configure IRQs which assert DIO pin
pub fn set_dio_irq_config_cmd(dio_num: DioNum, irqs: u32) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x01;
    cmd[1] = 0x15;

    cmd[2] |= (dio_num as u8) & 0xF;
    cmd[3] |= ((irqs >> 24) & 0xFF) as u8;
    cmd[4] |= ((irqs >> 16) & 0xFF) as u8;
    cmd[5] |= ((irqs >> 8) & 0xFF) as u8;
    cmd[6] |= (irqs & 0xFF) as u8;
    cmd
}

/// Clear pending irqs
pub fn clear_irq_cmd(irqs: u32) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x01;
    cmd[1] = 0x16;

    cmd[2] |= ((irqs >> 24) & 0xFF) as u8;
    cmd[3] |= ((irqs >> 16) & 0xFF) as u8;
    cmd[4] |= ((irqs >> 8) & 0xFF) as u8;
    cmd[5] |= (irqs & 0xFF) as u8;
    cmd
}

/// Read out the pending IRQs and clear all pending IRQs
pub fn get_and_clear_irq_req() -> [u8; 2] {
    [0x01, 0x17]
}

/// Configures the used Low Frequency clock
pub fn config_lf_clock_cmd(lf_clock: LfClock) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x18;

    cmd[2] |= (lf_clock as u8) & 0x3;
    cmd
}

/// Configure LF and HF clock configuration for DIO output
pub fn config_clk_outputs_cmd(clk_scaling: ClkScaling) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x19;

    cmd[2] |= clk_scaling as u8;
    cmd
}

/// Configure which FIFO status flags generate FIFO IRQs as well as the threshold levels
pub fn config_fifo_irq_cmd(rx_fifo_irq_enable: u8, tx_fifo_irq_enable: u8) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x01;
    cmd[1] = 0x1A;

    cmd[2] |= rx_fifo_irq_enable;
    cmd[3] |= tx_fifo_irq_enable;
    cmd
}

/// Configure which FIFO status flags generate FIFO IRQs as well as the threshold levels
pub fn config_fifo_irq_adv_cmd(rx_fifo_irq_enable: u8, tx_fifo_irq_enable: u8, rx_high_threshold: u16, tx_low_threshold: u16, rx_low_threshold: u16, tx_high_threshold: u16) -> [u8; 12] {
    let mut cmd = [0u8; 12];
    cmd[0] = 0x01;
    cmd[1] = 0x1A;

    cmd[2] |= rx_fifo_irq_enable;
    cmd[3] |= tx_fifo_irq_enable;
    cmd[4] |= ((rx_high_threshold >> 8) & 0xFF) as u8;
    cmd[5] |= (rx_high_threshold & 0xFF) as u8;
    cmd[6] |= ((tx_low_threshold >> 8) & 0xFF) as u8;
    cmd[7] |= (tx_low_threshold & 0xFF) as u8;
    cmd[8] |= ((rx_low_threshold >> 8) & 0xFF) as u8;
    cmd[9] |= (rx_low_threshold & 0xFF) as u8;
    cmd[10] |= ((tx_high_threshold >> 8) & 0xFF) as u8;
    cmd[11] |= (tx_high_threshold & 0xFF) as u8;
    cmd
}

/// Returns all FIFO flags that have been triggered since the last flags clear. It does not depend on enabled flags for the IRQs
pub fn get_fifo_irq_flags_req() -> [u8; 2] {
    [0x01, 0x1B]
}

/// Read out the RX FIFO level in bytes
pub fn get_rx_fifo_level_req() -> [u8; 2] {
    [0x01, 0x1C]
}

/// Read out the TX FIFO level in bytes
pub fn get_tx_fifo_level_req() -> [u8; 2] {
    [0x01, 0x1D]
}

/// Clears the RX FIFO
pub fn clear_rx_fifo_cmd() -> [u8; 2] {
    [0x01, 0x1E]
}

/// Clears the TX FIFO
pub fn clear_tx_fifo_cmd() -> [u8; 2] {
    [0x01, 0x1F]
}

/// Configures the SIMO mode and ramp times
pub fn set_reg_mode_cmd(simo_usage: SimoUsage) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x21;

    cmd[2] |= (simo_usage as u8) & 0x3;
    cmd
}

#[allow(clippy::too_many_arguments)]
/// Configures the SIMO mode and ramp times
pub fn set_reg_mode_adv_cmd(simo_usage: SimoUsage, ramp_time_rc2ru_unit: RampTimeRc2ruUnit, ramp_time_rc2ru: u8, ramp_time_tx2ru_unit: RampTimeTx2ruUnit, ramp_time_tx2ru: u8, ramp_time_ru2rc_unit: RampTimeRu2rcUnit, ramp_time_ru2rc: u8, ramp_time_ramp_down_unit: RampTimeRampDownUnit, ramp_time_ramp_down: u8) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x01;
    cmd[1] = 0x21;

    cmd[2] |= (simo_usage as u8) & 0x3;
    cmd[3] |= ((ramp_time_rc2ru_unit as u8) & 0x3) << 5;
    cmd[3] |= ramp_time_rc2ru & 0x1F;
    cmd[4] |= ((ramp_time_tx2ru_unit as u8) & 0x3) << 5;
    cmd[4] |= ramp_time_tx2ru & 0x1F;
    cmd[5] |= ((ramp_time_ru2rc_unit as u8) & 0x3) << 5;
    cmd[5] |= ramp_time_ru2rc & 0x1F;
    cmd[6] |= ((ramp_time_ramp_down_unit as u8) & 0x3) << 5;
    cmd[6] |= ramp_time_ramp_down & 0x1F;
    cmd
}

/// Calibrates the requested blocks defined by the blocks_to_calibrate parameter. Will work in any mode of the chip. On exit the chip will be in Standby RC
pub fn calibrate_cmd(pa_offset: bool, meas_unit: bool, aaf: bool, pll: bool, hf_rc: bool, lf_rc: bool) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x22;

    if pa_offset { cmd[2] |= 64; }
    if meas_unit { cmd[2] |= 16; }
    if aaf { cmd[2] |= 8; }
    if pll { cmd[2] |= 4; }
    if hf_rc { cmd[2] |= 2; }
    if lf_rc { cmd[2] |= 1; }
    cmd
}

/// Launches all Front End calibrations (ADC Offset, PPF, Image) at the given frequencies. Will not work if device is in Rx or Tx mode
pub fn calib_fe_cmd(freq1: u16, freq2: u16, freq3: u16) -> [u8; 8] {
    let mut cmd = [0u8; 8];
    cmd[0] = 0x01;
    cmd[1] = 0x23;

    cmd[2] |= ((freq1 >> 8) & 0xFF) as u8;
    cmd[3] |= (freq1 & 0xFF) as u8;
    cmd[4] |= ((freq2 >> 8) & 0xFF) as u8;
    cmd[5] |= (freq2 & 0xFF) as u8;
    cmd[6] |= ((freq3 >> 8) & 0xFF) as u8;
    cmd[7] |= (freq3 & 0xFF) as u8;
    cmd
}

/// Measure and return current VBAT value
pub fn get_v_bat_req(vbat_format: VbatFormat, adc_res: AdcRes) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x24;

    cmd[2] |= ((vbat_format as u8) & 0x1) << 3;
    cmd[2] |= (adc_res as u8) & 0x7;
    cmd
}

/// Measure and return temperature value
pub fn get_temp_req(temp_src: TempSrc, adc_res: AdcRes) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x25;

    cmd[2] |= ((temp_src as u8) & 0x3) << 4;
    cmd[2] |= 8; // Force format to Celsius
    cmd[2] |= (adc_res as u8) & 0x7;
    cmd
}

/// Return a 32-bit random number
pub fn get_random_number_req() -> [u8; 2] {
    [0x01, 0x26]
}

/// Return a 32-bit random number
pub fn get_random_number_adv_req(source: u8) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x26;

    cmd[2] |= source & 0x3;
    cmd
}

/// Put device in sleep mode
pub fn set_sleep_cmd(clk_32k_en: bool, ret_en: u8) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x27;

    if clk_32k_en { cmd[2] |= 1; }
    cmd[2] |= (ret_en & 0xF) << 1;
    cmd
}

/// Put device in sleep mode
pub fn set_sleep_adv_cmd(clk_32k_en: bool, ret_en: u8, sleep_time: u32) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x01;
    cmd[1] = 0x27;

    if clk_32k_en { cmd[2] |= 1; }
    cmd[2] |= (ret_en & 0xF) << 1;
    cmd[3] |= ((sleep_time >> 24) & 0xFF) as u8;
    cmd[4] |= ((sleep_time >> 16) & 0xFF) as u8;
    cmd[5] |= ((sleep_time >> 8) & 0xFF) as u8;
    cmd[6] |= (sleep_time & 0xFF) as u8;
    cmd
}

/// Put device in standby mode (XOSC or RC)
pub fn set_standby_cmd(standby_mode: StandbyMode) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x28;

    cmd[2] |= (standby_mode as u8) & 0x1;
    cmd
}

/// Put device in Frequency Synthesis Mode
pub fn set_fs_cmd() -> [u8; 2] {
    [0x01, 0x29]
}

/// Specify address of additional register to save for sleep modes with retention
pub fn set_additional_reg_to_retain_cmd(slot: u8, addr: u32) -> [u8; 6] {
    let mut cmd = [0u8; 6];
    cmd[0] = 0x01;
    cmd[1] = 0x2A;

    cmd[2] |= slot & 0x1F;
    cmd[3] |= ((addr >> 16) & 0xFF) as u8;
    cmd[4] |= ((addr >> 8) & 0xFF) as u8;
    cmd[5] |= (addr & 0xFF) as u8;
    cmd
}

/// Read out and clear FIFO flags that generated FIFO IRQs
pub fn get_and_clear_fifo_irq_flags_req() -> [u8; 2] {
    [0x01, 0x2E]
}

/// Enable/disable end of life detection and setup trigger voltage
pub fn set_eol_config_cmd(eol_trim: EolTrim, enable: bool) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x30;

    cmd[2] |= (eol_trim as u8) & 0x7;
    if enable { cmd[2] |= 8; }
    cmd
}

/// Configure the chip to use a TCXO
pub fn set_tcxo_mode_cmd(tcxo_voltage: TcxoVoltage, start_time: u32) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x01;
    cmd[1] = 0x20;

    cmd[2] |= (tcxo_voltage as u8) & 0x7;
    cmd[3] |= ((start_time >> 24) & 0xFF) as u8;
    cmd[4] |= ((start_time >> 16) & 0xFF) as u8;
    cmd[5] |= ((start_time >> 8) & 0xFF) as u8;
    cmd[6] |= (start_time & 0xFF) as u8;
    cmd
}

/// Configure XOSC foot capacitor trim
pub fn set_xosc_cp_trim_cmd(xta: u8, xtb: u8) -> [u8; 4] {
    let mut cmd = [0u8; 4];
    cmd[0] = 0x01;
    cmd[1] = 0x31;

    cmd[2] |= xta & 0x3F;
    cmd[3] |= xtb & 0x3F;
    cmd
}

/// Configure XOSC foot capacitor trim
pub fn set_xosc_cp_trim_adv_cmd(xta: u8, xtb: u8, delay_us: u8) -> [u8; 5] {
    let mut cmd = [0u8; 5];
    cmd[0] = 0x01;
    cmd[1] = 0x31;

    cmd[2] |= xta & 0x3F;
    cmd[3] |= xtb & 0x3F;
    cmd[4] |= delay_us;
    cmd
}

/// Configure temperature compensation mode for TX and RX
pub fn set_temp_comp_cfg_cmd(ntc_en: bool, comp_mode: CompMode) -> [u8; 3] {
    let mut cmd = [0u8; 3];
    cmd[0] = 0x01;
    cmd[1] = 0x32;

    if ntc_en { cmd[2] |= 4; }
    cmd[2] |= (comp_mode as u8) & 0x3;
    cmd
}

/// Configure NTC parameters
pub fn set_ntc_params_cmd(ntc_r_ratio: u16, ntc_beta: u16, delay: u8) -> [u8; 7] {
    let mut cmd = [0u8; 7];
    cmd[0] = 0x01;
    cmd[1] = 0x33;

    cmd[2] |= ((ntc_r_ratio >> 8) & 0xFF) as u8;
    cmd[3] |= (ntc_r_ratio & 0xFF) as u8;
    cmd[4] |= ((ntc_beta >> 8) & 0xFF) as u8;
    cmd[5] |= (ntc_beta & 0xFF) as u8;
    cmd[6] |= delay;
    cmd
}

// Response structs

/// Response for GetStatus command
#[derive(Default)]
pub struct StatusRsp([u8; 6]);

impl StatusRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Status register with command status, interrupt status, reset source, and chip mode
    pub fn stat(&self) -> u16 {
        (self.0[1] as u16) |
        ((self.0[0] as u16) << 8)
    }

    /// IRQ status register
    pub fn intr(&self) -> Intr {
        Intr::from_slice(&self.0[2..6])
    }
}

impl AsMut<[u8]> for StatusRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetVersion command
#[derive(Default)]
pub struct VersionRsp([u8; 4]);

impl VersionRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Firmware major version
    pub fn major(&self) -> u8 {
        self.0[2]
    }

    /// Firmware minor version
    pub fn minor(&self) -> u8 {
        self.0[3]
    }
}

impl AsMut<[u8]> for VersionRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
#[cfg(feature = "defmt")]
impl defmt::Format for VersionRsp {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{:02x}.{:02x}", self.major(), self.minor());
    }
}

/// Response for GetErrors command
#[derive(Default)]
pub struct ErrorsRsp([u8; 4]);

impl ErrorsRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// High frequency xosc did not start correctly. This is most likely a hardware problem, which might be resolved by doing a reset.
    pub fn hf_xosc_start(&self) -> bool {
        self.0[3] & 0x1 != 0
    }

    /// Low frequency xosc did not start correctly. This is either a hardware problem, which might be resolved by doing a reset, or there is a TCXO instead which must be enabled through SetTcxoMode command.
    pub fn lf_xosc_start(&self) -> bool {
        (self.0[3] >> 1) & 0x1 != 0
    }

    /// The pll did not lock. This can come from too high or too low a frequency, or if the pll was not calibrated. To fix it try redoing a pll calibration, or using other frequencies
    pub fn pll_lock(&self) -> bool {
        (self.0[3] >> 2) & 0x1 != 0
    }

    /// Error during calibration of the low frequency RC, thus no calibration is available. To fix it try redoing a calibration. Might come from a hardware issue.
    pub fn lf_rc_calib(&self) -> bool {
        (self.0[3] >> 3) & 0x1 != 0
    }

    /// Error during calibration of the high frequency RC, thus no calibration is available. To fix it try redoing a calibration. Might come from a hardware issue.
    pub fn hf_rc_calib(&self) -> bool {
        (self.0[3] >> 4) & 0x1 != 0
    }

    /// Error during calibration of the PLL (max-min, bank estim, type2), thus no calibration is available. To fix it redo the pll calibration. Might come from a hardware issue.
    pub fn pll_calib(&self) -> bool {
        (self.0[3] >> 5) & 0x1 != 0
    }

    /// Error during calibration of the aaf (anti-aliasing filter), thus no calibration is available. To fix it try redoing a calibration. Might come from a hardware issue.
    pub fn aaf_calib(&self) -> bool {
        (self.0[3] >> 6) & 0x1 != 0
    }

    /// Error during calibration of the image rejection (IQ comp), thus no calibration is available. To fix it redo the image calibration.
    pub fn img_calib(&self) -> bool {
        (self.0[3] >> 7) & 0x1 != 0
    }

    /// Error asserted when a DIO TX or RX trigger could not be executed because chip was busy changing mode.
    pub fn chip_busy(&self) -> bool {
        self.0[2] & 0x1 != 0
    }

    /// Front End (Image rejection, PPF, ADC offset) calibration was not available for rx operation with specified rf frequency. To fix this do the front end calibration for specific rx rf frequency.
    pub fn rxfreq_no_fe_cal(&self) -> bool {
        (self.0[2] >> 1) & 0x1 != 0
    }

    /// Error during calibration of the measure unit adc, thus no calibration is available. To fix it try redoing a calibration. Might come from a hardware issue.
    pub fn meas_unit_adc_calib(&self) -> bool {
        (self.0[2] >> 2) & 0x1 != 0
    }

    /// Error during calibration of the PA offset, thus no calibration is available. To fix it try redoing a calibration. Might come from a hardware issue.
    pub fn pa_offset_calib(&self) -> bool {
        (self.0[2] >> 3) & 0x1 != 0
    }

    /// Error during calibration of the PPF (Poly-Phase Filter), thus no calibration is available. To fix it try redoing a calibration. Might come from a hardware issue.
    pub fn ppf_calib(&self) -> bool {
        (self.0[2] >> 4) & 0x1 != 0
    }

    /// Error during calibration of the SRC (Self Reception Cancelation), thus no calibration is available. To fix it try redoing a calibration. Might come from a hardware issue.
    pub fn src_calib(&self) -> bool {
        (self.0[2] >> 5) & 0x1 != 0
    }
    /// 16 bits value
    pub fn value(&self) -> u16 {
        u16::from_be_bytes([self.0[2], self.0[3]])
    }

    /// Flag when no error are present
    pub fn none(&self) -> bool {
        self.0[2] == 0 && self.0[3] == 0
    }
}

impl AsMut<[u8]> for ErrorsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
#[cfg(feature = "defmt")]
impl defmt::Format for ErrorsRsp {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Errors: ");
        if self.none() {
            defmt::write!(f, "None");
            return;
        }
        if self.hf_xosc_start()       {defmt::write!(f, "HfXoscStart ")};
        if self.lf_xosc_start()       {defmt::write!(f, "LfXoscStart ")};
        if self.pll_lock()            {defmt::write!(f, "PllLock ")};
        if self.lf_rc_calib()         {defmt::write!(f, "LfRcCalib ")};
        if self.hf_rc_calib()         {defmt::write!(f, "HfRcCalib ")};
        if self.pll_calib()           {defmt::write!(f, "PllCalib ")};
        if self.aaf_calib()           {defmt::write!(f, "AafCalib ")};
        if self.img_calib()           {defmt::write!(f, "ImgCalib ")};
        if self.chip_busy()           {defmt::write!(f, "ChipBusy ")};
        if self.rxfreq_no_fe_cal()    {defmt::write!(f, "RxfreqNoFeCal ")};
        if self.meas_unit_adc_calib() {defmt::write!(f, "MeasUnitAdcCalib ")};
        if self.pa_offset_calib()     {defmt::write!(f, "PaOffsetCalib ")};
        if self.ppf_calib()           {defmt::write!(f, "PpfCalib ")};
        if self.src_calib()           {defmt::write!(f, "SrcCalib ")};
    }
}

/// Response for GetAndClearIrq command
#[derive(Default)]
pub struct AndClearIrqRsp([u8; 6]);

impl AndClearIrqRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Current pending IRQ status (cleared after reading)
    pub fn intr(&self) -> u32 {
        (self.0[5] as u32) |
        ((self.0[4] as u32) << 8) |
        ((self.0[3] as u32) << 16) |
        ((self.0[2] as u32) << 24)
    }
}

impl AsMut<[u8]> for AndClearIrqRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetFifoIrqFlags command
#[derive(Default)]
pub struct FifoIrqFlagsRsp([u8; 4]);

impl FifoIrqFlagsRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// RX FIFO flags status
    pub fn rx_fifo_flags(&self) -> u8 {
        self.0[2]
    }

    /// TX FIFO flags status
    pub fn tx_fifo_flags(&self) -> u8 {
        self.0[3]
    }
}

impl AsMut<[u8]> for FifoIrqFlagsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetRxFifoLevel command
#[derive(Default)]
pub struct RxFifoLevelRsp([u8; 4]);

impl RxFifoLevelRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// RX FIFO level in bytes
    pub fn level(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }
}

impl AsMut<[u8]> for RxFifoLevelRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetTxFifoLevel command
#[derive(Default)]
pub struct TxFifoLevelRsp([u8; 4]);

impl TxFifoLevelRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// TX FIFO level in bytes
    pub fn level(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }
}

impl AsMut<[u8]> for TxFifoLevelRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetVBat command
#[derive(Default)]
pub struct VBatRsp([u8; 4]);

impl VBatRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Raw VBAT measurement (format=0)
    pub fn vbat_raw(&self) -> u16 {
        (self.0[3] as u16) |
        (((self.0[2] & 0x1F) as u16) << 8)
    }

    /// VBAT in millivolts (format=1)
    pub fn vbat_mv(&self) -> u16 {
        (self.0[3] as u16) |
        ((self.0[2] as u16) << 8)
    }
}

impl AsMut<[u8]> for VBatRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetTemp command
#[derive(Default)]
pub struct TempRsp([u8; 4]);

impl TempRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// Temperature in degrees Celsius (format=1)
    pub fn temp_celsius(&self) -> i16 {
        let raw = ((self.0[3] >> 3) as u16) |
            ((self.0[2] as u16) << 5);
        raw as i16 - if (self.0[2] & 0x80) != 0 {1<<13} else {0}
    }
}

impl AsMut<[u8]> for TempRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetRandomNumber command
#[derive(Default)]
pub struct RandomNumberRsp([u8; 6]);

impl RandomNumberRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// 32-bit random number
    pub fn random_number(&self) -> u32 {
        (self.0[5] as u32) |
        ((self.0[4] as u32) << 8) |
        ((self.0[3] as u32) << 16) |
        ((self.0[2] as u32) << 24)
    }
}

impl AsMut<[u8]> for RandomNumberRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// Response for GetAndClearFifoIrqFlags command
#[derive(Default)]
pub struct AndClearFifoIrqFlagsRsp([u8; 4]);

impl AndClearFifoIrqFlagsRsp {
    /// Create a new response buffer
    pub fn new() -> Self {
        Self::default()
    }

    /// Return Status
    pub fn status(&mut self) -> Status {
        Status::from_slice(&self.0[..2])
    }

    /// RX FIFO flags
    pub fn rx_fifo_flags(&self) -> u8 {
        self.0[2]
    }

    /// TX FIFO flags
    pub fn tx_fifo_flags(&self) -> u8 {
        self.0[3]
    }
}

impl AsMut<[u8]> for AndClearFifoIrqFlagsRsp {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
