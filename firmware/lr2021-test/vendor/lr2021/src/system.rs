//! # System control and chip management API
//!
//! This module provides general APIs to control the LR2021 chip, including calibration, interrupts, 
//! mode/status management, and low-level register access. These are the core system
//! functions required for chip initialization, operation, and debugging across all communication protocols.
//!
//! ## Available Methods
//!
//! ### Status and Information
//! - [`get_status`](Lr2021::get_status) - Read current chip status and interrupt flags
//! - [`get_errors`](Lr2021::get_errors) - Get detailed error information from the chip
//! - [`get_version`](Lr2021::get_version) - Get chip firmware version information
//! - [`get_and_clear_irq`](Lr2021::get_and_clear_irq) - Read interrupt flags and clear them atomically
//! - [`clear_irqs`](Lr2021::clear_irqs) - Clear specific interrupt flags
//!
//! ### Chip Mode and Power Management
//! - [`set_chip_mode`](Lr2021::set_chip_mode) - Set chip operational mode (sleep, standby, FS, TX, RX)
//! - [`set_regulator_mode`](Lr2021::set_regulator_mode) - Choose regulator (LDO or SIMO)
//! - [`patch_simo`](Lr2021::patch_simo) - Update SIMO configuration for optimal performances
//! - [`add_register_to_retention`](Lr2021::add_register_to_retention) - Add a register to the retention list (i.e. the value is restored on wake-up)
//! - [`setup_retention`](Lr2021::setup_retention) - Setup which registers to add to retention
//!
//! ### Calibration
//! - [`calibrate`](Lr2021::calibrate) - Run calibration of different blocks
//! - [`calib_fe`](Lr2021::calib_fe) - Run front-end calibration on specified frequencies
//!
//! ### Clock Management
//! - [`set_lf_clk`](Lr2021::set_lf_clk) - Configure the LF clock
//! - [`set_tcxo`](Lr2021::set_tcxo) - Configure the chip to use a TCXO
//! - [`set_xosc_trim`](Lr2021::set_xosc_trim) - Configure XOsc foot capacitor
//!
//! ### I/O Management
//! - [`set_dio_function`](Lr2021::set_dio_function) - Configure a DIO pin function
//! - [`set_dio_irq`](Lr2021::set_dio_irq) - Configure a DIO pin for interrupt generation
//! - [`set_dio_rf_switch`](Lr2021::set_dio_rf_switch) - Configure a DIO pin to control an RF Switch
//! - [`set_dio_clk_scaling`](Lr2021::set_dio_clk_scaling) - Configure the clock scaling when output on a DIO
//!
//! ### Register and Memory Access
//! - [`rd_reg`](Lr2021::rd_reg) - Read a 32-bit register value
//! - [`wr_reg`](Lr2021::wr_reg) - Write a 32-bit register value
//! - [`wr_reg_mask`](Lr2021::wr_reg_mask) - Write a 32-bit register value with a mask
//! - [`wr_field`](Lr2021::wr_field) - Write to specific bit field in a register
//! - [`rd_mem`](Lr2021::rd_mem) - Read multiple 32-bit words from memory to internal buffer
//!
//! ### Measurements
//! - [`get_temperature`](Lr2021::get_temperature) -  Return temperature in degree Celsius with 5 fractional bits
//! - [`set_ntc_param`](Lr2021::set_ntc_param) -  Configure NTC parameters
//! - [`get_vbat`](Lr2021::get_vbat) -  Return the battery voltage in mV
//! - [`get_random_number`](Lr2021::get_random_number) -  Return a random number using entropy from PLL and ADC

use embassy_time::Duration;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::cmd::cmd_regmem::{read_reg_mem32_req, write_reg_mem32_cmd, write_reg_mem_mask32_cmd, ReadRegMem32Rsp};
use crate::constants::*;

use super::{BusyPin, Lr2021, Lr2021Error};
use super::status::{Intr, Status};

pub use super::cmd::cmd_system::*;
use super::radio::{set_rx_cmd, set_tx_cmd};

/// Chip Mode: Sleep/Standby/Fs/...
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChipMode {
    /// Set chip in sleep mode without retention: will wakeup on NSS
    DeepSleep,
    /// Set chip in sleep mode with retention: will wakeup on NSS.
    DeepRetention,
    /// Set chip in sleep mode without retention with timeout on 32k clock
    Sleep(u32),
    /// Set chip in sleep mode with retention and timeout on 32k clock
    Retention(u32),
    /// Set Chip in Standby using RC clock
    StandbyRc,
    /// Set Chip in Standby using crystal oscillator.
    StandbyXosc,
    /// Set Chip in Frequency Synthesis: allows immediate TX/RX
    Fs,
    /// Set Chip in Transmit mode
    Tx,
    /// Set Chip in Receive mode
    Rx,
}

/// SIMO frequency for low bandwidth in pll step (4.30MHz)
const SIMO_FREQ_LBW : u32 = 4_508_877;
/// SIMO frequency for high bandwidth in pll step (2.80MHz)
const SIMO_FREQ_HBW : u32 = 2_936_013;

/// SIMO Timing control for wide-band case
const SIMO_TIME_WIDE : u32 = 0xBD;
/// SIMO Timing control default value
const SIMO_TIME_DEFAULT : u32 = 0xFF;

pub fn pllstep_to_hz(val_step: u32) -> u32 {
    let val_scaled : u64 = (val_step as u64) * 15625;
    (val_scaled >> 14) as u32
}

#[derive(Default, Clone, Copy)]
/// List of additional registers to keep in retention
pub struct RetentionCfg(u8);
impl RetentionCfg {
    pub const RET_SIMO               : u8 = 1;
    pub const RET_LORA_SX127X_SF6_SW : u8 = 2;
    pub const RET_LORA_SX127X_HOP    : u8 = 4;
    pub const RET_CPFSK_DEMOD        : u8 = 8;

    /// Default configuration with no register
    pub fn new() -> Self {
        Self(0)
    }

    /// Add SIMO register to retention
    pub fn with_simo(self) -> Self {
        Self(self.0 | Self::RET_SIMO)
    }

    /// Add LoRa SX127x SF6/Syncword compatibility mode register to retention
    pub fn with_lora_sx127x_sf6_sw(self) -> Self {
        Self(self.0 | Self::RET_LORA_SX127X_SF6_SW)
    }

    /// Add LoRa SX127x Frequency Hopping compatibility mode register to retention
    pub fn with_lora_sx127x_hopping(self) -> Self {
        Self(self.0 | Self::RET_LORA_SX127X_HOP)
    }

    /// Add BLE Coded register to retention
    pub fn with_ble_coded(self) -> Self {
        Self(self.0 | Self::RET_CPFSK_DEMOD)
    }

    /// Add WISUN FDev tracking register to retention
    pub fn with_wisun_tracking(self) -> Self {
        Self(self.0 | Self::RET_CPFSK_DEMOD)
    }

    /// Flag when configuration enable SIMO register to retention
    pub fn has_simo(self) -> bool {
        (self.0 & Self::RET_SIMO) != 0
    }

    /// Flag when configuration enable LoRa SX127x SF6/Syncword compatibility mode register to retention
    pub fn has_lora_sx127x_sf6_sw(self) -> bool {
        (self.0 & Self::RET_LORA_SX127X_SF6_SW) != 0
    }

    /// Flag when configuration enable LoRa SX127x frequency hopping compatibility mode register to retention
    pub fn has_lora_sx127x_hopping(self) -> bool {
        (self.0 & Self::RET_LORA_SX127X_HOP) != 0
    }

    /// Flag when configuration enable BLE Coded register to retention
    pub fn has_cpfsk_demod(self) -> bool {
        (self.0 & Self::RET_CPFSK_DEMOD) != 0
    }
}


impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{
    /// Read status and interrupt from the chip
    pub async fn get_status(&mut self) -> Result<(Status,Intr), Lr2021Error> {
        let req = get_status_req();
        let mut rsp = StatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok((rsp.status(), rsp.intr()))
    }

    /// Read status and interrupt from the chip
    pub async fn get_errors(&mut self) -> Result<ErrorsRsp, Lr2021Error> {
        let req = get_errors_req();
        let mut rsp = ErrorsRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Read status and interrupt from the chip
    pub async fn get_version(&mut self) -> Result<VersionRsp, Lr2021Error> {
        let req = get_version_req();
        let mut rsp = VersionRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Read interrupt from the chip and clear them all
    pub async fn get_and_clear_irq(&mut self) -> Result<Intr, Lr2021Error> {
        let req = get_and_clear_irq_req();
        let mut rsp = StatusRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.intr())
    }

    /// Set the RF channel (in Hz)
    pub async fn clear_irqs(&mut self, intr: Intr) -> Result<(), Lr2021Error> {
        let req = clear_irq_cmd(intr.value());
        self.cmd_wr(&req).await
    }

    /// Run calibration of different blocks
    /// Work in any chip mode and on exit the chip goes into Standby RC
    /// Eventual calibration error can be read with get_errors
    pub async fn calibrate(&mut self, pa_offset: bool, meas_unit: bool, aaf: bool, pll: bool, hf_rc: bool, lf_rc: bool) -> Result<(), Lr2021Error> {
        let req = calibrate_cmd(pa_offset, meas_unit, aaf, pll, hf_rc, lf_rc);
        self.cmd_wr(&req).await
    }

    /// Run calibration on up to 3 frequencies on 16b (MSB encode RX Path)
    /// If none, use current frequency
    pub async fn calib_fe(&mut self, freqs_4m: &[u16]) -> Result<(), Lr2021Error> {
        let f0 = freqs_4m.first().copied().unwrap_or(0);
        let f1 = freqs_4m.get(1).copied().unwrap_or(0);
        let f2 = freqs_4m.get(2).copied().unwrap_or(0);
        let req = calib_fe_cmd(f0,f1,f2);
        let len = 2 + 2*freqs_4m.len();
        self.cmd_wr(&req[..len]).await
    }

    /// Set Tx power and ramp time
    pub async fn set_chip_mode(&mut self, chip_mode: ChipMode) -> Result<(), Lr2021Error> {
        match chip_mode {
            ChipMode::DeepSleep      => self.cmd_wr(&set_sleep_cmd(false, 0)).await,
            ChipMode::DeepRetention  => self.cmd_wr(&set_sleep_adv_cmd(false, 1, 0)).await,
            ChipMode::Sleep(t)       => self.cmd_wr(&set_sleep_adv_cmd(true, 0, t)).await,
            ChipMode::Retention(t)   => self.cmd_wr(&set_sleep_adv_cmd(true, 1, t)).await,
            ChipMode::StandbyRc      => self.cmd_wr(&set_standby_cmd(StandbyMode::Rc)).await,
            ChipMode::StandbyXosc    => self.cmd_wr(&set_standby_cmd(StandbyMode::Xosc)).await,
            ChipMode::Fs => self.cmd_wr(&set_fs_cmd()).await,
            ChipMode::Tx => self.cmd_wr(&set_tx_cmd()).await,
            ChipMode::Rx => self.cmd_wr(&set_rx_cmd()).await,
        }
    }

    /// Configure regulator (LDO or SIMO)
    /// Shall only be called while in Standby RC
    pub async fn set_regulator_mode(&mut self, simo_en: bool) -> Result<(), Lr2021Error> {
        let mode = if simo_en {SimoUsage::Auto} else {SimoUsage::Off};
        let req = set_reg_mode_cmd(mode);
        self.cmd_wr(&req).await
    }

    /// Add a register to the retention list (i.e. the value is restored on wake-up)
    pub async fn add_register_to_retention(&mut self, slot: u8, addr: u32) -> Result<(), Lr2021Error> {
        let req = set_additional_reg_to_retain_cmd(slot, addr);
        self.cmd_wr(&req).await
    }

    /// Add registers to the retention list using a configuration parameter
    /// Registers are: SIMO, LoRa SX127x SF6/Syncword, LoRa SX127x hopping and BLE/WISUN tracking
    pub async fn setup_retention(&mut self, cfg: RetentionCfg) -> Result<(), Lr2021Error> {
        let mut slot = 0;
        if cfg.has_simo() {
            self.add_register_to_retention(slot, ADDR_SIMO_CFG).await?;
            slot += 1;
        }
        if cfg.has_lora_sx127x_sf6_sw() {
            self.add_register_to_retention(slot, ADDR_LORA_PARAM).await?;
            slot += 1;
        }
        if cfg.has_lora_sx127x_hopping() {
            self.add_register_to_retention(slot, ADDR_LORA_TX_CFG1).await?;
            slot += 1;
        }
        if cfg.has_cpfsk_demod() {
            self.add_register_to_retention(slot, ADDR_CPFSK_DEMOD).await?;
            slot += 1;
            self.add_register_to_retention(slot, ADDR_CPFSK_DETECT).await?;
            // slot += 1;
        }
        Ok(())
    }

    /// Configure End-of-Life
    pub async fn set_eol_config(&mut self, thr: EolTrim, en: bool) -> Result<(), Lr2021Error> {
        let req = set_eol_config_cmd(thr, en);
        self.cmd_wr(&req).await
    }


    /// Update SIMO configuration for optimal performances
    /// Must be called after a the modulation parameters are set when SIMO is enabled (set_regulator_mode(true))
    /// The retention enable allows to define a register slot to save setting in retention
    pub async fn patch_simo(&mut self, ret_en: Option<u8>) -> Result<(), Lr2021Error> {
        let ana_dec = (self.rd_reg(ADDR_ADC_CTRL).await? >> 8) & 3;
        let is_hf = (self.rd_reg(ADDR_AAF_CFG).await? &3) == 1;
        // Set SIMO Timing
        let v = if !is_hf && ana_dec < 3 {SIMO_TIME_WIDE} else {SIMO_TIME_DEFAULT};
        self.wr_reg_mask(ADDR_SIMO_CFG, 0x00FF_0000, v << 16).await?;
        // Apply new frequency configuration if needed
        let new_freq = if ana_dec==1 {SIMO_FREQ_HBW}  else {SIMO_FREQ_LBW};
        let curr_freq = self.rd_reg(ADDR_SIMO_FREQ).await?;
        if curr_freq != new_freq {
            self.wr_reg(ADDR_SIMO_FREQ, new_freq).await?;
            // Need to call set_rf to be sure this is taken into account
            let rf_step = self.rd_reg(ADDR_FREQ_RF).await?;
            let rf_hz = pllstep_to_hz(rf_step);
            self.set_rf(rf_hz).await?;
        }
        if let Some(slot) = ret_en {
            self.add_register_to_retention(slot,ADDR_SIMO_CFG).await?;
        }
        Ok(())
    }

    /// Configure a DIO function (IRQ, RF Switch, Clock, ...)
    /// Note: LF clock can only be output on DIO 7 to 11
    pub async fn set_dio_function(&mut self, dio: DioNum, func: DioFunc, pull_drive: PullDrive) -> Result<(), Lr2021Error> {
        let req = set_dio_function_cmd(dio, func, pull_drive);
        self.cmd_wr(&req).await
    }

    /// Configure a pin as An RF Switch
    /// Each args flags when the IO should be high
    pub async fn set_dio_rf_switch(&mut self, dio_num: DioNum, tx_hf: bool, rx_hf: bool, tx_lf: bool, rx_lf: bool, standby: bool) -> Result<(), Lr2021Error> {
        let req = set_dio_rf_switch_config_cmd(dio_num, tx_hf, rx_hf, tx_lf, rx_lf, standby);
        self.cmd_wr(&req).await
    }

    /// Configure a pin as IRQ and enable interrupts for this pin
    pub async fn set_dio_irq(&mut self, dio: DioNum, intr_en: Intr) -> Result<(), Lr2021Error> {
        let sleep_pull = if dio==DioNum::Dio5 || dio==DioNum::Dio6 {PullDrive::PullAuto} else {PullDrive::PullUp};
        let req = set_dio_function_cmd(dio, DioFunc::Irq, sleep_pull);
        self.cmd_wr(&req).await?;
        let req = set_dio_irq_config_cmd(dio, intr_en.value());
        self.cmd_wr(&req).await
    }

    /// Configure the clock scaling when output on a DIO
    pub async fn set_dio_clk_scaling(&mut self, div_scaling: ClkScaling) -> Result<(), Lr2021Error> {
        let req = config_clk_outputs_cmd(div_scaling);
        self.cmd_wr(&req).await
    }

    /// Configure the LF clock
    pub async fn set_lf_clk(&mut self, sel: LfClock) -> Result<(), Lr2021Error> {
        let req = config_lf_clock_cmd(sel);
        self.cmd_wr(&req).await
    }

    /// Configure the chip to use a TCXO
    pub async fn set_tcxo(&mut self, volt: TcxoVoltage, start_time: u32) -> Result<(), Lr2021Error> {
        let req = set_tcxo_mode_cmd(volt, start_time);
        self.cmd_wr(&req).await
    }

    /// Configure XOsc foot capacitor
    /// XT A/B configure the foot capacitor for each pin with value ranging from 0 to 47
    /// 1 LSB is 0.47pF and min value starts at 11.3pF and 10.1pF for XTA and XTB respectively
    /// The optional delay allows to wait longer for the crystal to stabilize when it starts
    pub async fn set_xosc_trim(&mut self, xta: u8, xtb: u8, delay_us: Option<u8>) -> Result<(), Lr2021Error> {
        let req = set_xosc_cp_trim_adv_cmd(xta, xtb, delay_us.unwrap_or(0));
        let len = req.len() - if delay_us.is_some() {1} else {0};
        self.cmd_wr(&req[..len]).await
    }

    /// Return temperature in °C with 5 fractional bits
    /// When the selected source is an NTC, its parameter must be configure with [`set_ntc_param`](Lr2021::set_ntc_param)
    /// The resolution directly controls how long the measure take: from 8us (8b) to 256us (13b)
    pub async fn get_temperature(&mut self, src: TempSrc, res: AdcRes) -> Result<i16, Lr2021Error> {
        let req = get_temp_req(src, res);
        let mut rsp = TempRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.temp_celsius())
    }

    /// Configure NTC parameters
    /// `r_ratio` is the resistance bias ratio of the NTC at 25°C given with 9 fractional bits
    /// The beta coefficient is given in unit of 2 Kelvin
    /// The delay corresponds to a first order time delay coefficient used in the temperature compensation feature:
    ///  this depends on the PCB and how far the NTC is from the crystal
    pub async fn set_ntc_param(&mut self, r_ratio: u16, beta: u16, delay: u8) -> Result<(), Lr2021Error> {
        let req = set_ntc_params_cmd(r_ratio, beta, delay);
        self.cmd_wr(&req).await
    }

    /// Configure Temperature compensation
    /// Command will fail if a TCXO is configured
    /// External NTC is important when the board does not have sufficient thermal break
    /// and/or when transmission is particularly long (more than 2s)
    pub async fn set_temp_comp(&mut self, mode: CompMode, ntc: bool) -> Result<(), Lr2021Error> {
        let req = set_temp_comp_cfg_cmd(ntc, mode);
        self.cmd_wr(&req).await
    }

    /// Return the battery voltage in mV
    /// A resolution of 13b corresponds to roughly 0.82mV
    /// The resolution directly controls how long the measure take: from 8us (8b) to 256us (13b)
    pub async fn get_vbat(&mut self, res: AdcRes) -> Result<u16, Lr2021Error> {
        let req = get_v_bat_req(VbatFormat::Millivolts, res);
        let mut rsp = VBatRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.vbat_mv())
    }

    /// Return a random number using entropy from PLL and ADC
    pub async fn get_random_number(&mut self) -> Result<u32, Lr2021Error> {
        let req = get_random_number_req();
        let mut rsp = RandomNumberRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.random_number())
    }

    /// Read a register value
    pub async fn rd_reg(&mut self, addr: u32) -> Result<u32, Lr2021Error> {
        let req = read_reg_mem32_req(addr, 1);
        let mut rsp = ReadRegMem32Rsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.value())
    }

    /// Read nb32 qword (max 40) from memory and save them inside local buffer
    pub async fn rd_mem(&mut self, addr: u32, nb32: u8) -> Result<(), Lr2021Error> {
        if nb32 > 40 {
            return Err(Lr2021Error::CmdErr);
        }
        let req = read_reg_mem32_req(addr, nb32);
        self.cmd_wr(&req).await?;
        self.wait_ready(Duration::from_millis(1)).await?;
        self.nss.set_low().map_err(|_| Lr2021Error::Pin)?;
        self.buffer.nop();
        let rsp_buf = &mut self.buffer.0[..4*nb32 as usize];
        self.spi
            .transfer_in_place(rsp_buf).await
            .map_err(|_| Lr2021Error::Spi)?;
        self.nss.set_high().map_err(|_| Lr2021Error::Pin)?;
        self.buffer.cmd_status().check()
    }

    /// Write a register value
    pub async fn wr_reg(&mut self, addr: u32, value: u32) -> Result<(), Lr2021Error> {
        let req = write_reg_mem32_cmd(addr, value);
        self.cmd_wr(&req).await
    }

    /// Write a register value with a mask (only bit where mask is high are changed)
    pub async fn wr_reg_mask(&mut self, addr: u32, mask: u32, value: u32) -> Result<(), Lr2021Error> {
        let req = write_reg_mem_mask32_cmd(addr, mask, value);
        self.cmd_wr(&req).await
    }

    /// Write a field value
    pub async fn wr_field(&mut self, addr: u32, value: u32, pos: u8, width: u8) -> Result<(), Lr2021Error> {
        let mask =
            if width >= 32 {0xFFFFFFFF}
            else { ((1 << width) - 1) << pos };
        let req = write_reg_mem_mask32_cmd(addr, mask, value << pos);
        self.cmd_wr(&req).await
    }

}
