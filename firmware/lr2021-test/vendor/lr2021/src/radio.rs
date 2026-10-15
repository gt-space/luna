//! # Radio control and RF management API
//!
//! This module provides APIs for controlling the LR2021 chip's radio functionality, including RF channel 
//! configuration, power amplifier setup, antenna signal measurement, and transmission/reception control.
//! These are the core radio functions needed to configure the RF characteristics and manage wireless
//! communication across all supported protocols.
//!
//! ## Available Methods
//!
//! ### RF Configuration
//! - [`set_rf`](Lr2021::set_rf) - Set RF frequency channel in Hz
//! - [`set_rf_ranging`](Lr2021::set_rf_ranging) - Set the RF channel (in Hz) for ranging operation
//! - [`set_rx_path`](Lr2021::set_rx_path) - Configure RX path (LF/HF) with boost settings
//! - [`set_packet_type`](Lr2021::set_packet_type) - Set packet type (LoRa, FSK, BLE, Z-Wave, etc.)
//!
//! ### Power Amplifier Configuration
//! - [`set_tx_params`](Lr2021::set_tx_params) - Set TX power level and ramp time
//! - [`set_pa_lf`](Lr2021::set_pa_lf) - Configure Low Frequency Power Amplifier (sub-GHz)
//! - [`set_pa_hf`](Lr2021::set_pa_hf) - Configure High Frequency Power Amplifier (2.4GHz)
//! - [`set_pa_lf_ocp_threshold`](Lr2021::set_pa_lf_ocp_threshold) - Change PA LF Over-Current Protection Threshold
//!
//! ### Operation Mode Control
//! - [`set_fallback`](Lr2021::set_fallback) - Set fallback mode after TX/RX completion
//! - [`set_tx`](Lr2021::set_tx) - Enter transmission mode with timeout
//! - [`set_tx_test`](Lr2021::set_tx_test) - Start TX in test mode (infinite preamble, continuous wave or PRBS9)
//! - [`set_rx`](Lr2021::set_rx) - Enter reception mode with timeout and ready wait option
//! - [`set_rx_continous`](Lr2021::set_rx_continous) - Start RX in continuous mode
//! - [`set_rx_duty_cycle`](Lr2021::set_rx_duty_cycle) - Start periodic RX
//! - [`set_auto_rxtx`](Lr2021::set_auto_rxtx) - Configure automatic Transmission/reception after RxDone/TxDone
//!
//! ### Channel Activity Detection (CAD)
//! - [`set_cad_params`](Lr2021::set_cad_params) - Configure CAD parameters (timeout, threshold, exit mode)
//! - [`set_cad`](Lr2021::set_cad) - Start channel activity detection
//!
//! ### Clear Channel Assessment (CCA)
//! - [`set_cca`](Lr2021::set_cca) - Start clear channel assessment for specified duration
//! - [`get_cca_result`](Lr2021::get_cca_result) - Get CCA measurement results
//! - [`set_and_get_cca`](Lr2021::set_and_get_cca) - Run a Clear Channel Assesment for duration (31.25ns) and retrieve the result
//!
//! ### Gain and Signal Control
//! - [`set_rx_gain`](Lr2021::set_rx_gain) - Set manual RX gain (0=auto, max=13)
//! - [`get_rssi_inst`](Lr2021::get_rssi_inst) - Get instantaneous RSSI measurement
//! - [`get_rssi_avg`](Lr2021::get_rssi_avg) - Get average RSSI measurement over specified duration
//!
//! ### Reception Management
//! - [`clear_rx_stats`](Lr2021::clear_rx_stats) - Clear reception statistics
//! - [`get_rx_pkt_len`](Lr2021::get_rx_pkt_len) - Get length of last received packet
//! - [`force_crc_out`](Lr2021::force_crc_out) - Force CRC output to FIFO even when hardware-checked
//!
//! ### Timing
//! - [`set_timestamp_source`](Lr2021::set_timestamp_source) - Set source for a timestamp (up to 3 configurable)
//! - [`get_timestamp`](Lr2021::get_timestamp) - Get Timestamp (as number of HF tick elapsed until NSS)
//! - [`set_default_timeout`](Lr2021::set_default_timeout) - Set default timeout for TX/RX operation
//! - [`set_stop_timeout`](Lr2021::set_stop_timeout) - Set whether the RX timeout stops when preamble is detected or when the synchronization is confirmed
//!


use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;

use crate::{cmd::cmd_regmem::write_reg_mem_mask32_cmd, constants::*};

pub use super::cmd::cmd_common::*;
use super::{BusyPin, Lr2021, Lr2021Error};

#[derive(Clone, Copy)]
pub enum PaLfOcpThr {
    Default = 55, Low900Mhz = 41,
}

impl<O,SPI, M> Lr2021<O,SPI, M> where
    O: OutputPin, SPI: SpiBus<u8>, M: BusyPin
{

    /// Set the RF channel (in Hz)
    pub async fn set_rf(&mut self, freq: u32) -> Result<(), Lr2021Error> {
        let req = set_rf_frequency_cmd(freq);
        self.cmd_wr(&req).await
    }

    /// Set the RF channel (in Hz) for ranging operation
    /// Call only after set_packet_type(Ranging)
    pub async fn set_rf_ranging(&mut self, freq: u32) -> Result<(), Lr2021Error> {
        self.set_rf(freq).await?;
        self.wr_reg_mask(ADDR_FREQ_RF, 0x7F, 0).await
    }

    /// Set the RX Path (LF/HF)
    pub async fn set_rx_path(&mut self, rx_path: RxPath, rx_boost: RxBoost) -> Result<(), Lr2021Error> {
        let req = set_rx_path_adv_cmd(rx_path, rx_boost);
        self.cmd_wr(&req).await
    }

    /// Set the packet type
    pub async fn set_packet_type(&mut self, packet_type: PacketType) -> Result<(), Lr2021Error> {
        let req = set_packet_type_cmd(packet_type);
        self.cmd_wr(&req).await
    }

    /// Set Tx power and ramp time
    /// TX Power in given in half-dB unit. Range is -19..44 for LF Path and -39..24 for HF path
    /// Ramp-time is important to reduce Out-of-band emission. A safe rule of thumb is to set it to around 4/Bandwidth.
    pub async fn set_tx_params(&mut self, tx_power: i8, ramp_time: RampTime) -> Result<(), Lr2021Error> {
        let req = set_tx_params_cmd(tx_power, ramp_time);
        self.cmd_wr(&req).await
    }

    /// Configure LF Power Amplifier
    pub async fn set_pa_lf(&mut self, pa_lf_mode: PaLfMode, pa_lf_duty_cycle: u8, pa_lf_slices: u8) -> Result<(), Lr2021Error> {
        let req = set_pa_config_cmd(PaSel::LfPa, pa_lf_mode, pa_lf_duty_cycle, pa_lf_slices);
        self.cmd_wr(&req).await
    }

    /// Change PA LF Over-Current Protection Threshold
    /// The power amplifier consumption may increase depending on antenna matching.
    /// Some 900MHz band antenna have shown a power consumption increase close to the OCP limitation.
    /// Calling this function allows to increase the OCP limitation in these situations
    /// WARNING: USE THIS FUNCTION CAREFULLY AS AN INCORRECT USAGE MAY RESULT IN DESTRUCTION OF THE CHIP.
    pub async fn set_pa_lf_ocp_threshold(&mut self, thr: PaLfOcpThr) -> Result<(), Lr2021Error> {
        let value = (thr as u32) << 19;
        self.wr_reg(ADDR_PA_LOCK, 0xC0DE).await?;
        self.wr_reg_mask(ADDR_PA_CTRL, 0x1F80000, value).await?;
        self.wr_reg(ADDR_PA_LOCK, 0).await?;
        self.wr_reg_mask(ADDR_OCP_RETENTION, 0xFF, value).await
    }

    /// Configure HF Power Amplifier
    pub async fn set_pa_hf(&mut self) -> Result<(), Lr2021Error> {
        let req = set_pa_config_cmd(PaSel::HfPa, PaLfMode::LfPaFsm, 6, 7);
        self.cmd_wr(&req).await
    }

    /// Set the Fallback mode after TX/RX
    pub async fn set_fallback(&mut self, fallback_mode: FallbackMode) -> Result<(), Lr2021Error> {
        let req = set_rx_tx_fallback_mode_cmd(fallback_mode);
        self.cmd_wr(&req).await
    }

    /// Set chip in TX mode. Set timeout to 0 or to a value longer than the packet duration.
    /// Timeout is given in LF clock step (1/32.768kHz ~ 30.5us)
    pub async fn set_tx(&mut self, tx_timeout: u32) -> Result<(), Lr2021Error> {
        let req = set_tx_adv_cmd(tx_timeout);
        self.cmd_wr(&req).await
    }

    /// Start TX in test mode (infinite preamble, continuous wave or PRBS9)
    pub async fn set_tx_test(&mut self, mode: TestMode) -> Result<(), Lr2021Error> {
        let req = set_tx_test_mode_cmd(mode);
        self.cmd_wr(&req).await
    }

    /// Set chip in RX mode. A timeout equal to 0 means a single reception, the value 0xFFFFFF is for continuous RX (i.e. always restart reception)
    /// and any other value, the chip will go back to its fallback mode if a reception does not occur before the timeout is elapsed
    /// Timeout is given in LF clock step (1/32.768kHz ~ 30.5us)
    pub async fn set_rx(&mut self, rx_timeout: u32, wait_ready: bool) -> Result<(), Lr2021Error> {
        let req = set_rx_adv_cmd(rx_timeout);
        self.cmd_wr(&req).await?;
        if wait_ready {
            self.wait_ready(Duration::from_millis(100)).await?;
        }
        Ok(())
    }

    /// Set RX in continuous mode
    pub async fn set_rx_continous(&mut self) -> Result<(), Lr2021Error> {
        self.set_rx(0xFFFFFF,true).await
    }

    /// Start periodic RX
    /// Radio listens for `rx_max_time`: go to sleep once packet is received or no packet was detect
    /// Repeat operation every `cycle_time` (which must be bigger than rx_max_time)
    /// The `use_lora_cad` is only valid if packet type was set to LoRa and performs a CAD instead of a standard reception.
    /// In this case the exit mode of the CAD is performed, i.e. it can start a TX if configured as Listen-Before-Talk
    pub async fn set_rx_duty_cycle(&mut self, listen_time: u32, cycle_time: u32, use_lora_cad: bool, dram_ret: u8) -> Result<(), Lr2021Error> {
        let req = set_rx_duty_cycle_cmd(listen_time, cycle_time, use_lora_cad, dram_ret);
        self.cmd_wr(&req).await
    }

    /// Configure automatic Transmission/reception after RxDone/TxDone
    /// This mode triggers only once and must re-enabled.
    /// When clear is set, the auto_txrx is cleared even on RX timeout.
    pub async fn set_auto_rxtx(&mut self, clear: bool, mode: AutoTxrxMode, timeout: u32, delay: u32) -> Result<(), Lr2021Error> {
        let req = set_auto_rx_tx_cmd(clear, mode, timeout, delay);
        self.cmd_wr(&req).await
    }

    /// Configure parameters for Channel Activity Detection
    ///  - CAD Timeout is the maximum time spent measuring RSSI in 31.25ns step
    ///  - Threshold in -dBm to determine if a signal is present
    ///  - Exit Mode: controls what happens after CAD (nothing, TX if nothing, RX if something)
    ///  - TRX Timeout is the timeout used in case the exit_mode triggers a TX or RX
    pub async fn set_cad_params(&mut self, cad_timeout: u32, threshold: u8, exit_mode: ExitMode, trx_timeout: u32) -> Result<(), Lr2021Error> {
        let req = set_cad_params_cmd(cad_timeout, threshold, exit_mode, trx_timeout);
        self.cmd_wr(&req).await
    }

    /// Set chip in Channel activity Detection mode
    /// CAD is configured with set_cad_params
    pub async fn set_cad(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&set_cad_cmd()).await
    }

    /// Set chip in CCA (Clear Channel Assesment) for duration (31.25ns)
    /// Note: Chip must be standby or FS before issuing the command
    pub async fn set_cca(&mut self, duration: u32, gain: Option<u8>) -> Result<(), Lr2021Error> {
        let req = set_cca_adv_cmd(duration, gain.unwrap_or(0));
        let len = req.len() - if gain.is_none() {1} else {0};
        self.cmd_wr(&req[..len]).await
    }

    /// Get CCA measurement results
    pub async fn get_cca_result(&mut self) -> Result<CcaResultRsp, Lr2021Error> {
        let req = get_cca_result_req();
        let mut rsp = CcaResultRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp)
    }

    /// Run a Clear Channel Assesment for duration (31.25ns) and retrieve the result
    /// Note: Chip must be standby or FS before issuing the command
    pub async fn set_and_get_cca(&mut self, duration: u32, gain: Option<u8>) -> Result<CcaResultRsp, Lr2021Error> {
        let req = set_cca_adv_cmd(duration, gain.unwrap_or(0));
        let len = req.len() - if gain.is_none() {1} else {0};
        self.cmd_wr(&req[..len]).await?;
        // Approximate duration using 32ns for the LF clock period to avoid multiplication
        let dur_ns = (duration as u64 ) << 5;
        Timer::after_nanos(dur_ns).await;
        self.get_cca_result().await
    }

    /// Configure the radio gain manually:
    ///   - Gain 0 enable the automatic gain selection (default setting)
    ///   - Max gain is 13
    pub async fn set_rx_gain(&mut self, gain: u8) -> Result<(), Lr2021Error> {
        let req = set_agc_gain_manual_cmd(gain.min(13));
        self.cmd_wr(&req).await
    }

    /// Clear RX stats
    pub async fn clear_rx_stats(&mut self) -> Result<(), Lr2021Error> {
        self.cmd_wr(&reset_rx_stats_cmd()).await
    }

    /// Return length of last packet received
    pub async fn get_rx_pkt_len(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_rx_pkt_length_req();
        let mut rsp = RxPktLengthRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.pkt_length())
    }

    /// Output CRC to the FIFO even when already checked by hardware
    pub async fn force_crc_out(&mut self) -> Result<(), Lr2021Error> {
        let req = write_reg_mem_mask32_cmd(0xF30844, 0x01000000, 0);
        self.cmd_wr(&req).await
    }

    /// Measure RSSI instantaneous
    pub async fn get_rssi_inst(&mut self) -> Result<u16, Lr2021Error> {
        let req = get_rssi_inst_req();
        let mut rsp = RssiInstRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.rssi())
    }

    /// Measure an average RSSI (in -0.5dBm)
    /// Average is the result of n instantaneous RSSI measurement
    pub async fn get_rssi_avg(&mut self, nb_meas: u16) -> Result<u16, Lr2021Error> {
        let mut rssi = 0;
        for _ in 0..nb_meas {
            rssi += self.get_rssi_inst().await?;
        }
        Ok((rssi + (nb_meas>>1)) / nb_meas)
    }

    /// Set default timeout for TX/RX operation
    /// Used when started on DIO trigger
    pub async fn set_default_timeout(&mut self, tx: u32, rx: u32) -> Result<(), Lr2021Error> {
        let req = set_default_rx_tx_timeout_cmd(rx, tx);
        self.cmd_wr(&req).await
    }

    /// Set whether the RX timeout stops when preamble is detected or when the synchronization is confirmed
    pub async fn set_stop_timeout(&mut self, on_preamble: bool) -> Result<(), Lr2021Error> {
        let req = set_stop_timeout_cmd(on_preamble);
        self.cmd_wr(&req).await
    }

    /// Set source for a timestamp (up to 3 configurable)
    pub async fn set_timestamp_source(&mut self, index: TimestampIndex, source: TimestampSource) -> Result<(), Lr2021Error> {
        let req = set_timestamp_source_cmd(index, source);
        self.cmd_wr(&req).await
    }

    /// Get Timestamp (as number of HF tick elapsed until NSS)
    pub async fn get_timestamp(&mut self, index: TimestampIndex) -> Result<u32, Lr2021Error> {
        let req = get_timestamp_value_req(index);
        let mut rsp = TimestampValueRsp::new();
        self.cmd_rd(&req, rsp.as_mut()).await?;
        Ok(rsp.timestamp())
    }

}
