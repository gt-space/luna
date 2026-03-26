//! RECO (Recovery) Board Driver
//!
//! This driver provides SPI communication with the RECO board for recovery system control.
//! The RECO board handles recovery mechanisms and communicates with the flight computer
//! via SPI using a custom protocol.
//!
//! This driver is designed for Raspberry Pi using Linux spidev for SPI communication.
//! Hardware chip select (CE0/CE1) is automatically controlled by the kernel driver.

use std::fmt;
use std::fs::{File, OpenOptions};
use std::os::unix::io::{AsRawFd, RawFd};

pub use common::comm::reco::{
    AltimeterOffsets, EkfStateVector, InitialCovarianceMatrix, MeasurementNoiseMatrix,
    ProcessNoiseMatrix, TimerValues,
};

// SPI ioctl definitions (from Linux spidev.h)
const SPI_IOC_WR_MODE: u32 = 0x40016b01;
const SPI_IOC_WR_MAX_SPEED_HZ: u32 = 0x40046b04;
const SPI_IOC_MESSAGE_1: u32 = 0x40206b00;

#[repr(C)]
struct SpiIocTransfer {
    tx_buf: u64,
    rx_buf: u64,
    len: u32,
    speed_hz: u32,
    delay_usecs: u16,
    bits_per_word: u8,
    cs_change: u8,
    tx_nbits: u8,
    rx_nbits: u8,
    pad: u16,
}

/// Default SPI settings for RECO board
const DEFAULT_SPI_MODE: u8 = 0; // Mode 0 (CPOL=0, CPHA=0)
const DEFAULT_SPI_SPEED: u32 = 2_000_000; // 2 MHz
/// Message sizes
const RECO_BODY_SIZE: usize = 180;
const TOTAL_TRANSFER_SIZE: usize = RECO_BODY_SIZE;

/// Opcodes for messages to RECO
pub mod opcode {
    /// Opcode that tells RECO that the rocket has launched
    pub const LAUNCHED: u8 = 0x79;
    /// Opcode that sends RECO most recent GPS data and receives RECO telemetry
    pub const GPS_DATA: u8 = 0xF2;
    /// Opcode requesting that RECO initialize (or reinitialize) its EKF.
    pub const INIT_EKF: u8 = 0xCA; 
    /// Opcode that sends the EKF process-noise matrix to RECO.
    pub const PROCESS_NOISE_MATRIX: u8 = 0x51;
    /// Opcode that sends the EKF measurement-noise matrix to RECO.
    pub const MEASUREMENT_NOISE_MATRIX: u8 = 0x52;
    /// Opcode that sends the EKF state vector to RECO.
    pub const EKF_STATE_VECTOR: u8 = 0x78;
    /// Opcode that sends the initial covariance matrix to RECO.
    pub const INITIAL_COVARIANCE_MATRIX: u8 = 0x50;
    /// Opcode that sends timer values to RECO.
    pub const TIMER_VALUES: u8 = 0x54;
    /// Opcode that sends altimeter offsets to RECO.
    pub const ALTIMETER_OFFSETS: u8 = 0x42;
}

/// RECO driver structure
pub struct RecoDriver {
    spi_fd: RawFd,
    _spi_file: File, // Keep file open to maintain valid file descriptor
}

/// GPS data structure for opcode 0x02
#[derive(Debug, Clone, Copy)]
pub struct FcGpsBody {
    pub velocity_north: f32,
    pub velocity_east: f32,
    pub velocity_down: f32,
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
    pub valid: bool,
}

/// Data structure received from RECO
#[derive(Debug, Clone, Copy)]
pub struct RecoBody {
    /// Attitude of vehicle (quaternion)
    pub quaternion: [f32; 4],           
    /// Position [longitude, latitude, altitude]
    pub lla_pos: [f32; 3],              
    /// Velocity of vehicle
    pub velocity: [f32; 3],             
    /// Gyroscope bias offset
    pub g_bias: [f32; 3],               
    /// Accelerometer bias offset
    pub a_bias: [f32; 3],               
    /// Gyro scale factor
    pub g_sf: [f32; 3],                 
    /// Acceleration scale factor
    pub a_sf: [f32; 3],                 
    /// XYZ linear acceleration
    pub lin_accel: [f32; 3],            
    /// Angular rates (pitch, yaw, roll)
    pub angular_rate: [f32; 3],         
    /// XYZ magnetometer data
    pub mag_data: [f32; 3],             
    /// Temperature from barometer
    pub temperature: f32,               
    /// Pressure from barometer
    pub pressure: f32,                  
    /// Channel 1 Driver 1 Voltage (VREF-FB1-A)
    pub vref_ch1_dr1: f32,              
    /// Channel 1 Driver 2 Voltage (VREF-FB1-B)
    pub vref_ch1_dr2: f32,              
    /// Channel 2 Driver 1 Voltage (VREF-FB2-A)
    pub vref_ch2_dr1: f32,              
    /// Channel 2 Driver 2 Voltage (VREF-FB2-B)
    pub vref_ch2_dr2: f32,              
    /// Recovery Driver 1 current
    pub sns1_current: f32,              
    /// Recovery Driver 2 current
    pub sns2_current: f32,              
    /// 24 V Rail Voltage
    pub v_rail_24v: f32,                
    /// 3.3 V Rail Voltage
    pub v_rail_3v3: f32,                
    /// Pulled high when STM32 says to deploy drogue
    pub stage1_enabled: bool,           
    /// Pulled high when STM32 says to deploy main
    pub stage2_enabled: bool,      
    /// Pulled high by RECO when it has received the launch command
    pub reco_recvd_launch: bool,        
    /// Tells which of the 10 channels has fault
    pub reco_driver_faults: [u8; 10],   
    /// Whether EKF has blown up or not
    pub ekf_blown_up: bool,             
    /// When true, timer will be used over EKF for drogue
    pub drouge_timer_enable: bool,      
    /// When true, timer will be used over altimeter for main
    pub main_timer_enable: bool, 
}

/// Error types for RECO operations
#[derive(Debug)]
pub enum RecoError {
    Protocol(String),
    InvalidMessageSize(usize),
    Deserialization(String),
}

impl fmt::Display for RecoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RecoError::Protocol(msg) => write!(f, "Protocol error: {}", msg),
            RecoError::InvalidMessageSize(size) => write!(f, "Invalid message size: {} bytes", size),
            RecoError::Deserialization(msg) => write!(f, "Deserialization error: {}", msg),
        }
    }
}

impl std::error::Error for RecoError {}

trait Encode {
    fn encoded_len(&self) -> usize;
    fn encode_into(&self, writer: &mut MessageEncoder<'_>) -> Result<(), RecoError>;
}

trait Decode: Sized {
    fn decode_from(reader: &mut MessageReader<'_>) -> Result<Self, RecoError>;
}

struct MessageEncoder<'a> {
    buf: &'a mut [u8],
    offset: usize,
}

impl<'a> MessageEncoder<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, offset: 0 }
    }

    fn finish(&self) -> Result<(), RecoError> {
        if self.offset == self.buf.len() {
            Ok(())
        } else {
            Err(RecoError::Protocol(format!(
                "Encoded {} bytes but expected {}",
                self.offset,
                self.buf.len()
            )))
        }
    }

    fn write_f32(&mut self, value: f32) -> Result<(), RecoError> {
        self.write_bytes(&value.to_le_bytes())
    }

    fn write_bool(&mut self, value: bool) -> Result<(), RecoError> {
        self.write_u8(if value { 1 } else { 0 })
    }

    fn write_u8(&mut self, value: u8) -> Result<(), RecoError> {
        let slot = self.take_mut(1)?;
        slot[0] = value;
        Ok(())
    }

    fn write_f32_slice(&mut self, values: &[f32]) -> Result<(), RecoError> {
        for value in values {
            self.write_f32(*value)?;
        }
        Ok(())
    }

    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), RecoError> {
        let slot = self.take_mut(bytes.len())?;
        slot.copy_from_slice(bytes);
        Ok(())
    }

    fn take_mut(&mut self, len: usize) -> Result<&mut [u8], RecoError> {
        let end = self
            .offset
            .checked_add(len)
            .ok_or_else(|| RecoError::Protocol("MessageEncoder offset overflow".to_string()))?;
        if end > self.buf.len() {
            return Err(RecoError::Protocol(format!(
                "MessageEncoder overflow: need {} bytes, have {} remaining",
                len,
                self.buf.len().saturating_sub(self.offset)
            )));
        }

        let slot = &mut self.buf[self.offset..end];
        self.offset = end;
        Ok(slot)
    }
}

struct MessageReader<'a> {
    buf: &'a [u8],
    offset: usize,
}

impl<'a> MessageReader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self { buf, offset: 0 }
    }

    fn finish(&self) -> Result<(), RecoError> {
        if self.offset == self.buf.len() {
            Ok(())
        } else {
            Err(RecoError::Deserialization(format!(
                "Decoded {} bytes but expected {}",
                self.offset,
                self.buf.len()
            )))
        }
    }

    fn read_f32(&mut self) -> Result<f32, RecoError> {
        let bytes = self.read_exact::<4>()?;
        Ok(f32::from_le_bytes(bytes))
    }

    fn read_bool(&mut self) -> Result<bool, RecoError> {
        Ok(self.read_u8()? != 0)
    }

    fn read_u8(&mut self) -> Result<u8, RecoError> {
        Ok(self.take(1)?[0])
    }

    fn read_f32_array<const N: usize>(&mut self) -> Result<[f32; N], RecoError> {
        let mut values = [0.0; N];
        for value in &mut values {
            *value = self.read_f32()?;
        }
        Ok(values)
    }

    fn read_exact<const N: usize>(&mut self) -> Result<[u8; N], RecoError> {
        let bytes = self.take(N)?;
        let mut out = [0u8; N];
        out.copy_from_slice(bytes);
        Ok(out)
    }

    fn take(&mut self, len: usize) -> Result<&[u8], RecoError> {
        let end = self
            .offset
            .checked_add(len)
            .ok_or_else(|| RecoError::Deserialization("MessageReader offset overflow".to_string()))?;
        if end > self.buf.len() {
            return Err(RecoError::Deserialization(format!(
                "MessageReader underflow: need {} bytes, have {} remaining",
                len,
                self.buf.len().saturating_sub(self.offset)
            )));
        }

        let bytes = &self.buf[self.offset..end];
        self.offset = end;
        Ok(bytes)
    }
}

impl Encode for FcGpsBody {
    fn encoded_len(&self) -> usize {
        6 * 4 + 1
    }

    fn encode_into(&self, writer: &mut MessageEncoder<'_>) -> Result<(), RecoError> {
        writer.write_f32(self.velocity_north)?;
        writer.write_f32(self.velocity_east)?;
        writer.write_f32(self.velocity_down)?;
        writer.write_f32(self.latitude)?;
        writer.write_f32(self.longitude)?;
        writer.write_f32(self.altitude)?;
        writer.write_bool(self.valid)?;
        Ok(())
    }
}

impl Encode for ProcessNoiseMatrix {
    fn encoded_len(&self) -> usize {
        36 * 4
    }

    fn encode_into(&self, writer: &mut MessageEncoder<'_>) -> Result<(), RecoError> {
        writer.write_f32_slice(&self.nu_gv_mat)?;
        writer.write_f32_slice(&self.nu_gu_mat)?;
        writer.write_f32_slice(&self.nu_av_mat)?;
        writer.write_f32_slice(&self.nu_au_mat)?;
        Ok(())
    }
}

impl Encode for MeasurementNoiseMatrix {
    fn encoded_len(&self) -> usize {
        10 * 4
    }

    fn encode_into(&self, writer: &mut MessageEncoder<'_>) -> Result<(), RecoError> {
        writer.write_f32_slice(&self.gps_noise_matrix)?;
        writer.write_f32(self.barometer_noise)?;
        Ok(())
    }
}

impl Encode for EkfStateVector {
    fn encoded_len(&self) -> usize {
        22 * 4
    }

    fn encode_into(&self, writer: &mut MessageEncoder<'_>) -> Result<(), RecoError> {
        writer.write_f32_slice(&self.quaternion)?;
        writer.write_f32_slice(&self.lla_pos)?;
        writer.write_f32_slice(&self.velocity)?;
        writer.write_f32_slice(&self.g_bias)?;
        writer.write_f32_slice(&self.a_bias)?;
        writer.write_f32_slice(&self.g_sf)?;
        writer.write_f32_slice(&self.a_sf)?;
        Ok(())
    }
}

impl Encode for InitialCovarianceMatrix {
    fn encoded_len(&self) -> usize {
        21 * 4
    }

    fn encode_into(&self, writer: &mut MessageEncoder<'_>) -> Result<(), RecoError> {
        writer.write_f32_slice(&self.att_unc0)?;
        writer.write_f32_slice(&self.pos_unc0)?;
        writer.write_f32_slice(&self.vel_unc0)?;
        writer.write_f32_slice(&self.gbias_unc0)?;
        writer.write_f32_slice(&self.abias_unc0)?;
        writer.write_f32_slice(&self.gsf_unc0)?;
        writer.write_f32_slice(&self.asf_unc0)?;
        Ok(())
    }
}

impl Encode for TimerValues {
    fn encoded_len(&self) -> usize {
        2 * 4 + 2
    }

    fn encode_into(&self, writer: &mut MessageEncoder<'_>) -> Result<(), RecoError> {
        writer.write_f32(self.drouge_timer)?;
        writer.write_f32(self.main_timer)?;
        writer.write_u8(self.drouge_timer_enable)?;
        writer.write_u8(self.main_timer_enable)?;
        Ok(())
    }
}

impl Encode for AltimeterOffsets {
    fn encoded_len(&self) -> usize {
        4 * 4
    }

    fn encode_into(&self, writer: &mut MessageEncoder<'_>) -> Result<(), RecoError> {
        writer.write_f32(self.flight_baro_fmf_parameter)?;
        writer.write_f32(self.ground_baro_fmf_parameter)?;
        writer.write_f32(self.flight_gps_fmf_parameter)?;
        writer.write_f32(self.ground_gps_fmf_parameter)?;
        Ok(())
    }
}

impl Decode for RecoBody {
    fn decode_from(reader: &mut MessageReader<'_>) -> Result<Self, RecoError> {
        Ok(Self {
            quaternion: reader.read_f32_array::<4>()?,
            lla_pos: reader.read_f32_array::<3>()?,
            velocity: reader.read_f32_array::<3>()?,
            g_bias: reader.read_f32_array::<3>()?,
            a_bias: reader.read_f32_array::<3>()?,
            g_sf: reader.read_f32_array::<3>()?,
            a_sf: reader.read_f32_array::<3>()?,
            lin_accel: reader.read_f32_array::<3>()?,
            angular_rate: reader.read_f32_array::<3>()?,
            mag_data: reader.read_f32_array::<3>()?,
            temperature: reader.read_f32()?,
            pressure: reader.read_f32()?,
            vref_ch1_dr1: reader.read_f32()?,
            vref_ch1_dr2: reader.read_f32()?,
            vref_ch2_dr1: reader.read_f32()?,
            vref_ch2_dr2: reader.read_f32()?,
            sns1_current: reader.read_f32()?,
            sns2_current: reader.read_f32()?,
            v_rail_24v: reader.read_f32()?,
            v_rail_3v3: reader.read_f32()?,
            stage1_enabled: reader.read_bool()?,
            stage2_enabled: reader.read_bool()?,
            reco_recvd_launch: reader.read_bool()?,
            reco_driver_faults: reader.read_exact::<10>()?,
            ekf_blown_up: reader.read_bool()?,
            drouge_timer_enable: reader.read_bool()?,
            main_timer_enable: reader.read_bool()?,
        })
    }
}

impl RecoDriver {
    /// Creates a new RECO driver instance
    /// 
    /// # Arguments
    /// 
    /// * `device_path` - SPI device path (e.g., "/dev/spidev1.1")
    /// 
    /// # Example
    /// 
    /// ```no_run
    /// use reco::RecoDriver;
    /// 
    /// // Using hardware CS (CE1 on SPI1)
    /// let reco = RecoDriver::new("/dev/spidev1.1")?;
    /// # Ok::<(), reco::RecoError>(())
    /// ```
    pub fn new(device_path: &str) -> Result<Self, RecoError> {
        // Open SPI device
        let spi_file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(device_path)
            .map_err(|e| RecoError::Protocol(format!("Failed to open {}: {}", device_path, e)))?;
        
        let spi_fd = spi_file.as_raw_fd();

        // Configure SPI mode
        let mode: u8 = DEFAULT_SPI_MODE;
        unsafe {
            let result = libc::ioctl(spi_fd, SPI_IOC_WR_MODE as libc::c_ulong, &mode as *const u8);
            if result < 0 {
                return Err(RecoError::Protocol(format!(
                    "Failed to set SPI mode: {}",
                    std::io::Error::last_os_error()
                )));
            }
        }

        // Configure SPI speed
        let speed: u32 = DEFAULT_SPI_SPEED;
        unsafe {
            let result = libc::ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ as libc::c_ulong, &speed as *const u32);
            if result < 0 {
                return Err(RecoError::Protocol(format!(
                    "Failed to set SPI speed: {}",
                    std::io::Error::last_os_error()
                )));
            }
        }
        
        Ok(RecoDriver {
            spi_fd,
            _spi_file: spi_file,
        })
    }

    /// Perform SPI transfer (read and write simultaneously)
    /// 
    /// Uses Linux spidev ioctl for full-duplex SPI transfer.
    /// Both buffers must be mutable and the same size.
    /// 
    /// If a manual CS pin is provided, it will be controlled manually.
    /// If no CS pin is provided (None), the hardware CS line will be controlled
    /// automatically by the SPI driver during the transfer.
    fn spi_transfer(&mut self, tx_buf: &mut [u8], rx_buf: &mut [u8]) -> Result<(), RecoError> {
        if tx_buf.len() != rx_buf.len() {
            return Err(RecoError::Protocol(
                "TX and RX buffers must be the same size".to_string(),
            ));
        }
        
        // Verify we have data to transfer
        if tx_buf.is_empty() {
            return Err(RecoError::Protocol(
                "TX buffer is empty - no data to transfer".to_string(),
            ));
        }
        
        // Prepare SPI transfer structure
        let transfer = SpiIocTransfer {
            tx_buf: tx_buf.as_ptr() as u64,
            rx_buf: rx_buf.as_mut_ptr() as u64,
            len: tx_buf.len() as u32,
            speed_hz: DEFAULT_SPI_SPEED,
            delay_usecs: 0,
            bits_per_word: 8,
            cs_change: 0,
            tx_nbits: 0,
            rx_nbits: 0,
            pad: 0,
        };
        
        // Perform SPI transfer using ioctl
        // Hardware CS (CE0/CE1) is automatically controlled by spidev
        let result = unsafe {
            libc::ioctl(self.spi_fd, SPI_IOC_MESSAGE_1 as libc::c_ulong, &transfer as *const SpiIocTransfer)
        };
        
        // Check transfer result
        if result < 0 {
            return Err(RecoError::Protocol(format!(
                "SPI transfer failed: {}",
                std::io::Error::last_os_error()
            )));
        }
        
        Ok(())
    }

    fn send_payload(&mut self, opcode: u8, payload: Option<&dyn Encode>) -> Result<(), RecoError> {
        let payload_len = payload.map_or(0, |payload| payload.encoded_len());
        let payload_end = 1 + payload_len;
        if payload_end > TOTAL_TRANSFER_SIZE {
            return Err(RecoError::Protocol(format!(
                "Message size {} exceeds transfer size {}",
                payload_end, TOTAL_TRANSFER_SIZE
            )));
        }

        let mut tx_buf = [0u8; TOTAL_TRANSFER_SIZE];
        let mut rx_buf = [0u8; TOTAL_TRANSFER_SIZE];
        tx_buf[0] = opcode;

        if let Some(payload) = payload {
            let mut writer = MessageEncoder::new(&mut tx_buf[1..payload_end]);
            payload.encode_into(&mut writer)?;
            writer.finish()?;
        }

        if std::env::var("RECO_DEBUG").is_ok() {
            eprintln!("DEBUG: Sending opcode 0x{:02X}", opcode);
            eprintln!(
                "DEBUG: TX buffer (first {} bytes): {:02X?}",
                payload_end,
                &tx_buf[..payload_end]
            );
        }

        self.spi_transfer(&mut tx_buf, &mut rx_buf)
    }

    fn exchange_payload<R: Decode>(
        &mut self,
        opcode: u8,
        payload: Option<&dyn Encode>,
    ) -> Result<R, RecoError> {
        let payload_len = payload.map_or(0, |payload| payload.encoded_len());
        let payload_end = 1 + payload_len;
        if payload_end > TOTAL_TRANSFER_SIZE {
            return Err(RecoError::Protocol(format!(
                "Message size {} exceeds transfer size {}",
                payload_end, TOTAL_TRANSFER_SIZE
            )));
        }

        let mut tx_buf = [0u8; TOTAL_TRANSFER_SIZE];
        let mut rx_buf = [0u8; TOTAL_TRANSFER_SIZE];
        tx_buf[0] = opcode;

        if let Some(payload) = payload {
            let mut writer = MessageEncoder::new(&mut tx_buf[1..payload_end]);
            payload.encode_into(&mut writer)?;
            writer.finish()?;
        }

        if std::env::var("RECO_DEBUG").is_ok() {
            eprintln!("DEBUG: Sending opcode 0x{:02X}", opcode);
            eprintln!(
                "DEBUG: TX buffer (first {} bytes): {:02X?}",
                payload_end,
                &tx_buf[..payload_end]
            );
        }

        self.spi_transfer(&mut tx_buf, &mut rx_buf)?;
        Self::parse_reco_response(&rx_buf)
    }

    /// Send "launched" message (opcode 0x01) to RECO
    /// 
    /// This message indicates that the rocket has been launched.
    /// The body is all zeros (padding).
    /// 
    /// The full-duplex transfer reads RECO telemetry concurrently, which is discarded.
    pub fn send_launched(&mut self) -> Result<(), RecoError> {
        self.send_payload(opcode::LAUNCHED, None)
    }

    /// Send GPS data to RECO and receive RECO telemetry in a single full-duplex transfer.
    /// 
    /// # Arguments
    /// 
    /// * `gps_data` - GPS data structure containing velocity, position, and validity
    pub fn send_gps_data_and_receive_reco(&mut self, gps_data: &FcGpsBody) -> Result<RecoBody, RecoError> {
        self.exchange_payload(opcode::GPS_DATA, Some(gps_data))
    }

    /// Send EKF-initialization message (repurposed opcode 0x03) to RECO.
    ///
    /// The body is all zeros (padding); only the opcode is used by RECO to
    /// trigger EKF initialization.
    pub fn send_init_ekf(&mut self) -> Result<(), RecoError> {
        self.send_payload(opcode::INIT_EKF, None)
    }

    /// Send EKF process-noise matrix to RECO.
    /// Acts as a special transaction where we don't care about received bytes.
    pub fn send_process_noise_matrix(
        &mut self,
        q: &ProcessNoiseMatrix,
    ) -> Result<(), RecoError> {
        self.send_payload(opcode::PROCESS_NOISE_MATRIX, Some(q))
    }

    /// Send EKF measurement-noise matrix to RECO.
    /// Acts as a special transaction where we don't care about received bytes.
    pub fn send_measurement_noise_matrix(&mut self, m: &MeasurementNoiseMatrix) 
        -> Result<(), RecoError> {
        self.send_payload(opcode::MEASUREMENT_NOISE_MATRIX, Some(m))
    }

    /// Send timer values to RECO (opcode 0x54).
    /// Acts as a special transaction where we don't care about received bytes.
    pub fn send_timer_values(&mut self, t: &TimerValues) -> Result<(), RecoError> {
        self.send_payload(opcode::TIMER_VALUES, Some(t))
    }

    /// Send altimeter offsets to RECO
    /// Acts as a special transaction where we don't care about received bytes.
    pub fn send_altimeter_offsets(&mut self, o: &AltimeterOffsets) -> Result<(), RecoError> {
        self.send_payload(opcode::ALTIMETER_OFFSETS, Some(o))
    }

    /// Send EKF state vector to RECO .
    /// Acts as a special transaction where we don't care about received bytes.
    pub fn send_ekf_state_vector(&mut self, s: &EkfStateVector) -> Result<(), RecoError> {
        self.send_payload(opcode::EKF_STATE_VECTOR, Some(s))
    }

    /// Send initial covariance (P) matrix to RECO.
    /// Acts as a special transaction where we don't care about received bytes.
    pub fn send_initial_covariance_matrix(&mut self, p: &InitialCovarianceMatrix) 
        -> Result<(), RecoError> {
        self.send_payload(opcode::INITIAL_COVARIANCE_MATRIX, Some(p))
    }

    /// Receive data from RECO
    /// 
    /// This method sends a dummy message and receives the RECO body response.
    /// The response consists of the RecoBody structure.
    /// 
    /// # Returns
    /// 
    /// The received RecoBody structure if successful
    pub fn receive_data(&mut self) -> Result<RecoBody, RecoError> {
        // Send dummy bytes to initiate transfer (SPI requires simultaneous tx/rx)
        let mut tx_buf = [0u8; TOTAL_TRANSFER_SIZE];
        let mut rx_buf = [0u8; TOTAL_TRANSFER_SIZE];
        
        // Debug mode: Print TX buffer if enabled
        if std::env::var("RECO_DEBUG").is_ok() {
            eprintln!("DEBUG: Sending receive_data request (all zeros)");
        }
        
        self.spi_transfer(&mut tx_buf, &mut rx_buf)?;
        Self::parse_reco_response(&rx_buf)
    }

    fn parse_reco_response<R: Decode>(rx_buf: &[u8]) -> Result<R, RecoError> {
        // Verify message size
        if rx_buf.len() < TOTAL_TRANSFER_SIZE {
            return Err(RecoError::InvalidMessageSize(rx_buf.len()));
        }
        
        // Extract body
        let body_bytes = &rx_buf[0..RECO_BODY_SIZE];
        
        // Debug mode: Print raw bytes if RECO_DEBUG environment variable is set
        if std::env::var("RECO_DEBUG").is_ok() {
            eprintln!("DEBUG: Raw RX buffer ({} bytes):", rx_buf.len());
            eprintln!("DEBUG: Full buffer: {:02X?}", rx_buf);
            eprintln!("DEBUG: Body (first 64 bytes): {:02X?}", &body_bytes[0..body_bytes.len().min(64)]);
        }
        
        let mut reader = MessageReader::new(body_bytes);
        let reco_body = R::decode_from(&mut reader)?;
        reader.finish()?;
        Ok(reco_body)
    }

    /// Get the SPI file descriptor (for advanced use)
    /// 
    /// Returns the raw file descriptor for the SPI device.
    /// This can be used for low-level operations if needed.
    pub fn spi_fd(&self) -> RawFd {
        self.spi_fd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_f32_serialization() {
        let value = std::f32::consts::PI;
        let mut buf = [0u8; 4];
        let mut writer = MessageEncoder::new(&mut buf);
        writer.write_f32(value).unwrap();
        writer.finish().unwrap();

        let mut reader = MessageReader::new(&buf);
        let restored = reader.read_f32().unwrap();
        reader.finish().unwrap();

        assert!((value - restored).abs() < 0.0001);
    }

    #[test]
    fn test_bool_serialization() {
        let mut buf = [0u8; 3];
        let mut writer = MessageEncoder::new(&mut buf);
        writer.write_bool(true).unwrap();
        writer.write_bool(false).unwrap();
        writer.write_u8(42).unwrap();
        writer.finish().unwrap();

        let mut reader = MessageReader::new(&buf);
        assert!(reader.read_bool().unwrap());
        assert!(!reader.read_bool().unwrap());
        assert_eq!(reader.read_u8().unwrap(), 42);
        reader.finish().unwrap();
    }

    #[test]
    fn test_message_format() {
        // Test that launched message has correct format
        let gps = FcGpsBody {
            velocity_north: 0.0,
            velocity_east: 0.0,
            velocity_down: 0.0,
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            valid: false,
        };
        let mut message = [0u8; 26];
        message[0] = opcode::LAUNCHED;
        // Body (bytes 1-25) are zeros
        
        // Verify message size
        assert_eq!(message.len(), 1 + gps.encoded_len());
        assert_eq!(1 + gps.encoded_len(), 26);
    }

    #[test]
    fn test_fcgpsbody_encoding_matches_wire_format() {
        let gps = FcGpsBody {
            velocity_north: 1.0,
            velocity_east: 2.0,
            velocity_down: 3.0,
            latitude: 4.0,
            longitude: 5.0,
            altitude: 6.0,
            valid: true,
        };

        let mut buf = [0u8; 25];
        let mut writer = MessageEncoder::new(&mut buf);
        gps.encode_into(&mut writer).unwrap();
        writer.finish().unwrap();

        assert_eq!(&buf[0..4], &1.0f32.to_le_bytes());
        assert_eq!(&buf[4..8], &2.0f32.to_le_bytes());
        assert_eq!(&buf[8..12], &3.0f32.to_le_bytes());
        assert_eq!(&buf[12..16], &4.0f32.to_le_bytes());
        assert_eq!(&buf[16..20], &5.0f32.to_le_bytes());
        assert_eq!(&buf[20..24], &6.0f32.to_le_bytes());
        assert_eq!(buf[24], 1);
    }

    #[test]
    fn test_parse_reco_response_zeroed_body() {
        let rx_buf = [0u8; TOTAL_TRANSFER_SIZE];

        let reco_body =
            RecoDriver::parse_reco_response::<RecoBody>(&rx_buf).expect("Failed to parse reco body");
        assert_eq!(reco_body.quaternion, [0.0; 4]);
        assert_eq!(reco_body.lla_pos, [0.0; 3]);
        assert_eq!(reco_body.temperature, 0.0);
        assert_eq!(reco_body.pressure, 0.0);
    }

    // Note: Hardware-dependent tests require actual hardware and cannot run in CI
    #[test]
    #[ignore]
    fn test_send_launched() {
        // This test requires hardware
        // let mut reco = RecoDriver::new("/dev/spidev0.0")
        //     .expect("Failed to create RECO driver");
        // reco.send_launched().expect("Failed to send launched message");
    }
}

