//! RECO (Recovery) Board Driver
//!
//! This driver provides SPI communication with the RECO board for recovery system control.
//! The RECO board handles recovery mechanisms and communicates with the flight computer
//! via SPI using a custom protocol.
//!
//! This driver is designed for Raspberry Pi using Linux spidev for SPI communication.
//! Hardware chip select (CE0/CE1) is automatically controlled by the kernel driver.

use common::comm::reco::EkfBiasParameters;
use std::{
    fmt,
    fs::{File, OpenOptions},
    os::unix::io::{AsRawFd, RawFd},
};

// SPI ioctl definitions (from Linux spidev.h)
const SPI_IOC_WR_MODE: u32 = 0x40016b01;
const SPI_IOC_WR_MAX_SPEED_HZ: u32 = 0x40046b04;
const SPI_IOC_MESSAGE_1: u32 = 0x40206b00;
/// Default SPI settings for RECO board
const DEFAULT_SPI_MODE: u8 = 0; // Mode 0 (CPOL=0, CPHA=0)
const DEFAULT_SPI_SPEED: u32 = 2_000_000; // 2 MHz
// Size of the RECO message in bytes
const RECO_BODY_SIZE: usize = 152;
const TOTAL_TRANSFER_SIZE: usize = RECO_BODY_SIZE;

/// Information about a message to from FC to RECO
#[derive(Debug, Clone, Copy)]
pub struct MessageInfo {
    /// Opcode of the message
    pub opcode: u8,
    /// Size of message (opcode + body) in bytes
    pub message_size: usize,
}

/// Collection of all FC → RECO message descriptors.
#[derive(Debug, Clone, Copy)]
pub struct FcToRecoMessages {
    pub launch: MessageInfo,
    pub gps_data: MessageInfo,
    pub init_ekf: MessageInfo,
    pub set_ekf_params: MessageInfo,
}

/// All FC to RECO command messages and their information
/// Message size is determined by opcode + body size (in bytes)
pub const FC_RECO_MESSAGES: FcToRecoMessages = FcToRecoMessages {
    launch: MessageInfo {
        opcode: 0x79,
        message_size: 1 + 0,
    },
    gps_data: MessageInfo {
        opcode: 0xF2,
        message_size: 1 + 25,
    },
    init_ekf: MessageInfo {
        opcode: 0xCA,
        message_size: 1 + 0,
    },
    set_ekf_params: MessageInfo {
        opcode: 0x2E,
        message_size: 1 + 84,
    },
};

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
    pub quaternion: [f32; 4],           // attitude of vehicle
    pub lla_pos: [f32; 3],              // position [longitude, latitude, altitude]
    pub velocity: [f32; 3],             // velocity of vehicle
    pub g_bias: [f32; 3],               // gyroscope bias offset
    pub a_bias: [f32; 3],               // accelerometer bias offset
    pub g_sf: [f32; 3],                 // gyro scale factor
    pub a_sf: [f32; 3],                 // acceleration scale factor
    pub lin_accel: [f32; 3],            // XYZ Acceleration
    pub angular_rate: [f32; 3],         // Angular Rates (pitch, yaw, roll)
    pub mag_data: [f32; 3],             // XYZ Magnetometer Data
    pub temperature: f32,
    pub pressure: f32,
    pub stage1_enabled: bool,
    pub stage2_enabled: bool,
    pub vref_a_stage1: bool,
    pub vref_a_stage2: bool,
    pub vref_b_stage1: bool,
    pub vref_b_stage2: bool,
    pub vref_c_stage1: bool,
    pub vref_c_stage2: bool,
    pub vref_d_stage1: bool,
    pub vref_d_stage2: bool,
    pub vref_e_stage1_1: bool,
    pub vref_e_stage1_2: bool,
    pub reco_recvd_launch: bool,        // True if RECO has received the launch command, else False
    pub fault_driver_a: bool,
    pub fault_driver_b: bool,
    pub fault_driver_c: bool,
    pub fault_driver_d: bool,
    pub fault_driver_e: bool,
    pub ekf_blown_up: bool,
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

    /// Serialize f32 to little-endian bytes
    fn f32_to_bytes(val: f32) -> [u8; 4] {
        val.to_le_bytes()
    }

    /// Deserialize little-endian bytes to f32
    fn bytes_to_f32(bytes: &[u8]) -> Result<f32, RecoError> {
        if bytes.len() < 4 {
            return Err(RecoError::Deserialization("Insufficient bytes for f32".to_string()));
        }
        Ok(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    /// Serialize bool to byte (1 for true, 0 for false)
    fn bool_to_byte(val: bool) -> u8 {
        if val { 1 } else { 0 }
    }

    /// Deserialize byte to bool (non-zero is true)
    fn byte_to_bool(byte: u8) -> bool {
        byte != 0
    }

    /// Prepare a transfer buffer with the outbound message placed at the start.
    fn prepare_transfer_buffers(message: &[u8]) -> Result<([u8; TOTAL_TRANSFER_SIZE], [u8; TOTAL_TRANSFER_SIZE]), RecoError> {
        if message.len() > TOTAL_TRANSFER_SIZE {
            return Err(RecoError::Protocol(format!(
                "Message size {} exceeds transfer size {}",
                message.len(),
                TOTAL_TRANSFER_SIZE
            )));
        }

        let mut tx_buf = [0u8; TOTAL_TRANSFER_SIZE];
        tx_buf[..message.len()].copy_from_slice(message);
        let rx_buf = [0u8; TOTAL_TRANSFER_SIZE];
        Ok((tx_buf, rx_buf))
    }

    /// Send "launched" message (opcode 0x01) to RECO
    /// 
    /// This message indicates that the rocket has been launched.
    /// The body is all zeros (padding).
    /// 
    /// The full-duplex transfer reads RECO telemetry concurrently, which is discarded.
    pub fn send_launched(&mut self) -> Result<(), RecoError> {
        let mut message = [0u8; { FC_RECO_MESSAGES.launch.message_size }];
        
        // Set opcode
        message[0] = FC_RECO_MESSAGES.launch.opcode;
        
        // Body is already zeros (padding) - 25 bytes total
        
        let (mut tx_buf, mut rx_buf) = Self::prepare_transfer_buffers(&message)?;
        self.spi_transfer(&mut tx_buf, &mut rx_buf)?;
        Ok(())
    }

    /// Send GPS data to RECO and receive RECO telemetry in a single full-duplex transfer.
    /// 
    /// # Arguments
    /// 
    /// * `gps_data` - GPS data structure containing velocity, position, and validity
    pub fn send_gps_data_and_receive_reco(&mut self, gps_data: &FcGpsBody) -> Result<RecoBody, RecoError> {
        let mut message = [0u8; { FC_RECO_MESSAGES.gps_data.message_size }];
        
        // Set opcode
        message[0] = FC_RECO_MESSAGES.gps_data.opcode;
        
        // Serialize GPS body data (little-endian)
        let mut offset = 1;
        
        // velocity_north (4 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(gps_data.velocity_north));
        offset += 4;
        
        // velocity_east (4 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(gps_data.velocity_east));
        offset += 4;
        
        // velocity_down (4 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(gps_data.velocity_down));
        offset += 4;
        
        // latitude (4 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(gps_data.latitude));
        offset += 4;
        
        // longitude (4 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(gps_data.longitude));
        offset += 4;
        
        // altitude (4 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(gps_data.altitude));
        offset += 4;
        
        // valid (1 byte)
        message[offset] = Self::bool_to_byte(gps_data.valid);
        offset += 1;
                       
        let (mut tx_buf, mut rx_buf) = Self::prepare_transfer_buffers(&message)?;
        
        self.spi_transfer(&mut tx_buf, &mut rx_buf)?;
        Self::parse_reco_response(&rx_buf)
    }

    /// Send EKF bias parameters to RECO
    pub fn send_ekf_bias_parameters(&mut self, params: &EkfBiasParameters) -> Result<(), RecoError> {
        let mut message = [0u8; { FC_RECO_MESSAGES.set_ekf_params.message_size }];
        
        // Set opcode
        message[0] = FC_RECO_MESSAGES.set_ekf_params.opcode;
        let mut offset = 1;
        
        // quaternion (16 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.quaternion[0]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.quaternion[1]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.quaternion[2]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.quaternion[3]));
        offset += 4;
        
        // lla_pos (12 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.lla_pos[0]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.lla_pos[1]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.lla_pos[2]));
        offset += 4;

        // a_bias (12 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.a_bias[0]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.a_bias[1]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.a_bias[2]));
        offset += 4;

        // g_bias (12 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.g_bias[0]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.g_bias[1]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.g_bias[2]));
        offset += 4;

        // a_sf (12 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.a_sf[0]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.a_sf[1]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.a_sf[2]));
        offset += 4;

        // g_sf (12 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.g_sf[0]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.g_sf[1]));
        offset += 4;
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.g_sf[2]));
        offset += 4;

        // alt_press_off (4 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.alt_press_off));
        offset += 4;    

        // filter_press_off (4 bytes)
        message[offset..offset+4].copy_from_slice(&Self::f32_to_bytes(params.filter_press_off));
        offset += 4;    
        
        let (mut tx_buf, mut rx_buf) = Self::prepare_transfer_buffers(&message)?;
        self.spi_transfer(&mut tx_buf, &mut rx_buf)?;
        
        Ok(())
    }

    /// Send EKF-initialization message (repurposed opcode 0x03) to RECO.
    ///
    /// The body is all zeros (padding); only the opcode is used by RECO to
    /// trigger EKF initialization.
    pub fn send_init_ekf(&mut self) -> Result<(), RecoError> {
        let mut message = [0u8; { FC_RECO_MESSAGES.init_ekf.message_size }];

        // Set opcode
        message[0] = FC_RECO_MESSAGES.init_ekf.opcode;

        // Body (bytes 1-25) remain zeros.
        let (mut tx_buf, mut rx_buf) = Self::prepare_transfer_buffers(&message)?;
        self.spi_transfer(&mut tx_buf, &mut rx_buf)?;
        Ok(())
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
        
        self.spi_transfer(&mut tx_buf, &mut rx_buf)?;
        Self::parse_reco_response(&rx_buf)
    }

    fn parse_reco_response(rx_buf: &[u8]) -> Result<RecoBody, RecoError> {
        // Verify message size
        if rx_buf.len() < TOTAL_TRANSFER_SIZE {
            return Err(RecoError::InvalidMessageSize(rx_buf.len()));
        }
        
        // Extract body
        let body_bytes = &rx_buf[0..RECO_BODY_SIZE];
        
        // Deserialize RecoBody
        let mut offset = 0;
        
        // quaternion[4] (16 bytes)
        let quaternion = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
            Self::bytes_to_f32(&body_bytes[offset+12..offset+16])?,
        ];
        offset += 16;
        
        // lla_pos[3] (12 bytes)
        let lla_pos = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
        ];
        offset += 12;
        
        // velocity[3] (12 bytes)
        let velocity = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
        ];
        offset += 12;
        
        // g_bias[3] (12 bytes)
        let g_bias = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
        ];
        offset += 12;
        
        // a_bias[3] (12 bytes)
        let a_bias = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
        ];
        offset += 12;
        
        // g_sf[3] (12 bytes)
        let g_sf = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
        ];
        offset += 12;
        
        // a_sf[3] (12 bytes)
        let a_sf = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
        ];
        offset += 12;
        
        // lin_accel[3] (12 bytes)
        let lin_accel = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
        ];
        offset += 12;
        
        // angular_rate[3] (12 bytes)
        let angular_rate = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
        ];
        offset += 12;
        
        // mag_data[3] (12 bytes)
        let mag_data = [
            Self::bytes_to_f32(&body_bytes[offset..offset+4])?,
            Self::bytes_to_f32(&body_bytes[offset+4..offset+8])?,
            Self::bytes_to_f32(&body_bytes[offset+8..offset+12])?,
        ];
        offset += 12;
        
        // temperature (4 bytes)
        let temperature = Self::bytes_to_f32(&body_bytes[offset..offset+4])?;
        offset += 4;
        
        // pressure (4 bytes)
        let pressure = Self::bytes_to_f32(&body_bytes[offset..offset+4])?;
        offset += 4;
        
        // stage1_enabled (1 byte)
        let stage1_enabled = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // stage2_enabled (1 byte)
        let stage2_enabled = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_a_stage1 (1 byte)
        let vref_a_stage1 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_a_stage2 (1 byte)
        let vref_a_stage2 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_b_stage1 (1 byte)
        let vref_b_stage1 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_b_stage2 (1 byte)
        let vref_b_stage2 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_c_stage1 (1 byte)
        let vref_c_stage1 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_c_stage2 (1 byte)
        let vref_c_stage2 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_d_stage1 (1 byte)
        let vref_d_stage1 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_d_stage2 (1 byte)
        let vref_d_stage2 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_e_stage1_1 (1 byte)
        let vref_e_stage1_1 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // vref_e_stage1_2 (1 byte)
        let vref_e_stage1_2 = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        
        // reco_recvd_launch (1 byte)
        let reco_recvd_launch = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;

        // fault_driver_a..fault_driver_e 
        let fault_driver_a = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        let fault_driver_b = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        let fault_driver_c = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        let fault_driver_d = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;
        let fault_driver_e = Self::byte_to_bool(body_bytes[offset]);
        offset += 1;

        // ekf_blown_up (1 byte)
        let ekf_blown_up = Self::byte_to_bool(body_bytes[offset]);
        
        Ok(RecoBody {
            quaternion,
            lla_pos,
            velocity,
            g_bias,
            a_bias,
            g_sf,
            a_sf,
            lin_accel,
            angular_rate,
            mag_data,
            temperature,
            pressure,
            stage1_enabled,
            stage2_enabled,
            vref_a_stage1,
            vref_a_stage2,
            vref_b_stage1,
            vref_b_stage2,
            vref_c_stage1,
            vref_c_stage2,
            vref_d_stage1,
            vref_d_stage2,
            vref_e_stage1_1,
            vref_e_stage1_2,
            reco_recvd_launch,
            fault_driver_a,
            fault_driver_b,
            fault_driver_c,
            fault_driver_d,
            fault_driver_e,
            ekf_blown_up,
        })
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
        let val = std::f32::consts::PI;
        let bytes = RecoDriver::f32_to_bytes(val);
        let restored = RecoDriver::bytes_to_f32(&bytes).unwrap();
        assert!((val - restored).abs() < 0.0001);
    }

    #[test]
    fn test_bool_serialization() {
        assert_eq!(RecoDriver::bool_to_byte(true), 1);
        assert_eq!(RecoDriver::bool_to_byte(false), 0);
        assert_eq!(RecoDriver::byte_to_bool(1), true);
        assert_eq!(RecoDriver::byte_to_bool(0), false);
        assert_eq!(RecoDriver::byte_to_bool(42), true);
    }

    #[test]
    fn test_message_format() {
        // Test that launched message has correct format
        let mut message = [0u8; FC_RECO_MESSAGES.launch.message_size];
        message[0] = FC_RECO_MESSAGES.launch.opcode;
        // Body is empty; only opcode is used for this command.
        
        // Verify message size
        assert_eq!(message.len(), FC_RECO_MESSAGES.launch.message_size);
    }

    #[test]
    fn test_prepare_transfer_buffers_places_message() {
        let mut message = [0xAAu8; FC_RECO_MESSAGES.gps_data.message_size];
        message[0] = FC_RECO_MESSAGES.gps_data.opcode;

        let (tx_buf, rx_buf) = RecoDriver::prepare_transfer_buffers(&message).unwrap();
        assert_eq!(&tx_buf[..FC_RECO_MESSAGES.gps_data.message_size], &message);
        assert!(tx_buf[FC_RECO_MESSAGES.gps_data.message_size..].iter().all(|&byte| byte == 0));
        assert!(rx_buf.iter().all(|&byte| byte == 0));
    }

    #[test]
    fn test_parse_reco_response_zeroed_body() {
        let rx_buf = [0u8; TOTAL_TRANSFER_SIZE];

        let reco_body = RecoDriver::parse_reco_response(&rx_buf).expect("Failed to parse reco body");
        assert_eq!(reco_body.quaternion, [0.0; 4]);
        assert_eq!(reco_body.lla_pos, [0.0; 3]);
        assert_eq!(reco_body.temperature, 0.0);
        assert_eq!(reco_body.pressure, 0.0);
    }
}
