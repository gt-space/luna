//! RECO (Recovery) Board Driver
//!
//! This driver provides SPI communication with the RECO board for recovery system control.
//! The RECO board handles recovery mechanisms and communicates with the flight computer
//! via SPI using a custom protocol with CRC checksums.
//!
//! This driver is designed for Raspberry Pi using Linux spidev for SPI communication.
//! Hardware chip select (CE0/CE1) is automatically controlled by the kernel driver.

use crc::{Crc, CRC_32_ISO_HDLC};
use std::fmt;
use std::fs::{File, OpenOptions};
use std::os::unix::io::{AsRawFd, RawFd};

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
const DEFAULT_SPI_SPEED: u32 = 16_000_000; // 16 MHz
/// Message sizes
const MESSAGE_TO_RECO_SIZE: usize = 32; // opcode (1) + body (27) + checksum (4)
const BODY_SIZE: usize = 27; // 25 bytes of data + 2 bytes padding
const CHECKSUM_SIZE: usize = 4;
const RECO_BODY_SIZE: usize = 132;
const TOTAL_TRANSFER_SIZE: usize = RECO_BODY_SIZE + CHECKSUM_SIZE;

/// Opcodes for messages to RECO
pub mod opcode {
    pub const LAUNCHED: u8 = 0x01;
    pub const GPS_DATA: u8 = 0x02;
    pub const VOTING_LOGIC: u8 = 0x03;
}

/// CRC32 calculator instance
static CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

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

/// Voting logic structure for opcode 0x03
#[derive(Debug, Clone, Copy)]
pub struct VotingLogic {
    pub processor_1_enabled: bool,
    pub processor_2_enabled: bool,
    pub processor_3_enabled: bool,
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
}

/// Error types for RECO operations
#[derive(Debug)]
pub enum RecoError {
    Protocol(String),
    ChecksumMismatch,
    InvalidMessageSize(usize),
    Deserialization(String),
}

impl fmt::Display for RecoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RecoError::Protocol(msg) => write!(f, "Protocol error: {}", msg),
            RecoError::ChecksumMismatch => write!(f, "Checksum verification failed"),
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

    /// Calculate CRC32 checksum for a byte slice
    /// 
    /// NOTE: For messages sent TO RECO, the checksum is calculated on the opcode + body
    /// (bytes 0-28, which is opcode + 27-byte body), NOT just the body. This ensures the opcode is included in the
    /// checksum verification. Messages FROM RECO do not include an opcode, so only
    /// the body is checksummed.
    fn calculate_checksum(data: &[u8]) -> u32 {
        let mut digest = CRC32.digest();
        digest.update(data);
        digest.finalize()
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
    /// Checksum is calculated on opcode + body (bytes 0-28), then written to bytes 28-31.
    /// The full-duplex transfer reads RECO telemetry concurrently, which is discarded.
    pub fn send_launched(&mut self) -> Result<(), RecoError> {
        let mut message = [0u8; MESSAGE_TO_RECO_SIZE];
        
        // Set opcode
        message[0] = opcode::LAUNCHED;
        
        // Body is already zeros (padding) - 27 bytes total
        // Calculate checksum on opcode + body (bytes 0-28, which is opcode + 27-byte body)
        // NOTE: Opcode is included in checksum calculation
        let checksum = Self::calculate_checksum(&message[0..1+BODY_SIZE]);
        
        // Write checksum as little-endian u32 (bytes 28-31)
        let checksum_bytes = checksum.to_le_bytes();
        message[28..32].copy_from_slice(&checksum_bytes);
        
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
        let mut message = [0u8; MESSAGE_TO_RECO_SIZE];
        
        // Set opcode
        message[0] = opcode::GPS_DATA;
        
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
                       
        // Calculate checksum on opcode + body (bytes 0-28, which is opcode + 27-byte body)
        // NOTE: Opcode is included in checksum calculation
        let checksum = Self::calculate_checksum(&message[0..1+BODY_SIZE]);
        
        // Write checksum as little-endian u32 (bytes 28-31)
        let checksum_bytes = checksum.to_le_bytes();
        message[28..32].copy_from_slice(&checksum_bytes);
        
        let (mut tx_buf, mut rx_buf) = Self::prepare_transfer_buffers(&message)?;
        
        // Debug mode: Print TX buffer if enabled
        if std::env::var("RECO_DEBUG").is_ok() {
            eprintln!("DEBUG: Sending GPS data (opcode 0x{:02X})", opcode::GPS_DATA);
            eprintln!("DEBUG: TX buffer (first 32 bytes): {:02X?}", &tx_buf[0..tx_buf.len().min(32)]);
        }
        
        self.spi_transfer(&mut tx_buf, &mut rx_buf)?;
        Self::parse_reco_response(&rx_buf)
    }

    /// Send voting logic enable message (opcode 0x03) to RECO
    /// 
    /// # Arguments
    /// 
    /// * `voting_logic` - Voting logic structure with enable flags for each processor
    pub fn send_voting_logic(&mut self, voting_logic: &VotingLogic) -> Result<(), RecoError> {
        let mut message = [0u8; MESSAGE_TO_RECO_SIZE];
        
        // Set opcode
        message[0] = opcode::VOTING_LOGIC;
        
        // Serialize voting logic (3 bools)
        message[1] = Self::bool_to_byte(voting_logic.processor_1_enabled);
        message[2] = Self::bool_to_byte(voting_logic.processor_2_enabled);
        message[3] = Self::bool_to_byte(voting_logic.processor_3_enabled);
        
        // Remaining bytes (4-27) are padding (already zeros) - 24 bytes total
        
        // Calculate checksum on opcode + body (bytes 0-28, which is opcode + 27-byte body)
        // NOTE: Opcode is included in checksum calculation
        let checksum = Self::calculate_checksum(&message[0..1+BODY_SIZE]);
        
        // Write checksum as little-endian u32 (bytes 28-31)
        let checksum_bytes = checksum.to_le_bytes();
        message[28..32].copy_from_slice(&checksum_bytes);
        
        let (mut tx_buf, mut rx_buf) = Self::prepare_transfer_buffers(&message)?;
        self.spi_transfer(&mut tx_buf, &mut rx_buf)?;
        Ok(())
    }

    /// Receive data from RECO
    /// 
    /// This method sends a dummy message and receives the RECO body response.
    /// The response consists of the RecoBody structure followed by a 4-byte CRC checksum.
    /// 
    /// # Returns
    /// 
    /// The received RecoBody structure if successful
    pub fn receive_data(&mut self) -> Result<RecoBody, RecoError> {
        // Size of RecoBody: 
        // quaternion[4]: 4*4 = 16 bytes
        // lla_pos[3]: 3*4 = 12 bytes
        // velocity[3]: 3*4 = 12 bytes
        // g_bias[3]: 3*4 = 12 bytes
        // a_bias[3]: 3*4 = 12 bytes
        // g_sf[3]: 3*4 = 12 bytes
        // a_sf[3]: 3*4 = 12 bytes
        // lin_accel[3]: 3*4 = 12 bytes
        // angular_rate[3]: 3*4 = 12 bytes
        // mag_data[3]: 3*4 = 12 bytes
        // temperature: 4 bytes
        // pressure: 4 bytes
        // Total: 16 + 12*10 + 4 + 4 = 16 + 120 + 8 = 144 bytes
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

    fn parse_reco_response(rx_buf: &[u8]) -> Result<RecoBody, RecoError> {
        // Verify message size
        if rx_buf.len() < TOTAL_TRANSFER_SIZE {
            return Err(RecoError::InvalidMessageSize(rx_buf.len()));
        }
        
        // Extract body and checksum
        // Body: bytes 0-143 (144 bytes total)
        // Checksum: bytes 144-147 (4 bytes total, little-endian u32)
        let body_bytes = &rx_buf[0..RECO_BODY_SIZE];
        let checksum_bytes = &rx_buf[RECO_BODY_SIZE..TOTAL_TRANSFER_SIZE];
        
        // Debug mode: Print raw bytes if RECO_DEBUG environment variable is set
        // This prints BEFORE checksum verification, so you can see data even if checksum fails
        if std::env::var("RECO_DEBUG").is_ok() {
            eprintln!("DEBUG: Raw RX buffer ({} bytes):", rx_buf.len());
            eprintln!("DEBUG: Full buffer: {:02X?}", rx_buf);
            eprintln!("DEBUG: Body (first 64 bytes): {:02X?}", &body_bytes[0..body_bytes.len().min(64)]);
            eprintln!("DEBUG: Checksum bytes: {:02X?}", checksum_bytes);
        }
        
        // Verify checksum
        // NOTE: For messages FROM RECO, checksum is calculated on body only (no opcode)
        // This is different from messages TO RECO, where checksum includes opcode + body
        let calculated_checksum = Self::calculate_checksum(body_bytes);
        
        // Convert checksum bytes (little-endian) to u32
        // Safety: checksum_bytes is guaranteed to be 4 bytes by the slice bounds (RECO_BODY_SIZE..TOTAL_TRANSFER_SIZE)
        // Verify length for extra safety, then construct array
        if checksum_bytes.len() != 4 {
            return Err(RecoError::Deserialization(
                format!("Invalid checksum byte length: expected 4, got {}", checksum_bytes.len())
            ));
        }
        let checksum_array = [
            checksum_bytes[0],
            checksum_bytes[1],
            checksum_bytes[2],
            checksum_bytes[3],
        ];
        let received_checksum = u32::from_le_bytes(checksum_array);
        
        // Debug mode: Print checksum info
        if std::env::var("RECO_DEBUG").is_ok() {
            eprintln!("DEBUG: Calculated checksum: 0x{:08X}", calculated_checksum);
            eprintln!("DEBUG: Received checksum: 0x{:08X}", received_checksum);
        }
        
        if calculated_checksum != received_checksum {
            if std::env::var("RECO_DEBUG").is_ok() {
                eprintln!("DEBUG: Checksum mismatch! Calculated: 0x{:08X}, Received: 0x{:08X}", 
                    calculated_checksum, received_checksum);
            }
            return Err(RecoError::ChecksumMismatch);
        }
        
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
        let val = 3.14159f32;
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
    fn test_checksum_calculation() {
        // Test that checksum is consistent for same input
        let data = [0u8; 25];
        let checksum = RecoDriver::calculate_checksum(&data);
        assert_eq!(RecoDriver::calculate_checksum(&data), checksum);
        
        // Test that checksum changes when opcode is included (verifying opcode is in checksum)
        let mut data_with_opcode = [0u8; 26];
        data_with_opcode[0] = opcode::LAUNCHED;
        let checksum_with_opcode = RecoDriver::calculate_checksum(&data_with_opcode);
        
        // Checksum with opcode should be different from checksum without opcode
        assert_ne!(checksum, checksum_with_opcode);
        
        // Verify opcode is included: checksum should match when opcode + body are checksummed
        let mut full_message = [0u8; 26]; // opcode + 25 bytes of body
        full_message[0] = opcode::LAUNCHED;
        let full_checksum = RecoDriver::calculate_checksum(&full_message);
        assert_eq!(checksum_with_opcode, full_checksum);
    }

    #[test]
    fn test_message_format() {
        // Test that launched message has correct format with opcode included in checksum
        let mut message = [0u8; MESSAGE_TO_RECO_SIZE];
        message[0] = opcode::LAUNCHED;
        // Body (bytes 1-27) are zeros
        
        // Calculate checksum on opcode + body (bytes 0-28, which is opcode + 27-byte body)
        let checksum = RecoDriver::calculate_checksum(&message[0..1+BODY_SIZE]);
        let checksum_bytes = checksum.to_le_bytes();
        
        // Verify checksum bytes are valid
        assert_eq!(checksum_bytes.len(), 4);
        
        // Verify the checksum would be placed correctly
        message[28..32].copy_from_slice(&checksum_bytes);
        assert_eq!(message[28..32], checksum_bytes);
    }

    #[test]
    fn test_prepare_transfer_buffers_places_message() {
        let mut message = [0xAAu8; MESSAGE_TO_RECO_SIZE];
        message[0] = opcode::GPS_DATA;

        let (tx_buf, rx_buf) = RecoDriver::prepare_transfer_buffers(&message).unwrap();
        assert_eq!(&tx_buf[..MESSAGE_TO_RECO_SIZE], &message);
        assert!(tx_buf[MESSAGE_TO_RECO_SIZE..].iter().all(|&byte| byte == 0));
        assert!(rx_buf.iter().all(|&byte| byte == 0));
    }

    #[test]
    fn test_parse_reco_response_zeroed_body() {
        let mut rx_buf = [0u8; TOTAL_TRANSFER_SIZE];
        let checksum = RecoDriver::calculate_checksum(&rx_buf[..RECO_BODY_SIZE]);
        rx_buf[RECO_BODY_SIZE..TOTAL_TRANSFER_SIZE].copy_from_slice(&checksum.to_le_bytes());

        let reco_body = RecoDriver::parse_reco_response(&rx_buf).expect("Failed to parse reco body");
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

