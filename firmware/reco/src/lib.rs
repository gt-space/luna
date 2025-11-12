//! RECO (Recovery) Board Driver
//!
//! This driver provides SPI communication with the RECO board for recovery system control.
//! The RECO board handles recovery mechanisms and communicates with the flight computer
//! via SPI using a custom protocol with CRC checksums.
//!
//! This driver is designed for Raspberry Pi using the rppal crate.

use crc::{Crc, CRC_32_ISO_HDLC};
use rppal::gpio::OutputPin;
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use std::fmt;

/// Default SPI settings for RECO board
const DEFAULT_SPI_MODE: Mode = Mode::Mode0;
const DEFAULT_SPI_SPEED: u32 = 16_000_000; // 16 MHz - adjust per spec

/// Message sizes
const MESSAGE_TO_RECO_SIZE: usize = 30; // opcode (1) + body (25) + checksum (4)
const BODY_SIZE: usize = 25;
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
    spi: Spi,
    cs_pin: Option<OutputPin>,
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
    SPI(rppal::spi::Error),
    GPIO(rppal::gpio::Error),
    Protocol(String),
    ChecksumMismatch,
    InvalidMessageSize(usize),
    Deserialization(String),
}

impl fmt::Display for RecoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RecoError::SPI(err) => write!(f, "SPI error: {}", err),
            RecoError::GPIO(err) => write!(f, "GPIO error: {}", err),
            RecoError::Protocol(msg) => write!(f, "Protocol error: {}", msg),
            RecoError::ChecksumMismatch => write!(f, "Checksum verification failed"),
            RecoError::InvalidMessageSize(size) => write!(f, "Invalid message size: {} bytes", size),
            RecoError::Deserialization(msg) => write!(f, "Deserialization error: {}", msg),
        }
    }
}

impl std::error::Error for RecoError {}

impl From<rppal::spi::Error> for RecoError {
    fn from(err: rppal::spi::Error) -> Self {
        RecoError::SPI(err)
    }
}

impl From<rppal::gpio::Error> for RecoError {
    fn from(err: rppal::gpio::Error) -> Self {
        RecoError::GPIO(err)
    }
}

impl RecoDriver {
    /// Creates a new RECO driver instance
    /// 
    /// # Arguments
    /// 
    /// * `bus` - SPI bus to use (e.g., `Bus::Spi0`)
    /// * `slave_select` - SPI slave select (e.g., `SlaveSelect::Ss0`)
    /// * `cs_pin` - Optional chip select GPIO pin (active low)
    /// 
    /// # Example
    /// 
    /// ```no_run
    /// use reco::RecoDriver;
    /// use rppal::gpio::Gpio;
    /// use rppal::spi::{Bus, SlaveSelect};
    /// 
    /// let gpio = Gpio::new()?;
    /// let mut cs_pin = gpio.get(16)?.into_output();
    /// cs_pin.set_high(); // Active low, so start high (inactive)
    /// 
    /// let reco = RecoDriver::new(Bus::Spi0, SlaveSelect::Ss0, Some(cs_pin))?;
    /// # Ok::<(), reco::RecoError>(())
    /// ```
    pub fn new(
        bus: Bus,
        slave_select: SlaveSelect,
        mut cs_pin: Option<OutputPin>,
    ) -> Result<Self, RecoError> {
        // Ensure chip select pin is high (inactive) if provided
        // Note: cs_pin should already be configured as output before being passed in
        if let Some(ref mut pin) = cs_pin {
            pin.set_high(); // Active low, so start high (inactive)
        }

        // Open and configure SPI bus
        let spi = Spi::new(bus, slave_select, DEFAULT_SPI_SPEED, DEFAULT_SPI_MODE)?;
        
        Ok(RecoDriver { spi, cs_pin })
    }

    /// Enable chip select (pull low)
    fn enable_cs(&mut self) {
        if let Some(ref mut pin) = self.cs_pin {
            pin.set_low();
        }
    }

    /// Disable chip select (pull high)
    fn disable_cs(&mut self) {
        if let Some(ref mut pin) = self.cs_pin {
            pin.set_high();
        }
    }

    /// Perform SPI transfer (read and write simultaneously)
    /// 
    /// In rppal, transfer is full-duplex and takes separate tx and rx buffers.
    /// Both buffers must be mutable.
    fn spi_transfer(&mut self, tx_buf: &mut [u8], rx_buf: &mut [u8]) -> Result<(), RecoError> {
        if tx_buf.len() != rx_buf.len() {
            return Err(RecoError::Protocol(
                "TX and RX buffers must be the same size".to_string(),
            ));
        }
        
        self.enable_cs();
        
        self.spi.transfer(tx_buf, rx_buf)?;
        
        self.disable_cs();
        Ok(())
    }

    /// Calculate CRC32 checksum for a byte slice
    /// 
    /// NOTE: For messages sent TO RECO, the checksum is calculated on the opcode + body
    /// (bytes 0-25), NOT just the body. This ensures the opcode is included in the
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
    /// Checksum is calculated on opcode + body (bytes 0-25), then written to bytes 26-29.
    /// The full-duplex transfer reads RECO telemetry concurrently, which is discarded.
    pub fn send_launched(&mut self) -> Result<(), RecoError> {
        let mut message = [0u8; MESSAGE_TO_RECO_SIZE];
        
        // Set opcode
        message[0] = opcode::LAUNCHED;
        
        // Body is already zeros (padding)
        // Calculate checksum on opcode + body (bytes 0-25)
        // NOTE: Opcode is included in checksum calculation
        let checksum = Self::calculate_checksum(&message[0..1+BODY_SIZE]);
        
        // Write checksum as little-endian u32 (bytes 26-29)
        let checksum_bytes = checksum.to_le_bytes();
        message[26..30].copy_from_slice(&checksum_bytes);
        
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
        
        // Remaining bytes (24 - 25 = 0 bytes, but we have 1 byte used for valid)
        // Actually: 25 - 24 = 1 byte remaining, which is already set to 0
        // Total used: 4*6 + 1 = 25 bytes ✓
        
        // Calculate checksum on opcode + body (bytes 0-25)
        // NOTE: Opcode is included in checksum calculation
        let checksum = Self::calculate_checksum(&message[0..1+BODY_SIZE]);
        
        // Write checksum as little-endian u32 (bytes 26-29)
        let checksum_bytes = checksum.to_le_bytes();
        message[26..30].copy_from_slice(&checksum_bytes);
        
        let (mut tx_buf, mut rx_buf) = Self::prepare_transfer_buffers(&message)?;
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
        
        // Remaining bytes (4-25) are padding (already zeros)
        
        // Calculate checksum on opcode + body (bytes 0-25)
        // NOTE: Opcode is included in checksum calculation
        let checksum = Self::calculate_checksum(&message[0..1+BODY_SIZE]);
        
        // Write checksum as little-endian u32 (bytes 26-29)
        let checksum_bytes = checksum.to_le_bytes();
        message[26..30].copy_from_slice(&checksum_bytes);
        
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
        
        if calculated_checksum != received_checksum {
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

    /// Get the SPI device handle (for advanced use)
    pub fn spi(&mut self) -> &mut Spi {
        &mut self.spi
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
        // Body (bytes 1-25) are zeros
        
        // Calculate checksum on opcode + body (bytes 0-25)
        let checksum = RecoDriver::calculate_checksum(&message[0..1+BODY_SIZE]);
        let checksum_bytes = checksum.to_le_bytes();
        
        // Verify checksum bytes are valid
        assert_eq!(checksum_bytes.len(), 4);
        
        // Verify the checksum would be placed correctly
        message[26..30].copy_from_slice(&checksum_bytes);
        assert_eq!(message[26..30], checksum_bytes);
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
        // use rppal::gpio::Gpio;
        // use rppal::spi::{Bus, SlaveSelect};
        // 
        // let gpio = Gpio::new().expect("Failed to open GPIO");
        // let cs_pin = gpio.get(16).expect("Failed to get pin").into_output();
        // cs_pin.set_high();
        // 
        // let mut reco = RecoDriver::new(Bus::Spi0, SlaveSelect::Ss0, Some(cs_pin))
        //     .expect("Failed to create RECO driver");
        // reco.send_launched().expect("Failed to send launched message");
    }
}

