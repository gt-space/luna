//! GPS Driver for u-blox ZED-F9P using I2C
//! 
//! Documentation:
//! - Datasheet: https://content.u-blox.com/sites/default/files/ZED-F9P-04B_DataSheet_UBX-21044850.pdf
//! - Interface Description: https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf
//! - Ublox crate: https://docs.rs/ublox/latest/ublox/index.html
//! - RPPAL crate: https://docs.rs/rppal/latest/rppal/
//!
//! # I2C Protocol Notes
//! 
//! According to the u-blox interface description:
//! - The module acts as an I2C peripheral (slave) at address 0x42
//! - To read data:
//!   1. Read 2 bytes from register 0xFD to get the number of available bytes
//!   2. Read that many bytes from register 0xFF (data stream register)
//! - To write data: Simply write bytes directly to the module
//! - Maximum I2C speed: 400 kHz (Fast mode)

use chrono::{DateTime, Utc};
use rppal::i2c::I2c;
use std::{
  fmt, io,
  thread,
  time::Duration,
};
use ublox::{
  cfg_msg::CfgMsgAllPortsBuilder,
  cfg_rate::{AlignmentToReferenceTime, CfgRateBuilder},
  nav_pvt::proto23::NavPvt,
  packetref_proto23::PacketRef,
  mon_ver::MonVer,
  GnssFixType, Parser, Position, UbxPacket, UbxPacketRequest,
};

/// Default I2C address for u-blox GNSS modules
pub const UBLOX_I2C_ADDRESS: u16 = 0x42;

/// Register address to read the number of available bytes (high byte at 0xFD, low byte at 0xFE)
const UBLOX_NUM_BYTES_REG: u8 = 0xFD;

/// Data stream register address
const UBLOX_STREAM_REG: u8 = 0xFF;

/// Maximum payload length for u-blox messages
const MAX_PAYLOAD_LEN: usize = 1240;

/// Velocity in North-East-Down (NED) coordinate system
/// All values are in meters per second
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NedVelocity {
  /// North component of velocity (m/s)
  pub north: f64,
  /// East component of velocity (m/s)
  pub east: f64,
  /// Down component of velocity (m/s)
  pub down: f64,
}

/// Position, Velocity, and Time data structure
#[derive(Clone, Copy, Debug, Default)]
pub struct PVT {
  pub position: Option<Position>,
  pub velocity: Option<NedVelocity>,
  pub time: Option<DateTime<Utc>>,
}

/// GPS driver for u-blox ZED-F9P module using I2C
pub struct GPS {
  i2c: I2c,
  parser: Parser<Vec<u8>>,
}

/// Error types for GPS operations
#[derive(Debug)]
pub enum GPSError {
  I2C(rppal::i2c::Error),
  IO(io::Error),
  GPSMessage(io::Error),
  Configuration(String),
}

impl fmt::Display for GPSError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      GPSError::I2C(err) => write!(f, "I2C error: {}", err),
      GPSError::IO(err) => write!(f, "IO error: {}", err),
      GPSError::GPSMessage(err) => write!(f, "GPS Message error: {}", err),
      GPSError::Configuration(msg) => write!(f, "Configuration error: {}", msg),
    }
  }
}

impl std::error::Error for GPSError {}

impl From<rppal::i2c::Error> for GPSError {
  fn from(err: rppal::i2c::Error) -> Self {
    GPSError::I2C(err)
  }
}

impl From<io::Error> for GPSError {
  fn from(err: io::Error) -> Self {
    GPSError::IO(err)
  }
}

impl GPS {
  /// Creates a new GPS driver instance
  /// 
  /// # Arguments
  /// 
  /// * `i2c_bus` - I2C bus number (typically 1 for /dev/i2c-1 on Raspberry Pi)
  /// * `address` - Optional I2C address (defaults to 0x42 if None)
  /// 
  /// # Example
  /// 
  /// ```no_run
  /// use zedf9p04b::GPS;
  /// 
  /// let gps = GPS::new(1, None).expect("Failed to initialize GPS");
  /// ```
  pub fn new(i2c_bus: u8, address: Option<u16>) -> Result<Self, GPSError> {
    let mut i2c = I2c::with_bus(i2c_bus)?;
    let addr = address.unwrap_or(UBLOX_I2C_ADDRESS);
    i2c.set_slave_address(addr)?;
    
    let parser = Parser::default();
    
    Ok(GPS { i2c, parser })
  }

  /// Sends a UBX-MON-VER request to query module version information
  /// 
  /// This is useful for testing I2C communication with the module.
  /// See Interface Description Section 3.14.15
  pub fn mon_ver(&mut self) -> Result<(), GPSError> {
    // Send the MON-VER request
    let request = UbxPacketRequest::request_for::<MonVer>().into_packet_bytes();
    self.write_packet(&request)?;
    
    // Wait a bit for the module to process
    thread::sleep(Duration::from_millis(100));
    
    let mut found_mon_ver = false;
    let mut attempts = 0;
    const MAX_ATTEMPTS: u32 = 100;
    
    // Try reading packets until we find MON-VER or exceed max attempts
    while !found_mon_ver && attempts < MAX_ATTEMPTS {
      let found_packet = self.read_packets(|packet| {
        match packet {
          PacketRef::MonVer(packet) => {
            found_mon_ver = true;
            println!(
              "SW version: {} HW version: {}; Extensions: {:?}",
              packet.software_version(),
              packet.hardware_version(),
              packet.extension().collect::<Vec<&str>>()
            );
          }
          _ => {
            // Some other packet, ignore
          }
        }
      })?;
      
      if !found_packet {
        // No data available yet, wait a bit
        thread::sleep(Duration::from_millis(50));
      }
      attempts += 1;
    }
    
    if found_mon_ver {
      Ok(())
    } else {
      Err(GPSError::GPSMessage(std::io::Error::new(
        std::io::ErrorKind::NotFound,
        "No UBX-MON-VER response received from the device.",
      )))
    }
  }

  /// Configures the measurement rate (CFG-RATE)
  /// 
  /// # Arguments
  /// 
  /// * `meas_rate_ms` - Measurement period in milliseconds (e.g., 50 for 20 Hz, 100 for 10 Hz)
  /// * `nav_rate` - Navigation rate (number of measurement cycles per navigation solution, typically 1)
  /// * `time_ref` - Time reference (0 = UTC, 1 = GPS time)
  /// 
  /// # Example
  /// 
  /// ```no_run
  /// // Configure for 20 Hz (50 ms period)
  /// gps.set_measurement_rate(50, 1, 0)?;
  /// ```
  /// 
  /// See Interface Description Section 3.10.4
  /// See [ublox crate documentation](https://docs.rs/ublox/latest/ublox/cfg_rate/index.html)
  pub fn set_measurement_rate(
    &mut self,
    meas_rate_ms: u16,
    nav_rate: u16,
    time_ref: u16,
  ) -> Result<(), GPSError> {
    // Use ublox crate's CfgRateBuilder to construct the CFG-RATE message
    // UBX-CFG-RATE (Class 0x06, ID 0x08)
    let time_ref_enum = match time_ref {
      0 => AlignmentToReferenceTime::Utc,
      1 => AlignmentToReferenceTime::Gps,
      _ => AlignmentToReferenceTime::Utc, // Default to UTC for invalid values
    };
    
    let cfg_rate = CfgRateBuilder {
      measure_rate_ms: meas_rate_ms,
      nav_rate,
      time_ref: time_ref_enum,
    };
    
    // Convert to packet bytes
    let config = cfg_rate.into_packet_bytes();
    self.write_packet(&config)?;
    thread::sleep(Duration::from_millis(100));
    Ok(())
  }

  /// Configures the module to send periodic NAV-PVT messages at a specified rate
  /// 
  /// # Arguments
  /// 
  /// * `rate` - Rate for each port [DDC, UART1, UART2, USB, SPI, Reserved]
  ///            For I2C (DDC), typically set rate[0] to desired value (e.g., 1 for every solution)
  /// 
  /// See Interface Description Section 3.10.10
  pub fn set_nav_pvt_rate(&mut self, rate: [u8; 6]) -> Result<(), GPSError> {
    let config = CfgMsgAllPortsBuilder::set_rate_for::<NavPvt>(rate).into_packet_bytes();
    self.write_packet(&config)?;
    thread::sleep(Duration::from_millis(100));
    Ok(())
  }

  /// Polls the GPS module for a PVT (Position, Velocity, Time) message
  /// 
  /// See Interface Description Section 3.15.13
  pub fn poll_pvt(&mut self) -> Result<Option<PVT>, GPSError> {
    let mut pvt = PVT {
      position: None,
      velocity: None,
      time: None,
    };
    
    // Send NAV-PVT poll request
    let request = UbxPacketRequest::request_for::<NavPvt>().into_packet_bytes();
    self.write_packet(&request)?;
    
    // Wait for response
    thread::sleep(Duration::from_millis(100));
    
    let mut got_pvt = false;
    let mut attempts = 0;
    const MAX_ATTEMPTS: u32 = 10;
    
    while !got_pvt && attempts < MAX_ATTEMPTS {
      let found_packet = self.read_packets(|packet| {
        match packet {
          PacketRef::NavPvt(sol) => {
            got_pvt = true;

            println!("NavPvt: {:?}", sol);
            let has_time = sol.fix_type() == GnssFixType::Fix3D
              || sol.fix_type() == GnssFixType::GPSPlusDeadReckoning
              || sol.fix_type() == GnssFixType::TimeOnlyFix;
            let has_posvel = sol.fix_type() == GnssFixType::Fix3D
              || sol.fix_type() == GnssFixType::GPSPlusDeadReckoning;

            if has_posvel {
              let pos: Position = Position {
                lon: sol.longitude(),
                lat: sol.latitude(),
                alt: sol.height_above_ellipsoid(),
              };

              // Extract NED velocity from NavPvt (values are in m/s)
              let vel_ned = NedVelocity {
                north: sol.vel_north(),
                east: sol.vel_east(),
                down: sol.vel_down(),
              };
              
              pvt.position = Some(pos);
              pvt.velocity = Some(vel_ned);
            }

            if has_time {
              if let Ok(time) = (&sol).try_into() {
                let time: DateTime<Utc> = time;
                pvt.time = Some(time);
              }
            }
          }
          _ => {
            // Some other packet
          }
        }
      })?;
      
      if !found_packet {
        thread::sleep(Duration::from_millis(50));
      }
      attempts += 1;
    }
    
    if pvt.position.is_some() || pvt.velocity.is_some() || pvt.time.is_some() {
      Ok(Some(pvt))
    } else {
      Ok(None)
    }
  }

  /// Reads available packets and extracts PVT data if found
  /// 
  /// Unlike `poll_pvt()`, this function does not send a poll request.
  /// It simply reads any available NAV-PVT packets from the module's buffer.
  /// This is useful in periodic mode where the module automatically sends NAV-PVT messages.
  /// 
  /// # Returns
  /// 
  /// * `Ok(Some(PVT))` - If a NAV-PVT packet was found and parsed
  /// * `Ok(None)` - If no NAV-PVT packet was found in the available data
  /// * `Err(GPSError)` - If an I2C error occurred
  /// 
  /// # Example
  /// 
  /// ```no_run
  /// // In periodic mode, read PVT data as it arrives
  /// if let Some(pvt) = gps.read_pvt()? {
  ///     if let Some(pos) = pvt.position {
  ///         println!("Position: {:?}", pos);
  ///     }
  /// }
  /// ```
  pub fn read_pvt(&mut self) -> Result<Option<PVT>, GPSError> {
    let mut pvt = PVT {
      position: None,
      velocity: None,
      time: None,
    };
    
    let mut found_pvt = false;
    
    // Read available packets and look for NAV-PVT
    self.read_packets(|packet| {
      match packet {
        PacketRef::NavPvt(sol) => {
          found_pvt = true;
          
          let has_time = sol.fix_type() == GnssFixType::Fix3D
            || sol.fix_type() == GnssFixType::GPSPlusDeadReckoning
            || sol.fix_type() == GnssFixType::TimeOnlyFix;
          let has_posvel = sol.fix_type() == GnssFixType::Fix3D
            || sol.fix_type() == GnssFixType::GPSPlusDeadReckoning;

          if has_posvel {
            let pos: Position = Position {
              lon: sol.longitude(),
              lat: sol.latitude(),
              alt: sol.height_above_ellipsoid(),
            };

            // Extract NED velocity from NavPvt (values are in m/s)
            let vel_ned = NedVelocity {
              north: sol.vel_north(),
              east: sol.vel_east(),
              down: sol.vel_down(),
            };
            
            pvt.position = Some(pos);
            pvt.velocity = Some(vel_ned);
          }

          if has_time {
            if let Ok(time) = (&sol).try_into() {
              let time: DateTime<Utc> = time;
              pvt.time = Some(time);
            }
          }
        }
        _ => {
          // Some other packet, ignore
        }
      }
    })?;
    
    if found_pvt && (pvt.position.is_some() || pvt.velocity.is_some() || pvt.time.is_some()) {
      Ok(Some(pvt))
    } else {
      Ok(None)
    }
  }

  /// Reads all available packets from the module and prints them
  pub fn read_all(&mut self) -> Result<(), GPSError> {
    self.read_packets(|packet| {
      println!("{:?}", packet);
    })?;
    Ok(())
  }

  /// Writes a UBX packet to the module via I2C
  fn write_packet(&mut self, data: &[u8]) -> Result<(), GPSError> {
    self.i2c.write(data)?;
    Ok(())
  }

  /// Reads packets from the I2C bus and processes them with a callback
  /// 
  /// The u-blox I2C protocol recommends first reading the number of available
  /// bytes from registers 0xFD/0xFE, then reading exactly that many bytes from
  /// the data stream register 0xFF. This avoids wasting time reading a large
  /// fixed-size buffer full of 0xFF when little or no data is available.
  fn read_packets<T: FnMut(PacketRef)>(
    &mut self,
    mut cb: T,
  ) -> Result<bool, GPSError> {
    // First, query how many bytes are available in the module's buffer.
    // According to the u-blox documentation, the high byte is at 0xFD and
    // the low byte is at 0xFE. We use I2C write_read to read both bytes.
    let mut count_buf = [0u8; 2];
    // Select the "number of bytes" register and read 2 bytes from it.
    self.i2c.write_read(&[UBLOX_NUM_BYTES_REG], &mut count_buf)?;

    // The datasheet specifies MSB at 0xFD and LSB at 0xFE, so this is big-endian.
    let available = u16::from_be_bytes(count_buf) as usize;

    if available == 0 {
      // No data available; return quickly.
      return Ok(false);
    }

    // Clamp the number of bytes to our maximum payload length to avoid
    // over-reading into our buffer.
    let to_read = available.min(MAX_PAYLOAD_LEN);

    // Read exactly `to_read` bytes from the data stream register 0xFF.
    let mut local_buf = vec![0u8; to_read];
    self.i2c.write_read(&[UBLOX_STREAM_REG], &mut local_buf)?;

    let mut got_good_packet = false;

    // Parse the received data
    let mut it = self.parser.consume_ubx(&local_buf);
    loop {
      match it.next() {
        Some(Ok(ubx_packet)) => {
          got_good_packet = true;
          // Convert UbxPacket to PacketRef by extracting the Proto23 variant
          // We only support Proto23, so ignore other protocol versions
          if let UbxPacket::Proto23(packet_ref) = ubx_packet {
            cb(packet_ref);
          }
          // Note: Other protocol versions (Proto14, Proto27, Proto31) are ignored
        }
        Some(Err(_)) => {
          // Malformed packet, ignore
        }
        None => {
          // No more packets
          break;
        }
      }
    }

    Ok(got_good_packet)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  // Note: These tests require actual hardware and cannot run in CI
  // They are here as examples of how to use the driver

  #[test]
  #[ignore]
  fn test_mon_ver() {
    let mut gps = GPS::new(1, None).expect("Failed to create GPS");
    gps.mon_ver().expect("Failed to get MON-VER");
  }

  #[test]
  #[ignore]
  fn test_poll_pvt() {
    let mut gps = GPS::new(1, None).expect("Failed to create GPS");
    let pvt = gps.poll_pvt().expect("Failed to poll PVT");
    println!("PVT: {:?}", pvt);
  }
}
