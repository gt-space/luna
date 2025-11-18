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
  CfgMsgAllPortsBuilder, GpsFix, MonVer, NavPvt, PacketRef,
  Parser, Position, UbxPacketRequest,
};

/// Default I2C address for u-blox GNSS modules
pub const UBLOX_I2C_ADDRESS: u16 = 0x42;

/// Register address to read the number of available bytes (high byte at 0xFD, low byte at 0xFE)
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
    const MAX_ATTEMPTS: u32 = 10;
    
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
            
            let has_time = sol.fix_type() == GpsFix::Fix3D
              || sol.fix_type() == GpsFix::GPSPlusDeadReckoning
              || sol.fix_type() == GpsFix::TimeOnlyFix;
            let has_posvel = sol.fix_type() == GpsFix::Fix3D
              || sol.fix_type() == GpsFix::GPSPlusDeadReckoning;

            if has_posvel {
              let pos: Position = Position {
                lon: sol.lon_degrees(),
                lat: sol.lat_degrees(),
                alt: sol.height_meters(),
              };

              // Extract NED velocity from NavPvt (values are in mm/s, convert to m/s)
              let vel_ned = NedVelocity {
                north: sol.vel_north() as f64,
                east: sol.vel_east() as f64,
                down: sol.vel_down() as f64,
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
  /// The u-blox I2C protocol works by reading a buffer of data directly.
  /// If no data is available, the module returns 0xFF bytes.
  fn read_packets<T: FnMut(PacketRef)>(
    &mut self,
    mut cb: T,
  ) -> Result<bool, GPSError> {
    // Read a buffer of data from the I2C bus
    // The module will fill it with data if available, or 0xFF if not
    let mut local_buf = vec![0u8; MAX_PAYLOAD_LEN];
    
    match self.i2c.read(&mut local_buf) {
      Ok(_) => {
        // Check if we got any real data (not all 0xFF)
        let has_data = local_buf.iter().any(|&b| b != 0xFF);
        
        if !has_data {
          return Ok(false);
        }
        
        let mut got_good_packet = false;
        
        // Parse the received data
        let mut it = self.parser.consume(&local_buf);
        loop {
          match it.next() {
            Some(Ok(packet)) => {
              got_good_packet = true;
              cb(packet);
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
      Err(e) => {
        Err(GPSError::I2C(e))
      }
    }
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
