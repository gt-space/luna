extern crate spidev;
use std::{io, fmt};
use std::io::prelude::*;
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SpiModeFlags};
use common::comm::gpio::{Gpio, Pin, PinMode::*, PinValue::*};
use std::{thread::sleep, time::Duration, io::{Error, ErrorKind}};

/// Error handling for Mag
#[derive(Debug)]
pub enum MagError {
    SPI(io::Error),
    InitializationError(String),
}

impl fmt::Display for MagError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
      match self {
        MagError::SPI(err) => write!(f, "SPI error: {}", err),
        MagError::InitializationError(msg) => write!(f, "Initialization Error: {}", msg),
      }
    }
  }

impl std::error::Error for MagError {}

impl From<io::Error> for MagError {
  fn from(err: io::Error) -> Self {
    MagError::SPI(err)
  }
}

/// Structure to hold magnetometer data
#[derive(Clone, Copy, Debug, Default)]
pub struct MagnetometerData {
    // X, Y, Z axes magnetic field in microTesla (µT)
    pub x: f32,
    pub y: f32,
    pub z: f32
}

/// Function to display data 
impl fmt::Display for MagnetometerData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f, "Magnetic Field: X: {:.4} gauss, Y: {:.4} gauss, Z: {:.4} gauss",
            self.x,
            self.y,
            self.z,
        )
    }
}

mod Registers {
    pub const WHO_AM_I: u8 = 0x0F;
    pub const OFFSET_X_REG_L_M: u8 = 0x05;
    pub const OFFSET_X_REG_H_M: u8 = 0x06;
    pub const OFFSET_Y_REG_L_M: u8= 0x07;
    pub const OFFSET_Y_REG_H_M: u8 = 0x08;
    pub const OFFSET_Z_REG_L_M: u8 = 0x09;
    pub const OFFSET_Z_REG_H_M: u8 = 0x0A;
    pub const CTRL_REG1: u8 = 0x20;
    pub const CTRL_REG2: u8 = 0x21;
    pub const CTRL_REG3: u8 = 0x22;
    pub const CTRL_REG4: u8 = 0x23;
    pub const CTRL_REG5: u8 = 0x24;
    pub const STATUS_REG: u8 = 0x27;
    pub const OUT_X_L: u8 = 0x28;
    pub const OUT_X_H: u8 = 0x29;
    pub const OUT_Y_L: u8 = 0x2A;
    pub const OUT_Y_H: u8 = 0x2B;
    pub const OUT_Z_L: u8 = 0x2C;
    pub const OUT_Z_H: u8 = 0x2D;
    pub const TEMP_OUT_L: u8 = 0x2E;
    pub const TEMP_OUT_H: u8 = 0x2F;
    pub const INT_CFG: u8 = 0x30;
    pub const INT_SRC: u8 = 0x31;
    pub const INT_THS_L: u8 = 0x32;
    pub const INT_THS_H: u8= 0x33;
}

const DEV_ID: u8 = 0x3D;

pub struct LIS3MDLDriver {
    spi : Spidev,
  
    drdy : Pin, 
  
    cs : Pin,

    // interrupt_pin: Pin 
}

impl LIS3MDLDriver {
    pub fn new(bus: &str, drdy: Pin, cs: Pin) -> Result<Self, MagError> {
        
        // Initialize SPI device 
        let mut spi = Spidev::open(bus).map_err(MagError::SPI)?;

        let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(500000)
        .mode(SpiModeFlags::SPI_MODE_0)
        .lsb_first(false)
        .build();
    
        println!("Set options");
    
        spi.configure(&options).map_err(MagError::SPI)?;
        println!("Configured spi");

        // Create instance of LIS3MDL driver
        println!("Initializing driver");
        let mut driver = LIS3MDLDriver {
            spi,
            drdy,
            cs,
        };
        // Configure pins
        driver.cs.mode(Output);
        driver.drdy.mode(Input);

        // Verify device id
        driver.enable_cs()?;
        let who_am_i = driver.read_register(Registers::WHO_AM_I)?;
        driver.disable_cs()?;
        if who_am_i != DEV_ID {
            return Err(MagError::InitializationError("Device ID does not match".into()));
        }

        // Initialize sensor CTRL registers
        driver.init()?;

        Ok(driver)

    }

    // Read single register
    fn read_register(&mut self, reg: u8) -> Result<u8, MagError> {
        // self.enable_cs()?;

        let addr = reg & 0x3F;
        let tx_buf: [u8; 2] = [addr | 0x80, 0]; // MSB set to 1 for read
        let mut rx_buf: [u8; 2] = [0x00, 0x00];
        println!("{:?}", tx_buf);
        println!("{:?}", rx_buf);
        let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
        self.spi.transfer(&mut transfer)?;
        
        // self.disable_cs()?;
        println!("{:?}", tx_buf);
        println!("{:?}", rx_buf);

        Ok(rx_buf[1])
    }

    /// Writes to a single register
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), MagError> {
        // self.enable_cs()?;

        let addr = reg & 0x3F;
        let tx_buf: [u8; 2] = [addr & 0x7F, value]; // MSB set to 0 for write 
        let mut transfer = SpidevTransfer::write(&tx_buf);
        self.spi.transfer(&mut transfer)?;
        
        // self.disable_cs()?;
        
        Ok(())
    }

    // Initialize config registers
    fn init(&mut self) -> Result<(), MagError> {
        self.enable_cs()?;
        self.write_register(Registers::CTRL_REG1, 0x5C)?; // 01011100
        self.write_register(Registers::CTRL_REG2, 0x00)?; // Set full-scale to +/-4 gauss 
        self.write_register(Registers::CTRL_REG3, 0x00)?;
        self.write_register(Registers::CTRL_REG4, 0x08)?; // 0000-1000
        self.write_register(Registers::CTRL_REG5, 0x00)?; 
        self.disable_cs()?;
        Ok(())
    }

    /// Enable CS (active low)
    fn enable_cs(&mut self) -> std::io::Result<()> {
        self.cs.digital_write(Low);
        Ok(())
    }
    
    /// Disable the chip select line (inactive high).
    fn disable_cs(&mut self) -> std::io::Result<()> {
        self.cs.digital_write(High);
        Ok(())
    }

    pub fn read_magnetic_field(&mut self) -> Result<MagnetometerData, MagError> {
        while self.drdy.digital_read() == Low {
        }

        self.enable_cs()?;
        let x_l = self.read_register(Registers::OUT_X_L)?;
        let x_h = self.read_register(Registers::OUT_X_H)?;
        let y_l = self.read_register(Registers::OUT_Y_L)?;
        let y_h = self.read_register(Registers::OUT_Y_H)?;
        let z_l = self.read_register(Registers::OUT_Z_H)?;
        let z_h = self.read_register(Registers::OUT_Z_L)?;
        self.disable_cs()?;

        let x = ((x_h as i16) << 8) | (x_l as i16);
        let y = ((y_h as i16) << 8) | (y_l as i16);
        let z = ((z_h as i16) << 8) | (z_l as i16);

        let scale = 0.000122; // 4 / 32767

        Ok(MagnetometerData {
            x: x as f32 * scale * 100.0, // Convert Gauss to µT (1 Gauss = 100 µT)
            y: y as f32 * scale * 100.0,
            z: z as f32 * scale * 100.0,
        })
    }
}