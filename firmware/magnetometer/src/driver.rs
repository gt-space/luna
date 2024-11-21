extern crate spidev;
use std::{io, fmt};
use std::io::prelude::*;
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SpiModeFlags};
use common::comm::gpio::{Gpio, Pin, PinMode::*, PinValue::*};
use std::{thread::sleep, time::Duration, io::{Error, ErrorKind}};

use internals::*;

use crate::internals::{self, DriverInternals};

const POWER_ON_START_UP_TIME : Duration = Duration::from_millis(100);
const RESET_DOWNTIME : Duration = Duration::from_millis(100);

/// Structure to hold magnetometer data
#[derive(Clone, Debug)]
pub struct MagnetometerData {
    pub magnetic_field: [f32; 3], // X, Y, Z magnetic field readings
}

// function to display data 
impl fmt::Display for MagnetometerData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f, "Magnetic Field: X: {:010.4} gauss, Y: {:010.4} gauss, Z: {:010.4} gauss",
            self.magnetic_field[0] as f32,
            self.magnetic_field[1] as f32,
            self.magnetic_field[2] as f32,
        )
    }
}

#[derive(Copy, Clone)]
pub enum Registers {
    WHO_AM_I_MAG,
	OFFSET_X_REG_L_M,
	OFFSET_X_REG_H_M,
	OFFSET_Y_REG_L_M,
	OFFSET_Y_REG_H_M,
	OFFSET_Z_REG_L_M,
	OFFSET_Z_REG_H_M,
    CTRL_REG1,
    CTRL_REG2,
    CTRL_REG3,
    CTRL_REG4,
    CTRL_REG5,
    STATUS_REG,
    OUT_X_L,
    OUT_X_H,
    OUT_Y_L,
    OUT_Y_H,
    OUT_Z_L,
    OUT_Z_H,
    TEMP_OUT_L,
	TEMP_OUT_H,
	INT_CFG,
	INT_SRC,
	INT_THS_L,
	INT_THS_H,
}

impl Registers {
    fn get_address(&self) -> u8 {
        match self {
            Registers::WHO_AM_I_MAG => 0x0F,
            Registers::OFFSET_X_REG_L_M => 0x05,
            Registers::OFFSET_X_REG_H_M => 0x06,
            Registers::OFFSET_Y_REG_L_M => 0x07,
            Registers::OFFSET_Y_REG_H_M => 0x08,
            Registers::OFFSET_Z_REG_L_M => 0x09,
            Registers::OFFSET_Z_REG_H_M => 0x0A,
            Registers::CTRL_REG1 => 0x20,
            Registers::CTRL_REG2 => 0x21,
            Registers::CTRL_REG3 => 0x22,
            Registers::CTRL_REG4 => 0x23,
            Registers::CTRL_REG5 => 0x24,
            Registers::STATUS_REG => 0x27,
            Registers::OUT_X_L => 0x28,
            Registers::OUT_X_H => 0x29,
            Registers::OUT_Y_L => 0x2A,
            Registers::OUT_Y_H => 0x2B,
            Registers::OUT_Z_L => 0x2C,
            Registers::OUT_Z_H => 0x2D,
            Registers::TEMP_OUT_L => 0x2E,
            Registers::TEMP_OUT_H => 0x2F,
            Registers::INT_CFG => 0x30,
            Registers::INT_SRC => 0x31,
            Registers::INT_THS_L => 0x32,
            Registers::INT_THS_H => 0x33,
        }
    }
}

pub struct LIS3MDLDriver<'a> {
    internals: DriverInternals<'a>,
    // config: ConfigValues,
}

impl<'a> LIS3MDLDriver<'a> {
    pub fn initialize(mut spi: Spidev, data_ready: Pin<'a>, nchip_select: Pin<'a>, interrupt_pin: Pin<'a>) -> Result<LIS3MDLDriver<'a>, Error> {
        println!("Initializing driver");
        let mut driver = LIS3MDLDriver {
            internals: DriverInternals::initialize(spi, data_ready, nchip_select, interrupt_pin)?,
        };
        println!("Driver created");

        sleep(POWER_ON_START_UP_TIME);
        driver.internals.disable_chip_select();
        println!("Chip select disabled");

        driver.write_config()?;
        println!("Config regs initialized");

        Ok(driver)
    }

    // Write to config registers
    fn write_config(&mut self) -> Result<(), Error> {
        self.write_mag_register(Registers::CTRL_REG1, 0x7C)?; // Set data rate and operating mode
        self.write_mag_register(Registers::CTRL_REG2, 0x00)?; // Set full-scale to +/-4 gauss
        self.write_mag_register(Registers::CTRL_REG3, 0x00)?; // Enable continuous conversion mode
        self.write_mag_register(Registers::CTRL_REG4, 0x0C)?; // Enable high perf mode for z-axis 
        self.write_mag_register(Registers::CTRL_REG5, 0x40)?; // Enable block data updates
        Ok(())
    }

    // Generate register address with flags
    fn generate_mag_address(reg: u8, read: bool, consecutive: bool) -> u8 {
        let mut address = reg & 0x3F; 
        if read {
            address |= 1 << 7;
        }
        if consecutive {
            address |= 1 << 6;
        }
        address
    }

    // Check if register is reserved
    fn ensure_mag_not_reserved(&self, reg: u8) -> Result<(), Error> {
        const MAG_RESERVED_REG_HASH: [u8; 52] = [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1,
                                                1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        if reg as usize >= MAG_RESERVED_REG_HASH.len() || MAG_RESERVED_REG_HASH[reg as usize] == 1 {
            Err(Error::new(ErrorKind::Other, "Reserved or invalid register"))
        } else {
            Ok(())
        }
    }

    // Write to a magnetometer register
    fn write_mag_register(&mut self, reg: Registers, value: u8) -> Result<(), Error> {
        self.ensure_mag_not_reserved(reg.get_address())?;
        let addr = Self::generate_mag_address(reg.get_address(), false, false);
        let mut buffer = [addr, value];
        
        self.internals.spi_write(&buffer)?;
        Ok(())
    }

    // Read single register 
    pub fn read_8_bit(&mut self, reg: Registers) -> Result<u8, Error> {
        self.ensure_mag_not_reserved(reg.get_address())?;
        
        let mut addr = Self::generate_mag_address(reg.get_address(), true, false);
        let tx_buf = [addr, 0];
        let mut rx_buf: [u8; 2] = [0; 2];
        self.internals.spi_transfer(&tx_buf, &mut rx_buf)?;

        Ok(rx_buf[1])
    }

    // Read 2 registers as 16 bit value 
    fn read_16_bit(&mut self, reg_upper: Registers, reg_lower: Registers) -> Result<u16, Error> {
        let upper = self.read_8_bit(reg_upper)?;
        let lower = self.read_8_bit(reg_lower)?;
        Ok((upper as u16) << 8 | (lower as u16))
    }

    pub fn read_magnetic_field(&mut self) -> Result<MagnetometerData, Error> {
        let x = self.read_16_bit(Registers::OUT_X_H, Registers::OUT_X_L)? as u16;
        let y = self.read_16_bit(Registers::OUT_Y_H, Registers::OUT_Y_L)? as u16;
        let z = self.read_16_bit(Registers::OUT_Z_H, Registers::OUT_Z_L)? as u16;

        Ok(MagnetometerData {
            magnetic_field: [
                0.00014 * x as f32, 
                0.00014 * y as f32, 
                0.00014 * z as f32,
            ],
        })
    }

    fn getStatusRegister(&mut self) -> Result<u8, Error> {
        return self.read_8_bit(Registers::STATUS_REG);
    }
    
    fn getXMagRaw(&mut self) -> Result<u16, Error> {
        Ok(self.read_16_bit(Registers::OUT_X_H, Registers::OUT_X_L)?)
    }
    
    fn getYMagRaw(&mut self) -> Result<u16, Error> {
        Ok(self.read_16_bit(Registers::OUT_Y_H, Registers::OUT_Y_L)?)
    }
    
    fn getZMagRaw(&mut self) -> Result<u16, Error> {
        Ok(self.read_16_bit(Registers::OUT_Z_H, Registers::OUT_Z_L)?)
    }
    
    fn getXMag(&mut self) -> Result<f32, Error> {
        let value = self.read_16_bit(Registers::OUT_X_H, Registers::OUT_X_L)? as f32;
        Ok(0.00014 * value)
    }
    
    fn getYMag(&mut self)-> Result<f32, Error> {
        let value = self.read_16_bit(Registers::OUT_Y_H, Registers::OUT_Y_L)? as f32;
        Ok(0.00014 * value)
    }
    
    fn getZMag(&mut self) -> Result<f32, Error> {
        let value = self.read_16_bit(Registers::OUT_Z_H, Registers::OUT_Z_L)? as f32;
        Ok(0.00014 * value)
    }

}