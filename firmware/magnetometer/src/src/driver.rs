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
    pub magnetic_field: [i16; 3], // X, Y, Z magnetic field readings
}

impl fmt::Display for MagnetometerData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f, "Magnetic Field: X: gauss, Y: gauss, Z: gauss",
            self.magnetic_field[0] as f32,
            self.magnetic_field[1] as f32,
            self.magnetic_field[2] as f32,
        )
    }
}

pub struct ConfigValues {
    control_reg_1: u8,
    control_reg_2: u8,
    control_reg_3: u8,
    control_reg_4: u8,
}

impl ConfigValues {
    fn default() -> ConfigValues {
        ConfigValues {
            control_reg_1: 0x70,  // default performance mode, ODR = 10Hz
            control_reg_2: 0x00,  // default full-scale 4 gauss
            control_reg_3: 0x00,  // default continuous conversion mode
            control_reg_4: 0x0C,  // default Z-axis performance mode
        }
    }

    fn read_all_values(&self, driver: &mut LIS3MDLDriver) -> Result<(), Error> {
        driver.read_8_bit(Registers::CTRL_REG1)?;
        driver.read_8_bit(Registers::CTRL_REG2)?;
        driver.read_8_bit(Registers::CTRL_REG3)?;
        driver.read_8_bit(Registers::CTRL_REG4)?;
        Ok(())
    }
}

#[derive(Copy, Clone)]
pub enum Registers {
    CTRL_REG1,
    CTRL_REG2,
    CTRL_REG3,
    CTRL_REG4,
    OUT_X_L,
    OUT_X_H,
    OUT_Y_L,
    OUT_Y_H,
    OUT_Z_L,
    OUT_Z_H,
}

impl Registers {
    fn get_address(&self) -> u8 {
        match self {
            Registers::CTRL_REG1 => 0x20,
            Registers::CTRL_REG2 => 0x21,
            Registers::CTRL_REG3 => 0x22,
            Registers::CTRL_REG4 => 0x23,
            Registers::OUT_X_L => 0x28,
            Registers::OUT_X_H => 0x29,
            Registers::OUT_Y_L => 0x2A,
            Registers::OUT_Y_H => 0x2B,
            Registers::OUT_Z_L => 0x2C,
            Registers::OUT_Z_H => 0x2D,
        }
    }
}

pub struct LIS3MDLDriver<'a> {
    internals: DriverInternals<'a>,
    config: ConfigValues,
}

impl<'a> LIS3MDLDriver<'a> {
    pub fn initialize(
        mut spi: Spidev,
        data_ready: Pin<'a>,
        nchip_select: Pin<'a>,
        interrupt_pin: Pin<'a>
    ) -> Result<LIS3MDLDriver<'a>, Error> {
        let mut driver = LIS3MDLDriver {
            internals: DriverInternals::initialize(spi, data_ready, nchip_select, interrupt_pin)?,
            config: ConfigValues::default(),
        };

        sleep(POWER_ON_START_UP_TIME);
        driver.internals.disable_chip_select();
        driver.reset()?;

        driver.config.read_all_values(&mut driver)?;

        Ok(driver)
    }

    fn read_8_bit(&mut self, reg: Registers) -> Result<u8, Error> {
        
      let mut tx_buf : [u8; 6] = [0; 6];
      tx_buf[1] = reg.get_address()[0];
  
      let mut rx_buf : [u8; 6] = [0; 6];
  
      self.internals.spi_transfer(&tx_buf, &mut rx_buf)?;
      
      Ok(i16::from_le_bytes([rx_buf[2], rx_buf[5]]))

    }

    pub fn read_magnetic_field(&mut self) -> Result<MagnetometerData, Error> {
      let x_l = self.read_8_bit(Registers::OUT_X_L)?;
      let x_h = self.read_8_bit(Registers::OUT_X_H)?;
      let y_l = self.read_8_bit(Registers::OUT_Y_L)?;
      let y_h = self.read_8_bit(Registers::OUT_Y_H)?;
      let z_l = self.read_8_bit(Registers::OUT_Z_L)?;
      let z_h = self.read_8_bit(Registers::OUT_Z_H)?;

      Ok(MagnetometerData {
          magnetic_field: [
              ((x_h as i16) << 8 | x_l as i16),
              ((y_h as i16) << 8 | y_l as i16),
              ((z_h as i16) << 8 | z_l as i16),
          ],
      })
    }

    // pub fn handle_interrupt(&mut self) -> Result<(), Error> {
    //   if self.internals.is_interrupt_triggered() {
          
    //   }
    //   Ok(())
    // }
}