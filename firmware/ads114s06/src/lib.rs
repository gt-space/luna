/*
TODO: If reset event happens figure out how to handle current reg vals
Redo all of the bitwise stuff
Make necessary getter functions for each relevant section of each relevant reg
 */

use std::io;
use spidev::spidevioctl::{spi_ioc_transfer, SpidevTransfer};
use spidev::Spidev;

// bit resolution
const ADC_RESOLUTION = 16;

// Register locations
const ID_LOCATION : usize = 0;
const STATUS_LOCATION : usize = 1;
const INPMUX_LOCATION : usize = 2;
const PGA_LOCATION : usize = 3;
const DATARATE_LOCATION : usize = 4;
const REF_LOCATION : usize = 5;
const IDACMAG_LOCATION : usize = 6;
const IDACMUX_LOCATION : usize = 7;
const VBIAS_LOCATION : usize = 8;
const SYS_LOCATION : usize = 9;
const RESERVED0_LOCATION : usize = 10;
const OFCAL0_LOCATION : usize = 11;
const OFCAL1_LOCATION : usize = 12;
const RESERVED1_LOCATION : usize = 13;
const FSCAL0_LOCATION : usize = 14;
const FSCAL1_LOCATION : usize = 15;
const GPIODAT_LOCATION : usize = 16;
const GPIOCON_LOCATION : usize = 17;

#[repr(usize)]
pub enum RegisterLocation {
  ID = 0,
  STATUS = 1,
  INPMUX = 2,
  PGA = 3,
  DATARATE = 4,
  REF = 5,
  IDACMAG = 6,
  IDACMUX = 7,
  VBIAS = 8,
  SYS = 9,
  RESERVED0 = 10,
  OFCAL0 = 11,
  OFCAL1 = 12,
  RESERVED1 = 13,
  FSCAL0 = 14,
  FSCAL1 = 15,
  GPIODAT = 16,
  GPIOCON = 17,
}

pub enum ADCError {
  InvalidPositiveInputMux,
  InvalidNegativeInputMux,
  SamePositiveNegativeInputMux,
  InvalidPGAGain,
  InvalidProgrammableConversionDelay,
  InvalidDataRate,
  InvalidIDACMag,
  InvalidIDAC1Mux,
  InvalidIDAC2Mux,
  SameIDAC1IDAC2Mux,
  InvalidInternalTempSensePGAGain,
  SPI(io::Error)
}

impl From<io::Error> for ADCError {
  fn from(err: io::Error) -> ADCError {
    ADCError::SPI(err)
  }
}

#[derive(Clone, Copy)]
pub enum Channel {
  AIN0 = 0b0000,
  AIN1 = 0b0001,
  AIN2 = 0b0010,
  AIN3 = 0b0011,
  AIN4 = 0b0100,
  AIN5 = 0b0101,
  AINCOM = 0b1100,
}

pub struct ADC {
  resolution: u8
  spidev: Spidev,
  current_reg_vals: [u8; 18],
}

impl ADC {
  pub fn new(&mut self, spidev: Spidev) -> ADC {
      ADC {
        resolution: 16,
        spidev: spidev,
        current_reg_vals: {
          match self.spi_read_all_regs() {
            Ok(regs) => regs,
            Err(_) => {
              println!("Error in reading all initial register values");
              [0; 18] //default array for current register values
            }
          }
        }
      }
  }

  pub fn get_all_regs(&self) -> &[u8; 18] {
    &self.current_reg_vals
  }
  
  pub fn get_id_reg(&self) -> u8 {
    self.current_reg_vals[ID_LOCATION]
  }
  
  pub fn get_status_reg(&self) -> u8 {
    self.current_reg_vals[STATUS_LOCATION]
  }
  
  pub fn get_inpmux_reg(&self) -> u8 {
    self.current_reg_vals[INPMUX_LOCATION]
  }
  
  pub fn get_pga_reg(&self) -> u8 {
    self.current_reg_vals[PGA_LOCATION]
  }
  
  pub fn get_datarate_reg(&self) -> u8 {
    self.current_reg_vals[DATARATE_LOCATION]
  }
  
  pub fn get_ref_reg(&self) -> u8 {
    self.current_reg_vals[REF_LOCATION]
  }
  
  pub fn get_idacmag_reg(&self) -> u8 {
    self.current_reg_vals[IDACMAG_LOCATION]
  }
  
  pub fn get_idacmux_reg(&self) -> u8 {
    self.current_reg_vals[IDACMUX_LOCATION]
  }
  
  pub fn get_vbias_reg(&self) -> u8 {
    self.current_reg_vals[VBIAS_LOCATION]
  }
  
  pub fn get_sys_reg(&self) -> u8 {
    self.current_reg_vals[SYS_LOCATION]
  }
  
  pub fn get_reserved0_reg(&self) -> u8 {
    self.current_reg_vals[RESERVED0_LOCATION]
  }
  
  pub fn get_ofcal0_reg(&self) -> u8 {
    self.current_reg_vals[OFCAL0_LOCATION]
  }
  
  pub fn get_ofcal1_reg(&self) -> u8 {
    self.current_reg_vals[OFCAL1_LOCATION]
  }
  
  pub fn get_reserved1_reg(&self) -> u8 {
    self.current_reg_vals[RESERVED1_LOCATION]
  }
  
  pub fn get_fscal0_reg(&self) -> u8 {
    self.current_reg_vals[FSCAL0_LOCATION]
  }
  
  pub fn get_fscal1_reg(&self) -> u8 {
    self.current_reg_vals[FSCAL1_LOCATION]
  }
  
  pub fn get_gpiodat_reg(&self) -> u8 {
    self.current_reg_vals[GPIODAT_LOCATION]
  }
  
  pub fn get_gpiocon_reg(&self) -> u8 {
    self.current_reg_vals[GPIOCON_LOCATION]
  }

  // Input Multiplexer Register Functions Below

  pub fn set_positive_input_channel(&mut self, channel: Channel) -> Result<(), ADCError> {
    if (channel as u8) == self.get_negative_input_channel() {
      return Err(ADCError::SamePositiveNegativeInputMux)
    }

    let clear = 0b00001111;
    self.current_reg_vals[INPMUX_LOCATION] &= clear;
    self.current_reg_vals[INPMUX_LOCATION] |= (channel as u8) << 4;
    self.spi_write_reg(INPMUX_LOCATION, self.current_reg_vals[INPMUX_LOCATION])?;
    Ok(())
  }

  pub fn set_negative_input_channel(&mut self, channel: Channel) -> Result<(), ADCError> {
    if (channel as u8) == self.get_positive_input_channel() {
      return Err(ADCError::SamePositiveNegativeInputMux)
    }

    let clear = 0b11110000;
    self.current_reg_vals[INPMUX_LOCATION] &= clear;
    self.current_reg_vals[INPMUX_LOCATION] |= channel as u8;
    self.spi_write_reg(INPMUX_LOCATION, self.current_reg_vals[INPMUX_LOCATION])?;
    Ok(())
  }

  fn get_positive_input_channel(&self) -> u8 {
    self.get_inpmux_reg() >> 4
  }

  fn get_negative_input_channel(&self) -> u8 {
    self.get_inpmux_reg() & 0b00001111
  }

  // PGA Register Functions Below

  pub fn enable_pga(&mut self) -> Result<(), ADCError> {
    // clear bits 4 and 3, then set bit 3
    let clear: u8 = 0b11100111;
    let set: u8 = 0b00001000;
    self.current_reg_vals[PGA_LOCATION] &= clear;
    self.current_reg_vals[PGA_LOCATION] |= set;
    self.spi_write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION])?;
    Ok(())
  }

  pub fn disable_pga(&mut self) -> Result<(), ADCError> {
    // clear bits 4 and 3
    let clear: u8 = 0b11100111;
    self.current_reg_vals[PGA_LOCATION] &= clear;
    self.set_pga_gain(1)?;
    Ok(())
  }

  pub fn set_pga_gain(&mut self, gain: u8) -> Result<(), ADCError> {
    // clear bits 2-0
    let clear: u8 = 0b11111000;
    let set: u8 = match gain {
      1 => 0,

      2 => 0b00000001,

      4 => 0b00000010,

      8 => 0b00000011,

      16 => 0b00000100,

      32 => 0b00000101,

      64 => 0b00000110,

      128 => 0b00000111,

      _ => return Err(ADCError::InvalidPGAGain)
    };

    self.current_reg_vals[PGA_LOCATION] &= clear;
    self.current_reg_vals[PGA_LOCATION] |= set;
    self.spi_write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION])?;
    Ok(())
  }

  pub fn get_pga_gain(&self) -> u8 {
    1 << (self.get_pga_reg() & 0b00000111)
  }

  pub fn set_programmable_conversion_delay(&mut self, delay: u16) -> Result<(), ADCError> {
    let clear: u8 = 0b00011111;
    let set: u8 = match delay {
      14 => 0,

      25 => 0b00100000,

      64 => 0b01000000,

      256 => 0b01100000,

      1024 => 0b10000000,

      2048 => 0b10100000,

      4096 => 0b11000000,

      1 => 0b11100000,

      _ => return Err(ADCError::InvalidProgrammableConversionDelay)
    };

    self.current_reg_vals[PGA_LOCATION] &= clear;
    self.current_reg_vals[PGA_LOCATION] |= set;
    self.spi_write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION])?;
    Ok(())
  }

  // Data Rate Register Functions Below

  pub fn enable_global_chop(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] |= 1 << 7; // set bit 7
    self.spi_write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
    Ok(())
  }

  pub fn disable_global_chop(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 7); // clear bit 7
    self.spi_write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
    Ok(())
  }

  pub fn enable_internal_clock(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 6); // clear bit 6
    self.spi_write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
    Ok(())
  }

  pub fn enable_continious_conversion_mode(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 5); // clear bit 5
    self.spi_write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
    Ok(())
  }

  pub fn enable_single_shot_conversion_mode(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] |= 1 << 5; // set bit 5
    self.spi_write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
    Ok(())
  }

  pub fn enable_sinc_filter(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 4); // clear bit 4
    self.spi_write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
    Ok(())
  }

  pub fn enable_low_latency_filter(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] |= 1 << 4; // set bit 4
    self.spi_write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
    Ok(())
  }

  pub fn set_data_rate(&mut self, rate: f64) -> Result<(), ADCError> {
    let clear: u8 = 0b11110000;
    let set: u8 = match rate {
      2.5 => 0b00000000,

      5.0 => 0b00000001,

      10.0 => 0b00000010,

      16.6 => 0b00000011,

      20.0 => 0b00000100,

      50.0 => 0b00000101,

      60.0 => 0b00000110,

      100.0 => 0b00000111,

      200.0 => 0b00001000,

      400.0 => 0b00001001,

      800.0 => 0b00001010,

      1000.0 => 0b00001011,

      2000.0 => 0b00001100,

      4000.0 => 0b00001101,

      _ => return Err(ADCError::InvalidDataRate)
    };

    self.current_reg_vals[DATARATE_LOCATION] &= clear;
    self.current_reg_vals[DATARATE_LOCATION] |= set;
    self.spi_write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
    Ok(())
  }

  // Reference Register Functions Below

  pub fn enable_positive_reference_buffer(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 5); // clear bit 5
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }

  pub fn disable_positive_reference_buffer(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[REF_LOCATION] |= 1 << 5; // set bit 5
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }

  pub fn enable_negative_reference_buffer(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 4); // clear bit 4
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }

  pub fn disable_negative_reference_buffer(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[REF_LOCATION] |= 1 << 4; // set bit 4
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }
  
  pub fn set_ref_input_ref0(&mut self) -> Result<(), ADCError> {
    let clear = 0b11110011;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }

  pub fn set_ref_input_ref1(&mut self) -> Result<(), ADCError> {
    let clear = 0b11110011;
    let set = 0b00000100;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.current_reg_vals[REF_LOCATION] |= set;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }

  pub fn set_ref_input_internal_2v5_ref(&mut self) -> Result<(), ADCError> {
    let clear = 0b11110011;
    let set = 0b00001000;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.current_reg_vals[REF_LOCATION] |= set;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }

  pub fn disable_internal_voltage_reference(&mut self) -> Result<(), ADCError> {
    let clear = 0b11111100;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }

  pub fn enable_internal_voltage_reference_off_pwr_down(&mut self) -> Result<(), ADCError> {
    let clear = 0b11111100;
    let set = 0b00000001;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.current_reg_vals[REF_LOCATION] |= set;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }

  pub fn enable_internal_voltage_reference_on_pwr_down(&mut self) -> Result<(), ADCError> {
    let clear = 0b11111100;
    let set = 0b00000010;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.current_reg_vals[REF_LOCATION] |= set;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
    Ok(())
  }

  // IDACMAG functions below

  pub fn set_idac_magnitude(&mut self, mag: u16) -> Result<(), ADCError> {

    let clear: u8 = 0b11110000;
    let set = match mag {
      0 => 0,

      10 => 0b00000001,

      50 => 0b00000010,

      100 => 0b00000011,

      250 => 0b00000100,

      500 => 0b00000101,

      750 => 0b00000110,

      1000 => 0b00000111,

      1500 => 0b00001000,

      2000 => 0b00001001,

      _ => return Err(ADCError::InvalidIDACMag)
    };
    
    self.current_reg_vals[IDACMAG_LOCATION] &= clear;
    self.current_reg_vals[IDACMAG_LOCATION] |= set;
    self.spi_write_reg(IDACMAG_LOCATION, self.current_reg_vals[IDACMAG_LOCATION])?;
    Ok(())
  }

  pub fn get_idac_magnitude(&self) -> u16 {
    match self.get_idacmag_reg() & 0b00001111 {
      0b0000 => 0,

      0b0001 => 10,

      0b0010 => 50,

      0b0011 => 100,

      0b0100 => 250,

      0b0101 => 500,

      0b0110 => 750,

      0b0111 => 1000,

      0b1000 => 1500,

      0b1001 => 2000,

      _ => 0
    }
  }

  // IDACMUX functions below

  pub fn enable_idac1_output_channel(&mut self, channel: Channel) -> Result<(), ADCError> {
    if (channel as u8) == self.get_idac2_output_channel() {
      return Err(ADCError::SameIDAC1IDAC2Mux)
    }

    let clear: u8 = 0b11110000;
    self.current_reg_vals[IDACMUX_LOCATION] &= clear;
    self.current_reg_vals[IDACMUX_LOCATION] |= channel as u8;
    self.spi_write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])?;
    Ok(())
  }

  pub fn enable_idac2_output_channel(&mut self, channel: Channel) -> Result<(), ADCError> {
    if (channel as u8) == self.get_idac1_output_channel() {
      return Err(ADCError::SameIDAC1IDAC2Mux)
    }

    let clear = 0b00001111;
    self.current_reg_vals[IDACMUX_LOCATION] &= clear;
    self.current_reg_vals[IDACMUX_LOCATION] |= channel as u8;
    self.spi_write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])?;
    Ok(())
  }

  pub fn disable_idac1(&mut self) -> Result<(), ADCError> {
    let set: u8 = 0b11110000;
    self.current_reg_vals[IDACMUX_LOCATION] |= set;
    self.spi_write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])?;
    Ok(())
  }

  pub fn disable_idac2(&mut self) -> Result<(), ADCError> {
    let set: u8 = 0b00001111;
    self.current_reg_vals[IDACMUX_LOCATION] |= set;
    self.spi_write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])?;
    Ok(())
  }

  pub fn get_idac1_output_channel(&self) -> u8 {
    self.get_idacmux_reg() >> 4
  }

  pub fn get_idac2_output_channel(&self) -> u8 {
    self.get_idacmux_reg() & 0b00001111
  }

  // VBIAS Register Functions

  pub fn disable_vbias(&mut self) -> Result<(), ADCError> {
    // sets VBIAS to (AVDD + AVSS) / 2 and disconnects VBIAS from all AIN(X)
    self.current_reg_vals[VBIAS_LOCATION] = 0;
    self.spi_write_reg(VBIAS_LOCATION, self.current_reg_vals[VBIAS_LOCATION])?;
    Ok(())
  }

  // System Control Register Functions

  pub fn enable_internal_temp_sensor(&mut self, pga_gain: u8) -> Result<(), ADCError> {
    match pga_gain {
      1 => self.set_pga_gain(1)?,
      2 => self.set_pga_gain(2)?,
      4 => self.set_pga_gain(4)?,
      _ => return Err(ADCError::InvalidInternalTempSensePGAGain)
    }
    self.enable_pga()?;

    let clear = 0b00011111;
    let set = 0b01000000;
    self.current_reg_vals[SYS_LOCATION] &= clear;
    self.current_reg_vals[SYS_LOCATION] |= set;
    self.spi_write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])?;
    Ok(())
  }


    /* FOR THE FOLLOWING SPI COMMUNICATION COMMANDS BELOW
    For a read_write transfer, tx_buf is used to send the command and rx_buf
    is used to receive the data. For read_write, tx_buf and rx_buf must be
    of equal size and the kernel automatically modified rx_buf, which is why
    a mutable reference is passed to it. For the write_reg function it must be
    explored as to if providing an rx_buf will do anything.
     */
  
    pub fn spi_no_operation(&mut self) -> Result<(), ADCError> {
    let tx_buf: [u8; 1] = [0x00];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    match self.spidev.transfer(&mut transfer) {
      Ok(_) => Ok(()),
      Err(e) => ADCError::SPI(e)
    }
  }

  pub fn spi_wake_up_from_pwr_down_mode(&mut self) -> Result<(), ADCError> {
    let tx_buf: [u8; 1] = [0x02];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    match self.spidev.transfer(&mut transfer) {
      Ok(_) => Ok(()),
      Err(e) => ADCError::SPI(e)
    }
  }

  pub fn spi_enter_pwr_down_mode(&mut self) -> Result<(), ADCError> {
    let tx_buf: [u8; 1] = [0x04];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    match self.spidev.transfer(&mut transfer) {
      Ok(_) => Ok(()),
      Err(e) => ADCError::SPI(e)
    }
  }

  pub fn spi_reset(&mut self) -> Result<(), ADCError> {
    let tx_buf: [u8; 1] = [0x06];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    match self.spidev.transfer(&mut transfer) {
      Ok(_) => Ok(()),
      Err(e) => ADCError::SPI(e)
    }
  }

  pub fn spi_start_conversion(&mut self) -> Result<(), ADCError> {
    let tx_buf: [u8; 1] = [0x08];
    let mut transfer = SpidevTransfer::read(&tx_buf);
    match self.spidev.transfer(&mut transfer) {
      Ok(_) => Ok(()),
      Err(e) => ADCError::SPI(e)
    }
  }

  pub fn spi_stop_conversion(&mut self) -> Result<(), ADCError> {
    let tx_buf: [u8; 1] = [0x0A];
    let mut transfer = SpidevTransfer::read(&tx_buf);
    match self.spidev.transfer(&mut transfer) {
      Ok(_) => Ok(()),
      Err(e) => ADCError::SPI(e)
    }
  }

  pub fn spi_read_data(&mut self) -> Result<i16, ADCError> {
    /*
    old SAM code received data in 3 byte buffer even though CRC and STATUS
    bytes were disabled which leaves for 2 bytes of data. The tx_buf just
    needs to store one byte so going to investigate why this was done and if
    not needed will reduce tx_buf and rx_buf sizes to 2 bytes
     */
    let tx_buf: [u8; 2] = [0x12, 0x00];
    let mut rx_buf: [u8; 2] = [0x00, 0x00];
    let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    match result {
      Ok(_) => Ok(((rx_buf[0] as i16) << 8) | (rx_buf[1] as i16)), // confirm these array indices are correct
      Err(e) => ADCError::SPI(e)
    }
  }

  pub fn spi_read_reg(&mut self, reg: u8) -> Result<u8, ADCError> {
    let tx_buf: [u8; 2] = [0x20 | reg, 0x00];
    let mut rx_buf: [u8; 2] = [0x00, 0x00];
    let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    match self.spidev.transfer(&mut transfer) {
      Ok(_) => Ok(rx_buf[0]), // test if value goes to index 0 or 1
      Err(e) => ADCError::SPI(e)
    }
  }

  pub fn spi_read_all_regs(&mut self) -> Result<[u8; 18], ADCError> {
    let mut tx_buf: [u8; 18] = [0; 18];
    let mut rx_buf: [u8; 18] = [0; 18];
    tx_buf[0] = 0x20;
    tx_buf[1] = 17;
    let mut transfer: spi = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    match self.spidev.transfer(&mut transfer) {
      Ok(_) => Ok(rx_buf),
      Err(e) => ADCError::SPI(e)
    }
  }

  pub fn spi_write_reg(&mut self, reg: usize, data: u8) -> Result<(), ADCError> {
    // TODO: if an rx buffer is sent, look into what data it holds if modified
    let tx_buf: [u8; 3] = [0x40 | (reg as u8), 0x00, data];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    self.spidev.transfer(&mut transfer) // no need for extra error handling as nothing is returned in good case
  }


    /*
  GND is often used as negative end of differential measurement so it looks
  like a single ended measurement
   */
  pub fn calculate_differential_measurement(&self, code: i16) -> f64 {
    /*
    The voltage seen by the ADC is the digital output code multiplied
    by the smallest voltage difference produced by a change of 1 in the
    digital output code
     */
    // max_voltage is 2.5V
    let lsb: f64 = (2.0 * 2.5) / ((1 << (self.get_pga_gain() + ADC_RESOLUTION)) as f64);
    (code as f64) * lsb
  }

  pub fn calculate_four_wire_rtd_resistance(code: i16, ref_resistance: f64) -> f64 {
    /*
    The beauty of a ratiometric measurement is that the output code is
    proportional to a ratio between the input voltage and reference voltage.
    The two resistances creating these voltages are in series so with ohms law
    you can cancel out the current because current is the same in series and
    you are left with a ratio proportional to two resistances
     */
    (code as f64) * 2.0 * ref_resistance / ((1 << (self.get_pga_gain() + ADC_RESOLUTION)) as f64)
  }
}