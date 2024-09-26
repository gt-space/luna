use std::io;
use spidev::spidevioctl::{spi_ioc_transfer, SpidevTransfer};
use spidev::Spidev;

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

const ACCEPTABLE_IDAC_MAGNITUDES: [u16; 10] = [0, 10, 50, 100, 250, 500, 750, 1000, 1500, 2000];
const ACCEPTABLE_PGA_GAINS: [u8; 8] = [1, 2, 4, 8, 16, 32, 64, 128];

pub struct ADC {
  pub spidev: Spidev,
  current_reg_vals: [u8; 18],
}

pub enum ADCError {
  InvalidPositiveInputMux,
  InvalidNegativeInputMux,
  SamePositiveNegativeInputMux,
  InvalidIDACMag,
  InvalidIDAC1Mux,
  InvalidIDAC2Mux,
  SamePositiveNegativeIDACMux,
}

impl ADC {
  pub fn new(&mut self, spidev: Spidev) -> ADC {
      ADC {
        spidev: spidev,
        current_reg_vals: {
          match self.read_all_regs() {
            Ok(regs) => regs,
            Err(e) => {
              println!("Error in reading all initial register values");
              [0; 18] //default array for current register values
            }
          }
        }
      }
  }

  // Input Multiplexer Register Functions Below

  pub fn set_positive_input_channel(&mut self, channel: u8) -> Result<_, ADCError> {
    if (channel < 0 || channel > 5) {
      Err(ADCError::InvalidPositiveInputMux)
    }

    let negative_input_channel: u8 = self.current_reg_vals[INPMUX_LOCATION] >> 4;
    if (channel == negative_input_channel) {
      Err(ADCError::SamePositiveNegativeInputMux)
    }

    match channel {
      0 => {
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 4); // clear bit 4
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 5); // clear bit 5
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 6); // clear bit 6
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 7); // clear bit 7
      },

      1 => {
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 4; // set bit 4
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 5); // clear bit 5
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 6); // clear bit 6
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 7); // clear bit 7
      },

      2 => {
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 4); // clear bit 4
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 5; // set bit 5
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 6); // clear bit 6
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 7); // clear bit 7
      },

      3 => {
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 4); // clear bit 4
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 5); // clear bit 5
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 6; // set bit 6
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 7); // clear bit 7
      },

      4 => {
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 4); // clear bit 4
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 5); // clear bit 5
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 6); // clear bit 6
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 7; // set bit 7
      },

      5 => {
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 4; // set bit 4
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 5); // clear bit 5
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 6); // clear bit 6
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 7; // set bit 7
      }
    }
    self.write_reg(INPMUX_LOCATION, self.current_reg_vals[INPMUX_LOCATION]);
    Ok(())
  }


  pub fn set_negative_input_channel(&mut self, channel: u8) -> Result<_, ADCError> {
    if (channel < 0 || channel > 5) {
      Err(ADCError::InvalidNegativeInputMux)
    }

    let positive_input_channel: u8 = self.current_reg_vals[INPMUX_LOCATION] & 0x0F;
    if (channel == positive_input_channel) {
      Err(ADCError::SamePositiveNegativeInputMux)
    }

    match channel {
      0 => {
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 0); // clear bit 0
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 3); // clear bit 3
      },

      1 => {
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 0; // set bit 0
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 3); // clear bit 3
      },

      2 => {
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 0); // clear bit 0
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 1; // set bit 1
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 3); // clear bit 3
      },

      3 => {
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 0); // clear bit 0
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 2; // set bit 2
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 3); // clear bit 3
      },

      4 => {
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 0); // clear bit 0
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 3; // set bit 3
      },

      5 => {
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 0; // set bit 0
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[INPMUX_LOCATION] |= 1 << 3; // set bit 3
      }
    }
    self.write_reg(INPMUX_LOCATION, self.current_reg_vals[INPMUX_LOCATION]);
    Ok(())
  }

  // PGA Register Functions Below



  // Reference Register Functions Below

  pub fn enable_positive_reference_buffer(&mut self) {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 5); // clear bit 5
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }

  pub fn disable_positive_reference_buffer(&mut self) {
    self.current_reg_vals[REF_LOCATION] |= 1 << 5; // set bit 5
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }

  pub fn enable_negative_reference_buffer(&mut self) {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 4); // clear bit 4
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }

  pub fn disable_negative_reference_buffer(&mut self) {
    self.current_reg_vals[REF_LOCATION] |= 1 << 4; // set bit 4
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }
  
  pub fn set_ref_input_ref0(&mut self) {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 2); // clear bit 2
    self.current_reg_vals[REF_LOCATION] &= !(1 << 3); // clear bit 2
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }

  pub fn set_ref_input_ref1(&mut self) {
    self.current_reg_vals[REF_LOCATION] |= 1 << 2; // set bit 2
    self.current_reg_vals[REF_LOCATION] &= !(1 << 3); // clear bit 3
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }

  pub fn set_ref_input_internal_2v5_ref(&mut self) {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 2); // clear bit 2
    self.current_reg_vals[REF_LOCATION] |= 1 << 3; // set bit 3
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }

  pub fn disable_internal_voltage_reference(&mut self) {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 0); // clear bit 0
    self.current_reg_vals[REF_LOCATION] &= !(1 << 1); // clear bit 1
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }

  pub fn enable_internal_voltage_reference_off_pwr_down(&mut self) {
    self.current_reg_vals[REF_LOCATION] |= 1 << 0; // set bit 0
    self.current_reg_vals[REF_LOCATION] &= !(1 << 1); // clear bit 1
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }

  pub fn enable_internal_voltage_reference_on_pwr_down(&mut self) {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 0); // clear bit 0
    self.current_reg_vals[REF_LOCATION] |= 1 << 1; // set bit 1
    self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION]);
  }

  // IDACMAG functions below

  pub fn set_idac_magnitude(&mut self, mag: u16) -> Result<_, ADCError> {
    if !ACCEPTABLE_IDAC_MAGNITUDES.contains(&mag) {
      Err(ADCError::InvalidIDACMag)
    }
    match mag {
      0 => {
        // call disable function or set to 0?
      },

      10 => {
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
      },

      50 => {
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 0); // clear bit 0
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 1; // set bit 1
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
      },

      100 => {
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 1; // set bit 1
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
      },

      250 => {
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 0); // clear bit 0
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 2; // set bit 2
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
      },

      500 => {
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 2; // set bit 2
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
      },

      750 => {
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 0); // clear bit 0
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 1; // set bit 1
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 2; // set bit 2
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
      },

      1000 => {
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 1; // set bit 1
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 2; // set bit 2
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
      },

      1500 => {
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 0); // clear bit 0
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 3; // set bit 3
      },

      2000 => {
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
        self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
        self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 3; // set bit 3
      }
    }
    
    self.write_reg(IDACMAG_LOCATION, self.write_reg(reg, data));
    Ok(())
  }

  // IDACMUX functions below

  pub fn set_idac1_output_channel(&mut self, channel: u8) {
    if (channel < 0 || channel > 5) {
      Err(ADCError::)
    }
  }

  pub fn set_idac2_output_channel(&mut self, channel: u8) {

  }


    /* FOR THE FOLLOWING SPI COMMUNICATION COMMANDS BELOW
    For a read_write transfer, tx_buf is used to send the command and rx_buf
    is used to receive the data. For read_write, tx_buf and rx_buf must be
    of equal size and the kernel automatically modified rx_buf, which is why
    a mutable reference is passed to it. For the write_reg function it must be
    explored as to if providing an rx_buf will do anything.
     */

  pub fn read_data(&mut self) -> Result<i16, io::Error> {
    /*
    old SAM code received data in 3 byte buffer even though CRC and STATUS
    bytes were disabled which leaves for 2 bytes of data. The tx_buf just
    needs to store one byte so going to investigate why this was done and if
    not needed will reduce tx_buf and rx_buf sizes to 2 bytes
     */
    let tx_buf: [u8; 3] = [0x12, 0x00, 0x00];
    let mut rx_buf: [u8; 3] = [0x00, 0x00, 0x00];
    let mut transfer: spi_ioc_transfer<'_, '_> = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    match result {
      Ok(_) => Ok(((rx_buf[1] as i16) << 8) | (rx_buf[2] as i16)), // confirm these array indices are correct
      Err(e) => {
        Err(e)
      }
    }
  }

  pub fn read_reg(&mut self, reg: u8) -> Result<u8, io::Error> {
    let tx_buf: [u8; 2] = [0x20 | reg, 0x00];
    let mut rx_buf: [u8; 2] = [0x00, 0x00];
    let mut transfer: spi_ioc_transfer<'_, '_> = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    match result {
      Ok(_) => Ok(rx_buf[0]), // test if value goes to index 0 or 1
      Err(e) => {
        Err(e)
      }
    }
  }

  pub fn read_all_regs(&mut self) -> Result<[u8; 18], io::Error> {
    let mut tx_buf: [u8; 18] = [0; 18];
    let mut rx_buf: [u8; 18] = [0; 18];
    tx_buf[0] = 0x20;
    tx_buf[1] = 17;
    let mut transfer: spi_ioc_transfer<'_, '_> = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    match result {
      Ok(_) => Ok(rx_buf),
      Err(e) => {
        Err(e)
      }
    }
  }

  pub fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), io::Error> {
    // TODO: if an rx buffer is sent, look into what data it holds if modified
    let tx_buf: [u8; 3] = [0x40 | reg, 0x00, data];
    let mut transfer: spi_ioc_transfer<'_, '_> = SpidevTransfer::write(&tx_buf);
    self.spidev.transfer(&mut transfer) // no need for extra error handling as nothing is returned in good case
  }
}