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

// const ACCEPTABLE_IDAC_MAGNITUDES: [u16; 10] = [0, 10, 50, 100, 250, 500, 750, 1000, 1500, 2000];
// const ACCEPTABLE_PGA_GAINS: [u8; 8] = [1, 2, 4, 8, 16, 32, 64, 128];
// const ACCEPTABLE_PROGRAMMABLE_CONVERSION_DELAYS: [u8; 8] = [14, 25, 64, 256, 1024, 2048, 4096, 1];

pub struct ADC {
  pub spidev: Spidev,
  current_reg_vals: [u8; 18],
}

pub enum ADCUserError {
  InvalidPositiveInputMux,
  InvalidNegativeInputMux,
  SamePositiveNegativeInputMux,
  InvalidPGAGain,
  InvalidProgrammableConversionDelay,
  InvalidDataRate,
  InvalidIDACMag,
  InvalidIDAC1Mux,
  InvalidIDAC2Mux,
  SameIDAC1Idac2Mux,
  InvalidInternalTempSensePGAGain,
}

pub enum ADCError {
  User(ADCUserError),
  System(io::Error),
}

impl From<ADCUserError> for ADCError {
  fn from(err: ADCUserError) -> ADCError {
    ADCError::User(err)
  }
}

impl From<io::Error> for ADCError {
  fn from(err: io::Error) -> ADCError {
    ADCError::System(err)
  }
}

// This is constructed because there are int and float data rate values
pub enum DataRate {
  UInt(u16),
  UFloat(f32),
}

// const ACCEPTABLE_DATA_RATES[DataRate; 15] = [DataRate::UFloat(2.5),
//   DataRate::UInt(5), DataRate::UInt(10), DataRate::UFloat(16.6),
//   DataRate::UInt(20), DataRate::UInt(50), DataRate::UInt(60),
//   DataRate::UInt(100), DataRate::UInt(200), DataRate::UInt(400),
//   DataRate::UInt(800), DataRate::UInt(1000), DataRate::UInt(2000),
//   DataRate::UInt(4000)];

impl ADC {
  pub fn new(&mut self, spidev: Spidev) -> ADC {
      ADC {
        spidev: spidev,
        current_reg_vals: {
          match self.read_all_regs() {
            Ok(regs) => regs,
            Err(_) => {
              println!("Error in reading all initial register values");
              [0; 18] //default array for current register values
            }
          }
        }
      }
  }

  // Input Multiplexer Register Functions Below

  pub fn set_positive_input_channel(&mut self, channel: u8) -> Result<(), ADCError> {
    let negative_input_channel: u8 = self.current_reg_vals[INPMUX_LOCATION] >> 4;
    if channel == negative_input_channel {
      return Err(ADCUserError::SamePositiveNegativeInputMux.into())
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

      _ => return Err(ADCUserError::InvalidPositiveInputMux.into())
    }

    self.write_reg(INPMUX_LOCATION, self.current_reg_vals[INPMUX_LOCATION])?;
    Ok(())
  }


  // pub fn set_negative_input_channel(&mut self, channel: u8) -> Result<(), ADCError> {
  //   let positive_input_channel: u8 = self.current_reg_vals[INPMUX_LOCATION] & 0x0F;
  //   if channel == positive_input_channel {
  //     return Err(ADCError::User(ADCUserError::SamePositiveNegativeInputMux))
  //   }

  //   match channel {
  //     0 => {
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     1 => {
  //       self.current_reg_vals[INPMUX_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     2 => {
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[INPMUX_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     3 => {
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[INPMUX_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     4 => {
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[INPMUX_LOCATION] |= 1 << 3; // set bit 3
  //     },

  //     5 => {
  //       self.current_reg_vals[INPMUX_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[INPMUX_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[INPMUX_LOCATION] |= 1 << 3; // set bit 3
  //     }

  //     _ => return Err(ADCError::User(ADCUserError::InvalidNegativeInputMux))
  //   }

  //   self.write_reg(INPMUX_LOCATION, self.current_reg_vals[INPMUX_LOCATION])?;
  // }

  // // PGA Register Functions Below

  // pub fn enable_pga(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[PGA_LOCATION] &= !(1 << 3); // clear bit 3
  //   self.current_reg_vals[PGA_LOCATION] &= !(1 << 4); // clear bit 4
  //   self.write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION])?;
  // }

  // pub fn disable_pga(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[PGA_LOCATION] |= 1 << 3; // set bit 3
  //   self.current_reg_vals[PGA_LOCATION] &= !(1 << 4); // clear bit 4
  //   self.set_pga_gain(1);
  // }

  // pub fn set_pga_gain(&mut self, gain: u8) -> Result<(), ADCError> {
  //   match gain {
  //     1 => {
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 2); // clear bit 2
  //     },

  //     2 => {
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 2); // clear bit 2
  //     },

  //     4 => {
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 2); // clear bit 2
  //     },

  //     8 => {
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 2); // clear bit 2
  //     },

  //     16 => {
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 2; // set bit 2
  //     },

  //     32 => {
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 2; // set bit 2
  //     },

  //     64 => {
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 2; // set bit 2
  //     },

  //     128 => {
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 2; // set bit 2
  //     }

  //     _ => return Err(ADCError::User(ADCUserError::InvalidPGAGain))
  //   }

  //   self.write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION]);
  // }

  // pub fn set_programmable_conversion_delay(&mut self, delay: u16) -> Result<(), ADCError> {
  //   match delay {
  //     14 => {
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 5); // clear bit 5
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 6); // clear bit 6
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 7); // clear bit 7
  //     },

  //     25 => {
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 5; // set bit 5
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 6); // clear bit 6
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 7); // clear bit 7
  //     },

  //     64 => {
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 5); // clear bit 5
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 6; // set bit 6
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 7); // clear bit 7
  //     },

  //     256 => {
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 5; // set bit 5
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 6; // set bit 6
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 7); // clear bit 7
  //     },

  //     1024 => {
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 5); // clear bit 5
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 6); // clear bit 6
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 7; // set bit 7
  //     },

  //     2048 => {
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 5; // set bit 5
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 6); // clear bit 6
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 7; // set bit 7
  //     },

  //     4096 => {
  //       self.current_reg_vals[PGA_LOCATION] &= !(1 << 5); // clear bit 5
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 6; // set bit 6
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 7; // set bit 7
  //     },

  //     1 => {
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 5; // set bit 5
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 6; // set bit 6
  //       self.current_reg_vals[PGA_LOCATION] |= 1 << 7; // set bit 7
  //     }

  //     _ => return Err(ADCError::User(ADCUserError::InvalidProgrammableConversionDelay))
  //   }

  //   self.write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION])?;
  // }

  // // Data Rate Register Functions Below

  // pub fn enable_global_chop(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[DATARATE_LOCATION] |= 1 << 7; // set bit 7
  //   self.write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
  // }

  // pub fn disable_global_chop(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 7); // clear bit 7
  //   self.write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
  // }

  // pub fn enable_internal_clock(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 6); // clear bit 6
  //   self.write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
  // }

  // pub fn enable_continious_conversion_mode(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 5); // clear bit 5
  //   self.write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
  // }

  // pub fn enable_single_shot_conversion_mode(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[DATARATE_LOCATION] |= 1 << 5; // set bit 5
  //   self.write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
  // }

  // pub fn enable_sinc_filter(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 4); // clear bit 4
  //   self.write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
  // }

  // pub fn enable_low_latency_filter(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[DATARATE_LOCATION] |= 1 << 4; // set bit 4
  //   self.write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
  // }

  // pub fn set_data_rate(&mut self, rate: DataRate) -> Result<(), ADCError> {
  //   match rate {
  //     DataRate::UFloat(2.5) => {
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     DataRate::UInt(5) => {
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     DataRate::UInt(10) => {
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     DataRate::UFloat(16.6) => {
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     DataRate::UInt(20) => {
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     DataRate::UInt(50) => {
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     DataRate::UInt(60) => {
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     DataRate::UInt(100) => {
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     DataRate::UInt(200) => {
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 3; // set bit 3
  //     },

  //     DataRate::UInt(400) => {
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 3; // set bit 3
  //     },

  //     DataRate::UInt(800) => {
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 3; // set bit 3
  //     },

  //     DataRate::UInt(1000) => {
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 3; // set bit 3
  //     },

  //     DataRate::UInt(2000) => {
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 3; // set bit 3
  //     },

  //     DataRate::UInt(4000) => {
  //       self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[DATARATE_LOCATION] |= 1 << 3; // set bit 3
  //     },

  //     _ => return Err(ADCError::User(ADCUserError::InvalidDataRate))
  //   }

  //   self.write_reg(DATARATE_LOCATION, self.current_reg_vals[DATARATE_LOCATION])?;
  // }

  // // Reference Register Functions Below

  // pub fn enable_positive_reference_buffer(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 5); // clear bit 5
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }

  // pub fn disable_positive_reference_buffer(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] |= 1 << 5; // set bit 5
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }

  // pub fn enable_negative_reference_buffer(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 4); // clear bit 4
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }

  // pub fn disable_negative_reference_buffer(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] |= 1 << 4; // set bit 4
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }
  
  // pub fn set_ref_input_ref0(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 2); // clear bit 2
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 3); // clear bit 2
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }

  // pub fn set_ref_input_ref1(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] |= 1 << 2; // set bit 2
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 3); // clear bit 3
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }

  // pub fn set_ref_input_internal_2v5_ref(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 2); // clear bit 2
  //   self.current_reg_vals[REF_LOCATION] |= 1 << 3; // set bit 3
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }

  // pub fn disable_internal_voltage_reference(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 0); // clear bit 0
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 1); // clear bit 1
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }

  // pub fn enable_internal_voltage_reference_off_pwr_down(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] |= 1 << 0; // set bit 0
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 1); // clear bit 1
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }

  // pub fn enable_internal_voltage_reference_on_pwr_down(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[REF_LOCATION] &= !(1 << 0); // clear bit 0
  //   self.current_reg_vals[REF_LOCATION] |= 1 << 1; // set bit 1
  //   self.write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])?;
  // }

  // // IDACMAG functions below

  // pub fn set_idac_magnitude(&mut self, mag: u16) -> Result<(), ADCError> {
  //   match mag {
  //     0 => {
  //       // call disable function or set to 0?;
  //     },

  //     10 => {
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     50 => {
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     100 => {
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     250 => {
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     500 => {
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     750 => {
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     1000 => {
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     1500 => {
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 3; // set bit 3
  //     },

  //     2000 => {
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 3; // set bit 3
  //     }

  //     _ => return Err(ADCError::User(ADCUserError::InvalidIDACMag))
  //   }
    
  //   self.write_reg(IDACMAG_LOCATION, self.current_reg_vals[IDACMAG_LOCATION])?;
  // }

  // // IDACMUX functions below

  // pub fn set_idac1_output_channel(&mut self, channel: u8) -> Result<(), ADCError> {
  //   let idac2_channel = self.current_reg_vals[IDACMUX_LOCATION] >> 4;
  //   if channel == idac2_channel {
  //     return Err(ADCError::User(ADCUserError::SameIDAC1Idac2Mux))
  //   }

  //   match channel {
  //     0 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     1 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     2 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     3 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 1; // set bit 1
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 2); // clear bit 2
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     4 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 0); // clear bit 0
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     },

  //     5 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 0; // set bit 0
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 1); // clear bit 1
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 2; // set bit 2
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 3); // clear bit 3
  //     }

  //     _ => return Err(ADCError::User(ADCUserError::InvalidIDAC1Mux))
  //   }

  //   self.write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])?;
  // }

  // pub fn set_idac2_output_channel(&mut self, channel: u8) -> Result<(), ADCError> {
  //   let idac1_channel = self.current_reg_vals[IDACMUX_LOCATION] & 0x0F;
  //   if channel == idac1_channel {
  //     return Err(ADCError::User(ADCUserError::SameIDAC1Idac2Mux))
  //   }

  //   match channel {
  //     0 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 4); // clear bit 4
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 5); // clear bit 5
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 6); // clear bit 6
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 7); // clear bit 7
  //     },

  //     1 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 4; // set bit 4
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 5); // clear bit 5
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 6); // clear bit 6
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 7); // clear bit 7
  //     },

  //     2 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 4); // clear bit 4
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 5; // set bit 5
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 6); // clear bit 6
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 7); // clear bit 7
  //     },

  //     3 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 4; // set bit 4
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 5; // set bit 5
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 6); // clear bit 6
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 7); // clear bit 7
  //     },

  //     4 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 4); // clear bit 4
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 5); // clear bit 5
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 6; // set bit 6
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 7); // clear bit 7
  //     },

  //     5 => {
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 4; // set bit 4
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 5); // clear bit 5
  //       self.current_reg_vals[IDACMUX_LOCATION] |= 1 << 6; // set bit 6
  //       self.current_reg_vals[IDACMUX_LOCATION] &= !(1 << 7); // clear bit 7
  //     }

  //     _ => return Err(ADCError::User(ADCUserError::InvalidIDAC2Mux))
  //   }

  //   self.write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])?;
  // }

  // // VBIAS Register Functions

  // pub fn disable_vbias(&mut self) -> Result<(), ADCError> {
  //   // sets VBIAS to (AVDD + AVSS) / 2 and disconnects VBIAS from all AIN(X)
  //   self.current_reg_vals[VBIAS_LOCATION] = 0;
  //   self.write_reg(VBIAS_LOCATION, self.current_reg_vals[VBIAS_LOCATION])?;
  // }

  // // System Control Register Functions

  // pub fn enable_data_status_byte(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[SYS_LOCATION] |= 1 << 0; // set bit 0
  //   self.write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])?;
  // }

  // pub fn disable_data_status_byte(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[SYS_LOCATION] &= !(1 << 0); // clear bit 0
  //   self.write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])?;
  // }

  // pub fn enable_data_crc_byte(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[SYS_LOCATION] |= 1 << 1; // set bit 1
  //   self.write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])?;
  // }

  // pub fn disable_data_crc_byte(&mut self) -> Result<(), ADCError> {
  //   self.current_reg_vals[SYS_LOCATION] &= !(1 << 1); // clear bit 1
  //   self.write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])?;
  // }

  // pub fn enable_internal_temp_sensor(&mut self, pga_gain: u8) -> Result<(), ADCError> {
  //   match pga_gain {
  //     1 => self.set_pga_gain(1)?,
  //     2 => self.set_pga_gain(2)?,
  //     4 => self.set_pga_gain(4)?,
  //     _ => return Err(ADCError::User(ADCUserError::InvalidInternalTempSensePGAGain))
  //   }
  //   self.enable_pga()?;

  //   self.current_reg_vals[SYS_LOCATION] &= !(1 << 5); // clear bit 5
  //   self.current_reg_vals[SYS_LOCATION] |= 1 << 6; // set bit 6
  //   self.current_reg_vals[SYS_LOCATION] &= !(1 << 7); // clear bit 7

  //   self.write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])?;
  // }


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

  pub fn write_reg(&mut self, reg: usize, data: u8) -> Result<(), io::Error> {
    // TODO: if an rx buffer is sent, look into what data it holds if modified
    let tx_buf: [u8; 3] = [0x40 | (reg as u8), 0x00, data];
    let mut transfer: spi_ioc_transfer<'_, '_> = SpidevTransfer::write(&tx_buf);
    self.spidev.transfer(&mut transfer) // no need for extra error handling as nothing is returned in good case
  }
}