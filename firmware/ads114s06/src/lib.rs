use common::comm::{
  gpio::{
    Pin,
    PinMode::{self, Input, Output},
    PinValue::{self, High, Low},
  },
  ADCKind,
};
use spidev::{
  spidevioctl::SpidevTransfer,
  SpiModeFlags,
  Spidev,
  SpidevOptions,
};
use std::{io, thread, time};

// bit resolution
const ADC_RESOLUTION: u8 = 16;

// Register locations
const ID_LOCATION: usize = 0;
const STATUS_LOCATION: usize = 1;
const INPMUX_LOCATION: usize = 2;
const PGA_LOCATION: usize = 3;
const DATARATE_LOCATION: usize = 4;
const REF_LOCATION: usize = 5;
const IDACMAG_LOCATION: usize = 6;
const IDACMUX_LOCATION: usize = 7;
const VBIAS_LOCATION: usize = 8;
const SYS_LOCATION: usize = 9;
const RESERVED0_LOCATION: usize = 0x0A;
const OFCAL0_LOCATION: usize = 0x0B;
const OFCAL1_LOCATION: usize = 0x0C;
const RESERVED1_LOCATION: usize = 0x0D;
const FSCAL0_LOCATION: usize = 0x0E;
const FSCAL1_LOCATION: usize = 0x0F;
const GPIODAT_LOCATION: usize = 0x10;
const GPIOCON_LOCATION: usize = 0x11;

#[derive(Debug)]
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
  InvalidChannel,
  InvalidGpioNum,
  WritingToGpioInput,
  OutOfBoundsRegisterRead,
  ForbiddenRegisterWrite,
  SPI(io::Error),
}

impl From<io::Error> for ADCError {
  fn from(err: io::Error) -> ADCError {
    ADCError::SPI(err)
  }
}

pub struct ADC {
  spidev: Spidev,
  pub drdy_pin: Pin,
  pub cs_pin: Option<Pin>,
  pub kind: ADCKind,
  pub current_reg_vals: [u8; 18],
}

impl ADC {
  pub fn new(
    bus: &str,
    drdy_pin: Pin,
    mut cs_pin: Option<Pin>,
    kind: ADCKind,
  ) -> Result<ADC, ADCError> {
    // possibly redundant based on how user code handles chip selects
    if let Some(pin) = cs_pin.as_mut() {
      pin.mode(Output);
      pin.digital_write(High); // active low
    }

    let mut spidev = Spidev::open(bus)?;

    let options = SpidevOptions::new()
      .bits_per_word(8)
      .max_speed_hz(10_000_000)
      .lsb_first(false)
      .mode(SpiModeFlags::SPI_MODE_1)
      .build();

    spidev.configure(&options)?;

    let mut adc = ADC {
      spidev,
      drdy_pin,
      cs_pin,
      kind,
      current_reg_vals: [0; 18],
    };

    // possibly redundant based on how user handles drdy pin
    adc.drdy_pin.mode(Input);
    adc.spi_reset()?;
    adc.current_reg_vals = adc.spi_read_all_regs()?;
    Ok(adc)
  }

  pub fn enable_chip_select(&mut self) {
    if let Some(ref mut pin) = self.cs_pin {
      pin.digital_write(Low); // active low
    }
  }

  pub fn disable_chip_select(&mut self) {
    if let Some(ref mut pin) = self.cs_pin {
      pin.digital_write(High); // active low
    }
  }

  pub fn check_drdy(&self) -> PinValue {
    self.drdy_pin.digital_read()
  }

  pub fn get_id_reg(&self) -> u8 {
    self.current_reg_vals[ID_LOCATION]
  }

  pub fn get_status_reg(&mut self) -> Result<u8, ADCError> {
    self.spi_read_reg(STATUS_LOCATION)
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

  pub fn get_gpiodat_reg(&mut self) -> Result<u8, ADCError> {
    Ok(self.spi_read_reg(GPIODAT_LOCATION)?)
  }

  pub fn get_gpiocon_reg(&self) -> u8 {
    self.current_reg_vals[GPIOCON_LOCATION]
  }

  // Input Multiplexer Register Functions Below

  pub fn set_positive_input_channel(
    &mut self,
    channel: u8,
  ) -> Result<(), ADCError> {
    if channel == self.get_negative_input_channel() {
      return Err(ADCError::SamePositiveNegativeInputMux);
    }

    if channel > 5 {
      return Err(ADCError::InvalidPositiveInputMux);
    }

    // clear bits 7-4
    let clear = 0b00001111;
    self.current_reg_vals[INPMUX_LOCATION] &= clear;
    // shift input by 4 bits to configure bits 7-4
    self.current_reg_vals[INPMUX_LOCATION] |= channel << 4;
    self.spi_write_reg(INPMUX_LOCATION, self.current_reg_vals[INPMUX_LOCATION])
  }

  pub fn set_negative_input_channel(
    &mut self,
    channel: u8,
  ) -> Result<(), ADCError> {
    if channel == self.get_positive_input_channel() {
      return Err(ADCError::SamePositiveNegativeInputMux);
    }

    if channel > 5 {
      return Err(ADCError::InvalidNegativeInputMux);
    }

    // clear bits 3-0
    let clear = 0b11110000;
    self.current_reg_vals[INPMUX_LOCATION] &= clear;
    // configure bits 3-0
    self.current_reg_vals[INPMUX_LOCATION] |= channel;
    self.spi_write_reg(INPMUX_LOCATION, self.current_reg_vals[INPMUX_LOCATION])
  }

  pub fn set_negative_input_channel_to_aincom(
    &mut self,
  ) -> Result<(), ADCError> {
    let clear = 0b11110000; // clear bits 3-0
    let set = 0b00001100; // set bits 3-2
    self.current_reg_vals[INPMUX_LOCATION] &= clear;
    self.current_reg_vals[INPMUX_LOCATION] |= set;
    self.spi_write_reg(INPMUX_LOCATION, self.current_reg_vals[INPMUX_LOCATION])
  }

  fn get_positive_input_channel(&self) -> u8 {
    // shift right by 4 bits to return bits 7-4
    self.get_inpmux_reg() >> 4
  }

  fn get_negative_input_channel(&self) -> u8 {
    // return bits 3-0
    self.get_inpmux_reg() & 0b00001111
  }

  // PGA Register Functions Below

  pub fn enable_pga(&mut self) -> Result<(), ADCError> {
    // clear bits 4 and 3, then set bit 3
    let clear: u8 = 0b11100111;
    let set: u8 = 0b00001000;
    self.current_reg_vals[PGA_LOCATION] &= clear;
    self.current_reg_vals[PGA_LOCATION] |= set;
    self.spi_write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION])
  }

  pub fn disable_pga(&mut self) -> Result<(), ADCError> {
    // clear bits 4 and 3
    let clear: u8 = 0b11100111;
    self.current_reg_vals[PGA_LOCATION] &= clear;
    self.spi_write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION])
  }

  pub fn set_pga_gain(&mut self, gain: u8) -> Result<(), ADCError> {
    // clear bits 2-0
    let clear: u8 = 0b11111000;

    // configure bits 2-0
    let set: u8 = match gain {
      1 => {
        self.disable_pga()?;
        0
      }

      2 => 0b00000001,

      4 => 0b00000010,

      8 => 0b00000011,

      16 => 0b00000100,

      32 => 0b00000101,

      64 => 0b00000110,

      128 => 0b00000111,

      _ => return Err(ADCError::InvalidPGAGain),
    };

    self.current_reg_vals[PGA_LOCATION] &= clear;
    self.current_reg_vals[PGA_LOCATION] |= set;
    self.spi_write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION])
  }

  pub fn get_pga_gain(&self) -> u8 {
    1 << (self.get_pga_reg() & 0b00000111)
  }

  pub fn set_programmable_conversion_delay(
    &mut self,
    delay: u16,
  ) -> Result<(), ADCError> {
    // clear bits 7-5
    let clear: u8 = 0b00011111;
    // configure bits 7-5
    let set: u8 = match delay {
      14 => 0,

      25 => 0b00100000,

      64 => 0b01000000,

      256 => 0b01100000,

      1024 => 0b10000000,

      2048 => 0b10100000,

      4096 => 0b11000000,

      1 => 0b11100000,

      _ => return Err(ADCError::InvalidProgrammableConversionDelay),
    };

    self.current_reg_vals[PGA_LOCATION] &= clear;
    self.current_reg_vals[PGA_LOCATION] |= set;
    self.spi_write_reg(PGA_LOCATION, self.current_reg_vals[PGA_LOCATION])
  }

  pub fn get_programmable_conversion_delay(&self) -> Result<u16, ADCError> {
    // shift right by 5 bits to get bits 7-5
    let delay = match (self.get_pga_reg() & 0b11100000) >> 5 {
      0b000 => 14,

      0b001 => 25,

      0b010 => 64,

      0b011 => 256,

      0b100 => 1024,

      0b101 => 2048,

      0b110 => 4096,

      0b111 => 1,

      _ => return Err(ADCError::InvalidProgrammableConversionDelay),
    };
    Ok(delay)
  }

  // Data Rate Register Functions Below

  pub fn enable_global_chop(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] |= 1 << 7; // set bit 7
    self.spi_write_reg(
      DATARATE_LOCATION,
      self.current_reg_vals[DATARATE_LOCATION],
    )
  }

  pub fn disable_global_chop(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 7); // clear bit 7
    self.spi_write_reg(
      DATARATE_LOCATION,
      self.current_reg_vals[DATARATE_LOCATION],
    )
  }

  pub fn enable_internal_clock_disable_external(
    &mut self,
  ) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 6); // clear bit 6
    self.spi_write_reg(
      DATARATE_LOCATION,
      self.current_reg_vals[DATARATE_LOCATION],
    )
  }

  pub fn enable_external_clock_disable_internal(
    &mut self,
  ) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] |= 1 << 6; // set bit 6
    self.spi_write_reg(
      DATARATE_LOCATION,
      self.current_reg_vals[DATARATE_LOCATION],
    )
  }

  pub fn enable_continious_conversion_mode(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 5); // clear bit 5
    self.spi_write_reg(
      DATARATE_LOCATION,
      self.current_reg_vals[DATARATE_LOCATION],
    )
  }

  pub fn enable_single_shot_conversion_mode(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] |= 1 << 5; // set bit 5
    self.spi_write_reg(
      DATARATE_LOCATION,
      self.current_reg_vals[DATARATE_LOCATION],
    )
  }

  pub fn enable_sinc_filter(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] &= !(1 << 4); // clear bit 4
    self.spi_write_reg(
      DATARATE_LOCATION,
      self.current_reg_vals[DATARATE_LOCATION],
    )
  }

  pub fn enable_low_latency_filter(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[DATARATE_LOCATION] |= 1 << 4; // set bit 4
    self.spi_write_reg(
      DATARATE_LOCATION,
      self.current_reg_vals[DATARATE_LOCATION],
    )
  }

  pub fn set_data_rate(&mut self, rate: f64) -> Result<(), ADCError> {
    // cleat bits 3-0
    let clear: u8 = 0b11110000;
    // configure bits 3-0
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

      _ => return Err(ADCError::InvalidDataRate),
    };

    self.current_reg_vals[DATARATE_LOCATION] &= clear;
    self.current_reg_vals[DATARATE_LOCATION] |= set;
    self.spi_write_reg(
      DATARATE_LOCATION,
      self.current_reg_vals[DATARATE_LOCATION],
    )
  }

  pub fn get_data_rate(&self) -> Result<f64, ADCError> {
    // look at bits 3-0
    let rate = match self.get_datarate_reg() & 0b00001111 {
      0b0000 => 2.5,

      0b0001 => 5.0,

      0b0010 => 10.0,

      0b0011 => 16.6,

      0b0100 => 20.0,

      0b0101 => 50.0,

      0b0110 => 60.0,

      0b0111 => 100.0,

      0b1000 => 200.0,

      0b1001 => 400.0,

      0b1010 => 800.0,

      0b1011 => 1000.0,

      0b1100 => 2000.0,

      0b1101 => 4000.0,

      0b1110 => 4000.0,

      _ => return Err(ADCError::InvalidDataRate),
    };
    Ok(rate)
  }

  // Reference Register Functions Below

  pub fn disable_reference_monitor(&mut self) -> Result<(), ADCError> {
    let clear = 0b00111111;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn enable_positive_reference_buffer(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 5); // clear bit 5
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn disable_positive_reference_buffer(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[REF_LOCATION] |= 1 << 5; // set bit 5
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn enable_negative_reference_buffer(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[REF_LOCATION] &= !(1 << 4); // clear bit 4
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn disable_negative_reference_buffer(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[REF_LOCATION] |= 1 << 4; // set bit 4
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn set_ref_input_ref0(&mut self) -> Result<(), ADCError> {
    // clear bits 3-2
    let clear = 0b11110011;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn set_ref_input_ref1(&mut self) -> Result<(), ADCError> {
    // clear bits 3-2
    let clear = 0b11110011;
    // set bit 2
    let set = 0b00000100;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.current_reg_vals[REF_LOCATION] |= set;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn set_ref_input_internal_2v5_ref(&mut self) -> Result<(), ADCError> {
    // clear bits 3-2
    let clear = 0b11110011;
    // set bit 3
    let set = 0b00001000;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.current_reg_vals[REF_LOCATION] |= set;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn disable_internal_voltage_reference(&mut self) -> Result<(), ADCError> {
    // clear bits 1-0
    let clear = 0b11111100;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn enable_internal_voltage_reference_off_pwr_down(
    &mut self,
  ) -> Result<(), ADCError> {
    // clear bits 1-0
    let clear = 0b11111100;
    // set bit 1
    let set = 0b00000001;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.current_reg_vals[REF_LOCATION] |= set;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  pub fn enable_internal_voltage_reference_on_pwr_down(
    &mut self,
  ) -> Result<(), ADCError> {
    // clear bits 1-0
    let clear = 0b11111100;
    // set bit 1
    let set = 0b00000010;
    self.current_reg_vals[REF_LOCATION] &= clear;
    self.current_reg_vals[REF_LOCATION] |= set;
    self.spi_write_reg(REF_LOCATION, self.current_reg_vals[REF_LOCATION])
  }

  // IDACMAG functions below

  pub fn disable_pga_output_monitoring(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 7); // clear bit 7
    self
      .spi_write_reg(IDACMAG_LOCATION, self.current_reg_vals[IDACMAG_LOCATION])
  }

  pub fn open_low_side_pwr_switch(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[IDACMAG_LOCATION] &= !(1 << 6); // clear bit 6
    self
      .spi_write_reg(IDACMAG_LOCATION, self.current_reg_vals[IDACMAG_LOCATION])
  }

  pub fn close_low_side_pwr_switch(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[IDACMAG_LOCATION] |= 1 << 6; // set bit 6
    self
      .spi_write_reg(IDACMAG_LOCATION, self.current_reg_vals[IDACMAG_LOCATION])
  }

  pub fn set_idac_magnitude(&mut self, mag: u16) -> Result<(), ADCError> {
    // clear bits 3-0
    let clear: u8 = 0b11110000;
    // configure bits 3-0
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

      _ => return Err(ADCError::InvalidIDACMag),
    };

    self.current_reg_vals[IDACMAG_LOCATION] &= clear;
    self.current_reg_vals[IDACMAG_LOCATION] |= set;
    self
      .spi_write_reg(IDACMAG_LOCATION, self.current_reg_vals[IDACMAG_LOCATION])
  }

  pub fn get_idac_magnitude(&self) -> u16 {
    // look at bits 3-0
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

      _ => 0,
    }
  }

  // IDACMUX functions below

  pub fn enable_idac1_output_channel(
    &mut self,
    channel: u8,
  ) -> Result<(), ADCError> {
    if channel == self.get_idac2_output_channel() {
      return Err(ADCError::SameIDAC1IDAC2Mux);
    }

    if channel > 5 {
      return Err(ADCError::InvalidIDAC1Mux);
    }

    // clear bits 3-0
    let clear: u8 = 0b11110000;
    self.current_reg_vals[IDACMUX_LOCATION] &= clear;
    // configure bits 3-0
    self.current_reg_vals[IDACMUX_LOCATION] |= channel;
    self
      .spi_write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])
  }

  pub fn enable_idac2_output_channel(
    &mut self,
    channel: u8,
  ) -> Result<(), ADCError> {
    if channel == self.get_idac1_output_channel() {
      return Err(ADCError::SameIDAC1IDAC2Mux);
    }

    if channel > 5 {
      return Err(ADCError::InvalidIDAC2Mux);
    }

    // clear bits 7-4
    let clear = 0b00001111;
    self.current_reg_vals[IDACMUX_LOCATION] &= clear;
    // configure bits 7-4
    self.current_reg_vals[IDACMUX_LOCATION] |= channel << 4;
    self
      .spi_write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])
  }

  pub fn disable_idac1(&mut self) -> Result<(), ADCError> {
    // set bits 7-4 to 1111
    let set: u8 = 0b11110000;
    self.current_reg_vals[IDACMUX_LOCATION] |= set;
    self
      .spi_write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])
  }

  pub fn disable_idac2(&mut self) -> Result<(), ADCError> {
    // set bits 3-0 to 1111
    let set: u8 = 0b00001111;
    self.current_reg_vals[IDACMUX_LOCATION] |= set;
    self
      .spi_write_reg(IDACMUX_LOCATION, self.current_reg_vals[IDACMUX_LOCATION])
  }

  pub fn get_idac1_output_channel(&self) -> u8 {
    // look at bits 3-0
    self.get_idacmux_reg() & 0b00001111
  }

  pub fn get_idac2_output_channel(&self) -> u8 {
    // look at bits 7-4
    self.get_idacmux_reg() >> 4
  }

  // VBIAS Register Functions

  pub fn disable_vbias(&mut self) -> Result<(), ADCError> {
    // sets VBIAS to (AVDD + AVSS) / 2 and disconnects VBIAS from all AIN(X)
    self.current_reg_vals[VBIAS_LOCATION] = 0;
    self.spi_write_reg(VBIAS_LOCATION, self.current_reg_vals[VBIAS_LOCATION])
  }

  // System Control Register Functions

  pub fn enable_internal_temp_sensor(
    &mut self,
    pga_gain: u8,
  ) -> Result<(), ADCError> {
    // pga gain must be <= 4
    match pga_gain {
      1 => self.set_pga_gain(1)?,
      2 => self.set_pga_gain(2)?,
      4 => self.set_pga_gain(4)?,
      _ => return Err(ADCError::InvalidInternalTempSensePGAGain),
    }
    self.enable_pga()?;

    // clear bits 7-4
    let clear = 0b00011111;
    // set bit 6
    let set = 0b01000000;
    self.current_reg_vals[SYS_LOCATION] &= clear;
    self.current_reg_vals[SYS_LOCATION] |= set;
    self.spi_write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])
  }

  pub fn disable_system_monitoring(&mut self) -> Result<(), ADCError> {
    let clear = 0b00011111;
    self.current_reg_vals[SYS_LOCATION] &= clear;
    self.spi_write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])
  }

  pub fn disable_spi_timeout(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[SYS_LOCATION] &= !(1 << 2); // clear bit 2
    self.spi_write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])
  }

  pub fn disable_crc_byte(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[SYS_LOCATION] &= !(1 << 1); // clear bit 1
    self.spi_write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])
  }

  pub fn disable_status_byte(&mut self) -> Result<(), ADCError> {
    self.current_reg_vals[SYS_LOCATION] &= !(1 << 0); // clear bit 0
    self.spi_write_reg(SYS_LOCATION, self.current_reg_vals[SYS_LOCATION])
  }

  // GPIO Functions

  pub fn set_gpio_mode(
    &mut self,
    pin: u8,
    mode: PinMode,
  ) -> Result<(), ADCError> {
    if pin > 3 {
      return Err(ADCError::InvalidGpioNum);
    }

    match mode {
      Output => {
        self.current_reg_vals[GPIODAT_LOCATION] &= !(1 << (pin + 4));
      }

      Input => {
        self.current_reg_vals[GPIODAT_LOCATION] |= 1 << (pin + 4);
      }
    }

    self
      .spi_write_reg(GPIODAT_LOCATION, self.current_reg_vals[GPIODAT_LOCATION])
  }

  pub fn get_gpio_mode(&self, pin: u8) -> Result<PinMode, ADCError> {
    if pin > 3 {
      return Err(ADCError::InvalidGpioNum);
    }

    match (self.current_reg_vals[GPIODAT_LOCATION] >> (pin + 4)) & 1 {
      0 => Ok(Output),
      1 => Ok(Input),
      _ => unreachable!(),
    }
  }

  pub fn gpio_digital_write(
    &mut self,
    pin: u8,
    val: PinValue,
  ) -> Result<(), ADCError> {
    if self.get_gpio_mode(pin)? == Input {
      return Err(ADCError::WritingToGpioInput);
    }

    match val {
      Low => {
        self.current_reg_vals[GPIODAT_LOCATION] &= !(1 << pin);
      }

      High => {
        self.current_reg_vals[GPIODAT_LOCATION] |= 1 << pin;
      }
    }

    self
      .spi_write_reg(GPIODAT_LOCATION, self.current_reg_vals[GPIODAT_LOCATION])
  }

  pub fn gpio_digital_read(&mut self, pin: u8) -> Result<PinValue, ADCError> {
    if pin > 3 {
      return Err(ADCError::InvalidGpioNum);
    }

    self.current_reg_vals[GPIODAT_LOCATION] =
      self.spi_read_reg(GPIODAT_LOCATION)?;
    match (self.current_reg_vals[GPIODAT_LOCATION] >> pin) & 1 {
      0 => Ok(Low),
      1 => Ok(High),
      _ => unreachable!(),
    }
  }

  pub fn config_gpio_as_gpio(&mut self, pin: u8) -> Result<(), ADCError> {
    if pin > 3 {
      return Err(ADCError::InvalidGpioNum);
    }

    // always write 0 to bits 7-4 in GPIOCON
    self.current_reg_vals[GPIOCON_LOCATION] &= 0b00001111;
    self.current_reg_vals[GPIOCON_LOCATION] |= 1 << pin;

    self
      .spi_write_reg(GPIOCON_LOCATION, self.current_reg_vals[GPIOCON_LOCATION])
  }

  pub fn config_gpio_as_analog_input(
    &mut self,
    pin: u8,
  ) -> Result<(), ADCError> {
    if pin > 3 {
      return Err(ADCError::InvalidGpioNum);
    }

    // always write 0 to bits 7-4 in GPIOCON
    self.current_reg_vals[GPIOCON_LOCATION] &= 0b00001111;
    self.current_reg_vals[GPIOCON_LOCATION] &= !(1 << pin);

    self
      .spi_write_reg(GPIOCON_LOCATION, self.current_reg_vals[GPIOCON_LOCATION])
  }

  /* FOR THE FOLLOWING SPI COMMUNICATION COMMANDS BELOW
  For a read_write transfer, tx_buf is used to send the command and rx_buf
  is used to receive the data. For read_write, tx_buf and rx_buf must be
  of equal size and the kernel automatically modified rx_buf, which is why
  a mutable reference is passed to it. For the write_reg function it must be
  explored as to if providing an rx_buf will do anything.
   */

  pub fn spi_no_operation(&mut self) -> Result<(), ADCError> {
    self.enable_chip_select();
    let tx_buf: [u8; 1] = [0x00];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    match result {
      Ok(_) => Ok(()),
      Err(e) => Err(ADCError::SPI(e)),
    }
  }

  pub fn spi_wake_up_from_pwr_down_mode(&mut self) -> Result<(), ADCError> {
    self.enable_chip_select();
    let tx_buf: [u8; 1] = [0x02];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    match result {
      Ok(_) => Ok(()),
      Err(e) => Err(ADCError::SPI(e)),
    }
  }

  pub fn spi_enter_pwr_down_mode(&mut self) -> Result<(), ADCError> {
    self.enable_chip_select();
    let tx_buf: [u8; 1] = [0x04];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    match result {
      Ok(_) => Ok(()),
      Err(e) => Err(ADCError::SPI(e)),
    }
  }

  /*
  After a reset command a delay of t d(RSSC) is needed before sending a
  command or starting a conversion. This value is 4096 * t clock where t clock
  is the inverse of the frequency of the clock of the ADC. For us avionics
  people it is grounded and the internal oscillator with a
  frequency of 4.096 MHz is used. The math results in a needed delay of 1ms
  or 1000 microseconds, and I simply add a little bit more to play safe. The
  registers are set to their default states, so assuming the reset worked, the
  delay is executed and the registers are all re-read to get the current state
  of the ADC
   */
  pub fn spi_reset(&mut self) -> Result<(), ADCError> {
    self.enable_chip_select();
    let tx_buf: [u8; 1] = [0x06];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    // wait 1 ms before any other commands
    thread::sleep(time::Duration::from_micros(1100));
    match result {
      Ok(_) => {
        self.current_reg_vals = self.spi_read_all_regs()?;
        Ok(())
      }

      Err(e) => Err(ADCError::SPI(e)),
    }
  }

  pub fn spi_start_conversion(&mut self) -> Result<(), ADCError> {
    self.enable_chip_select();
    let tx_buf: [u8; 1] = [0x08];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    thread::sleep(time::Duration::from_micros(1100));
    match result {
      Ok(_) => Ok(()),
      Err(e) => Err(ADCError::SPI(e)),
    }
  }

  pub fn spi_stop_conversion(&mut self) -> Result<(), ADCError> {
    self.enable_chip_select();
    let tx_buf: [u8; 1] = [0x0A];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    match result {
      Ok(_) => Ok(()),
      Err(e) => Err(ADCError::SPI(e)),
    }
  }

  pub fn spi_read_data(&mut self) -> Result<i16, ADCError> {
    /*
    old SAM code received data in 3 byte buffer even though CRC and STATUS
    bytes were disabled which leaves for 2 bytes of data. The tx_buf just
    needs to store one byte so going to investigate why this was done and if
    not needed will reduce tx_buf and rx_buf sizes to 2 bytes
     */
    self.enable_chip_select();
    let tx_buf: [u8; 3] = [0x12, 0x00, 0x00];
    let mut rx_buf: [u8; 3] = [0x00, 0x00, 0x00];
    let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    match result {
      Ok(_) => Ok(((rx_buf[1] as i16) << 8) | (rx_buf[2] as i16)),
      Err(e) => Err(ADCError::SPI(e)),
    }
  }

  pub fn spi_read_reg(&mut self, reg: usize) -> Result<u8, ADCError> {
    // usize is non negative so that would not compile or fail beforehand
    if reg > 17 {
      return Err(ADCError::OutOfBoundsRegisterRead);
    }
    self.enable_chip_select();
    let tx_buf: [u8; 2] = [0x20 | (reg as u8), 0x00];
    let mut rx_buf: [u8; 2] = [0x00, 0x00];
    let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    match result {
      Ok(_) => Ok(rx_buf[1]),
      Err(e) => Err(ADCError::SPI(e)),
    }
  }

  pub fn spi_read_all_regs(&mut self) -> Result<[u8; 18], ADCError> {
    self.enable_chip_select();
    /*
    There are 18 registers to read from, but 2 bytes are needed for the
    command. Increased size of array to 20 because first register will appear
    at the 3rd byte of rx_buf
     */
    let tx_buf: [u8; 20] = [
      0x20, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];
    let mut rx_buf: [u8; 20] = [0; 20];
    let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    match result {
      Ok(_) => {
        let mut regs: [u8; 18] = [0; 18];
        regs.copy_from_slice(&rx_buf[2..]);
        Ok(regs)
      }
      Err(e) => Err(ADCError::SPI(e)),
    }
  }

  fn spi_write_reg(&mut self, reg: usize, data: u8) -> Result<(), ADCError> {
    if reg == RESERVED0_LOCATION || reg == RESERVED1_LOCATION {
      return Err(ADCError::ForbiddenRegisterWrite);
    }
    self.enable_chip_select();
    let tx_buf: [u8; 3] = [0x40 | (reg as u8), 0x00, data];
    let mut transfer = SpidevTransfer::write(&tx_buf);
    let result = self.spidev.transfer(&mut transfer);
    self.disable_chip_select();
    match result {
      Ok(_) => Ok(()),
      Err(e) => Err(ADCError::SPI(e)),
    }
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
    let lsb: f64 =
      (2.0 * 2.5) / ((1 << (self.get_pga_gain() + ADC_RESOLUTION - 1)) as f64);
    (code as f64) * lsb
  }

  //   pub fn calculate_four_wire_rtd_resistance(&self, code: i16,
  // ref_resistance: f64) -> f64 {     /*
  //     The beauty of a ratiometric measurement is that the output code is
  //     proportional to a ratio between the input voltage and reference
  // voltage.     The two resistances creating these voltages are in series so
  // with ohms law     you can cancel out the current because current is the
  // same in series and     you are left with a ratio proportional to two
  // resistances      */
  //     (code as f64) * 2.0 * ref_resistance / ((1 << (self.get_pga_gain() +
  // ADC_RESOLUTION)) as f64)   }
}
