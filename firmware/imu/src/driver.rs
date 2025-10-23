extern crate spidev;
use common::comm::gpio::{Gpio, Pin, PinMode::*, PinValue::*};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::{
  error,
  fmt,
  io::{self, prelude::*},
  thread::sleep,
  time::Duration,
};

use crate::bit_mappings::*;
use crate::internals::*;

/// From page 4 of documentation
const POWER_ON_START_UP_TIME: Duration = Duration::from_millis(310);
/// From page 4 of documentation
const RESET_DOWNTIME: Duration = Duration::from_millis(255);

#[derive(Clone, Debug)]
pub struct DeltaReadData {
  pub delta_angle: [i32; 3],

  pub delta_velocity: [i32; 3],
}

impl DeltaReadData {
  pub fn add(&mut self, other: DeltaReadData, multiplier: i32) {
    self.delta_angle[0] += multiplier * other.delta_angle[0];
    self.delta_angle[1] += multiplier * other.delta_angle[1];
    self.delta_angle[2] += multiplier * other.delta_angle[2];
    self.delta_velocity[0] += multiplier * other.delta_velocity[0];
    self.delta_velocity[1] += multiplier * other.delta_velocity[1];
    self.delta_velocity[2] += multiplier * other.delta_velocity[2];
  }

  pub fn divide(self, amount: i32) -> DeltaReadData {
    return DeltaReadData {
      delta_angle: [
        self.delta_angle[0] / amount,
        self.delta_angle[1] / amount,
        self.delta_angle[2] / amount,
      ],
      delta_velocity: [
        self.delta_velocity[0] / amount,
        self.delta_velocity[1] / amount,
        self.delta_velocity[2] / amount,
      ],
    };
  }

  pub fn get_angle_float(&self) -> [f32; 3] {
    [
      self.delta_angle[0] as f32 * 2160.0 / CONVERSION_DIVISOR_CONST as f32,
      self.delta_angle[1] as f32 * 2160.0 / CONVERSION_DIVISOR_CONST as f32,
      self.delta_angle[2] as f32 * 2160.0 / CONVERSION_DIVISOR_CONST as f32,
    ]
  }
  pub fn get_velocity_float(&self) -> [f32; 3] {
    [
      self.delta_velocity[0] as f32 * 400.0 / CONVERSION_DIVISOR_CONST as f32,
      self.delta_velocity[1] as f32 * 400.0 / CONVERSION_DIVISOR_CONST as f32,
      self.delta_velocity[2] as f32 * 400.0 / CONVERSION_DIVISOR_CONST as f32,
    ]
  }
}

const CONVERSION_DIVISOR_CONST: u32 = 0x80000000;
impl fmt::Display for DeltaReadData {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let del_ang = self.get_angle_float();
    let del_vel = self.get_velocity_float();
    write!(f, "angle : ({:010.4}, {:010.4}, {:010.4}) deg | velocity : ({:010.4}, {:010.4}, {:010.4}) m/s",
    del_ang[0],
    del_ang[1],
    del_ang[2],
    del_vel[0],
    del_vel[1],
    del_vel[2],
    )
  }
}

#[derive(Clone, Debug)]
pub struct GenericData {
  pub temp: i16,

  pub data_counter: i16,
}

#[derive(Clone, Debug)]
pub struct GyroReadData {
  pub gyro: [i32; 3],

  pub accel: [i32; 3],
}

impl GyroReadData {
  pub fn get_gyro_float(&self) -> [f32; 3] {
    [
      self.gyro[0] as f32 * 0.1 / 0x10000 as f32,
      self.gyro[1] as f32 * 0.1 / 0x10000 as f32,
      self.gyro[2] as f32 * 0.1 / 0x10000 as f32,
    ]
  }
  pub fn get_accel_float(&self) -> [f32; 3] {
    [
      self.accel[0] as f32 * 392.0 / 0x7D000000 as f32,
      self.accel[1] as f32 * 392.0 / 0x7D000000 as f32,
      self.accel[2] as f32 * 392.0 / 0x7D000000 as f32,
    ]
  }
}

impl fmt::Display for GyroReadData {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let conv_gyro = self.get_gyro_float();
    let conv_accel = self.get_accel_float();
    write!(f, "gyro : ({:010.4}, {:010.4}, {:010.4}) deg | accel : ({:010.4}, {:010.4}, {:010.4}) m/s",
    conv_gyro[0],
    conv_gyro[1],
    conv_gyro[2],
    conv_accel[0],
    conv_accel[1],
    conv_accel[2],
    )
  }
}

struct ConfigValues {
  msc_control_reg: u16,
  dec_rate_reg: u16,
}

impl ConfigValues {
  fn default() -> ConfigValues {
    ConfigValues {
      msc_control_reg: 0x00C1,
      dec_rate_reg: 0x0000,
    }
  }

  fn get_last_burst_sel(&self) -> bool {
    return (self.msc_control_reg & (1 << 8)) != 0;
  }

  /// DOES NOT WRITE TO REGISTER
  fn set_burst_sel(&mut self, value: bool) {
    if value {
      self.msc_control_reg |= 0x0100;
    } else {
      self.msc_control_reg &= 0xFEFF;
    }
  }

  fn read_all_values(
    &mut self,
    driver: &mut AdisIMUDriver,
  ) -> DriverResult<()> {
    self.msc_control_reg =
      driver.repeat_read_16_bit_redundant(Registers::MSC_CTRL, 3)? as u16;

    self.msc_control_reg =
      driver.repeat_read_16_bit_redundant(Registers::MSC_CTRL, 3)? as u16;

    Ok(())
  }
}

const GLOB_CMD: [u8; 2] = [0x68, 0x69];

#[derive(Copy, Clone)]
#[allow(non_camel_case_types)]
enum Registers {
  DIAG_STAT,
  X_GYRO_LOW,
  X_GYRO_OUT,
  Y_GYRO_LOW,
  Y_GYRO_OUT,
  Z_GYRO_LOW,
  Z_GYRO_OUT,
  X_ACCL_LOW,
  X_ACCL_OUT,
  Y_ACCL_LOW,
  Y_ACCL_OUT,
  Z_ACCL_LOW,
  Z_ACCL_OUT,
  TEMP_OUT,
  TIME_STAMP,

  DATA_CNTR,
  X_DELTANG_LOW,
  X_DELTANG_OUT,
  Y_DELTANG_LOW,
  Y_DELTANG_OUT,
  Z_DELTANG_LOW,
  Z_DELTANG_OUT,
  X_DELTVEL_LOW,
  X_DELTVEL_OUT,
  Y_DELTVEL_LOW,
  Y_DELTVEL_OUT,
  Z_DELTVEL_LOW,
  Z_DELTVEL_OUT,

  XG_BIAS_LOW,
  XG_BIAS_HIGH,
  YG_BIAS_LOW,
  YG_BIAS_HIGH,
  ZG_BIAS_LOW,
  ZG_BIAS_HIGH,
  XA_BIAS_LOW,
  XA_BIAS_HIGH,
  YA_BIAS_LOW,
  YA_BIAS_HIGH,
  ZA_BIAS_LOW,
  ZA_BIAS_HIGH,

  FILT_CTRL,
  RANG_MDL,
  MSC_CTRL,
  UP_SCALE,
  DEC_RATE,

  FIRM_REV,
  FIRM_DM,
  FIRM_Y,
  PROD_ID,
  SERIAL_NUM,
  USER_SCR_1,
  USER_SCR_2,
  USER_SCR_3,
  FLSHCNT_LOW,
  FLSHCNT_HIGH,
}

impl Registers {
  fn get_address(&self) -> [u8; 2] {
    match (self) {
      Registers::DIAG_STAT => [0x02, 0x03],
      Registers::X_GYRO_LOW => [0x04, 0x05],
      Registers::X_GYRO_OUT => [0x06, 0x07],
      Registers::Y_GYRO_LOW => [0x08, 0x09],
      Registers::Y_GYRO_OUT => [0x0A, 0x0B],
      Registers::Z_GYRO_LOW => [0x0C, 0x0D],
      Registers::Z_GYRO_OUT => [0x0E, 0x0F],
      Registers::X_ACCL_LOW => [0x10, 0x11],
      Registers::X_ACCL_OUT => [0x12, 0x13],
      Registers::Y_ACCL_LOW => [0x14, 0x15],
      Registers::Y_ACCL_OUT => [0x16, 0x17],
      Registers::Z_ACCL_LOW => [0x18, 0x19],
      Registers::Z_ACCL_OUT => [0x1A, 0x1B],
      Registers::TEMP_OUT => [0x1C, 0x1D],
      Registers::TIME_STAMP => [0x1E, 0x1F],

      Registers::DATA_CNTR => [0x22, 0x23],
      Registers::X_DELTANG_LOW => [0x24, 0x25],
      Registers::X_DELTANG_OUT => [0x26, 0x27],
      Registers::Y_DELTANG_LOW => [0x28, 0x29],
      Registers::Y_DELTANG_OUT => [0x2A, 0x2B],
      Registers::Z_DELTANG_LOW => [0x2C, 0x2D],
      Registers::Z_DELTANG_OUT => [0x2E, 0x2F],
      Registers::X_DELTVEL_LOW => [0x30, 0x31],
      Registers::X_DELTVEL_OUT => [0x32, 0x33],
      Registers::Y_DELTVEL_LOW => [0x34, 0x35],
      Registers::Y_DELTVEL_OUT => [0x36, 0x37],
      Registers::Z_DELTVEL_LOW => [0x38, 0x39],
      Registers::Z_DELTVEL_OUT => [0x3A, 0x3B],

      Registers::XG_BIAS_LOW => [0x40, 0x41],
      Registers::XG_BIAS_HIGH => [0x42, 0x43],
      Registers::YG_BIAS_LOW => [0x44, 0x45],
      Registers::YG_BIAS_HIGH => [0x46, 0x47],
      Registers::ZG_BIAS_LOW => [0x48, 0x49],
      Registers::ZG_BIAS_HIGH => [0x4A, 0x4B],
      Registers::XA_BIAS_LOW => [0x4C, 0x4D],
      Registers::XA_BIAS_HIGH => [0x4E, 0x4F],
      Registers::YA_BIAS_LOW => [0x50, 0x51],
      Registers::YA_BIAS_HIGH => [0x52, 0x53],
      Registers::ZA_BIAS_LOW => [0x54, 0x55],
      Registers::ZA_BIAS_HIGH => [0x56, 0x57],

      Registers::FILT_CTRL => [0x5C, 0x5D],
      Registers::RANG_MDL => [0x5E, 0x5F],
      Registers::MSC_CTRL => [0x60, 0x61],
      Registers::UP_SCALE => [0x62, 0x63],
      Registers::DEC_RATE => [0x64, 0x65],

      Registers::FIRM_REV => [0x6C, 0x6D],
      Registers::FIRM_DM => [0x6E, 0x6F],
      Registers::FIRM_Y => [0x70, 0x71],
      Registers::PROD_ID => [0x72, 0x73],
      Registers::SERIAL_NUM => [0x74, 0x75],
      Registers::USER_SCR_1 => [0x76, 0x77],
      Registers::USER_SCR_2 => [0x78, 0x79],
      Registers::USER_SCR_3 => [0x7A, 0x7B],
      Registers::FLSHCNT_LOW => [0x7C, 0x7D],
      Registers::FLSHCNT_HIGH => [0x7E, 0x7F],
    }
  }

  fn is_writeable(&self) -> bool {
    match (self) {
      Registers::DIAG_STAT => false,
      Registers::X_GYRO_LOW => false,
      Registers::X_GYRO_OUT => false,
      Registers::Y_GYRO_LOW => false,
      Registers::Y_GYRO_OUT => false,
      Registers::Z_GYRO_LOW => false,
      Registers::Z_GYRO_OUT => false,
      Registers::X_ACCL_LOW => false,
      Registers::X_ACCL_OUT => false,
      Registers::Y_ACCL_LOW => false,
      Registers::Y_ACCL_OUT => false,
      Registers::Z_ACCL_LOW => false,
      Registers::Z_ACCL_OUT => false,
      Registers::TEMP_OUT => false,
      Registers::TIME_STAMP => false,

      Registers::DATA_CNTR => false,
      Registers::X_DELTANG_LOW => false,
      Registers::X_DELTANG_OUT => false,
      Registers::Y_DELTANG_LOW => false,
      Registers::Y_DELTANG_OUT => false,
      Registers::Z_DELTANG_LOW => false,
      Registers::Z_DELTANG_OUT => false,
      Registers::X_DELTVEL_LOW => false,
      Registers::X_DELTVEL_OUT => false,
      Registers::Y_DELTVEL_LOW => false,
      Registers::Y_DELTVEL_OUT => false,
      Registers::Z_DELTVEL_LOW => false,
      Registers::Z_DELTVEL_OUT => false,

      Registers::XG_BIAS_LOW => true,
      Registers::XG_BIAS_HIGH => true,
      Registers::YG_BIAS_LOW => true,
      Registers::YG_BIAS_HIGH => true,
      Registers::ZG_BIAS_LOW => true,
      Registers::ZG_BIAS_HIGH => true,
      Registers::XA_BIAS_LOW => true,
      Registers::XA_BIAS_HIGH => true,
      Registers::YA_BIAS_LOW => true,
      Registers::YA_BIAS_HIGH => true,
      Registers::ZA_BIAS_LOW => true,
      Registers::ZA_BIAS_HIGH => true,

      Registers::FILT_CTRL => true,
      Registers::RANG_MDL => false,
      Registers::MSC_CTRL => true,
      Registers::UP_SCALE => true,
      Registers::DEC_RATE => true,

      Registers::FIRM_REV => false,
      Registers::FIRM_DM => false,
      Registers::FIRM_Y => false,
      Registers::PROD_ID => false,
      Registers::SERIAL_NUM => false,
      Registers::USER_SCR_1 => true,
      Registers::USER_SCR_2 => true,
      Registers::USER_SCR_3 => true,
      Registers::FLSHCNT_LOW => false,
      Registers::FLSHCNT_HIGH => false,
    }
  }
}

impl fmt::Display for Registers {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "{}",
      match (self) {
        Registers::DIAG_STAT => "DIAG_STAT",
        Registers::X_GYRO_LOW => "X_GYRO_LOW",
        Registers::X_GYRO_OUT => "X_GYRO_OUT",
        Registers::Y_GYRO_LOW => "Y_GYRO_LOW",
        Registers::Y_GYRO_OUT => "Y_GYRO_OUT",
        Registers::Z_GYRO_LOW => "Z_GYRO_LOW",
        Registers::Z_GYRO_OUT => "Z_GYRO_OUT",
        Registers::X_ACCL_LOW => "X_ACCL_LOW",
        Registers::X_ACCL_OUT => "X_ACCL_OUT",
        Registers::Y_ACCL_LOW => "Y_ACCL_LOW",
        Registers::Y_ACCL_OUT => "Y_ACCL_OUT",
        Registers::Z_ACCL_LOW => "Z_ACCL_LOW",
        Registers::Z_ACCL_OUT => "Z_ACCL_OUT",
        Registers::TEMP_OUT => "TEMP_OUT",
        Registers::TIME_STAMP => "TIME_STAMP",

        Registers::DATA_CNTR => "DATA_CNTR",
        Registers::X_DELTANG_LOW => "X_DELTANG_LOW",
        Registers::X_DELTANG_OUT => "X_DELTANG_OUT",
        Registers::Y_DELTANG_LOW => "Y_DELTANG_LOW",
        Registers::Y_DELTANG_OUT => "Y_DELTANG_OUT",
        Registers::Z_DELTANG_LOW => "Z_DELTANG_LOW",
        Registers::Z_DELTANG_OUT => "Z_DELTANG_OUT",
        Registers::X_DELTVEL_LOW => "X_DELTVEL_LOW",
        Registers::X_DELTVEL_OUT => "X_DELTVEL_OUT",
        Registers::Y_DELTVEL_LOW => "Y_DELTVEL_LOW",
        Registers::Y_DELTVEL_OUT => "Y_DELTVEL_OUT",
        Registers::Z_DELTVEL_LOW => "Z_DELTVEL_LOW",
        Registers::Z_DELTVEL_OUT => "Z_DELTVEL_OUT",

        Registers::XG_BIAS_LOW => "XG_BIAS_LOW",
        Registers::XG_BIAS_HIGH => "XG_BIAS_HIGH",
        Registers::YG_BIAS_LOW => "YG_BIAS_LOW",
        Registers::YG_BIAS_HIGH => "YG_BIAS_HIGH",
        Registers::ZG_BIAS_LOW => "ZG_BIAS_LOW",
        Registers::ZG_BIAS_HIGH => "ZG_BIAS_HIGH",
        Registers::XA_BIAS_LOW => "XA_BIAS_LOW ",
        Registers::XA_BIAS_HIGH => "XA_BIAS_HIGH",
        Registers::YA_BIAS_LOW => "YA_BIAS_LOW",
        Registers::YA_BIAS_HIGH => "YA_BIAS_HIGH",
        Registers::ZA_BIAS_LOW => "ZA_BIAS_LOW",
        Registers::ZA_BIAS_HIGH => "ZA_BIAS_HIGH",

        Registers::FILT_CTRL => "FILT_CTRL",
        Registers::RANG_MDL => "RANG_MDL",
        Registers::MSC_CTRL => "MSC_CTRL",
        Registers::UP_SCALE => "UP_SCALE",
        Registers::DEC_RATE => "DEC_RATE",

        Registers::FIRM_REV => "FIRM_REV",
        Registers::FIRM_DM => "FIRM_DM",
        Registers::FIRM_Y => "FIRM_Y",
        Registers::PROD_ID => "PROD_ID",
        Registers::SERIAL_NUM => "SERIAL_NUM",
        Registers::USER_SCR_1 => "USER_SCR_1",
        Registers::USER_SCR_2 => "USER_SCR_2",
        Registers::USER_SCR_3 => "USER_SCR_3",
        Registers::FLSHCNT_LOW => "FLSHCNT_LOW",
        Registers::FLSHCNT_HIGH => "FLSHCNT_HIGH",
      }
    )
  }
}

/// The controlling data structure for the ADI IMU we use
/// Abstracts almost all actual usage of the device
///
/// Whatever is public should work, but for now just stick to
///
/// reading / writing to dec rate,
/// validate,
/// and using burst reads.
///
/// printing has conversions for types. I am too busy to turn that into
/// it's own functions, so someone (or I) will have to make them their own
/// functions later
pub struct AdisIMUDriver {
  /// The internal pins and spi of the device
  internals: DriverInternals,

  config: ConfigValues,
}

impl AdisIMUDriver {
  pub fn reset(&mut self) {
    self.internals.enable_reset();
    sleep(Duration::from_millis(500)); // Arbitrary
    self.internals.disable_reset();
    // Documented required time + some leeway
    sleep(RESET_DOWNTIME + Duration::from_millis(100));
  }

  /// Initialize the driver using established GPIO pins
  pub fn initialize(
    mut spi: Spidev,
    data_ready: Pin,
    nreset: Pin,
    nchip_select: Pin,
  ) -> DriverResult<AdisIMUDriver> {
    // initialize everything
    let mut driver = AdisIMUDriver {
      internals: DriverInternals::initialize(
        spi,
        data_ready,
        nreset,
        nchip_select,
      )?,
      config: ConfigValues::default(),
    };
    // Wait until the time to power on has passed / the IMU just powered on
    sleep(POWER_ON_START_UP_TIME + Duration::from_millis(100));
    // Disable chip select to not fry this thing
    driver.internals.disable_chip_select();
    // Reset in case this is NOT the first initialization / the IMU powered on
    // a long time ago and to clear all internals
    driver.reset();
    Ok(driver)
  }

  pub fn validate(&mut self) -> bool {
    self.read_prod_id().unwrap_or(0) == 0x4074
  }

  fn read_16_bit(&mut self, reg: Registers) -> DriverResult<i16> {
    // Setup buffers
    let mut tx_buf: [u8; 6] = [0; 6];
    tx_buf[1] = reg.get_address()[0];

    let mut rx_buf: [u8; 6] = [0; 6];

    // Do the transfer (CS is handled internally)
    self.internals.spi_transfer(&tx_buf, &mut rx_buf)?;

    // Parse recieved bytes
    Ok(i16::from_be_bytes([rx_buf[2], rx_buf[5]]))
  }

  /// Reads a 16 bit register with 3x redundancy to ensure validity.
  ///
  ///
  /// If a majority of the results contain a value, it will return it.
  ///
  /// If all values are different, it will return a `ErrorKind::Other` Error.
  ///
  /// This is primaryily done for values that may be corrupted by DR overlap
  fn read_16_bit_redundant(&mut self, reg: Registers) -> DriverResult<i16> {
    // Setup buffers
    let mut tx_buf: [u8; 10] = [0; 10];
    tx_buf[1] = reg.get_address()[0];
    tx_buf[3] = tx_buf[1];
    tx_buf[5] = tx_buf[1];

    let mut rx_buf: [u8; 10] = [0; 10];

    // Do the transfer (CS is handled internally)
    self.internals.spi_transfer(&tx_buf, &mut rx_buf)?;

    let results = [
      i16::from_be_bytes([rx_buf[2], rx_buf[5]]),
      i16::from_be_bytes([rx_buf[4], rx_buf[7]]),
      i16::from_be_bytes([rx_buf[6], rx_buf[9]]),
    ];

    if results[0] == results[1] || results[0] == results[2] {
      Ok(results[0])
    } else if results[1] == results[2] {
      Ok(results[1])
    } else {
      Err(
        InvalidDataError::new(
          "Potentially corrupted read value from register. No majority result.",
        )
        .into(),
      )
    }
  }

  /// Attempts a redundant read up to `n` times, returning on the first success
  fn repeat_read_16_bit_redundant(
    &mut self,
    reg: Registers,
    repeat_count: usize,
  ) -> DriverResult<i16> {
    // attempt twice
    for _ in 0..repeat_count {
      if let Ok(x) = self.read_16_bit_redundant(reg) {
        return Ok(x);
      }
    }
    // on third attempt, just return result either way
    self.read_16_bit_redundant(reg)
  }

  fn write_to_reg(&mut self, reg: Registers, data: u16) -> DriverResult<()> {
    if !reg.is_writeable() {
      return Err(
        io::Error::new(
          io::ErrorKind::InvalidInput,
          format!("Cannot write to register {}", reg),
        )
        .into(),
      );
    }
    // We need be bits, le bytes
    let data_bytes: [u8; 2] = data.to_be_bytes();

    let location = reg.get_address();
    // Setup buffers
    let mut tx_buf: [u8; 4] = [0; 4];
    tx_buf[1] = location[0] | 0x80;
    tx_buf[0] = data_bytes[1];
    tx_buf[3] = location[1] | 0x80;
    tx_buf[2] = data_bytes[0];

    // Do the transfer (CS is handled internally)
    self.internals.spi_write(&tx_buf)?;

    // Parse recieved bytes
    Ok(())
  }

  pub fn write_dec_rate(&mut self, rate: u16) -> DriverResult<()> {
    self.write_to_reg(Registers::DEC_RATE, rate)?;
    sleep(Duration::from_micros(200 + 100));
    Ok(())
  }
  pub fn read_dec_rate(&mut self) -> DriverResult<i16> {
    self.read_16_bit(Registers::DEC_RATE)
  }
  pub fn read_prod_id(&mut self) -> DriverResult<i16> {
    self.read_16_bit(Registers::PROD_ID)
  }
  pub fn read_data_counter(&mut self) -> DriverResult<i16> {
    self.read_16_bit(Registers::DATA_CNTR)
  }
  pub fn read_msc_ctrl(&mut self) -> DriverResult<i16> {
    let new = self.read_16_bit(Registers::MSC_CTRL);
    if let Ok(x) = new {
      self.config.msc_control_reg = x as u16;
      return Ok(x);
    } else {
      return new;
    }
  }

  pub fn burst_read_gyro_16(
    &mut self,
  ) -> DriverResult<(GenericData, GyroReadData)> {
    // TODO Configure burst mode + burst select
    if self.config.get_last_burst_sel() {
      self.config.set_burst_sel(false);
      self.write_to_reg(Registers::MSC_CTRL, self.config.msc_control_reg)?;
      sleep(Duration::from_micros(100)); // prob unneeded : TODO : CHECK
    }

    // Setup buffers
    let mut tx_buf: [u8; 22] = [0; 22];
    tx_buf[1] = GLOB_CMD[0];

    let mut rx_buf: [u8; 22] = [0; 22];

    self.internals.spi_transfer(&tx_buf, &mut rx_buf)?;

    // 16 bit data
    let gyro = [
      (i16::from_le_bytes(rx_buf[4..6].try_into().unwrap()) as i32) << 16,
      (i16::from_le_bytes(rx_buf[6..8].try_into().unwrap()) as i32) << 16,
      (i16::from_le_bytes(rx_buf[8..10].try_into().unwrap()) as i32) << 16,
    ];

    let accel = [
      (i16::from_le_bytes(rx_buf[10..12].try_into().unwrap()) as i32) << 16,
      (i16::from_le_bytes(rx_buf[12..14].try_into().unwrap()) as i32) << 16,
      (i16::from_le_bytes(rx_buf[14..16].try_into().unwrap()) as i32) << 16,
    ];

    // 16 bit data
    let diagnostic_stat: DiagnosticStats =
      u16::from_le_bytes(rx_buf[2..4].try_into().unwrap()).into();
    let temp = i16::from_le_bytes(rx_buf[16..18].try_into().unwrap());
    let data_counter = i16::from_le_bytes(rx_buf[18..20].try_into().unwrap());

    let mut sum: u16 = 0;
    for i in 2..20 {
      sum += rx_buf[i] as u16;
    }
    if !diagnostic_stat.is_empty() {
      return Err(diagnostic_stat.into());
    }

    return if sum == u16::from_le_bytes(rx_buf[20..22].try_into().unwrap()) {
      Ok((
        GenericData { temp, data_counter },
        GyroReadData { gyro, accel },
      ))
    } else {
      Err(InvalidDataError::new("Checksum Failure").into())
    };
  }

  pub fn burst_read_delta_16(
    &mut self,
  ) -> DriverResult<(GenericData, DeltaReadData)> {
    // TODO Configure burst mode + burst select
    if !self.config.get_last_burst_sel() {
      self.config.set_burst_sel(true);
      self.write_to_reg(Registers::MSC_CTRL, self.config.msc_control_reg)?;
      sleep(Duration::from_micros(100)); // prob unneeded : TODO : CHECK
    }

    // Setup buffers
    let mut tx_buf: [u8; 22] = [0; 22];
    tx_buf[1] = GLOB_CMD[0];

    let mut rx_buf: [u8; 22] = [0; 22];

    self.internals.spi_transfer(&tx_buf, &mut rx_buf)?;

    // 16 bit data
    let delta_angle = [
      (i16::from_le_bytes(rx_buf[4..6].try_into().unwrap()) as i32) << 16,
      (i16::from_le_bytes(rx_buf[6..8].try_into().unwrap()) as i32) << 16,
      (i16::from_le_bytes(rx_buf[8..10].try_into().unwrap()) as i32) << 16,
    ];

    let delta_velocity = [
      (i16::from_le_bytes(rx_buf[10..12].try_into().unwrap()) as i32) << 16,
      (i16::from_le_bytes(rx_buf[12..14].try_into().unwrap()) as i32) << 16,
      (i16::from_le_bytes(rx_buf[14..16].try_into().unwrap()) as i32) << 16,
    ];

    // 16 bit data
    let diagnostic_stat: DiagnosticStats =
      u16::from_le_bytes(rx_buf[2..4].try_into().unwrap()).into();
    let temp = i16::from_le_bytes(rx_buf[16..18].try_into().unwrap());
    let data_counter = i16::from_le_bytes(rx_buf[18..20].try_into().unwrap());

    let mut sum: u16 = 0;
    for i in 2..20 {
      sum += rx_buf[i] as u16;
    }
    if !diagnostic_stat.is_empty() {
      return Err(diagnostic_stat.into());
    }

    //pub fn read_control_registers(&mut self) -> DriverResult<ConfigValues> {

    //}

    return if sum == u16::from_le_bytes(rx_buf[20..22].try_into().unwrap()) {
      Ok((
        GenericData { temp, data_counter },
        DeltaReadData {
          delta_angle,
          delta_velocity,
        },
      ))
    } else {
      Err(InvalidDataError::new("Checksum Failure").into())
    };
  }

  /// Does both gyro and delta burst reads
  fn burst_read_gyro_and_delta(
    &mut self,
  ) -> DriverResult<((GenericData, GyroReadData), (GenericData, DeltaReadData))>
  {
    const READ_START_OFFSET: usize = 16;

    let mut tx_buf: [u8; 22 + READ_START_OFFSET] = [0; 22 + READ_START_OFFSET];

    let mut tx_buf_b: [u8; 22 + READ_START_OFFSET] =
      [0; 22 + READ_START_OFFSET];

    // swap to gyro burst
    let msc_reg_addr = Registers::MSC_CTRL.get_address();
    self.config.msc_control_reg &= 0xFEFF;
    {
      let msc_write_bytes: [u8; 2] = self.config.msc_control_reg.to_be_bytes();

      tx_buf[1] = msc_reg_addr[0] | 0x80;
      tx_buf[0] = msc_write_bytes[1];
      tx_buf[3] = msc_reg_addr[1] | 0x80;
      tx_buf[2] = msc_write_bytes[0];
    }
    tx_buf[READ_START_OFFSET + 1] = GLOB_CMD[0];

    // swap to delta burst
    self.config.msc_control_reg |= 0x0100;
    {
      let msc_write_bytes: [u8; 2] = self.config.msc_control_reg.to_be_bytes();

      tx_buf_b[1] = msc_reg_addr[0] | 0x80;
      tx_buf_b[0] = msc_write_bytes[1];
      tx_buf_b[3] = msc_reg_addr[1] | 0x80;
      tx_buf_b[2] = msc_write_bytes[0];
    }
    tx_buf_b[READ_START_OFFSET + 1] = GLOB_CMD[0];

    let mut rx_buf: [u8; 22 + READ_START_OFFSET] = [0; 22 + READ_START_OFFSET];
    let mut rx_buf_b: [u8; 22 + READ_START_OFFSET] =
      [0; 22 + READ_START_OFFSET];

    // do spi stuff
    self.internals.spi_transfer(&tx_buf, &mut rx_buf)?;
    sleep(Duration::from_micros(30));
    self.internals.spi_transfer(&tx_buf_b, &mut rx_buf_b)?;

    let gyro_read = GyroReadData {
      gyro: [
        (i16::from_le_bytes(
          rx_buf[READ_START_OFFSET + 4..READ_START_OFFSET + 6]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
        (i16::from_le_bytes(
          rx_buf[READ_START_OFFSET + 6..READ_START_OFFSET + 8]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
        (i16::from_le_bytes(
          rx_buf[READ_START_OFFSET + 8..READ_START_OFFSET + 10]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
      ],

      accel: [
        (i16::from_le_bytes(
          rx_buf[READ_START_OFFSET + 10..READ_START_OFFSET + 12]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
        (i16::from_le_bytes(
          rx_buf[READ_START_OFFSET + 12..READ_START_OFFSET + 14]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
        (i16::from_le_bytes(
          rx_buf[READ_START_OFFSET + 14..READ_START_OFFSET + 16]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
      ],
    };

    // 16 bit data
    {
      let diagnostic_stat: DiagnosticStats = u16::from_le_bytes(
        rx_buf[READ_START_OFFSET + 2..READ_START_OFFSET + 4]
          .try_into()
          .unwrap(),
      )
      .into();
      if !diagnostic_stat.is_empty() {
        return Err(diagnostic_stat.into());
      }
    }
    let gyro_gen = GenericData {
      temp: i16::from_le_bytes(
        rx_buf[READ_START_OFFSET + 16..READ_START_OFFSET + 18]
          .try_into()
          .unwrap(),
      ),
      data_counter: i16::from_le_bytes(
        rx_buf[READ_START_OFFSET + 18..READ_START_OFFSET + 20]
          .try_into()
          .unwrap(),
      ),
    };

    {
      let mut sum: u16 = 0;
      for i in READ_START_OFFSET + 2..READ_START_OFFSET + 20 {
        sum += rx_buf[i] as u16;
      }
      if sum
        != u16::from_le_bytes(
          rx_buf[READ_START_OFFSET + 20..READ_START_OFFSET + 22]
            .try_into()
            .unwrap(),
        )
      {
        return Err(InvalidDataError::new("Gyro Checksum Failure").into());
      }
    }

    let delta_read = DeltaReadData {
      delta_angle: [
        (i16::from_le_bytes(
          rx_buf_b[READ_START_OFFSET + 4..READ_START_OFFSET + 6]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
        (i16::from_le_bytes(
          rx_buf_b[READ_START_OFFSET + 6..READ_START_OFFSET + 8]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
        (i16::from_le_bytes(
          rx_buf_b[READ_START_OFFSET + 8..READ_START_OFFSET + 10]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
      ],

      delta_velocity: [
        (i16::from_le_bytes(
          rx_buf_b[READ_START_OFFSET + 10..READ_START_OFFSET + 12]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
        (i16::from_le_bytes(
          rx_buf_b[READ_START_OFFSET + 12..READ_START_OFFSET + 14]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
        (i16::from_le_bytes(
          rx_buf_b[READ_START_OFFSET + 14..READ_START_OFFSET + 16]
            .try_into()
            .unwrap(),
        ) as i32)
          << 16,
      ],
    };

    // 16 bit data
    {
      let diagnostic_stat: DiagnosticStats = u16::from_le_bytes(
        rx_buf_b[READ_START_OFFSET + 2..READ_START_OFFSET + 4]
          .try_into()
          .unwrap(),
      )
      .into();
      if !diagnostic_stat.is_empty() {
        return Err(diagnostic_stat.into());
      }
    }
    let delta_gen = GenericData {
      temp: i16::from_le_bytes(
        rx_buf_b[READ_START_OFFSET + 16..READ_START_OFFSET + 18]
          .try_into()
          .unwrap(),
      ),
      data_counter: i16::from_le_bytes(
        rx_buf_b[READ_START_OFFSET + 18..READ_START_OFFSET + 20]
          .try_into()
          .unwrap(),
      ),
    };
    {
      let mut sum: u16 = 0;
      for i in READ_START_OFFSET + 2..READ_START_OFFSET + 20 {
        sum += rx_buf_b[i] as u16;
      }
      if sum
        != u16::from_le_bytes(
          rx_buf_b[READ_START_OFFSET + 20..READ_START_OFFSET + 22]
            .try_into()
            .unwrap(),
        )
      {
        return Err(InvalidDataError::new("Delta Checksum Failure").into());
      }
    }

    Ok(((gyro_gen, gyro_read), (delta_gen, delta_read)))
  }
}
