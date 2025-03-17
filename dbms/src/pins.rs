use common::comm::gpio::Gpio;
use common::comm::{ADCKind, DarcyBmsADC};
use std::sync::LazyLock;
use std::{collections::HashMap, process::Command};

pub static GPIO_CONTROLLERS: LazyLock<Vec<Gpio>> =
  LazyLock::new(open_controllers);
pub static SPI_INFO: LazyLock<HashMap<ADCKind, SpiInfo>> =
  LazyLock::new(get_spi_info);

pub struct GpioInfo {
  pub controller: usize,
  pub pin_num: usize,
}

pub struct SpiInfo {
  pub spi_bus: &'static str,
  pub cs: Option<GpioInfo>,
  pub drdy: Option<GpioInfo>,
}

pub fn open_controllers() -> Vec<Gpio> {
  (0..=3).map(Gpio::open_controller).collect()
}

pub fn get_spi_info() -> HashMap<ADCKind, SpiInfo> {
  let mut map = HashMap::new();

  map.insert(
    ADCKind::DarcyBms(DarcyBmsADC::VBatAnd5V),
    SpiInfo {
      spi_bus: "/dev/spidev0.0",
      cs: Some(GpioInfo {
        controller: 0,
        pin_num: 5,
      }),
      drdy: Some(GpioInfo {
        controller: 1,
        pin_num: 30,
      }),
    },
  );

  map
}

pub fn config_pins() {
  // P9 GPIO
  config_pin("p9.5", "gpio"); // CS#
  config_pin("p9.11", "gpio"); // DRDY#
  config_pin("p9.13", "gpio"); // FLT#1
  config_pin("p9.14", "gpio"); // FLT#2
  config_pin("p9.15", "gpio"); // FLT#3
  config_pin("p9.16", "gpio"); // FLT#4
  config_pin("p9.24", "gpio"); // EN1
  config_pin("p9.25", "gpio"); // EN2
  config_pin("p9.26", "gpio"); // EN3
  config_pin("p9.27", "gpio"); // EN4

  // SPI 0
  config_pin("p9_18", "spi"); // MISO
  config_pin("p9_21", "spi"); // MOSI
  config_pin("p9_22", "spi_sclk"); // SCLK
}

/* The purpose of this function is to deprecate the pins.sh file by handling
all of the 'config-pin' calls internally
 */
fn config_pin(pin: &str, mode: &str) {
  match Command::new("config-pin").args([pin, mode]).output() {
    Ok(result) => {
      if result.status.success() {
        println!("Configured {pin} as {mode}");
      } else {
        println!("Configuration did not work for {pin} -> {mode}");
      }
    }

    Err(e) => {
      eprintln!("Failed to execute config-pin for {pin} -> {mode}, Error: {e}");
    }
  }
}
