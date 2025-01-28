use std::{collections::HashMap, process::Command};
use std::sync::LazyLock;
use common::comm::gpio::Gpio;
use common::comm::{ADCKind, VespulaBmsADC};


pub static GPIO_CONTROLLERS: LazyLock<Vec<Gpio>> = LazyLock::new(|| open_controllers());
pub static SPI_INFO: LazyLock<HashMap<ADCKind, SpiInfo>> = LazyLock::new(|| get_spi_info());

pub struct GpioInfo {
  pub controller: usize,
  pub pin_num: usize
}

pub struct SpiInfo {
  pub spi_bus: &'static str,
  pub cs: Option<GpioInfo>,
  pub drdy: Option<GpioInfo>
}

pub fn open_controllers() -> Vec<Gpio> {
  (0..=3).map(Gpio::open_controller).collect()
}

pub fn get_spi_info() -> HashMap<ADCKind, SpiInfo> {
  let mut map = HashMap::new();

  map.insert(ADCKind::VespulaBms(VespulaBmsADC::VBatUmbCharge), SpiInfo {spi_bus: "/dev/spidev0.0", cs: Some(GpioInfo { controller: 0, pin_num: 30 }), drdy: Some(GpioInfo { controller: 1, pin_num: 28 })});
  map.insert(ADCKind::VespulaBms(VespulaBmsADC::SamAnd5V), SpiInfo {spi_bus: "/dev/spidev0.0", cs: Some(GpioInfo { controller: 0, pin_num: 31 }), drdy: Some(GpioInfo { controller: 1, pin_num: 18 })});

  map
}

pub fn config_pins() {
  // P9 GPIO
  config_pin("p9.11", "gpio");
  config_pin("p9.12", "gpio");
  config_pin("p9.13", "gpio");
  config_pin("p9.14", "gpio");
  config_pin("p9.15", "gpio");
  config_pin("p9.16", "gpio");
  config_pin("p9.23", "gpio");
  config_pin("p9.24", "gpio");
  config_pin("p9.26", "gpio");

  // P8 GPIO
  config_pin("p8.07", "gpio");
  config_pin("p8.08", "gpio");
  config_pin("p8.09", "gpio");
  config_pin("p8.10", "gpio");
  config_pin("p8.11", "gpio");
  config_pin("p8.12", "gpio");
  config_pin("p8.13", "gpio");
  config_pin("p8.14", "gpio");
  config_pin("p8.18", "gpio");
  config_pin("p8.19", "gpio");
  config_pin("p8.21", "gpio");
  config_pin("p8.23", "gpio");
  config_pin("p8.30", "gpio");

  // SPI 0
  config_pin("p9_18", "spi");
  config_pin("p9_21", "spi");
  config_pin("p9_22", "spi_sclk");
}


/* The purpose of this function is to deprecate the pins.sh file by handling
all of the 'config-pin' calls internally
 */
fn config_pin(pin: &str, mode: &str) {
  match Command::new("config-pin")
    .args([pin, mode])
    .output() {
      Ok(result) => {
        if result.status.success() {
          println!("Configured {pin} as {mode}");
        } else {
          println!("Configuration did not work for {pin} -> {mode}");
        }
      },

      Err(e) => {
        eprintln!("Failed to execute config-pin for {pin} -> {mode}, Error: {e}");
      }
    }
}