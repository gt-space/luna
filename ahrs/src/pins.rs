use common::comm::gpio::Gpio;
use once_cell::sync::Lazy;
use std::process::Command;

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> =
  Lazy::new(|| (0..=3).map(Gpio::open_controller).collect());

pub fn config_pins() {
  // P8 GPIO
  config_pin("p8.34", "gpio"); // IMU-NRESET
  config_pin("p8.30", "gpio"); // IMU-DR
  config_pin("p8.46", "gpio"); // CAM-EN

  // SPI1
  config_pin("p9.31", "spi_sclk");
  config_pin("p9.29", "spi");
  config_pin("p9.30", "spi");
  config_pin("p9.19", "spi_cs"); // MAG-CS
  config_pin("p9.20", "spi_cs"); // BAR-CS

  // SPI0
  config_pin("p9.22", "spi_sclk");
  config_pin("p9.21", "spi");
  config_pin("p9.18", "spi");
  config_pin("p9.17", "spi_cs"); // IMU-CS
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
