use crate::pins::{GPIO_CONTROLLERS, SPI_INFO};
use common::comm::gpio::{PinMode::Output, PinValue::High};
use common::comm::gpio::Gpio;
use common::comm::{ADCKind, ThermocoupleModADC};
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

fn open_controllers() -> Vec<Gpio> {
    (0..=3).map(Gpio::open_controller).collect()
}

fn get_spi_info() -> HashMap<ADCKind, SpiInfo> {
    let mut map = HashMap::new();

    map.insert(
        ADCKind::Thermocouple(ThermocoupleModADC::Bank1),
        SpiInfo {
            spi_bus: "/dev/spidev0.0",
            cs: Some(GpioInfo { controller: 0, pin_num: 30 }), // p9.11
            drdy: None,
        },
    );
    map.insert(
        ADCKind::Thermocouple(ThermocoupleModADC::Bank2),
        SpiInfo {
            spi_bus: "/dev/spidev0.0",
            cs: Some(GpioInfo { controller: 1, pin_num: 28 }), // p9.12
            drdy: None,
        },
    );
    map.insert(
        ADCKind::Thermocouple(ThermocoupleModADC::Bank3),
        SpiInfo {
            spi_bus: "/dev/spidev0.0",
            cs: Some(GpioInfo { controller: 1, pin_num: 18 }), // p9.14
            drdy: None,
        },
    );
    map.insert(
        ADCKind::Thermocouple(ThermocoupleModADC::Bank4),
        SpiInfo {
            spi_bus: "/dev/spidev0.0",
            cs: Some(GpioInfo { controller: 0, pin_num: 5 }),  // p9.17
            drdy: None,
        },
    );

    map
}

pub fn init_cs() {
  for spi in SPI_INFO.values() {
      if let Some(cs) = &spi.cs {
          let mut pin = GPIO_CONTROLLERS[cs.controller].get_pin(cs.pin_num);
          pin.mode(Output);
          pin.digital_write(High);
      }
  }
}

pub fn config_pins() {
    config_pin("p9.18", "spi");      // MOSI
    config_pin("p9.21", "spi");      // MISO
    config_pin("p9.22", "spi_sclk"); // SCLK

    config_pin("p9.11", "gpio"); 
    config_pin("p9.12", "gpio"); 
    config_pin("p9.14", "gpio"); 
    config_pin("p9.17", "gpio"); 
}

fn config_pin(pin: &str, mode: &str) {
    match Command::new("config-pin").args([pin, mode]).output() {
        Ok(result) if result.status.success() => {
            println!("Configured {} as {}", pin, mode);
        }
        Ok(_) => {
            eprintln!("Configuration failed for {} as {}", pin, mode);
        }
        Err(e) => {
            eprintln!(
                "Error executing config-pin for {} as {}: {}",
                pin,
                mode,
                e
            );
        }
    }
}
