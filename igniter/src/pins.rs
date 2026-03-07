use crate::{IgniterId, IGNITER_ID};
use common::comm::{
  gpio::RpiGpioController,
  IgniterRev1ADC,
};
use std::{
  collections::HashMap, 
  sync::LazyLock
};

// TODO: We do not need maps, can definitely do with vectors since we are
// not changing the mappings after initialization

/// GPIO controller that interacts with the kernel to give us access to GPIO pins
pub static GPIO_CONTROLLER: LazyLock<RpiGpioController> =
  LazyLock::new(|| RpiGpioController::open_controller().expect("Failed to open RPi GPIO"));

/// Mapping of igniter channel number to the GPIO pin number that enables it
pub static IGNITER_CHANNEL_ENABLE_PINS: LazyLock<HashMap<u8, u8>> =
  LazyLock::new(get_igniter_channel_enable_pins);

/// Mapping of cc channel to the GPIO pin that reads fault for that channel
pub static CC_FAULT_PINS: LazyLock<HashMap<u8, u8>> =
  LazyLock::new(get_cc_fault_pins);

/// Mapping of adcs on igniter board to the relevant spi information
pub static SPI_INFO: LazyLock<HashMap<IgniterRev1ADC, SpiInfo>> =
  LazyLock::new(get_spi_info);

pub struct SpiInfo {
  pub spi_bus: &'static str,
  pub cs: Option<u8>,
  pub drdy: Option<u8>,
}

/// Returns a map consisting of (igniter channel #, gpio pin number),
/// where gpio pin number is the # of the pin that enables the channel
pub fn get_igniter_channel_enable_pins() -> HashMap<u8, u8> {
  let mut map = HashMap::new();
  map.insert(1, 2);
  map.insert(2, 3);
  map.insert(3, 4);
  map.insert(4, 12);
  map.insert(5, 13);
  map.insert(6, 14);
  map
}

/// Returns a map consisting of (cc channel number, gpio pin number),
/// where gpio pin number is the # of the pin that reads fault for that channel.
/// Takes into account different cc fault pinouts for different igniter board 
/// devices (A or B).
pub fn get_cc_fault_pins() -> HashMap<u8, u8> {
  let mut map = HashMap::new();
  match *IGNITER_ID {
    IgniterId::Igniter1 => {
      map.insert(4, 0);
      map.insert(5, 1);
      map.insert(6, 5);
    }
    IgniterId::Igniter2 => {
      map.insert(4, 6);
      map.insert(5, 15);
      map.insert(6, 26);
    }
  }
  
  map
}

// Returns a map consisting of (adc type, relevant spi information)
pub fn get_spi_info() -> HashMap<IgniterRev1ADC, SpiInfo> {
  let mut map = HashMap::new();

  map.insert(
    IgniterRev1ADC::Continuity,
    SpiInfo {
      spi_bus: "/dev/spidev0.0",
      cs: Some(8),
      drdy: Some(16),
    },
  );
  map.insert(
    IgniterRev1ADC::ConstantCurrent,
    SpiInfo {
      spi_bus: "/dev/spidev1.0",
      cs: Some(18),
      drdy: Some(23),
    },
  );
  map.insert(
    IgniterRev1ADC::ConstantVoltage,
    SpiInfo {
      spi_bus: "/dev/spidev0.1",
      cs: Some(7),
      drdy: Some(22),
    },
  );
  map.insert(
    IgniterRev1ADC::PowerMonitoring,
    SpiInfo {
      spi_bus: "/dev/spidev1.1",
      cs: Some(17),
      drdy: Some(24),
    },
  );

  map
}
