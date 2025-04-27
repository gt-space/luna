use crate::communication::get_hostname;
use crate::{SamVersion, SAM_VERSION};
use common::comm::gpio::Gpio;
use common::comm::ADCKind;
use common::comm::{SamRev3ADC, SamRev4FlightADC, SamRev4GndADC};
use std::sync::LazyLock;
use std::{collections::HashMap, process::Command};

pub static GPIO_CONTROLLERS: LazyLock<Vec<Gpio>> =
  LazyLock::new(open_controllers);
pub static VALVE_PINS: LazyLock<HashMap<u32, GpioInfo>> =
  LazyLock::new(get_valve_mappings);
pub static VALVE_CURRENT_PINS: LazyLock<HashMap<u8, GpioInfo>> =
  LazyLock::new(get_valve_current_sel_mappings);
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

// valve pinouts checked!
// u32 cuz that is the integer type that FC sends to SAM
pub fn get_valve_mappings() -> HashMap<u32, GpioInfo> {
  let mut map = HashMap::new();

  match *SAM_VERSION {
    SamVersion::Rev3 => {
      map.insert(
        1,
        GpioInfo {
          controller: 0,
          pin_num: 8,
        },
      );
      map.insert(
        2,
        GpioInfo {
          controller: 2,
          pin_num: 16,
        },
      );
      map.insert(
        3,
        GpioInfo {
          controller: 2,
          pin_num: 17,
        },
      );
      map.insert(
        4,
        GpioInfo {
          controller: 2,
          pin_num: 25,
        },
      );
      map.insert(
        5,
        GpioInfo {
          controller: 2,
          pin_num: 1,
        },
      );
      map.insert(
        6,
        GpioInfo {
          controller: 1,
          pin_num: 14,
        },
      );
    }

    SamVersion::Rev4Ground => {
      map.insert(
        1,
        GpioInfo {
          controller: 1,
          pin_num: 0,
        },
      );
      map.insert(
        2,
        GpioInfo {
          controller: 1,
          pin_num: 4,
        },
      );
      map.insert(
        3,
        GpioInfo {
          controller: 1,
          pin_num: 14,
        },
      );
      map.insert(
        4,
        GpioInfo {
          controller: 1,
          pin_num: 15,
        },
      );
      map.insert(
        5,
        GpioInfo {
          controller: 0,
          pin_num: 15,
        },
      );
      map.insert(
        6,
        GpioInfo {
          controller: 1,
          pin_num: 17,
        },
      );
    }

    SamVersion::Rev4Flight => {
      map.insert(
        1,
        GpioInfo {
          controller: 2,
          pin_num: 16,
        },
      );
      map.insert(
        2,
        GpioInfo {
          controller: 1,
          pin_num: 16,
        },
      );
      map.insert(
        3,
        GpioInfo {
          controller: 2,
          pin_num: 13,
        },
      );
      map.insert(
        4,
        GpioInfo {
          controller: 1,
          pin_num: 17,
        },
      );
      map.insert(
        5,
        GpioInfo {
          controller: 3,
          pin_num: 19,
        },
      );
      map.insert(
        6,
        GpioInfo {
          controller: 2,
          pin_num: 8,
        },
      );
    }
  };

  map
}

// pinouts checked!
pub fn get_valve_current_sel_mappings() -> HashMap<u8, GpioInfo> {
  let mut map: HashMap<u8, GpioInfo> = HashMap::new();

  match *SAM_VERSION {
    SamVersion::Rev3 => {}

    SamVersion::Rev4Ground => {
      map.insert(
        1,
        GpioInfo {
          controller: 0,
          pin_num: 22,
        },
      );
      map.insert(
        2,
        GpioInfo {
          controller: 0,
          pin_num: 23,
        },
      );
      map.insert(
        3,
        GpioInfo {
          controller: 3,
          pin_num: 19,
        },
      );
    }

    SamVersion::Rev4Flight => {
      map.insert(
        1,
        GpioInfo {
          controller: 0,
          pin_num: 30,
        },
      );
      map.insert(
        2,
        GpioInfo {
          controller: 2,
          pin_num: 15,
        },
      );
      map.insert(
        3,
        GpioInfo {
          controller: 3,
          pin_num: 21,
        },
      );
    }
  }

  map
}

// pinouts checked!
/* What this hash map holds is for a given ADCKind key, the spibus file, the
optional chip select info, and optional data ready info. Because there is a
global GPIO_CONTROLLERS this hash map holds the information needed to access the
right pin in that variable instead of the memory itself.
 */
pub fn get_spi_info() -> HashMap<ADCKind, SpiInfo> {
  let mut map = HashMap::new();

  match *SAM_VERSION {
    // pinouts checked!
    SamVersion::Rev3 => {
      map.insert(
        ADCKind::SamRev3(SamRev3ADC::CurrentLoopPt),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 0,
            pin_num: 30,
          }),
          drdy: Some(GpioInfo {
            controller: 1,
            pin_num: 28,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev3(SamRev3ADC::DiffSensors),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 3,
            pin_num: 16,
          }),
          drdy: Some(GpioInfo {
            controller: 3,
            pin_num: 15,
          }),
        },
      );

      // sam-05 does not support I/V feedback because it uses relays
      if get_hostname() != "sam-05" {
        map.insert(
          ADCKind::SamRev3(SamRev3ADC::IValve),
          SpiInfo {
            spi_bus: "/dev/spidev0.0",
            cs: Some(GpioInfo {
              controller: 2,
              pin_num: 4,
            }),
            drdy: Some(GpioInfo {
              controller: 2,
              pin_num: 3,
            }),
          },
        );
        map.insert(
          ADCKind::SamRev3(SamRev3ADC::VValve),
          SpiInfo {
            spi_bus: "/dev/spidev0.0",
            cs: Some(GpioInfo {
              controller: 0,
              pin_num: 26,
            }),
            drdy: Some(GpioInfo {
              controller: 1,
              pin_num: 12,
            }),
          },
        );
      }

      map.insert(
        ADCKind::SamRev3(SamRev3ADC::IPower),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 2,
            pin_num: 15,
          }),
          drdy: Some(GpioInfo {
            controller: 2,
            pin_num: 14,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev3(SamRev3ADC::VPower),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 2,
            pin_num: 13,
          }),
          drdy: Some(GpioInfo {
            controller: 2,
            pin_num: 12,
          }),
        },
      );

      map.insert(
        ADCKind::SamRev3(SamRev3ADC::Tc1),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 0,
            pin_num: 10,
          }),
          drdy: None,
        },
      );
      map.insert(
        ADCKind::SamRev3(SamRev3ADC::Tc2),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 0,
            pin_num: 20,
          }),
          drdy: None,
        },
      );
    }

    // pinouts fixed and checked!
    SamVersion::Rev4Ground => {
      map.insert(
        ADCKind::SamRev4Gnd(SamRev4GndADC::CurrentLoopPt),
        SpiInfo {
          spi_bus: "/dev/spidev1.1",
          cs: None,
          drdy: Some(GpioInfo {
            controller: 3,
            pin_num: 17,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev4Gnd(SamRev4GndADC::DiffSensors),
        SpiInfo {
          spi_bus: "/dev/spidev1.0",
          cs: Some(GpioInfo {
            controller: 0,
            pin_num: 30,
          }),
          drdy: Some(GpioInfo {
            controller: 1,
            pin_num: 28,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev4Gnd(SamRev4GndADC::IValve),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 0,
            pin_num: 31,
          }),
          drdy: Some(GpioInfo {
            controller: 1,
            pin_num: 18,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev4Gnd(SamRev4GndADC::VValve),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 1,
            pin_num: 16,
          }),
          drdy: Some(GpioInfo {
            controller: 1,
            pin_num: 19,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev4Gnd(SamRev4GndADC::Rtd1),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 1,
            pin_num: 13,
          }),
          drdy: Some(GpioInfo {
            controller: 1,
            pin_num: 12,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev4Gnd(SamRev4GndADC::Rtd2),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 2,
            pin_num: 5,
          }),
          drdy: Some(GpioInfo {
            controller: 2,
            pin_num: 4,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev4Gnd(SamRev4GndADC::Rtd3),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 2,
            pin_num: 2,
          }),
          drdy: Some(GpioInfo {
            controller: 2,
            pin_num: 3,
          }),
        },
      );
    }

    // pinouts checked!
    SamVersion::Rev4Flight => {
      map.insert(
        ADCKind::SamRev4Flight(SamRev4FlightADC::CurrentLoopPt),
        SpiInfo {
          spi_bus: "/dev/spidev1.1",
          cs: None,
          drdy: Some(GpioInfo {
            controller: 0,
            pin_num: 7,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev4Flight(SamRev4FlightADC::DiffSensors),
        SpiInfo {
          spi_bus: "/dev/spidev1.0",
          cs: None,
          drdy: Some(GpioInfo {
            controller: 2,
            pin_num: 14,
          }),
        },
      );

      map.insert(
        ADCKind::SamRev4Flight(SamRev4FlightADC::IValve),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 2,
            pin_num: 9,
          }),
          drdy: Some(GpioInfo {
            controller: 0,
            pin_num: 14,
          }),
        },
      );
      map.insert(
        ADCKind::SamRev4Flight(SamRev4FlightADC::VValve),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 2,
            pin_num: 11,
          }),
          drdy: Some(GpioInfo {
            controller: 2,
            pin_num: 12,
          }),
        },
      );

      map.insert(
        ADCKind::SamRev4Flight(SamRev4FlightADC::Rtd1),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 1,
            pin_num: 28,
          }),
          drdy: Some(GpioInfo {
            controller: 1,
            pin_num: 18,
          }),
        },
      );

      // sam-21 does not have Rtd2 ADC soldered so its pointless
      if get_hostname() != "sam-21" {
        map.insert(
          ADCKind::SamRev4Flight(SamRev4FlightADC::Rtd2),
          SpiInfo {
            spi_bus: "/dev/spidev0.0",
            cs: Some(GpioInfo {
              controller: 2,
              pin_num: 2,
            }),
            drdy: Some(GpioInfo {
              controller: 2,
              pin_num: 3,
            }),
          },
        );
      }

      map.insert(
        ADCKind::SamRev4Flight(SamRev4FlightADC::Rtd3),
        SpiInfo {
          spi_bus: "/dev/spidev0.0",
          cs: Some(GpioInfo {
            controller: 2,
            pin_num: 6,
          }),
          drdy: Some(GpioInfo {
            controller: 2,
            pin_num: 10,
          }),
        },
      );
    }
  }

  map
}

// checked!
pub fn config_pins() {
  if *SAM_VERSION == SamVersion::Rev3 {
    // P8 GPIO
    config_pin("p8.10", "gpio");
    config_pin("p8.11", "gpio");
    config_pin("p8.12", "gpio");
    config_pin("p8.13", "gpio");
    config_pin("p8.14", "gpio");
    config_pin("p8.15", "gpio");
    config_pin("p8.16", "gpio");
    config_pin("p8.17", "gpio");
    config_pin("p8.18", "gpio");
    config_pin("p8.19", "gpio");
    config_pin("p8.20", "gpio");
    config_pin("p8.21", "gpio");
    config_pin("p8.22", "gpio");
    config_pin("p8.23", "gpio");
    config_pin("p8.24", "gpio");
    config_pin("p8.25", "gpio");
    config_pin("p8.26", "gpio");
    config_pin("p8.27", "gpio");
    config_pin("p8.28", "gpio");
    config_pin("p8.29", "gpio");
    config_pin("p8.30", "gpio");
    config_pin("p8.31", "gpio");
    config_pin("p8.34", "gpio");
    config_pin("p8.35", "gpio");
    config_pin("p8.36", "gpio");
    config_pin("p8.41", "gpio");
    config_pin("p8.42", "gpio");
    config_pin("p8.43", "gpio");
    config_pin("p8.44", "gpio");
    config_pin("p8.45", "gpio");
    config_pin("p8.46", "gpio");

    // P9 GPIO
    config_pin("p9.11", "gpio");
    config_pin("p9.12", "gpio");
    config_pin("p9.13", "gpio");
    config_pin("p9.14", "gpio");
    config_pin("p9.15", "gpio");
    config_pin("p9.16", "gpio");
    config_pin("p9.17", "gpio");
    config_pin("p9.18", "gpio");
    config_pin("p9.19", "gpio");
    config_pin("p9.20", "gpio");
    config_pin("p9.21", "gpio");
    config_pin("p9.22", "gpio");
    config_pin("p9.23", "gpio");
    config_pin("p9.24", "gpio");
    config_pin("p9.25", "gpio");
    config_pin("p9.26", "gpio");
    config_pin("p9.27", "gpio");
    config_pin("p9.28", "gpio");
    config_pin("p9.29", "gpio");
    config_pin("p9.30", "gpio");
    config_pin("p9.31", "gpio");
    config_pin("p9.41", "gpio");
    config_pin("p9.42", "gpio");

    // SPI (removed unconnected chip select)
    config_pin("p9_18", "spi");
    config_pin("p9_21", "spi");
    config_pin("p9_22", "spi_sclk");
  } else if *SAM_VERSION == SamVersion::Rev4Ground {
    // P8 GPIO (added 0 to numbers below 10)
    config_pin("p8.07", "gpio");
    config_pin("p8.08", "gpio");
    config_pin("p8.09", "gpio");
    config_pin("p8.10", "gpio");
    config_pin("p8.11", "gpio");
    config_pin("p8.12", "gpio");
    config_pin("p8.13", "gpio");
    config_pin("p8.14", "gpio");
    config_pin("p8.15", "gpio");
    config_pin("p8.16", "gpio");
    config_pin("p8.19", "gpio");
    config_pin("p8.21", "gpio");
    config_pin("p8.23", "gpio");
    config_pin("p8.25", "gpio");

    // P9 GPIO
    config_pin("p9.11", "gpio");
    config_pin("p9.12", "gpio");
    config_pin("p9.13", "gpio");
    config_pin("p9.14", "gpio");
    config_pin("p9.15", "gpio");
    config_pin("p9.16", "gpio");
    config_pin("p9.23", "gpio");
    config_pin("p9.24", "gpio");
    config_pin("p9.25", "gpio");
    config_pin("p9.26", "gpio");
    config_pin("p9.27", "gpio");
    config_pin("p9.28", "gpio"); // somehow works even tho its SPI

    // SPI 0 (slow)
    config_pin("p9_18", "spi");
    config_pin("p9_21", "spi");
    config_pin("p9_22", "spi_sclk");

    // SPI 1 (fast)
    config_pin("p9_19", "spi_cs");
    config_pin("p9_29", "spi");
    config_pin("p9_30", "spi");
    config_pin("p9_31", "spi_sclk");
  } else if *SAM_VERSION == SamVersion::Rev4Flight {
    // P8 GPIO
    config_pin("p8.7", "gpio");
    config_pin("p8.8", "gpio");
    config_pin("p8.9", "gpio");
    config_pin("p8.10", "gpio");
    config_pin("p8.11", "gpio");
    config_pin("p8.12", "gpio");
    config_pin("p8.13", "gpio");
    config_pin("p8.14", "gpio");
    config_pin("p8.15", "gpio");
    config_pin("p8.16", "gpio");
    config_pin("p8.17", "gpio");
    config_pin("p8.18", "gpio");
    config_pin("p8.19", "gpio");
    config_pin("p8.21", "gpio");
    config_pin("p8.36", "gpio");
    config_pin("p8.37", "gpio");
    config_pin("p8.38", "gpio");
    config_pin("p8.39", "gpio");
    config_pin("p8.40", "gpio");
    config_pin("p8.41", "gpio");
    config_pin("p8.42", "gpio");
    config_pin("p8.43", "gpio");
    config_pin("p8.44", "gpio");
    config_pin("p8.45", "gpio");
    config_pin("p8.46", "gpio");

    // P9 GPIO
    config_pin("p9.11", "gpio");
    config_pin("p9.12", "gpio");
    config_pin("p9.14", "gpio");
    config_pin("p9.15", "gpio");
    config_pin("p9.16", "gpio");
    config_pin("p9.23", "gpio");
    config_pin("p9.24", "gpio");
    config_pin("p9.25", "gpio");
    config_pin("p9.26", "gpio");
    config_pin("p9.27", "gpio");

    // SPI 0 (slow)
    config_pin("p9_17", "spi_cs");
    config_pin("p9_18", "spi");
    config_pin("p9_21", "spi");
    config_pin("p9_22", "spi_sclk");

    // SPI 1 (fast)
    config_pin("p9_19", "spi_cs");
    config_pin("p9_28", "spi_cs");
    config_pin("p9_29", "spi");
    config_pin("p9_30", "spi");
    config_pin("p9_31", "spi_sclk");
  }
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
