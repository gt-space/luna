
use std::{
    collections::HashMap,
    sync::LazyLock,
};

use common::comm::gpio::RpiGpioController;

// One shared GPIO controller for the Raspberry Pi 5.
pub static GPIO_CONTROLLER: LazyLock<RpiGpioController> =
    LazyLock::new(|| {
        RpiGpioController::open_controller()
            .expect("Failed to open Raspberry Pi GPIO")
    });

// Information about a Raspberry Pi GPIO pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GpioInfo {
    /// BCM GPIO number used by the software.
    pub pin_num: u8,

    /// Physical pin on the Raspberry Pi 40-pin header.
    pub header_pin: u8,
}

// Helper function to create a GPIO mapping.
fn gpio(pin_num: u8, header_pin: u8) -> GpioInfo {
    GpioInfo {
        pin_num,
        header_pin,
    }
}

// ============================================================
// POWER CONTROL GPIO
// ============================================================

// Precharge enable: BCM23, physical pin 16.
pub static PRECHARGE_ENABLE: LazyLock<GpioInfo> =
    LazyLock::new(|| gpio(23, 16));

// Mains enable: BCM24, physical pin 18.
pub static MAINS_ENABLE: LazyLock<GpioInfo> =
    LazyLock::new(|| gpio(24, 18));

// ============================================================
// ESC CONTROL GPIO
// ============================================================

// ESC A enable: BCM25, physical pin 22.
pub static ESC_A_ENABLE: LazyLock<GpioInfo> =
    LazyLock::new(|| gpio(25, 22));

// ESC B enable: BCM27, physical pin 13.
pub static ESC_B_ENABLE: LazyLock<GpioInfo> =
    LazyLock::new(|| gpio(27, 13));

// PWM0: BCM12, physical pin 32.
pub static PWM0: LazyLock<GpioInfo> =
    LazyLock::new(|| gpio(12, 32));

// PWM1: BCM13, physical pin 33.
pub static PWM1: LazyLock<GpioInfo> =
    LazyLock::new(|| gpio(13, 33));

// Get the corresponding ESC enable pin.
// Channel 0 = ESC A.
// Channel 1 = ESC B.
pub fn esc_enable(channel: u8) -> Option<GpioInfo> {
    match channel {
        0 => Some(*ESC_A_ENABLE),
        1 => Some(*ESC_B_ENABLE),
        _ => None,
    }
}

// Get the corresponding PWM pin.
// Channel 0 = PWM0.
// Channel 1 = PWM1.
pub fn pwm_enable(channel: u8) -> Option<GpioInfo> {
    match channel {
        0 => Some(*PWM0),
        1 => Some(*PWM1),
        _ => None,
    }
}

// ============================================================
// SPI BUS
// ============================================================

// Raspberry Pi SPI0 bus.
pub const SPI_BUS_NUMBER: u8 = 0;

// SPI0 MOSI: BCM10, physical pin 19.
pub static SPI0_MOSI: LazyLock<GpioInfo> =
    LazyLock::new(|| gpio(10, 19));

// SPI0 MISO: BCM9, physical pin 21.
pub static SPI0_MISO: LazyLock<GpioInfo> =
    LazyLock::new(|| gpio(9, 21));

// SPI0 SCLK: BCM11, physical pin 23.
pub static SPI0_SCLK: LazyLock<GpioInfo> =
    LazyLock::new(|| gpio(11, 23));

// ============================================================
// SPI DEVICES
// ============================================================

// Devices connected to SPI0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SpiDevice {
    PowerAdc,
    CurrentLoopAdc,
    Thermocouple0,
    Thermocouple1,
    Thermocouple2,
}

// SPI configuration for an individual device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
// bus_number is always zero since all 5 SPI devices share SPI0
pub struct SpiInfo {
    // SPI bus number.
    pub bus_number: u8,

    // Chip select pin.
    pub cs: GpioInfo,

    // Optional data-ready pin.
    pub drdy: Option<GpioInfo>,

    // Linux SPI device path, when configured.
    pub spi_device: Option<&'static str>,
}

// Shared SPI device mapping.
pub static SPI_INFO: LazyLock<HashMap<SpiDevice, SpiInfo>> =
    LazyLock::new(get_spi_info);

// Create the SPI device mapping.
pub fn get_spi_info() -> HashMap<SpiDevice, SpiInfo> {
    use self::SpiDevice::*;

    let mut map = HashMap::new();

    // --------------------------------------------------------
    // POWER ADC
    // --------------------------------------------------------

    // ADS124S06 power-monitoring ADC.
    // CS0: BCM8, physical pin 24.
    // DRDY: BCM17, physical pin 11.
    map.insert(
        PowerAdc,
        SpiInfo {
            bus_number: SPI_BUS_NUMBER,
            cs: gpio(8, 24),
            drdy: Some(gpio(17, 11)),
            spi_device: Some("/dev/spidev0.0"),
        },
    );

    // --------------------------------------------------------
    // CURRENT LOOP ADC
    // --------------------------------------------------------

    // ADS124S06 current-loop ADC.
    // CS1: BCM7, physical pin 26.
    // DRDY: BCM22, physical pin 15.
    map.insert(
        CurrentLoopAdc,
        SpiInfo {
            bus_number: SPI_BUS_NUMBER,
            cs: gpio(7, 26),
            drdy: Some(gpio(22, 15)),
            spi_device: Some("/dev/spidev0.1"),
        },
    );

    // --------------------------------------------------------
    // THERMOCOUPLE 0
    // --------------------------------------------------------

    // MAX6675 thermocouple.
    // CS: BCM5, physical pin 29.
    map.insert(
        Thermocouple0,
        SpiInfo {
            bus_number: SPI_BUS_NUMBER,
            cs: gpio(5, 29),
            drdy: None,
            spi_device: None,
        },
    );

    // --------------------------------------------------------
    // THERMOCOUPLE 1
    // --------------------------------------------------------

    // MAX6675 thermocouple.
    // CS: BCM6, physical pin 31.
    map.insert(
        Thermocouple1,
        SpiInfo {
            bus_number: SPI_BUS_NUMBER,
            cs: gpio(6, 31),
            drdy: None,
            spi_device: None,
        },
    );

    // --------------------------------------------------------
    // THERMOCOUPLE 2
    // --------------------------------------------------------

    // MAX6675 thermocouple.
    // CS: BCM16, physical pin 36.
    map.insert(
        Thermocouple2,
        SpiInfo {
            bus_number: SPI_BUS_NUMBER,
            cs: gpio(16, 36),
            drdy: None,
            spi_device: None,
        },
    );

    map
}
