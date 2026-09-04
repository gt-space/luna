//! Simple app that interfaces with an ADS1115](https://www.ti.com/lit/ds/symlink/ads1115.pdf) ADC
//! over I2C from an STM32H745I-DISCO board](https://www.st.com/resource/en/user_manual/DM00547983.pdf)
//! using the stm32h7xx-hal crate.

//! Prevent linkage to the standard runtime, which helps out with stack
//! unwinding, heap allocation, etc.
#![no_main]
//! Prevent linkage to the standard library. Instead, libcore will be used.
#![no_std]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal::{pac, prelude::*};

// -- ADS1115 constants (ADS1115 datasheet, section 8) --

/// Device address (addr pin tied to GND).
const ADS1115_ADDR: u8 = 0x48;
/// Conversion register address.
const CONVERSION_REG_ADDR: u8 = 0x00;
/// Configuration register address.
const CONFIG_REG_ADDR: u8 = 0x01;

/// Config register reset value.
const CONFIG_REG_RST_VAL: u16 = 0x8583;

/// Mask for config register MUX bits.
const CONFIG_REG_MUX_MASK: u16 = 0b111 << 12;
/// Mask for config register MODE bit.
const CONFIG_REG_MODE_MASK: u16 = 1 << 8;

/// AINP = AIN0, AINN = GND
const CONFIG_REG_MUX: u16 = 0b100 << 12;
/// Continuous conversion mode
const CONFIG_REG_MODE_CONTINUOUS: u16 = 0 << 8;

/// Start of program indicator to the cortex-m reset handler.
/// The cortex-m reset handler is called by the micro after initial board boot.
#[entry]
fn main() -> ! {
    // Take ownership of all device peripherals
    let peripherals = pac::Peripherals::take().unwrap();

    // -- Power config --
    defmt::info!("Power config");

    // constrain() returns the HAL's representation of the PWR peripheral
    let pwr = peripherals.PWR.constrain();
    // Set to SMPS mode (disco board manual, section 6.4 (subsection SMPS/LDO power
    // supply)), and freeze this hardware configuration. This can only be undone
    // through power-on-reset.
    let pwr_cfg = pwr.smps().freeze();

    // -- Clock config --
    defmt::info!("Clock config");

    // constrain() returns the HAL's representation of the RCC (reset and control
    // clock) peripheral
    let rcc = peripherals.RCC.constrain();
    // Configure the system clock to 100 MHz and freeze this configuration in
    // hardware. This can only be undone through power-on-reset.
    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwr_cfg, &peripherals.SYSCFG);

    // -- I2C / GPIO config --
    defmt::info!("I2C / GPIO config");

    // split() takes the GPIOD bank and splits it into individual GPIO pins
    // (represented as ZSTs). passing in the clock for GPIOD allows for proper
    // configuration.
    let gpiod = peripherals.GPIOD.split(ccdr.peripheral.GPIOD);

    // Configure the I2C4 pins for SCL and SDA (disco board manual, section 7.1
    // table 8)
    let scl = gpiod.pd12.into_alternate_open_drain();
    let sda = gpiod.pd13.into_alternate_open_drain();

    // Configure the I2C peripheral
    let mut i2c = peripherals
        .I2C4
        .i2c((scl, sda), 100.kHz(), ccdr.peripheral.I2C4, &ccdr.clocks);

    // -- ADS1115 config --
    defmt::info!("ADS1115 config");

    // ADDRESS CHECK
    let mut found_addr = false;
    defmt::info!("Scanning I2C bus...");
    for addr in 0x08..=0x77 {
        if let Ok(()) = i2c.write(addr, &[]) {
            defmt::info!("Found device at {:#04x}", addr);
            found_addr = true;
            break;
        }
    }
    if !found_addr {
        defmt::error!("No ADS1115 found on I2C bus");
        panic!("No ADS1115 found on I2C bus");
    }

    // Start from the reset value and set mux and mode fields appropriately.
    // This lands us in continuous conversion mode using AIN0 as AINP and GND as
    // AINN.
    let config_reg_val = CONFIG_REG_RST_VAL & (!CONFIG_REG_MUX_MASK & !CONFIG_REG_MODE_MASK)
        | CONFIG_REG_MUX
        | CONFIG_REG_MODE_CONTINUOUS;
    // ADS1115 uses big-endian.
    let config_reg_bytes = config_reg_val.to_be_bytes();

    // Write the configuration to the config register.
    i2c.write(
        ADS1115_ADDR,
        &[CONFIG_REG_ADDR, config_reg_bytes[0], config_reg_bytes[1]],
    )
    .unwrap();

    // -- Sample data --

    let mut buf = [0u8; 2];
    // Read conversion register and print sampled data once per second..
    loop {
        i2c.write_read(ADS1115_ADDR, &[CONVERSION_REG_ADDR], &mut buf)
            .unwrap();
        let sampled_data = u16::from_be_bytes(buf);
        defmt::info!("Sampled data: {}", sampled_data);
        cortex_m::asm::delay(100_000_000);
    }
}
