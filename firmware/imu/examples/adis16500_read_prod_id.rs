//! Standalone example: configures IMU GPIO pins (same as flight2/sensors.rs), writes DEC_RATE,
//! then reads PROD_ID from an ADIS 16500 over SPI.

use common::comm::gpio::{GpioPin, PinMode::*, PinValue::*, RpiGpioController};
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};
use std::io;

// SPI and GPIO match flight2/src/sensors.rs init_imu()
const SPI_PATH: &str = "/dev/spidev5.0";

// BCM pin numbers (same as IMUPins in flight2/src/sensors.rs)
const IMU_CS_BCM: u8 = 12;
const IMU_DR_BCM: u8 = 22;
const IMU_NRESET_BCM: u8 = 23;

const REG_DEC_RATE: u8 = 0x64;
const REG_DEC_RATE_HI: u8 = 0x65;
const REG_PROD_ID: u8 = 0x72;

fn main() -> io::Result<()> {
    // --- GPIO setup (match flight2/sensors.rs init_imu: configure pins, then set defaults) ---
    let controller = RpiGpioController::open_controller()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    let mut cs = controller.get_pin(IMU_CS_BCM);
    cs.mode(Output);
    cs.digital_write(High); // Chip select active low, high = disabled

    let mut dr = controller.get_pin(IMU_DR_BCM);
    dr.mode(Input); // Data ready input

    let mut nreset = controller.get_pin(IMU_NRESET_BCM);
    nreset.mode(Output);
    nreset.digital_write(High); // Reset active low, high = disabled

    println!("GPIO configured: CS and nreset high, data_ready input");

    // --- SPI open and configure ---
    let mut spi = Spidev::open(SPI_PATH).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    let options = SpidevOptions::new()
        .max_speed_hz(1_000_000)
        .mode(SpiModeFlags::SPI_MODE_3)
        .bits_per_word(8)
        .lsb_first(false)
        .build();
    spi.configure(&options).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

    // --- Write decimation rate (same as flight2: write_dec_rate(8)) ---
    let dec_rate: u16 = 8;
    let data_be = dec_rate.to_be_bytes();
    let tx_write = [
        data_be[1],
        REG_DEC_RATE | 0x80,
        data_be[0],
        REG_DEC_RATE_HI | 0x80,
    ];
    println!(
        "TX write DEC_RATE: {:?}",
        tx_write.iter().map(|b| format!("{:#04x}", b)).collect::<Vec<_>>()
    );
    {
        let mut transfer = SpidevTransfer::write(&tx_write);
        spi.transfer(&mut transfer).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    }
    println!("DEC_RATE write done (value = {:#06x})", dec_rate);

    // --- Read PROD_ID ---
    let first_byte = REG_PROD_ID & 0x7F;
    let tx_request = [first_byte, 0x00];
    println!(
        "TX request: {:?}",
        tx_request.iter().map(|b| format!("{:#04x}", b)).collect::<Vec<_>>()
    );

    let mut rx_request = [0u8; 2];
    {
        let mut transfer = SpidevTransfer::read_write(&tx_request, &mut rx_request);
        spi.transfer(&mut transfer).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    }
    println!(
        "RX during request: {:?}",
        rx_request.iter().map(|b| format!("{:#04x}", b)).collect::<Vec<_>>()
    );

    let tx_dummy = [0x00u8; 6];
    let mut rx_data = [0u8; 6];
    {
        let mut transfer = SpidevTransfer::read_write(&tx_dummy, &mut rx_data);
        spi.transfer(&mut transfer).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    }
    println!(
        "RX data: {:?}",
        rx_data.iter().map(|b| format!("{:#04x}", b)).collect::<Vec<_>>()
    );

    let raw_value = (rx_data[0] as u16) << 8 | rx_data[1] as u16;
    println!("PROD_ID (hex): {:#06x}", raw_value);
    println!("PROD_ID (signed): {}", raw_value as i16);

    Ok(())
}
