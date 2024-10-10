#![warn(missing_docs)]
#![warn(clippy::correctness)]

//! Servo is the library/binary hybrid written for the Yellow Jacket Space
//! Program's control server.

/// Components related to interacting with the terminal and developer display.
pub mod interface;

/// Components related to the server, including route functions, forwarding,
/// flight communication, and the interface.
pub mod server;

/// Everything related to the Servo command line tool.
pub mod tool;

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f4xx_hal::{pac, prelude::*, serial::Serial};

#[entry]
fn main() -> ! {
    // Get the peripherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Configure the system clock
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

    // Configure the GPIO pins
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa2.into_alternate_af7();
    let rx_pin = gpioa.pa3.into_alternate_af7();

    // Set up the UART at 9600 baud rate
    let serial = Serial::usart2(
        dp.USART2,
        (tx_pin, rx_pin),
        9600.bps(),
        clocks,
    )
    .unwrap();

    let (mut tx, mut rx) = serial.split();

    // Write and read from UART
    tx.write(b'H').unwrap();
    tx.write(b'e').unwrap();
    tx.write(b'l').unwrap();
    tx.write(b'l').unwrap();
    tx.write(b'o').unwrap();

    loop {
        if let Ok(received) = rx.read() {
            tx.write(received).unwrap(); // Echo received data back
        }
    }
}

