//! Basic RECO driver test
//! 
//! This example demonstrates basic communication with the RECO board:
//! - Initialization
//! - Status reading
//! - Heartbeat verification
//! 
//! To run:
//! ```bash
//! cargo run --example basic_test
//! ```

use common::comm::gpio::{Gpio, Pin, PinMode::Output, PinValue::High};
use once_cell::sync::Lazy;
use reco::RecoDriver;
use std::env;

// GPIO controller 1, pin 16 for RECO chip select (from BMS code)
const RECO_CS_PIN_CONTROLLER: usize = 1;
const RECO_CS_PIN_NUM: usize = 16;

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> = Lazy::new(|| {
    (0..=3).map(Gpio::open_controller).collect()
});

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    
    println!("RECO Driver Basic Test");
    println!("======================");
    
    // Initialize GPIO
    println!("Initializing GPIO...");
    let gpio = &GPIO_CONTROLLERS[RECO_CS_PIN_CONTROLLER];
    let mut cs_pin = gpio.get_pin(RECO_CS_PIN_NUM);
    cs_pin.mode(Output);
    cs_pin.digital_write(High); // Active low, start high
    
    // Initialize RECO driver
    println!("Initializing RECO driver on /dev/spidev0.0...");
    let mut reco = match RecoDriver::new("/dev/spidev0.0", Some(cs_pin)) {
        Ok(driver) => {
            println!("✓ RECO driver initialized successfully");
            driver
        }
        Err(e) => {
            eprintln!("✗ Failed to initialize RECO driver: {}", e);
            return Err(Box::new(e));
        }
    };
    
    // Test heartbeat
    println!("\nTesting heartbeat...");
    match reco.heartbeat() {
        Ok(true) => println!("✓ RECO board is responding");
        Ok(false) => println!("✗ RECO board did not respond correctly");
        Err(e) => {
            eprintln!("✗ Heartbeat failed: {}", e);
            return Err(Box::new(e));
        }
    }
    
    // Read status
    println!("\nReading RECO status...");
    match reco.read_status() {
        Ok(status) => {
            println!("✓ Status read successfully:");
            println!("  System Status: 0x{:02X}", status.system_status);
            println!("  Error Flags:   0x{:02X}", status.error_flags);
            println!("  Channel Status: 0x{:02X}", status.channel_status);
            println!("  Channels enabled: {:?}", 
                (1..=3).filter(|&ch| (status.channel_status & (1 << (ch - 1))) != 0).collect::<Vec<_>>());
        }
        Err(e) => {
            eprintln!("✗ Failed to read status: {}", e);
            return Err(Box::new(e));
        }
    }
    
    println!("\n✓ Basic test completed successfully!");
    Ok(())
}

