//! RECO channel control test
//! 
//! This example demonstrates controlling recovery channels:
//! - Enabling channels
//! - Disabling channels
//! - Reading channel status
//! 
//! To run:
//! ```bash
//! cargo run --example channel_test
//! ```

use common::comm::gpio::{Gpio, Pin, PinMode::Output, PinValue::High};
use once_cell::sync::Lazy;
use reco::RecoDriver;
use std::env;
use std::thread;
use std::time::Duration;

const RECO_CS_PIN_CONTROLLER: usize = 1;
const RECO_CS_PIN_NUM: usize = 16;

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> = Lazy::new(|| {
    (0..=3).map(Gpio::open_controller).collect()
});

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    
    println!("RECO Channel Control Test");
    println!("=========================");
    
    // Initialize GPIO
    println!("Initializing GPIO...");
    let gpio = &GPIO_CONTROLLERS[RECO_CS_PIN_CONTROLLER];
    let mut cs_pin = gpio.get_pin(RECO_CS_PIN_NUM);
    cs_pin.mode(Output);
    cs_pin.digital_write(High);
    
    // Initialize RECO driver
    println!("Initializing RECO driver...");
    let mut reco = RecoDriver::new("/dev/spidev0.0", Some(cs_pin))?;
    println!("✓ Driver initialized");
    
    // Read initial status
    println!("\nReading initial status...");
    let initial_status = reco.read_status()?;
    println!("  Initial channel status: 0x{:02X}", initial_status.channel_status);
    
    // Test each channel
    for channel in 1..=3 {
        println!("\n--- Testing Channel {} ---", channel);
        
        // Enable channel
        println!("  Enabling channel {}...", channel);
        match reco.enable_channel(channel) {
            Ok(_) => println!("    ✓ Channel {} enabled", channel),
            Err(e) => {
                eprintln!("    ✗ Failed to enable channel {}: {}", channel, e);
                continue;
            }
        }
        
        // Wait a bit
        thread::sleep(Duration::from_millis(100));
        
        // Read status to verify
        match reco.read_status() {
            Ok(status) => {
                let channel_enabled = (status.channel_status & (1 << (channel - 1))) != 0;
                if channel_enabled {
                    println!("    ✓ Channel {} confirmed enabled", channel);
                } else {
                    println!("    ⚠ Channel {} status not showing as enabled", channel);
                }
            }
            Err(e) => {
                eprintln!("    ✗ Failed to read status: {}", e);
            }
        }
        
        // Disable channel
        println!("  Disabling channel {}...", channel);
        match reco.disable_channel(channel) {
            Ok(_) => println!("    ✓ Channel {} disabled", channel),
            Err(e) => {
                eprintln!("    ✗ Failed to disable channel {}: {}", channel, e);
            }
        }
        
        // Wait a bit
        thread::sleep(Duration::from_millis(100));
        
        // Read status to verify
        match reco.read_status() {
            Ok(status) => {
                let channel_enabled = (status.channel_status & (1 << (channel - 1))) != 0;
                if !channel_enabled {
                    println!("    ✓ Channel {} confirmed disabled", channel);
                } else {
                    println!("    ⚠ Channel {} status still showing as enabled", channel);
                }
            }
            Err(e) => {
                eprintln!("    ✗ Failed to read status: {}", e);
            }
        }
    }
    
    println!("\n✓ Channel test completed!");
    Ok(())
}

