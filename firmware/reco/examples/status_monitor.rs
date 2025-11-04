//! RECO status monitoring
//! 
//! This example continuously monitors the RECO board status:
//! - Reads status at regular intervals
//! - Displays status information
//! - Monitors for changes
//! 
//! To run:
//! ```bash
//! cargo run --example status_monitor
//! ```
//! 
//! Press Ctrl+C to exit.

use common::comm::gpio::{Gpio, Pin, PinMode::Output, PinValue::High};
use once_cell::sync::Lazy;
use reco::RecoDriver;
use std::env;
use std::thread;
use std::time::{Duration, Instant};

const RECO_CS_PIN_CONTROLLER: usize = 1;
const RECO_CS_PIN_NUM: usize = 16;
const MONITOR_INTERVAL_MS: u64 = 500;

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> = Lazy::new(|| {
    (0..=3).map(Gpio::open_controller).collect()
});

fn format_status(status: &reco::RecoStatus) -> String {
    let enabled_channels: Vec<u8> = (1..=3)
        .filter(|&ch| (status.channel_status & (1 << (ch - 1))) != 0)
        .collect();
    
    format!(
        "System: 0x{:02X} | Errors: 0x{:02X} | Channels: {:?}",
        status.system_status,
        status.error_flags,
        if enabled_channels.is_empty() {
            "None".to_string()
        } else {
            enabled_channels.iter().map(|c| c.to_string()).collect::<Vec<_>>().join(", ")
        }
    )
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    
    println!("RECO Status Monitor");
    println!("===================");
    println!("Press Ctrl+C to exit\n");
    
    // Initialize GPIO
    let gpio = &GPIO_CONTROLLERS[RECO_CS_PIN_CONTROLLER];
    let mut cs_pin = gpio.get_pin(RECO_CS_PIN_NUM);
    cs_pin.mode(Output);
    cs_pin.digital_write(High);
    
    // Initialize RECO driver
    let mut reco = RecoDriver::new("/dev/spidev0.0", Some(cs_pin))?;
    
    // Verify communication
    println!("Verifying communication...");
    match reco.heartbeat() {
        Ok(true) => println!("✓ RECO board is responding\n"),
        Ok(false) => {
            eprintln!("⚠ RECO board heartbeat failed");
            return Ok(());
        }
        Err(e) => {
            eprintln!("✗ Communication error: {}", e);
            return Err(Box::new(e));
        }
    }
    
    // Read initial status
    let mut last_status = reco.read_status()?;
    println!("Initial status: {}", format_status(&last_status));
    println!("\nMonitoring status ({}ms interval)...\n", MONITOR_INTERVAL_MS);
    
    let start_time = Instant::now();
    let mut iteration = 0;
    
    loop {
        iteration += 1;
        
        match reco.read_status() {
            Ok(status) => {
                // Check if status changed
                if status != last_status {
                    let elapsed = start_time.elapsed();
                    println!(
                        "[{:6.2}s] Status changed: {}",
                        elapsed.as_secs_f64(),
                        format_status(&status)
                    );
                    last_status = status;
                } else if iteration % 10 == 0 {
                    // Print status every 10 iterations (every 5 seconds at 500ms interval)
                    let elapsed = start_time.elapsed();
                    println!(
                        "[{:6.2}s] Status: {}",
                        elapsed.as_secs_f64(),
                        format_status(&status)
                    );
                }
                
                // Check for errors
                if status.error_flags != 0 {
                    eprintln!(
                        "⚠ ERROR FLAGS SET: 0x{:02X}",
                        status.error_flags
                    );
                }
            }
            Err(e) => {
                eprintln!("✗ Error reading status: {}", e);
                // Continue monitoring despite errors
            }
        }
        
        thread::sleep(Duration::from_millis(MONITOR_INTERVAL_MS));
    }
}

