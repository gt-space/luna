//! RECO data monitoring
//! 
//! This example continuously receives data from the RECO board:
//! - Reads RECO body data at regular intervals
//! - Displays quaternion, position, velocity, and sensor data
//! - Monitors for data changes
//! 
//! To run:
//! ```bash
//! cargo run --example status_monitor
//! ```
//! 
//! Press Ctrl+C to exit.

use reco::RecoDriver;
use rppal::gpio::Gpio;
use rppal::spi::{Bus, SlaveSelect};
use std::env;
use std::thread;
use std::time::{Duration, Instant};

const RECO_CS_PIN: u8 = 16;
const MONITOR_INTERVAL_MS: u64 = 500;

fn format_reco_data(data: &reco::RecoBody) -> String {
    format!(
        "Q:[{:.3},{:.3},{:.3},{:.3}] | Pos:[{:.4},{:.4},{:.1}] | Vel:[{:.2},{:.2},{:.2}] | T:{:.1}°C P:{:.1}Pa",
        data.quaternion[0], data.quaternion[1], data.quaternion[2], data.quaternion[3],
        data.lla_pos[0], data.lla_pos[1], data.lla_pos[2],
        data.velocity[0], data.velocity[1], data.velocity[2],
        data.temperature, data.pressure
    )
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    
    println!("RECO Data Monitor");
    println!("=================");
    println!("Press Ctrl+C to exit\n");
    
    // Initialize GPIO and chip select pin
    println!("Initializing GPIO...");
    let gpio = Gpio::new()?;
    let mut cs_pin = gpio.get(RECO_CS_PIN)?.into_output();
    cs_pin.set_high(); // Active low, start high (inactive)
    
    // Initialize RECO driver
    println!("Initializing RECO driver...");
    let mut reco = RecoDriver::new(Bus::Spi0, SlaveSelect::Ss0, Some(cs_pin))?;
    println!("✓ RECO driver initialized\n");
    
    // Try to receive initial data to verify communication
    println!("Verifying communication...");
    match reco.receive_data() {
        Ok(data) => {
            println!("✓ RECO board is responding");
            println!("Initial data: {}", format_reco_data(&data));
        }
        Err(e) => {
            eprintln!("⚠ Warning: Failed to receive initial data: {}", e);
            eprintln!("  Continuing to monitor anyway...");
        }
    }
    
    println!("\nMonitoring RECO data ({}ms interval)...\n", MONITOR_INTERVAL_MS);
    
    let start_time = Instant::now();
    let mut iteration = 0;
    let mut last_data: Option<reco::RecoBody> = None;
    let mut success_count = 0;
    let mut error_count = 0;
    
    loop {
        iteration += 1;
        
        match reco.receive_data() {
            Ok(data) => {
                success_count += 1;
                
                // Check if data changed significantly
                let changed = if let Some(ref last) = last_data {
                    // Compare key fields for changes
                    (data.temperature - last.temperature).abs() > 0.1 ||
                    (data.pressure - last.pressure).abs() > 1.0 ||
                    data.velocity.iter().zip(last.velocity.iter())
                        .any(|(a, b)| (a - b).abs() > 0.1)
                } else {
                    true // First data point
                };
                
                if changed {
                    let elapsed = start_time.elapsed().as_secs_f64();
                    println!(
                        "[{:6.2}s] Data: {}",
                        elapsed,
                        format_reco_data(&data)
                    );
                    last_data = Some(data);
                } else if iteration % 10 == 0 {
                    // Print status every 10 iterations (every 5 seconds at 500ms interval)
                    let elapsed = start_time.elapsed().as_secs_f64();
                    println!(
                        "[{:6.2}s] Status: Data OK | Success: {} | Errors: {} | {}",
                        elapsed,
                        success_count,
                        error_count,
                        format_reco_data(&data)
                    );
                }
            }
            Err(e) => {
                error_count += 1;
                
                // Only print errors occasionally to avoid spam
                if error_count == 1 || error_count % 10 == 0 {
                    let elapsed = start_time.elapsed().as_secs_f64();
                    eprintln!(
                        "[{:6.2}s] ⚠ Error receiving data (count: {}): {}",
                        elapsed,
                        error_count,
                        e
                    );
                }
            }
        }
        
        thread::sleep(Duration::from_millis(MONITOR_INTERVAL_MS));
    }
}

