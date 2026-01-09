//! Basic usage example for the GPS driver
//! 
//! This example demonstrates how to:
//! 1. Initialize the GPS module via I2C
//! 2. Query version information
//! 3. Poll for position/velocity/time data
//! 
//! To run this example:
//! ```bash
//! cargo run --example basic_usage
//! ```
//! 
//! Note: You must enable I2C on your Raspberry Pi first:
//! ```bash
//! sudo raspi-config
//! # Go to Interface Options -> I2C -> Enable
//! ```

use zedf9p04b::GPS;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing GPS module on I2C bus 1...");
    
    // Create GPS instance on I2C bus 1 with default address (0x42)
    let mut gps = GPS::new(1, None)?;
    
    println!("GPS initialized successfully!");
    
    // Query version information
    println!("\nQuerying module version...");
    match gps.mon_ver() {
        Ok(_) => println!("Version query successful!"),
        Err(e) => println!("Version query failed: {}", e),
    }
    
    // Poll for position data in a loop
    // Note: We're using polling mode, so we don't need to configure periodic output
    println!("\nPolling for GPS data (press Ctrl+C to exit)...\n");
    
    loop {
        match gps.poll_pvt() {
            Ok(Some(pvt)) => {
                if let Some(pos) = pvt.position {
                    println!("Position: lat={:.7}°, lon={:.7}°, alt={:.2}m",
                        pos.lat, pos.lon, pos.alt);
                }
                
                if let Some(vel) = pvt.velocity {
                    println!("Velocity (NED): north={:.2} m/s, east={:.2} m/s, down={:.2} m/s",
                        vel.north, vel.east, vel.down);
                }
                
                if let Some(time) = pvt.time {
                    println!("Time: {}", time);
                }
                
                println!();
            }
            Ok(None) => {
                println!("No GPS fix available yet...");
            }
            Err(e) => {
                println!("Error polling PVT: {}", e);
            }
        }
        
        // Wait before next poll
        thread::sleep(Duration::from_secs(1));
    }
}

