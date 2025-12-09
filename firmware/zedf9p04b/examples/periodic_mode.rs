//! Periodic mode example
//! 
//! This example demonstrates PERIODIC mode where the GPS module
//! automatically sends NAV-PVT messages at a configured rate.
//! 
//! Contrast this with basic_usage.rs which uses POLLING mode.
//! 
//! Periodic mode is more efficient when you want continuous updates,
//! but polling mode gives you more control over when data is retrieved.

use zedf9p04b::GPS;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing GPS module (Periodic Mode)...");
    
    let mut gps = GPS::new(1, None)?;
    println!("GPS initialized successfully!\n");
    
    // Query version
    println!("Querying module version...");
    gps.mon_ver()?;
    
    // Configure measurement rate to 20 Hz (50 ms period)
    println!("\nConfiguring measurement rate to 20 Hz...");
    gps.set_measurement_rate(50, 1, 0)?;
    // Arguments: meas_rate_ms=50 (20 Hz), nav_rate=1 (every measurement), time_ref=0 (UTC)
    println!("✓ Measurement rate configured to 20 Hz (50 ms period)!");
    
    // Configure module to automatically send NAV-PVT messages on every solution
    println!("\nConfiguring periodic NAV-PVT output...");
    println!("Setting message rate to 1 (one message per navigation solution)");
    
    // Rate array: [I2C, UART1, UART2, USB, SPI, Reserved]
    // Setting 1 means send on every navigation solution
    // Setting 5 would mean send every 5 solutions, etc.
    gps.set_nav_pvt_rate([1, 0, 0, 0, 0, 0])?;
    println!("✓ Periodic output configured!");
    
    println!("\nWaiting for GPS data at 20 Hz (press Ctrl+C to exit)...");
    println!("Module will automatically send NAV-PVT messages at 20 Hz\n");
    
    // In periodic mode, we just continuously read incoming PVT data
    // The module will send NAV-PVT automatically at 20 Hz without us polling
    let mut pvt_count = 0;
    
    loop {
        // Use read_pvt() to extract PVT data from available packets
        // This is more efficient than polling since the module pushes data to us
        // At 20 Hz, we should receive a NAV-PVT message approximately every 50 ms
        match gps.read_pvt() {
            Ok(Some(pvt)) => {
                pvt_count += 1;
                
                // Display PVT data
                if let Some(pos) = pvt.position {
                    println!("PVT #{}: lat={:.7}°, lon={:.7}°, alt={:.2}m",
                        pvt_count, pos.lat, pos.lon, pos.alt);
                }
                
                if let Some(vel) = pvt.velocity {
                    println!("  Velocity (NED): north={:.2} m/s, east={:.2} m/s, down={:.2} m/s",
                        vel.north, vel.east, vel.down);
                }
                
                if let Some(time) = pvt.time {
                    println!("  Time: {}", time);
                }
                
                if pvt_count % 20 == 0 {
                    println!("\n[Received {} PVT messages (~{} seconds)]\n", pvt_count, pvt_count / 20);
                }
            }
            Ok(None) => {
                // No PVT data available yet, this is normal
            }
            Err(e) => {
                eprintln!("Error reading PVT: {}", e);
            }
        }
        
        // Small delay to avoid busy-waiting
        // At 20 Hz, messages arrive every 50 ms, so 10 ms delay is fine
        thread::sleep(Duration::from_millis(10));
    }
}

