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
    
    // Configure module to automatically send NAV-PVT messages
    println!("\nConfiguring periodic NAV-PVT output...");
    println!("Setting rate to 1 Hz (one message per navigation solution)");
    
    // Rate array: [I2C, UART1, UART2, USB, SPI, Reserved]
    // Setting 1 means send on every navigation solution
    // Setting 5 would mean send every 5 solutions, etc.
    gps.set_nav_pvt_rate([1, 0, 0, 0, 0, 0])?;
    println!("✓ Periodic output configured!");
    
    println!("\nWaiting for GPS data (press Ctrl+C to exit)...");
    println!("Module will automatically send NAV-PVT messages\n");
    
    // In periodic mode, we just continuously read incoming data
    // The module will send NAV-PVT automatically without us polling
    let mut packet_count = 0;
    
    loop {
        // Read all available data
        // This is more efficient than polling since the module pushes data to us
        match gps.read_all() {
            Ok(_) => {
                packet_count += 1;
                // Data was processed (read_all prints debug info)
                // In a real application, you'd parse specific packets here
            }
            Err(e) => {
                eprintln!("Error reading data: {}", e);
            }
        }
        
        if packet_count % 10 == 0 && packet_count > 0 {
            println!("\n[Received {} packets so far]", packet_count);
        }
        
        // Small delay to avoid busy-waiting
        thread::sleep(Duration::from_millis(100));
    }
}

