//! Continuous reading example
//! 
//! This example demonstrates how to continuously read all available data
//! from the GPS module. This is useful for debugging and understanding
//! what messages the module is sending.
//! 
//! To run:
//! ```bash
//! cargo run --example continuous_read
//! ```

use zedf9p04b::GPS;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Initializing GPS module on I2C bus 1...");
    
    // Create GPS instance
    let mut gps = GPS::new(1, None)?;
    
    println!("GPS initialized successfully!");
    println!("Reading all available data (press Ctrl+C to exit)...\n");
    
    // Continuously read and display all packets
    loop {
        match gps.read_all() {
            Ok(_) => {
                // Data was read and printed
            }
            Err(e) => {
                eprintln!("Error reading data: {}", e);
            }
        }
        
        // Small delay to avoid overwhelming the CPU
        thread::sleep(Duration::from_millis(100));
    }
}

