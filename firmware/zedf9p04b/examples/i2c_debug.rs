//! I2C Debug Tool
//! 
//! This tool helps diagnose I2C communication issues with the GPS module.
//! Run this to check if the module is responding.

use rppal::i2c::I2c;
use std::{thread, time::Duration};

const UBLOX_I2C_ADDRESS: u16 = 0x42;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== GPS I2C Debug Tool ===\n");
    
    // Initialize I2C
    println!("1. Initializing I2C bus 1...");
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(UBLOX_I2C_ADDRESS)?;
    println!("   ✓ I2C initialized at address 0x{:02X}\n", UBLOX_I2C_ADDRESS);
    
    // Try to read raw bytes
    println!("2. Attempting to read raw bytes from I2C...");
    let mut buf = [0u8; 32];
    
    for attempt in 1..=5 {
        print!("   Attempt {}: ", attempt);
        match i2c.read(&mut buf) {
            Ok(_) => {
                println!("Success! Read {} bytes", buf.len());
                print!("   Data: ");
                for (i, byte) in buf.iter().enumerate() {
                    print!("{:02X} ", byte);
                    if (i + 1) % 16 == 0 {
                        println!();
                        print!("         ");
                    }
                }
                println!("\n");
                
                // Check if we got any non-zero or non-FF data
                let has_data = buf.iter().any(|&b| b != 0x00 && b != 0xFF);
                if has_data {
                    println!("   ✓ Received valid data from module!");
                } else {
                    println!("   ⚠ All bytes are 0x00 or 0xFF (no data available)");
                }
            }
            Err(e) => {
                println!("Failed: {}", e);
            }
        }
        
        thread::sleep(Duration::from_millis(500));
    }
    
    println!("\n3. Trying to write a UBX poll request...");
    // UBX-MON-VER poll request
    let mon_ver_request: [u8; 8] = [
        0xB5, 0x62,  // Sync chars
        0x0A, 0x04,  // Class, ID (MON-VER)
        0x00, 0x00,  // Length (0)
        0x0E, 0x34,  // Checksum
    ];
    
    print!("   Writing: ");
    for byte in &mon_ver_request {
        print!("{:02X} ", byte);
    }
    println!();
    
    match i2c.write(&mon_ver_request) {
        Ok(_) => {
            println!("   ✓ Write successful!");
            
            // Wait for response
            println!("\n4. Waiting 200ms for response...");
            thread::sleep(Duration::from_millis(200));
            
            println!("5. Reading response...");
            for attempt in 1..=10 {
                let mut response = vec![0u8; 128];
                match i2c.read(&mut response) {
                    Ok(_) => {
                        // Check if we got UBX response (starts with 0xB5 0x62)
                        if response[0] == 0xB5 && response[1] == 0x62 {
                            println!("   ✓ Got UBX response!");
                            print!("   Response: ");
                            for (i, byte) in response.iter().take(40).enumerate() {
                                print!("{:02X} ", byte);
                                if (i + 1) % 16 == 0 {
                                    println!();
                                    print!("             ");
                                }
                            }
                            println!("\n");
                            break;
                        } else if response.iter().any(|&b| b != 0x00 && b != 0xFF) {
                            println!("   Attempt {}: Got data but not UBX format", attempt);
                            print!("   First 20 bytes: ");
                            for byte in response.iter().take(20) {
                                print!("{:02X} ", byte);
                            }
                            println!();
                        } else {
                            println!("   Attempt {}: No data yet", attempt);
                        }
                    }
                    Err(e) => {
                        println!("   Attempt {}: Read error: {}", attempt, e);
                    }
                }
                thread::sleep(Duration::from_millis(100));
            }
        }
        Err(e) => {
            println!("   ✗ Write failed: {}", e);
        }
    }
    
    println!("\n=== Debug Complete ===");
    println!("\nIf you see data but no UBX responses, the module might be in a different mode.");
    println!("Check your wiring and ensure the module is powered correctly.");
    
    Ok(())
}

