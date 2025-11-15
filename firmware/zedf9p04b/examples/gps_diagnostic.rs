//! GPS Diagnostic Tool
//! 
//! This tool helps diagnose why you're not getting a GPS fix.
//! It checks various aspects of the GPS module and satellite reception.

use zedf9p04b::GPS;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== GPS Diagnostic Tool ===\n");
    
    let mut gps = GPS::new(1, None)?;
    println!("✓ GPS module initialized\n");
    
    // Test 1: Basic communication
    println!("1. Testing basic communication...");
    match gps.mon_ver() {
        Ok(_) => println!("   ✓ Module responds to commands"),
        Err(e) => {
            println!("   ✗ Communication error: {}", e);
            return Ok(());
        }
    }
    
    // Test 2: Check if module is receiving any data
    println!("\n2. Checking for any incoming data...");
    let mut data_count = 0;
    for i in 1..=5 {
        print!("   Attempt {}: ", i);
        match gps.read_all() {
            Ok(_) => {
                data_count += 1;
                println!("Got data");
            }
            Err(e) => {
                println!("No data - {}", e);
            }
        }
        thread::sleep(Duration::from_millis(500));
    }
    
    if data_count == 0 {
        println!("   ⚠ No data received - module may not be configured properly");
    } else {
        println!("   ✓ Module is sending data");
    }
    
    // Test 3: Try polling for PVT with detailed output
    println!("\n3. Testing PVT polling with detailed output...");
    for attempt in 1..=10 {
        print!("   Attempt {}: ", attempt);
        
        match gps.poll_pvt() {
            Ok(Some(pvt)) => {
                println!("Got PVT data!");
                
                if let Some(pos) = pvt.position {
                    println!("     ✓ Position: {:.7}°, {:.7}°", pos.lat, pos.lon);
                    println!("     ✓ Altitude: {:.2}m", pos.alt);
                } else {
                    println!("     • No position data (no fix yet)");
                }
                
                if let Some(vel) = pvt.velocity {
                    println!("     ✓ Velocity: {:.2} m/s", vel.speed);
                } else {
                    println!("     • No velocity data");
                }
                
                if let Some(time) = pvt.time {
                    println!("     ✓ Time: {}", time);
                } else {
                    println!("     • No time data");
                }
                
                // If we got any data, the module is working
                break;
            }
            Ok(None) => {
                println!("No PVT data available");
            }
            Err(e) => {
                println!("Error: {}", e);
            }
        }
        
        thread::sleep(Duration::from_millis(1000));
    }
    
    // Test 4: Check antenna and signal strength
    println!("\n4. Antenna and signal diagnostics:");
    println!("   • Is the antenna connected securely?");
    println!("   • Is the antenna positioned with clear sky view?");
    println!("   • Are you indoors? (GPS signals don't penetrate buildings well)");
    println!("   • Try moving closer to a window or going outside");
    
    // Test 5: Time-based diagnostics
    println!("\n5. Time-based diagnostics:");
    println!("   • Cold start can take 30-60 seconds");
    println!("   • Hot start should be 2-5 seconds");
    println!("   • If module was powered off >2 hours, it's a cold start");
    
    // Test 6: Configuration check
    println!("\n6. Configuration check:");
    println!("   • Module should be configured for UBX protocol (not NMEA)");
    println!("   • Run 'configure_ubx' if you haven't already");
    println!("   • Check that I2C is working: sudo i2cdetect -y 1");
    
    println!("\n=== Diagnostic Complete ===");
    println!("\nNext steps:");
    println!("1. If no data at all: Check wiring and run configure_ubx");
    println!("2. If data but no fix: Move to better location and wait longer");
    println!("3. If still no fix: Check antenna connection");
    
    Ok(())
}

