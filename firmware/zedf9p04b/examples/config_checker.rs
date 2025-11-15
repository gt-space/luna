//! Configuration Checker
//! 
//! This tool checks if the GPS module is properly configured
//! and suggests fixes if needed.

use rppal::i2c::I2c;
use std::{thread, time::Duration};
use ublox::{Parser, PacketRef, UbxPacketRequest, MonVer};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== GPS Configuration Checker ===\n");
    
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(0x42)?;
    let mut parser = Parser::default();
    
    // Test 1: Basic I2C communication
    println!("1. Testing I2C communication...");
    let request = UbxPacketRequest::request_for::<MonVer>().into_packet_bytes();
    
    match i2c.write(&request) {
        Ok(_) => println!("   ✓ Write successful"),
        Err(e) => {
            println!("   ✗ Write failed: {}", e);
            return Ok(());
        }
    }
    
    thread::sleep(Duration::from_millis(200));
    
    // Test 2: Check response format
    println!("\n2. Checking response format...");
    let mut buf = vec![0u8; 512];
    
    match i2c.read(&mut buf) {
        Ok(_) => {
            // Check if we got UBX format (starts with 0xB5 0x62)
            if buf[0] == 0xB5 && buf[1] == 0x62 {
                println!("   ✓ Receiving UBX binary format");
                
                // Try to parse
                let mut it = parser.consume(&buf);
                let mut found_valid_packet = false;
                
                while let Some(result) = it.next() {
                    match result {
                        Ok(packet) => {
                            found_valid_packet = true;
                            if let PacketRef::MonVer(mon_ver) = packet {
                                println!("   ✓ Module version: {}", mon_ver.software_version());
                                println!("   ✓ Hardware: {}", mon_ver.hardware_version());
                            }
                        }
                        Err(_) => {
                            println!("   ⚠ Got UBX sync bytes but malformed packet");
                        }
                    }
                }
                
                if !found_valid_packet {
                    println!("   ⚠ UBX format detected but no valid packets");
                }
            } else {
                // Check if it's NMEA text
                let has_nmea = buf.iter().any(|&b| b == b'$');
                if has_nmea {
                    println!("   ✗ Receiving NMEA text format (should be UBX)");
                    println!("   → Run 'configure_ubx' to fix this");
                } else {
                    println!("   ✗ Unknown data format");
                    print!("   First 20 bytes: ");
                    for byte in buf.iter().take(20) {
                        print!("{:02X} ", byte);
                    }
                    println!();
                }
            }
        }
        Err(e) => {
            println!("   ✗ Read failed: {}", e);
        }
    }
    
    // Test 3: Check for continuous data
    println!("\n3. Checking for continuous data flow...");
    let mut data_samples = 0;
    let mut non_ff_samples = 0;
    
    for i in 1..=10 {
        let mut buf = vec![0u8; 64];
        match i2c.read(&mut buf) {
            Ok(_) => {
                data_samples += 1;
                let non_ff_count = buf.iter().filter(|&&b| b != 0xFF).count();
                if non_ff_count > 0 {
                    non_ff_samples += 1;
                }
            }
            Err(_) => {
                // No data
            }
        }
        thread::sleep(Duration::from_millis(100));
    }
    
    println!("   Data samples: {}/10", data_samples);
    println!("   Non-0xFF samples: {}/10", non_ff_samples);
    
    if non_ff_samples == 0 {
        println!("   ⚠ No real data received - module may not be configured");
    } else if non_ff_samples < 5 {
        println!("   ⚠ Intermittent data - check connections");
    } else {
        println!("   ✓ Good data flow");
    }
    
    // Summary and recommendations
    println!("\n=== Summary ===");
    
    if non_ff_samples == 0 {
        println!("❌ Module appears to be misconfigured or not responding");
        println!("   Try:");
        println!("   1. Run 'configure_ubx'");
        println!("   2. Check wiring (SDA, SCL, GND, VCC)");
        println!("   3. Verify I2C is enabled: sudo i2cdetect -y 1");
    } else {
        println!("✅ Module is configured and responding");
        println!("   If you're still not getting GPS fixes:");
        println!("   1. Check antenna connection");
        println!("   2. Move to location with clear sky view");
        println!("   3. Wait 30-60 seconds for cold start");
        println!("   4. Run 'satellite_monitor' to see satellite signals");
    }
    
    Ok(())
}

