//! Verify UBX configuration
//! 
//! This tool checks if the module is now responding with UBX format

use rppal::i2c::I2c;
use std::{thread, time::Duration};
use ublox::{Parser, PacketRef, UbxPacketRequest, MonVer};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Verify UBX Configuration ===\n");
    
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(0x42)?;
    let mut parser = Parser::default();
    
    println!("Sending MON-VER request...");
    let request = UbxPacketRequest::request_for::<MonVer>().into_packet_bytes();
    i2c.write(&request)?;
    
    thread::sleep(Duration::from_millis(200));
    
    println!("Reading response...\n");
    
    for attempt in 1..=10 {
        let mut buf = vec![0u8; 512];
        
        match i2c.read(&mut buf) {
            Ok(_) => {
                // Check for UBX sync bytes
                if buf.iter().any(|&b| b == 0xB5) {
                    println!("Attempt {}: Found UBX sync byte (0xB5)!", attempt);
                    
                    // Try to parse
                    let mut it = parser.consume(&buf);
                    let mut found_valid_packet = false;
                    
                    while let Some(result) = it.next() {
                        match result {
                            Ok(packet) => {
                                found_valid_packet = true;
                                println!("\n✓ SUCCESS! Received valid UBX packet:");
                                
                                match packet {
                                    PacketRef::MonVer(mon_ver) => {
                                        println!("  Type: MON-VER");
                                        println!("  SW version: {}", mon_ver.software_version());
                                        println!("  HW version: {}", mon_ver.hardware_version());
                                        println!("  Extensions: {:?}", mon_ver.extension().collect::<Vec<&str>>());
                                    }
                                    _ => {
                                        println!("  Type: {:?}", packet);
                                    }
                                }
                            }
                            Err(_) => {
                                // Malformed packet
                            }
                        }
                    }
                    
                    if found_valid_packet {
                        println!("\n=== Configuration Verified! ===");
                        println!("The module is now properly configured for UBX.");
                        return Ok(());
                    }
                } else {
                    // Check if it's still NMEA
                    let has_nmea = buf.iter().any(|&b| b == b'$');
                    let has_data = buf.iter().filter(|&&b| b != 0xFF && b != 0x00).count() > 10;
                    
                    if has_nmea {
                        println!("Attempt {}: Still receiving NMEA data!", attempt);
                        print!("  Sample: ");
                        for &byte in buf.iter().take(40).filter(|&&b| b != 0xFF) {
                            if byte >= 32 && byte <= 126 {
                                print!("{}", byte as char);
                            } else {
                                print!(".");
                            }
                        }
                        println!();
                    } else if has_data {
                        println!("Attempt {}: Got data, but not UBX or NMEA", attempt);
                    } else {
                        println!("Attempt {}: No data (all 0xFF)", attempt);
                    }
                }
            }
            Err(e) => {
                println!("Attempt {}: Read error: {}", attempt, e);
            }
        }
        
        thread::sleep(Duration::from_millis(100));
    }
    
    println!("\n⚠ Configuration may not have worked.");
    println!("Try running 'configure_ubx' again, or reset the module.");
    
    Ok(())
}

