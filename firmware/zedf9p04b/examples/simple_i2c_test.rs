//! Simple I2C Test
//! 
//! This bypasses our driver and tests I2C communication directly

use rppal::i2c::I2c;
use std::{thread, time::Duration};
use ublox::{Parser, PacketRef, UbxPacketRequest, MonVer};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Simple I2C GPS Test\n");
    
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(0x42)?;
    println!("✓ I2C initialized\n");
    
    // Create parser
    let mut parser = Parser::default();
    
    // Send MON-VER request
    let request = UbxPacketRequest::request_for::<MonVer>().into_packet_bytes();
    println!("Sending MON-VER request...");
    i2c.write(&request)?;
    println!("✓ Request sent\n");
    
    // Wait and try reading multiple times
    thread::sleep(Duration::from_millis(100));
    
    println!("Reading responses (trying 20 times)...\n");
    let mut found_response = false;
    
    for attempt in 1..=20 {
        // Try to read data directly
        let mut buf = vec![0u8; 512];
        
        match i2c.read(&mut buf) {
            Ok(_) => {
                // Check if there's any real data (not all 0xFF)
                let non_ff_count = buf.iter().filter(|&&b| b != 0xFF).count();
                
                if non_ff_count > 0 {
                    println!("Attempt {}: Got {} non-0xFF bytes", attempt, non_ff_count);
                    
                    // Show first 32 bytes
                    print!("First 32 bytes: ");
                    for (i, &byte) in buf.iter().take(32).enumerate() {
                        print!("{:02X} ", byte);
                        if (i + 1) % 16 == 0 {
                            println!();
                            print!("                ");
                        }
                    }
                    println!();
                    
                    // Try parsing
                    let mut it = parser.consume(&buf);
                    while let Some(result) = it.next() {
                        match result {
                            Ok(packet) => {
                                found_response = true;
                                println!("\n✓ Found packet: {:?}\n", packet);
                                
                                if let PacketRef::MonVer(mon_ver) = packet {
                                    println!("SW version: {}", mon_ver.software_version());
                                    println!("HW version: {}", mon_ver.hardware_version());
                                }
                            }
                            Err(e) => {
                                // Malformed packet, that's ok
                            }
                        }
                    }
                } else {
                    println!("Attempt {}: All 0xFF (no data)", attempt);
                }
            }
            Err(e) => {
                println!("Attempt {}: Read error: {}", attempt, e);
            }
        }
        
        if found_response {
            break;
        }
        
        thread::sleep(Duration::from_millis(100));
    }
    
    if !found_response {
        println!("\n⚠ No valid UBX responses received");
        println!("\nTroubleshooting steps:");
        println!("1. Check if module appears in: sudo i2cdetect -y 1");
        println!("2. Verify wiring (SDA to GPIO2, SCL to GPIO3)");
        println!("3. Check power supply to module");
        println!("4. Try different I2C address if module is configured differently");
    }
    
    Ok(())
}

