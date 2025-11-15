//! Satellite Signal Monitor
//! 
//! This tool continuously monitors satellite signals to help debug
//! why you're not getting a GPS fix.

use rppal::i2c::I2c;
use std::{thread, time::Duration};
use ublox::{Parser, PacketRef, UbxPacketRequest, NavSat};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Satellite Signal Monitor ===\n");
    
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(0x42)?;
    let mut parser = Parser::default();
    
    println!("Requesting satellite information...");
    
    // Send NAV-SAT poll request
    let request = UbxPacketRequest::request_for::<NavSat>().into_packet_bytes();
    i2c.write(&request)?;
    
    thread::sleep(Duration::from_millis(200));
    
    println!("Monitoring satellite signals (press Ctrl+C to exit)...\n");
    
    let mut last_sat_count = 0;
    let mut attempts = 0;
    
    loop {
        attempts += 1;
        
        // Try to read NAV-SAT data
        let mut buf = vec![0u8; 512];
        
        match i2c.read(&mut buf) {
            Ok(_) => {
                let mut it = parser.consume(&buf);
                let mut found_sat_data = false;
                
                while let Some(result) = it.next() {
                    match result {
                        Ok(packet) => {
                            if let PacketRef::NavSat(nav_sat) = packet {
                                found_sat_data = true;
                                let sat_count = nav_sat.num_svs();
                                
                                if sat_count != last_sat_count {
                                    println!("Satellites visible: {}", sat_count);
                                    last_sat_count = sat_count;
                                }
                                
                                if sat_count > 0 {
                                    println!("  Signal details:");
                                    
                                    for sv in nav_sat.svs() {
                                        let constellation = match sv.gnss_id() {
                                            0 => "GPS",
                                            1 => "SBAS", 
                                            2 => "GAL",
                                            3 => "BDS",
                                            6 => "GLO",
                                            5 => "QZSS",
                                            _ => "Unknown",
                                        };
                                        
                                        let signal_strength = sv.cno();
                                        let elevation = sv.elev();
                                        let azimuth = sv.azim();
                                        
                                        if signal_strength > 0 {
                                            println!("    {} SV{}: {}dB, elev:{}°, azim:{}°", 
                                                constellation, sv.sv_id(), signal_strength, elevation, azimuth);
                                        }
                                    }
                                    
                                    println!();
                                } else {
                                    if attempts % 10 == 0 {
                                        println!("  No satellites visible (attempt {})", attempts);
                                        println!("  Check antenna connection and sky view");
                                    }
                                }
                            }
                        }
                        Err(_) => {
                            // Malformed packet
                        }
                    }
                }
                
                if !found_sat_data {
                    // Try requesting again
                    if attempts % 20 == 0 {
                        println!("Requesting satellite data again...");
                        i2c.write(&request)?;
                    }
                }
            }
            Err(_) => {
                // No data
            }
        }
        
        thread::sleep(Duration::from_millis(1000));
    }
}

