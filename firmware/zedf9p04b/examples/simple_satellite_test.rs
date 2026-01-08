//! Simple Satellite Test
//! 
//! This tool tries multiple approaches to get satellite information
//! and shows exactly what's happening.

use rppal::i2c::I2c;
use std::{thread, time::Duration};
use ublox::{
    nav_sat::NavSat,
    nav_status::NavStatus,
    packetref_proto23::PacketRef,
    Parser, UbxPacket, UbxPacketRequest,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Simple Satellite Test ===\n");
    
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(0x42)?;
    let mut parser = Parser::default();
    
    println!("Testing different approaches to get satellite data...\n");
    
    // Test 1: Try NAV-SAT (satellite info)
    println!("1. Requesting NAV-SAT (satellite information)...");
    let nav_sat_request = UbxPacketRequest::request_for::<NavSat>().into_packet_bytes();
    i2c.write(&nav_sat_request)?;
    thread::sleep(Duration::from_millis(500));
    
    // Test 2: Try NAV-STATUS (navigation status)
    println!("2. Requesting NAV-STATUS (navigation status)...");
    let nav_status_request = UbxPacketRequest::request_for::<NavStatus>().into_packet_bytes();
    i2c.write(&nav_status_request)?;
    thread::sleep(Duration::from_millis(500));
    
    println!("3. Reading responses...\n");
    
    for attempt in 1..=20 {
        let mut buf = vec![0u8; 1024];
        
        match i2c.read(&mut buf) {
            Ok(_) => {
                let mut it = parser.consume_ubx(&buf);
                let mut found_any_packet = false;
                
                while let Some(result) = it.next() {
                    match result {
                        Ok(ubx_packet) => {
                            found_any_packet = true;
                            
                            // Extract Proto23 variant from UbxPacket
                            if let UbxPacket::Proto23(packet) = ubx_packet {
                                match packet {
                                    PacketRef::NavSat(nav_sat) => {
                                    let sat_count = nav_sat.num_svs();
                                    println!("✓ NAV-SAT: {} satellites visible", sat_count);
                                    
                                    if sat_count > 0 {
                                        for sv in nav_sat.svs() {
                                            let signal = sv.cno();
                                            if signal > 0 {
                                                println!("  SV{}: {}dB signal", sv.sv_id(), signal);
                                            }
                                        }
                                    } else {
                                        println!("  → No satellites visible (antenna issue?)");
                                    }
                                    PacketRef::NavStatus(nav_status) => {
                                        println!("✓ NAV-STATUS: Fix type = {:?}", nav_status.fix_type());
                                        println!("  Flags: {:?}", nav_status.flags());
                                    }
                                    _ => {
                                        println!("✓ Other packet: {:?}", packet);
                                    }
                                }
                            }
                        }
                        Err(_) => {
                            // Malformed packet
                        }
                    }
                }
                
                if !found_any_packet {
                    if attempt % 5 == 0 {
                        println!("Attempt {}: No valid packets received", attempt);
                    }
                }
            }
            Err(_) => {
                if attempt % 5 == 0 {
                    println!("Attempt {}: No data", attempt);
                }
            }
        }
        
        thread::sleep(Duration::from_millis(500));
    }
    
    println!("\n=== Analysis ===");
    println!("If you see:");
    println!("  • 'No satellites visible' → Antenna connection issue");
    println!("  • 'No valid packets' → Module not responding to requests");
    println!("  • Satellite signals <20dB → Weak signals (location issue)");
    println!("  • Satellite signals >30dB → Good signals (should get fix)");
    
    println!("\nNext steps:");
    println!("1. Check antenna connection visually");
    println!("2. Try moving outdoors");
    println!("3. Wait 2-3 minutes for cold start");
    println!("4. If still no satellites, antenna may be faulty");
    
    Ok(())
}


