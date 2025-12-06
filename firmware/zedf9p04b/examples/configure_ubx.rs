//! Configure GPS module to use UBX protocol on I2C
//! 
//! This tool configures the u-blox module to output UBX messages instead of NMEA

use rppal::i2c::I2c;
use std::{thread, time::Duration};

const UBLOX_I2C_ADDRESS: u16 = 0x42;

fn calculate_checksum(payload: &[u8]) -> (u8, u8) {
    let mut ck_a = 0u8;
    let mut ck_b = 0u8;
    for byte in payload {
        ck_a = ck_a.wrapping_add(*byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

fn send_ubx_message(i2c: &mut I2c, class: u8, id: u8, payload: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
    let mut message = vec![0xB5, 0x62]; // Sync chars
    message.push(class);
    message.push(id);
    message.push((payload.len() & 0xFF) as u8); // Length LSB
    message.push(((payload.len() >> 8) & 0xFF) as u8); // Length MSB
    message.extend_from_slice(payload);
    
    let (ck_a, ck_b) = calculate_checksum(&message[2..]);
    message.push(ck_a);
    message.push(ck_b);
    
    i2c.write(&message)?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== GPS UBX Configuration Tool ===\n");
    
    let mut i2c = I2c::new()?;
    i2c.set_slave_address(UBLOX_I2C_ADDRESS)?;
    println!("✓ I2C initialized\n");
    
    println!("Step 1: Disabling NMEA messages on I2C/DDC port...");
    
    // UBX-CFG-PRT for I2C/DDC port
    // Configure port 0 (DDC/I2C) to use UBX protocol only
    // See Interface Description section 3.10.13
    let cfg_prt_payload: Vec<u8> = vec![
        0x00,       // Port ID: 0 = DDC (I2C)
        0x00,       // Reserved
        0x00, 0x00, // txReady (not used)
        0x84, 0x00, 0x00, 0x00, // mode (slave address 0x42)
        0x00, 0x00, 0x00, 0x00, // reserved
        0x01, 0x00, // inProtoMask: UBX only (bit 0 = UBX)
        0x01, 0x00, // outProtoMask: UBX only (bit 0 = UBX)
        0x00, 0x00, // flags
        0x00, 0x00, // reserved
    ];
    
    send_ubx_message(&mut i2c, 0x06, 0x00, &cfg_prt_payload)?;
    println!("  ✓ Sent CFG-PRT (configure I2C port for UBX)");
    thread::sleep(Duration::from_millis(500));
    
    println!("\nStep 2: Enabling NAV-PVT messages on I2C...");
    
    // UBX-CFG-MSG: Enable NAV-PVT on DDC port
    // Rate array: [DDC, UART1, UART2, USB, SPI, Reserved]
    let cfg_msg_payload: Vec<u8> = vec![
        0x01,       // Class: NAV
        0x07,       // ID: PVT
        0x00,       // Rate on DDC/I2C: 0 = disabled for now (we'll poll)
        0x00,       // Rate on UART1
        0x00,       // Rate on UART2
        0x00,       // Rate on USB
        0x00,       // Rate on SPI
        0x00,       // Reserved
    ];
    
    send_ubx_message(&mut i2c, 0x06, 0x01, &cfg_msg_payload)?;
    println!("  ✓ Sent CFG-MSG (configure NAV-PVT)");
    thread::sleep(Duration::from_millis(500));
    
    println!("\nStep 3: Saving configuration to flash...");
    
    // UBX-CFG-CFG: Save configuration
    let cfg_cfg_payload: Vec<u8> = vec![
        0x00, 0x00, 0x00, 0x00, // Clear mask
        0xFF, 0xFF, 0x00, 0x00, // Save mask (save all)
        0x00, 0x00, 0x00, 0x00, // Load mask
        0x01,                   // Device: BBR (battery-backed RAM)
    ];
    
    send_ubx_message(&mut i2c, 0x06, 0x09, &cfg_cfg_payload)?;
    println!("  ✓ Sent CFG-CFG (save to flash)");
    thread::sleep(Duration::from_millis(1000));
    
    println!("\n=== Configuration Complete! ===");
    println!("\nThe module should now:");
    println!("  • Output UBX binary format (not NMEA)");
    println!("  • Respond to UBX poll requests");
    println!("  • Work with the GPS driver");
    println!("\nTry running the basic_usage example now!");
    
    Ok(())
}

