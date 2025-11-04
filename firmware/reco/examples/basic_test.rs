//! Basic RECO driver test
//! 
//! This example demonstrates basic communication with the RECO board:
//! - Initialization
//! - Sending "launched" message
//! - Sending GPS data
//! - Sending voting logic configuration
//! - Receiving data from RECO
//! 
//! To run:
//! ```bash
//! cargo run --example basic_test
//! ```

use reco::{RecoDriver, FcGpsBody, VotingLogic, opcode};
use rppal::gpio::Gpio;
use rppal::spi::{Bus, SlaveSelect};
use std::env;

// GPIO pin 16 for RECO chip select (active low)
const RECO_CS_PIN: u8 = 16;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    
    println!("RECO Driver Basic Test");
    println!("======================");
    
    // Initialize GPIO and chip select pin
    println!("Initializing GPIO...");
    let gpio = Gpio::new()?;
    let mut cs_pin = gpio.get(RECO_CS_PIN)?.into_output();
    cs_pin.set_high(); // Active low, start high (inactive)
    println!("✓ GPIO initialized (CS pin: {})", RECO_CS_PIN);
    
    // Initialize RECO driver
    println!("Initializing RECO driver on SPI0, CS0...");
    let mut reco = match RecoDriver::new(Bus::Spi0, SlaveSelect::Ss0, Some(cs_pin)) {
        Ok(driver) => {
            println!("✓ RECO driver initialized successfully");
            driver
        }
        Err(e) => {
            eprintln!("✗ Failed to initialize RECO driver: {}", e);
            return Err(Box::new(e));
        }
    };
    
    // Test 1: Send "launched" message
    println!("\n--- Test 1: Sending 'launched' message (opcode {:#02X}) ---", opcode::LAUNCHED);
    match reco.send_launched() {
        Ok(_) => println!("✓ 'Launched' message sent successfully"),
        Err(e) => {
            eprintln!("✗ Failed to send 'launched' message: {}", e);
            return Err(Box::new(e));
        }
    }
    
    // Test 2: Send GPS data
    println!("\n--- Test 2: Sending GPS data (opcode {:#02X}) ---", opcode::GPS_DATA);
    let gps_data = FcGpsBody {
        velocity_north: 10.5,
        velocity_east: 2.3,
        velocity_down: -5.1,
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 100.0,
        valid: true,
    };
    
    match reco.send_gps_data(&gps_data) {
        Ok(_) => {
            println!("✓ GPS data sent successfully");
            println!("  Velocity: N={:.2}, E={:.2}, D={:.2} m/s", 
                gps_data.velocity_north, gps_data.velocity_east, gps_data.velocity_down);
            println!("  Position: Lat={:.4}, Lon={:.4}, Alt={:.2} m", 
                gps_data.latitude, gps_data.longitude, gps_data.altitude);
            println!("  Valid: {}", gps_data.valid);
        }
        Err(e) => {
            eprintln!("✗ Failed to send GPS data: {}", e);
            return Err(Box::new(e));
        }
    }
    
    // Test 3: Send voting logic configuration
    println!("\n--- Test 3: Sending voting logic (opcode {:#02X}) ---", opcode::VOTING_LOGIC);
    let voting_logic = VotingLogic {
        processor_1_enabled: true,
        processor_2_enabled: true,
        processor_3_enabled: false,
    };
    
    match reco.send_voting_logic(&voting_logic) {
        Ok(_) => {
            println!("✓ Voting logic sent successfully");
            println!("  Processor 1: {}", if voting_logic.processor_1_enabled { "enabled" } else { "disabled" });
            println!("  Processor 2: {}", if voting_logic.processor_2_enabled { "enabled" } else { "disabled" });
            println!("  Processor 3: {}", if voting_logic.processor_3_enabled { "enabled" } else { "disabled" });
        }
        Err(e) => {
            eprintln!("✗ Failed to send voting logic: {}", e);
            return Err(Box::new(e));
        }
    }
    
    // Test 4: Receive data from RECO
    println!("\n--- Test 4: Receiving data from RECO ---");
    match reco.receive_data() {
        Ok(data) => {
            println!("✓ Data received successfully");
            println!("  Quaternion: [{:.4}, {:.4}, {:.4}, {:.4}]", 
                data.quaternion[0], data.quaternion[1], data.quaternion[2], data.quaternion[3]);
            println!("  Position (LLA): [{:.6}, {:.6}, {:.2}]", 
                data.lla_pos[0], data.lla_pos[1], data.lla_pos[2]);
            println!("  Velocity: [{:.2}, {:.2}, {:.2}]", 
                data.velocity[0], data.velocity[1], data.velocity[2]);
            println!("  Temperature: {:.2}°C", data.temperature);
            println!("  Pressure: {:.2} Pa", data.pressure);
        }
        Err(e) => {
            eprintln!("✗ Failed to receive data: {}", e);
            eprintln!("  Note: This may fail if RECO board is not connected or not responding");
            // Don't return error here, as this might be expected in testing
        }
    }
    
    println!("\n✓ Basic test completed successfully!");
    println!("\nNote: Checksums are calculated on opcode + body (bytes 0-25) for all messages sent to RECO.");
    Ok(())
}

