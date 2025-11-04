//! RECO message protocol test
//! 
//! This example demonstrates the RECO message protocol:
//! - Sending different message types (launched, GPS, voting logic)
//! - Receiving data from RECO
//! - Demonstrating checksum calculation (includes opcode)
//! 
//! To run:
//! ```bash
//! cargo run --example channel_test
//! ```

use reco::{RecoDriver, FcGpsBody, VotingLogic};
use rppal::gpio::Gpio;
use rppal::spi::{Bus, SlaveSelect};
use std::env;
use std::thread;
use std::time::Duration;

// GPIO pin 16 for RECO chip select (active low)
const RECO_CS_PIN: u8 = 16;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    
    println!("RECO Message Protocol Test");
    println!("==========================");
    
    // Initialize GPIO and chip select pin
    println!("Initializing GPIO...");
    let gpio = Gpio::new()?;
    let mut cs_pin = gpio.get(RECO_CS_PIN)?.into_output();
    cs_pin.set_high(); // Active low, start high (inactive)
    
    // Initialize RECO driver
    println!("Initializing RECO driver...");
    let mut reco = RecoDriver::new(Bus::Spi0, SlaveSelect::Ss0, Some(cs_pin))?;
    println!("✓ Driver initialized");
    
    // Test 1: Send multiple "launched" messages
    println!("\n--- Test 1: Sending 'launched' messages ---");
    for i in 1..=3 {
        println!("  Sending 'launched' message #{}...", i);
        match reco.send_launched() {
            Ok(_) => println!("    ✓ Message #{} sent", i),
            Err(e) => {
                eprintln!("    ✗ Failed to send message #{}: {}", i, e);
                continue;
            }
        }
        thread::sleep(Duration::from_millis(100));
    }
    
    // Test 2: Send GPS data with different values
    println!("\n--- Test 2: Sending GPS data messages ---");
    let gps_test_cases = vec![
        ("Test 1", FcGpsBody {
            velocity_north: 0.0, velocity_east: 0.0, velocity_down: 0.0,
            latitude: 0.0, longitude: 0.0, altitude: 0.0, valid: false,
        }),
        ("Test 2", FcGpsBody {
            velocity_north: 10.5, velocity_east: 2.3, velocity_down: -5.1,
            latitude: 37.7749, longitude: -122.4194, altitude: 100.0, valid: true,
        }),
        ("Test 3", FcGpsBody {
            velocity_north: -15.2, velocity_east: 8.7, velocity_down: 12.3,
            latitude: -33.8688, longitude: 151.2093, altitude: 500.0, valid: true,
        }),
    ];
    
    for (name, gps_data) in gps_test_cases {
        println!("  Sending GPS data ({})...", name);
        match reco.send_gps_data(&gps_data) {
            Ok(_) => {
                println!("    ✓ GPS data sent");
                println!("      Valid: {}, Alt: {:.1}m", gps_data.valid, gps_data.altitude);
            }
            Err(e) => {
                eprintln!("    ✗ Failed to send GPS data: {}", e);
            }
        }
        thread::sleep(Duration::from_millis(100));
    }
    
    // Test 3: Send different voting logic configurations
    println!("\n--- Test 3: Sending voting logic configurations ---");
    let voting_configs = vec![
        ("All enabled", VotingLogic {
            processor_1_enabled: true,
            processor_2_enabled: true,
            processor_3_enabled: true,
        }),
        ("Processors 1 and 2", VotingLogic {
            processor_1_enabled: true,
            processor_2_enabled: true,
            processor_3_enabled: false,
        }),
        ("Only processor 1", VotingLogic {
            processor_1_enabled: true,
            processor_2_enabled: false,
            processor_3_enabled: false,
        }),
        ("All disabled", VotingLogic {
            processor_1_enabled: false,
            processor_2_enabled: false,
            processor_3_enabled: false,
        }),
    ];
    
    for (name, voting_logic) in voting_configs {
        println!("  Sending voting logic ({})...", name);
        match reco.send_voting_logic(&voting_logic) {
            Ok(_) => {
                println!("    ✓ Voting logic sent");
                println!("      P1: {}, P2: {}, P3: {}", 
                    voting_logic.processor_1_enabled,
                    voting_logic.processor_2_enabled,
                    voting_logic.processor_3_enabled);
            }
            Err(e) => {
                eprintln!("    ✗ Failed to send voting logic: {}", e);
            }
        }
        thread::sleep(Duration::from_millis(100));
    }
    
    // Test 4: Attempt to receive data multiple times
    println!("\n--- Test 4: Receiving data from RECO ---");
    for i in 1..=3 {
        println!("  Receiving data (attempt #{})...", i);
        match reco.receive_data() {
            Ok(data) => {
                println!("    ✓ Data received");
                println!("      Temperature: {:.2}°C, Pressure: {:.2} Pa", 
                    data.temperature, data.pressure);
            }
            Err(e) => {
                eprintln!("    ✗ Failed to receive data: {}", e);
                eprintln!("      (This may be expected if RECO is not connected)");
            }
        }
        thread::sleep(Duration::from_millis(100));
    }
    
    println!("\n✓ Message protocol test completed!");
    println!("\nNote: All checksums include the opcode (bytes 0-25) for verification.");
    Ok(())
}

