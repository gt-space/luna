//! RECO message protocol test
//! 
//! This example demonstrates the RECO message protocol:
//! - Sending different message types (launched, GPS, voting logic)
//! - Receiving data from RECO
//! 
//! To run:
//! ```bash
//! cargo run --example channel_test
//! ```

use reco::{RecoDriver, FcGpsBody};
use std::env;
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    
    println!("RECO Message Protocol Test");
    println!("==========================");
    
    println!("Initializing RECO driver on /dev/spidev1.1...");
    let mut reco = RecoDriver::new("/dev/spidev1.1")?;
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
        match reco.send_gps_data_and_receive_reco(&gps_data) {
            Ok(reco_data) => {
                println!("    ✓ GPS data exchanged");
                println!("      Valid: {}, Alt: {:.1}m", gps_data.valid, gps_data.altitude);
                println!("      RECO quaternion w: {:.4}", reco_data.quaternion[3]);
            }
            Err(e) => {
                eprintln!("    ✗ Failed to send GPS data: {}", e);
            }
        }
        thread::sleep(Duration::from_millis(100));
    }
    
    // Test 3: Send EKF-initialization command
    println!("\n--- Test 3: Sending EKF-init command ---");
    for i in 1..=3 {
        println!("  Sending EKF-init message #{}...", i);
        match reco.send_init_ekf() {
            Ok(_) => println!("    ✓ EKF-init message #{} sent", i),
            Err(e) => {
                eprintln!("    ✗ Failed to send EKF-init message #{}: {}", i, e);
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
                println!("      Stages: S1={}, S2={}", 
                    data.stage1_enabled, data.stage2_enabled);
            }
            Err(e) => {
                eprintln!("    ✗ Failed to receive data: {}", e);
                eprintln!("      (This may be expected if RECO is not connected)");
            }
        }
        thread::sleep(Duration::from_millis(100));
    }
    
    println!("\n✓ Message protocol test completed!");
    Ok(())
}

