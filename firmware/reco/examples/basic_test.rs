//! Basic RECO driver test
//! 
//! This example demonstrates basic communication with the RECO board:
//! - Initialization
//! - Sending "launched" message
//! - Exchanging GPS data and reading RECO telemetry
//! - Sending voting logic configuration
//! - Receiving data from RECO
//! 
//! To run:
//! ```bash
//! cargo run --example basic_test
//! ```

use reco::{RecoDriver, FcGpsBody, VotingLogic, opcode};
use std::env;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env::set_var("RUST_BACKTRACE", "1");
    
    // Enable debug mode if RECO_DEBUG environment variable is set
    // This will print raw bytes received
    // Usage: RECO_DEBUG=1 cargo run --example basic_test
    // Or: export RECO_DEBUG=1 && cargo run --example basic_test
    if std::env::var("RECO_DEBUG").is_ok() {
        println!("DEBUG MODE ENABLED - Raw bytes will be printed");
    }
    
    println!("RECO Driver Basic Test");
    println!("======================");
    
    println!("Initializing RECO driver on /dev/spidev1.0...");
    let mut reco = match RecoDriver::new("/dev/spidev1.0") {
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
    
    match reco.send_gps_data_and_receive_reco(&gps_data) {
        Ok(reco_data) => {
            println!("✓ GPS data exchanged successfully");
            println!("  Velocity: N={:.2}, E={:.2}, D={:.2} m/s", 
                gps_data.velocity_north, gps_data.velocity_east, gps_data.velocity_down);
            println!("  Position: Lat={:.4}, Lon={:.4}, Alt={:.2} m", 
                gps_data.latitude, gps_data.longitude, gps_data.altitude);
            println!("  Valid: {}", gps_data.valid);
            println!("  RECO temperature: {:.2}°C", reco_data.temperature);
            println!("  RECO pressure: {:.2} Pa", reco_data.pressure);
            println!("  RECO stage 1 enabled: {}", reco_data.stage1_enabled);
            println!("  RECO stage 2 enabled: {}", reco_data.stage2_enabled);
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
            println!("  Stage 1 enabled: {}", data.stage1_enabled);
            println!("  Stage 2 enabled: {}", data.stage2_enabled);
            println!("  Vref A: [{}, {}]", data.vref_a_stage1, data.vref_a_stage2);
            println!("  Vref B: [{}, {}]", data.vref_b_stage1, data.vref_b_stage2);
            println!("  Vref C: [{}, {}]", data.vref_c_stage1, data.vref_c_stage2);
            println!("  Vref D: [{}, {}]", data.vref_d_stage1, data.vref_d_stage2);
            println!("  Vref E Stage1: [{}, {}]", data.vref_e_stage1_1, data.vref_e_stage1_2);
        }
        Err(e) => {
            eprintln!("✗ Failed to receive data: {}", e);
            eprintln!("  Note: This may fail if RECO board is not connected or not responding");
            // Don't return error here, as this might be expected in testing
        }
    }
    
    println!("\n✓ Basic test completed successfully!");
    Ok(())
}

