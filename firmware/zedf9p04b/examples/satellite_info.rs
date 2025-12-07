//! Satellite Information Tool
//! 
//! Shows which satellites the module can see and signal strength

use zedf9p04b::GPS;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== GPS Satellite Information ===\n");
    
    let mut gps = GPS::new(1, None)?;
    println!("GPS initialized\n");
    
    println!("Checking for satellites...");
    println!("(This will show if the antenna is working)\n");
    
    for i in 1..=10 {
        println!("Attempt {}:", i);
        
        match gps.poll_pvt() {
            Ok(Some(pvt)) => {
                if let Some(pos) = pvt.position {
                    println!("  ✓ GPS FIX!");
                    println!("  Position: {:.7}°, {:.7}°", pos.lat, pos.lon);
                    println!("  Altitude: {:.2}m", pos.alt);
                    break;
                } else {
                    println!("  • Receiving data, but no position fix yet");
                    println!("  • Module is working, waiting for satellites...");
                }
                
                if let Some(time) = pvt.time {
                    println!("  • Time: {} (time fix acquired!)", time);
                }
            }
            Ok(None) => {
                println!("  • No fix data available");
                println!("  • Check antenna connection");
            }
            Err(e) => {
                println!("  ✗ Error: {}", e);
            }
        }
        
        println!();
        thread::sleep(Duration::from_secs(3));
    }
    
    println!("\nTips:");
    println!("  • Move antenna to location with clear sky view");
    println!("  • Wait 30-60 seconds for cold start");
    println!("  • Check antenna is properly connected");
    println!("  • Module needs line-of-sight to at least 4 satellites");
    
    Ok(())
}

