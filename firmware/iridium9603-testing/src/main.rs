use iridium9603::Iridium9603;
use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    // Device path to the serial port
    let device_path = "/dev/serial0";
    let email = "test@example.com";

    // Initialize the Iridium9603 driver
    let mut iridium = Iridium9603::new(device_path, email)?;
    println!("Iridium9603 driver initialized successfully!");

    // Retrieve device details
    // match iridium.get_device_details() {
    //     Ok(details) => {
    //         println!("Manufacturer Name: {}", details.manuf_name);
    //         println!("Model Number: {}", details.model_number);
    //         println!("Revision: {}", details.revision);
    //         println!("IMEI: {}", details.imei);
    //     }
    //     Err(e) => {
    //         eprintln!("Failed to get device details: {}", e);
    //     }
    // }
    match iridium.send_email("hello") {
        Ok(_) => {
            println!("Success writing sbd data!");
        }
        Err(e) => {
            eprintln!("Failed to get device details: {}", e);
        }
    }
     

    Ok(())
}

