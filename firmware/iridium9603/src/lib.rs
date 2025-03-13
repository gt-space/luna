use core::num;
use std::{array, error::Error};
use std::time::Duration;
use std::thread;
use chrono::Utc;

use rppal::uart::{Parity, Uart};

pub struct Iridium9603 {
  uart_port: Uart,
  pub momsn: u16
}

#[derive(Clone, Debug, Default, Eq, Hash, PartialEq)]
pub struct DeviceDetails {
  pub manuf_name: String,
  pub model_number: String,
  pub revision: String,
  pub imei: String
}

impl Iridium9603 {
  /// Initialize the Iridium9603 device with the given serial port path and email address 
  pub fn new(device_path: &str) -> Result<Self, Box<dyn Error>> { // Device path will probably be "/dev/serial0"
    // Serial port setup
    let mut uart_port = Uart::new(19200, Parity::None, 8, 1)?;
    uart_port.set_read_mode(1, Duration::default())?;
 
    let mut iridium = Self{
      uart_port: uart_port,
      momsn: 0  
    };
    // Reset
    iridium.sw_reset()?;

    Ok(iridium)
  }

  fn sw_reset(&mut self) -> Result<(), Box<dyn Error>> {
    // let mut buffer = [0u8; 1024];
    self.transfer("AT&F0\r")?;
    self.transfer("AT&K0\r")?;
    // self.serial_port.write_all("AT+CGMI\r".as_bytes())?;
    // let mut num_bytes_read = self.serial_port.read(&mut buffer)?; 
    Ok(())
  }

  pub fn get_device_details(&mut self) -> Result<DeviceDetails, Box<dyn Error>> {
    
    let manuf_name = self.transfer("AT+CGMI\r")?;
    let model_number = self.transfer("AT+CGMM\r")?;
    let revision = self.transfer("AT+CGMR\r")?;
    let imei = self.transfer("AT+CGSN\r")?;

    Ok(DeviceDetails{manuf_name, model_number, revision, imei})
    
  }

  pub fn send_email(&mut self, message: &str) -> Result<(), Box<dyn Error>> {
    println!("Clearing buffer of previous data...");
    self.transfer("AT+SBDD0\r")?;
    println!("Cleared!");
    println!("Writing {} to buffer...", message);
    let mut response = self.transfer(&format!("AT+SBDWB={}\r", message))?;
    println!("OK! Response: {}", response);
    println!("Initiating session...");
    response = self.transfer("AT+SBDI\r")?;
    println!("Response: {}", response);
    Ok(())
  }

  pub fn transfer(&mut self, command: &str)->Result<String, Box<dyn Error>>{
    self.uart_port.write(command.as_bytes())?;
    let mut buffer = [0u8; 64];
    let mut result = String::from("");
    let mut num_bytes_read = self.uart_port.read(&mut buffer)?;
    while num_bytes_read != 0 {
      let data = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();
      // println!("data: {:?}", data);
      result.push_str(&data);
      
      if result.ends_with("OK\r\n") {
        break;
      }
      if result.ends_with("ERROR\r\n") {
        return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidData, "Request resulted in an error")));
      }
    
      num_bytes_read = self.uart_port.read(&mut buffer)?;
    }

    // println!("result: {}", result);

    if let Some(parsed) = self.parse_iridium_response(&result, command) {
      Ok(parsed)
    } else {
      return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidData, "Failed to parse Iridium response")));
    }

  }

  fn parse_iridium_response(&mut self, response: &String, command: &str) -> Option<String> {
    response
        .strip_prefix(command) // Remove the command prefix
        .and_then(|s| s.strip_suffix("OK\r\n")) // Remove the "OK" suffix
        .map(|s| s.trim().to_string()) // Trim spaces and convert to String
  }
}
