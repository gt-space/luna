// use serialport::*;
use core::num;
use std::error::Error;
use std::time::Duration;
use std::thread;

use rppal::uart::{Parity, Uart};

// extern crate chrono;


use sbd::mo::{InformationElement, Header, SessionStatus, Message};


pub struct Iridium9603 {
  email: String,
  uart_port: Uart,
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
  pub fn new(device_path: &str, email: &str) -> Result<Self, Box<dyn Error>> { // Device path will probably be "/dev/serial0"
    // Serial port setup
    let mut uart_port = Uart::new(19200, Parity::None, 8, 1)?;
    uart_port.set_read_mode(1, Duration::default())?;
    
    let mut iridium = Self{
      email: email.into(),
      uart_port: uart_port
    };
    // Reset
    // iridium.sw_reset()?;
    
    Ok(iridium)
  }

  // Find the correct error type
  fn sw_reset(&mut self) -> Result<(), Box<dyn Error>> {
    // let mut buffer = [0u8; 1024];
    self.uart_port.write("AT&F0\r".as_bytes())?;
    self.uart_port.write("AT&K0\r".as_bytes())?;
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

  pub fn transfer(&mut self, command: &str)->Result<String, Box<dyn Error>>{
    self.uart_port.write(command.as_bytes())?;
    let mut buffer = [0u8; 64];
    let mut result = String::from("");
    let mut num_bytes_read = self.uart_port.read(&mut buffer)?;
    while num_bytes_read != 0 {
      let data = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();
      result.push_str(&data);
      
      if result.ends_with("OK\r\n") {
        break;
      }
      if result.ends_with("ERROR\r\n") {
        return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidData, "Request resulted in an error")));
      }
    
      num_bytes_read = self.uart_port.read(&mut buffer)?;
    }

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

  pub fn send_email(&mut self, message: &str) {
    // let initiated = self.transfer("AT+SBDIX\r"); // Initiate an SBD Session Extended

    let header = InformationElement::Header(Header {
      auto_id: 1,
      imei: [0; 15],
      session_status: SessionStatus::Ok,
      momsn: 1,
      mtmsn: 0,
      time_of_session: Utc.ymd(2017, 10, 1).and_hms(0, 0, 0),
    });
    let payload = InformationElement::Payload(message.as_bytes().to_vec());
    let message = Message::new(vec![header, payload]);
    let formatted_message = message.as_bytes();
    

    // let response = self.transfer("AT+SBDWB={%d}", formatted_message.len());
    let response = self.transfer(&format!("AT+SBDWB={}", formatted_message.len()));

    let initiated = self.transfer("AT+SBDIX\r"); // Initiate an SBD Session Extended
    uart_port.write(formatted_message);  

  }
}
