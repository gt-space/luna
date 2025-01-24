// use serialport::*;
use core::num;
use std::error::Error;
use std::time::Duration;
use std::thread;

use rppal::uart::{Parity, Uart};

pub struct Iridium9603 {
  email: String,
  uart_port: Uart,
}

#[derive(Clone, Debug, Default, Eq, Hash, PartialEq)]
pub struct DeviceDetails {
  pub manuf_name: String,
  // pub model_number: String,
  // pub revision: String,
  // pub imei: String
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
    //iridium.sw_reset()?;
    
    Ok(iridium)
  }

  // fn sw_reset(&mut self) -> Result<()> {
  //   // let mut buffer = [0u8; 1024];
  //   self.serial_port.write_all("AT&F0\r".as_bytes())?;
  //   self.serial_port.write_all("AT&K0\r".as_bytes())?;
  //   // self.serial_port.write_all("AT+CGMI\r".as_bytes())?;
  //   // let mut num_bytes_read = self.serial_port.read(&mut buffer)?; 
  //   Ok(())
  // }

  pub fn get_device_details(&mut self) -> Result<DeviceDetails, Box<dyn Error>> {
    // let mut buffer = [0u8; 1024];

    self.uart_port.write("AT+CGMI\r".as_bytes())?;
    println!("Wrote command!");
    transfer(self);
    // let mut num_bytes_read = self.uart_port.read(&mut buffer)?;
    println!("Read!");
    // let manuf_name = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();
    // println!("Manufacturer: {}", manuf_name);
  
    // self.uart_port.write("AT+CGMM\r".as_bytes())?;
    
    // num_bytes_read = self.uart_port.read(&mut buffer)?;
    // let model_number = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();
    // println!("Model Number: {}" , model_number);
    

    // self.uart_port.write("AT+CGMR\r".as_bytes())?;
    
    // num_bytes_read = self.uart_port.read(&mut buffer)?;
    // let revision = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();
    // println!("Revision: {}", revision);
    

    // self.uart_port.write("AT+CGSN\r".as_bytes())?;
    // num_bytes_read = self.uart_port.read(&mut buffer)?;
    // let imei = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();
    // println!("Imei: {}", imei);

    Ok(DeviceDetails{manuf_name})
    
  } 

  fn transfer(&mut self,)->Result<String, std::io::Error>{
    let mut buffer = [0u8; 1024];
    let mut result = String::from("");
    let mut num_bytes_read = uart.read(&mut buffer)?;
    while num_bytes_read != 0 {
      let data = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();
      // println!("{}", manuf_name);
      result.push_str(&data);
      num_bytes_read = uart.read(&mut buffer)?;
      println!("Data: {}", data);
    }

    Ok((result))
  }
}
