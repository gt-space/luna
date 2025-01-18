use serialport::*;

pub struct Iridium9603 {
  email: String,
  serial_port: Box<dyn SerialPort> 
}

#[derive(Clone, Debug, Default, Eq, Hash, PartialEq)]
pub struct DeviceDetails {
  pub manuf_name: String,
  pub model_number: String,
  pub revision: String,
  pub imei: u64
}

impl Iridium9603 {
  /// Initialize the Iridium9603 device with the given serial port path and email address 
  pub fn new(device_path: &str, email: &str) -> Result<Self> { // Device path will probably be "/dev/serial0"
    // Serial port setup
    let serial_port = serialport::new(device_path, 19200)
    .data_bits(DataBits::Eight)
    .parity(Parity::None)
    .stop_bits(StopBits::One)
    .open()?;
    
    let mut iridium = Self{
      email: email.into(),
      serial_port
    };
    // Reset
    iridium.sw_reset()?;
    
    Ok(iridium)
  }

  fn sw_reset(&mut self) -> Result<()> {
    // let mut buffer = [0u8; 1024];
    self.serial_port.write_all("AT&F0\r".as_bytes())?;
    self.serial_port.write_all("AT&K0\r".as_bytes())?;
    // self.serial_port.write_all("AT+CGMI\r".as_bytes())?;
    // let mut num_bytes_read = self.serial_port.read(&mut buffer)?; 
    Ok(())
  }

  pub fn get_device_details(&mut self) -> Result<DeviceDetails> {
    let mut buffer = [0u8; 1024];

    self.serial_port.write_all("AT+CGMI\r".as_bytes())?;
    println!("Wrote command!");
    self.serial_port.flush()?;
    println!("Flushed!");
    let mut num_bytes_read = self.serial_port.read(&mut buffer)?;
    println!("Read!");
    let manuf_name = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();
    println!("Manufacturer: {}", manuf_name);

    self.serial_port.write_all("AT+CGMM\r".as_bytes())?;
    self.serial_port.flush()?;
    num_bytes_read = self.serial_port.read(&mut buffer)?;
    let model_number = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();

    self.serial_port.write_all("AT+CGMR\r".as_bytes())?;
    self.serial_port.flush()?;
    num_bytes_read = self.serial_port.read(&mut buffer)?;
    let revision = String::from_utf8_lossy(&buffer[..num_bytes_read]).to_string();

    let mut imei_buffer = [0u8; 8];
    self.serial_port.write_all("AT+CGSN\r".as_bytes())?;
    self.serial_port.flush()?;
    self.serial_port.read_exact(&mut imei_buffer)?;
    let imei = u64::from_le_bytes(imei_buffer);

    Ok(DeviceDetails{manuf_name, model_number, revision, imei})
    
  } 
}
