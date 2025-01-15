use std::time::Duration;
use serialport::*;
use sbd::mo::Message;

pub struct Iridium9603 {
  email: String
}

impl Iridium9603 {
  /// Initialize the Iridium9603 device with the given serial port path and email address 
  pub fn new(device_path: &str, email: &str) -> Result<Self> { // Device path will probably be "/dev/serial0"
    // Serial port setup
    let serial_port = serialport::new(device_path, 19200)
    .data_bits(DataBits::Eight)
    .parity(Parity::None)
    .stop_bits(StopBits::One).open().map_err(|e| format!("Failed to open serial port: {}", e))?;
    Ok(Self{
      email: email.into()
    })
  }

  /// Send a text message via the Iridium modem
  pub fn send_message(&mut self, message: &str) -> Result<()> {
      println!("Sending message: {}", message);

      // Write the message to the modem
      self.device.send_text_message(message)?;

      // Initiate the SBD session
      match self.device.session_mailbox_check() {
          Ok(response) => {
              println!("SBD session successful: {:?}", response);
              Ok(())
          }
          Err(e) => {
              eprintln!("Failed to complete SBD session: {:?}", e);
              Err(Box::new(e))
          }
      }
  }
}
