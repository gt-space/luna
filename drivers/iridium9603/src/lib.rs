use std::time::Duration;
use serialport::prelud::*;
use sbd::{SbdDevice, SbdError};

struct iridium9603 {
  device: sbdDevice,
  email: String,
  baud: u32,
}

impl iridium9603 {
  pub fn new(device_path: &str) -> Result<Self, SbdError> {
    // initialize the iridium module - use SBD crate for this
    // establish uart connection with rpi - use UART crate (serialport)
    let port_name = "/dev/ttyUSB0"; // THIS HAS TO BE CHANGED TO THE CORRECT DEVICE FILE

    let serial_settings = SerialPortSettings {
      baud_rate: 19200,
      data_bits: DataBits::Eight,
      parity: Parity::None,
      stop_bits: StopBits::One,
      flow_control: FlowControl::None,
      timeout: Duration::from_secs(2),
    }

    // Open the serial port
    let port = serialport::open_with_settings(port_name, &serial_settings)?;

    // Initialize the SBD modem with the serial port
    let mut modem = IridiumModem::new(port);
  }

  // Function to send gps coordinates to email
  pub fn sendData(&self, String text) -> Result<(), Box<dyn Error>> {
    modem.write_message(text)?;

    modem.initiate_sbd_session()?;

    println!("Message sent successfully!");
    Ok(())
  }
}