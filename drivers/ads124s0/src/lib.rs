use spidev::spidevioctl::SpidevTransfer;
use spidev::Spidev;
use std::{thread, time};
use std::rc::Rc;

pub struct ADC {
  pub spidev: Spidev,
}

impl ADC {
  pub fn new(spidev: Spidev) -> ADC {
      ADC {
          spidev: spidev,
      }
  }


  pub fn init_regs(&mut self) {
      self.read_regs(0, 17);
      //todo: uncomment the above
  }

  // reset status function - pass a reference to the adc as a parameter (we can call adc.method() without actually passing a parameter)
  // completely resets adc
  pub fn reset_status(&mut self) {
      let tx_buf_reset = [0x06]; // define a value for resetting
      let mut transfer = SpidevTransfer::write(&tx_buf_reset); // write to the memory created for the reset
      let _status = self.spidev.transfer(&mut transfer); // set the status of the adc
  }
  // turns on the adc??
  // todo: see waht it actually does
  pub fn start_conversion(&mut self) {
      let mut tx_buf_rdata = [0x08]; // turns on?
      let mut rx_buf_rdata = [0x00];
      let mut transfer = SpidevTransfer::read_write(&mut tx_buf_rdata, &mut rx_buf_rdata);
      let _status = self.spidev.transfer(&mut transfer);
      thread::sleep(time::Duration::from_millis(1000)); // delay between instantiating adc and writing to adc
  }
  // TODO: return/pass data back to calling function
  pub fn read_regs(&self, reg: u8, num_regs: u8)-> [u8; 18] {
      let mut tx_buf_readreg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
      let mut rx_buf_readreg = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
      tx_buf_readreg[0] = 0x20 | reg;
      tx_buf_readreg[1] = num_regs;
      let mut transfer = SpidevTransfer::read_write(&mut tx_buf_readreg, &mut rx_buf_readreg);
      let _status = self.spidev.transfer(&mut transfer);
      println!("{:?}", rx_buf_readreg);

      if rx_buf_readreg == [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] {
          panic!("Failed to read correct register values"); // was fail before, doesn't compile with fail
      }
      rx_buf_readreg; // return rx_buf_readreg to the main file calling this function
  }
  // writes one byte at a time to a given register
  pub fn write_regs(&mut self, reg: u8, data: u8) {
      let mut tx_buf_writereg = [0x40, 0x00, 0x00];
      let mut rx_buf_writereg = [0x40, 0x00, 0x00];
      tx_buf_writereg[0] = 0x40 | reg;
      tx_buf_writereg[2] = data;
      let mut transfer = SpidevTransfer::read_write(&mut tx_buf_writereg, &mut rx_buf_writereg);
      let _status = self.spidev.transfer(&mut transfer);
      // println!("{:?}", rx_buf_writereg);
  }

  // Review channel stuff for ADC and implement a function for reading from a channel
}
