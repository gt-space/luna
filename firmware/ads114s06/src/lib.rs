use std::io;
use spidev::spidevioctl::{spi_ioc_transfer, SpidevTransfer};
use spidev::Spidev;

// Register locations
const ID_LOCATION : usize = 0;
const STATUS_LOCATION : usize = 1;
const INPMUX_LOCATION : usize = 2;
const PGA_LOCATION : usize = 3;
const DATARATE_LOCATION : usize = 4;
const REF_LOCATION : usize = 5;
const IDACMUG_LOCATION : usize = 6;
const IDACMUX_LOCATION : usize = 7;
const VBIAS_LOCATION : usize = 8;
const SYS_LOCATION : usize = 9;
const RESERVED0_LOCATION : usize = 10;
const OFCAL0_LOCATION : usize = 11;
const OFCAL1_LOCATION : usize = 12;
const RESERVED1_LOCATION : usize = 13;
const FSCAL0_LOCATION : usize = 14;
const FSCAL1_LOCATION : usize = 15;
const GPIODAT_LOCATION : usize = 16;
const GPIOCON_LOCATION : usize = 17;

const ACCEPTABLE_IDAC_MAGNITUDES: [u16; 10] = [0, 10, 50, 100, 250, 500, 750, 1000, 1500, 2000];
const ACCEPTABLE_PGA_GAINS: [u8; 8] = [1, 2, 4, 8, 16, 32, 64, 128];

pub struct ADC {
  pub spidev: Spidev,
  current_reg_vals: [u8; 18],
}

impl ADC {
  pub fn new(&mut self, spidev: Spidev) -> ADC {
      ADC {
        spidev: spidev,
        current_reg_vals: {
          match self.read_all_regs() {
            Ok(regs) => regs,
            Err(e) => {
              println!("Error in reading all initial register values");
              [0; 18] //default array for current register values
            }
          }
        }
      }
  }

    /* FOR THE FOLLOWING SPI COMMUNICATION COMMANDS BELOW
    For a read_write transfer, tx_buf is used to send the command and rx_buf
    is used to receive the data. For read_write, tx_buf and rx_buf must be
    of equal size and the kernel automatically modified rx_buf, which is why
    a mutable reference is passed to it. For the write_reg function it must be
    explored as to if providing an rx_buf will do anything.
     */

  pub fn read_data(&mut self) -> Result<i16, io::Error> {
    /*
    old SAM code received data in 3 byte buffer even though CRC and STATUS
    bytes were disabled which leaves for 2 bytes of data. The tx_buf just
    needs to store one byte so going to investigate why this was done and if
    not needed will reduce tx_buf and rx_buf sizes to 2 bytes
     */
    let tx_buf: [u8; 3] = [0x12, 0x00, 0x00];
    let mut rx_buf: [u8; 3] = [0x00, 0x00, 0x00];
    let mut transfer: spi_ioc_transfer<'_, '_> = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    match result {
      Ok(_) => Ok(((rx_buf[1] as i16) << 8) | (rx_buf[2] as i16)), // confirm these array indices are correct
      Err(e) => {
        Err(e)
      }
    }
  }

  pub fn read_reg(&mut self, reg: u8) -> Result<u8, io::Error> {
    let tx_buf: [u8; 2] = [0x20 | reg, 0x00];
    let mut rx_buf: [u8; 2] = [0x00, 0x00];
    let mut transfer: spi_ioc_transfer<'_, '_> = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    match result {
      Ok(_) => Ok(rx_buf[0]), // test if value goes to index 0 or 1
      Err(e) => {
        Err(e)
      }
    }
  }

  pub fn read_all_regs(&mut self) -> Result<[u8; 18], io::Error> {
    let mut tx_buf: [u8; 18] = [0; 18];
    let mut rx_buf: [u8; 18] = [0; 18];
    tx_buf[0] = 0x20;
    tx_buf[1] = 17;
    let mut transfer: spi_ioc_transfer<'_, '_> = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
    let result = self.spidev.transfer(&mut transfer);
    match result {
      Ok(_) => Ok(rx_buf),
      Err(e) => {
        Err(e)
      }
    }
  }

  pub fn write_reg(&mut self, reg: u8, data: u8) -> Result<(), io::Error> {
    // TODO: if an rx buffer is sent, look into what data it holds if modified
    let tx_buf: [u8; 3] = [0x40 | reg, 0x00, data];
    let mut transfer: spi_ioc_transfer<'_, '_> = SpidevTransfer::write(&tx_buf);
    self.spidev.transfer(&mut transfer) // no need for extra error handling as nothing is returned in good case
  }
}