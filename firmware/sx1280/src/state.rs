use common::comm::gpio::{Pin, 
  PinValue::{self, Low, High}, 
  PinMode::{Input, Output}
};

use spidev::{
  spidevioctl::SpidevTransfer,
  SpiModeFlags,
  Spidev,
  SpidevOptions,
};

use std::{default, io, thread, time::Duration};
use crate::metadata::{Command, Irq, Dio, CircuitMode, CommandStatus};

static NOP: u8 = 0x00;

pub struct SX1280 {
  state: StateEnum,
  pin_info: PinInfo,
  irq_info: IRQInfo,
  auto_fs: bool
}

pub enum SX1280Error {
  InvalidPowerRegulatorMode,
  InvalidDataRetentionMode,
  InvalidNumOfDio,
  InvalidCircuitMode
  Spi(io::Error)
}

impl From<io::Error> for SX1280Error {
  fn from(err: io::Error) -> SX1280Error {
    SX1280Error::Spi(err)
  }
}

pub struct PinInfo {
  spidev: Spidev,
  cs_pin: Pin,
  busy_pin: Pin,
  reset_pin: Pin,
  dio_1: Pin,
  dio_2: Pin,
  dio_3: Pin,
}

pub struct IRQInfo {
  irq_mask: u16,
  dio_masks: [u16; 3],
}

impl Default for IRQInfo {
  fn default() -> Self {
    IRQInfo {
      irq_mask: 0,
      dio_masks: [0; 3]
    }
  }
}

pub struct SleepData {

}

pub struct StandbyData {

}

pub struct FsData {

}

pub struct RxDutyCycleData {

}

pub struct TxData {

}

pub struct SX1280<State> {
  pin_info: PinInfo,
  irq_info: IRQInfo,
  auto_fs: bool,
  state: State
}

impl SX1280<StandbyData> {
  pub fn new(bus: &str, 
    mut cs_pin: Pin, 
    mut busy_pin: Pin, 
    mut reset_pin: Pin, 
    mut dio_1: Pin, 
    mut dio_2: Pin, 
    mut dio_3: Pin) -> SX1280<StandbyData> {
      cs_pin.mode(Output);
      cs_pin.digital_write(High); // active low
      // set up SPI bus
      let mut spidev = Spidev::open(bus).unwrap();
      println!("I opened the spidev file");
      let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(10_000_000)
        .lsb_first(false)
        .mode(SpiModeFlags::SPI_MODE_0)
        .build();

      spidev.configure(&options);

      // From Reset -> Startup -> Standby_RC
      // toggle reset pin
      reset_pin.mode(Output);
      reset_pin.digital_write(Low); // active low
      thread::sleep(Duration::from_nanos(100));
      reset_pin.digital_write(High);

      cs_pin.mode(Output);
      busy_pin.mode(Input);
      // Interrupt sources from SX1280
      dio_1.mode(Input);
      dio_2.mode(Input);
      dio_3.mode(Input);

      let pin_info: PinInfo = PinInfo {
        spidev,
        cs_pin,
        busy_pin,
        reset_pin,
        dio_1,
        dio_2,
        dio_3 
      };

      let driver: SX1280<StandbyData> = SX1280 {
        pin_info,
        irq_info: IRQInfo::default(),
        auto_fs: false,
        state: StandbyData {  }
      };

      driver.wait(); // wait for busy pin to go low

      // Configuration commands but leave most to user probably

      driver
    }


    pub fn enable_chip_select(&mut self) {
      self.pin_info.cs_pin.digital_write(Low); // active low
    }

    pub fn disable_chip_select(&mut self) {
      self.pin_info.cs_pin.digital_write(High); // active low
    }

    pub fn wait(&self) {
      loop {
        if self.pin_info.busy_pin.digital_read() == Low {
          break;
        }
      }
    }

    pub fn enable_irq(&mut self, irq: Irq, dio_nums: Vec<Dio>) -> Result<(), SX1280Error>{
      if dio_nums.len() > 3 {
        eprintln!("Maximum of 3 Dio pins can be provided for an IRQ");
        return Err(SX1280Error::InvalidNumOfDio)
      } else if dio_nums.len() == 0 {
        eprintln!("Need to provide at least 1 Dio pin for an IRQ");
        return Err(SX1280Error::InvalidNumOfDio)
      }

      self.enable_chip_select();

      self.irq_info.irq_mask |= 1 << (irq as u8);
      // user might want to route irq to multiple Dio pins
      for dio_num in dio_nums.iter() {
        self.irq_info.dio_masks[*dio_num as usize] |= 1 << (irq as u8);
      }

      let tx: [u8; 9] = [
        Command::SetDioIrqParams as u8,
        (self.irq_info.irq_mask >> 8) as u8,
        (self.irq_info.irq_mask & 0x00FF) as u8,
        (self.irq_info.dio_masks[0] >> 8) as u8,
        (self.irq_info.dio_masks[0] & 0x00FF) as u8,
        (self.irq_info.dio_masks[1] >> 8) as u8,
        (self.irq_info.dio_masks[1] & 0x00FF) as u8,
        (self.irq_info.dio_masks[2] >> 8) as u8,
        (self.irq_info.dio_masks[2] & 0x00FF) as u8
      ];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();
      result.map_err(SX1280Error::from)
    }

    pub fn disable_irq(&mut self, irq: Irq, dio_nums: Vec<Dio>) -> Result<(), SX1280Error> {
      /* 
      User may have routed irq to multiple dio pins but may only want
      to disable this irq for some subset of those dio pins
       */
      if dio_nums.len() > 3 {
        eprintln!("Maximum of 3 Dio pins can be provided for an IRQ");
        return Err(SX1280Error::InvalidNumOfDio)
      } else if dio_nums.len() == 0 {
        eprintln!("Need to provide at least 1 Dio pin for an IRQ");
        return Err(SX1280Error::InvalidNumOfDio)
      }

      self.enable_chip_select();

      self.irq_info.irq_mask &= !(1 << (irq as u8));
      for dio_num in dio_nums.iter() {
        self.irq_info.dio_masks[*dio_num as usize] &= !(1 << (irq as u8));
      }

      let tx: [u8; 9] = [
        Command::SetDioIrqParams as u8,
        (self.irq_info.irq_mask >> 8) as u8,
        (self.irq_info.irq_mask & 0x00FF) as u8,
        (self.irq_info.dio_masks[0] >> 8) as u8,
        (self.irq_info.dio_masks[0] & 0x00FF) as u8,
        (self.irq_info.dio_masks[1] >> 8) as u8,
        (self.irq_info.dio_masks[1] & 0x00FF) as u8,
        (self.irq_info.dio_masks[2] >> 8) as u8,
        (self.irq_info.dio_masks[2] & 0x00FF) as u8
      ];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();
      result.map_err(SX1280Error::from)
    }

    pub fn get_irq_status(&mut self, irq: Irq) -> Result<bool, SX1280Error> {
      self.enable_chip_select();

      let tx: [u8; 4] = [0x15, NOP, NOP, NOP];
      let mut rx: [u8; 4] = [0; 4];
      let mut transfer = SpidevTransfer::read_write(&tx, &mut rx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();

      match result {
        Ok(()) => {
          // check status info in bytes 0 and 1

          // parsing bytes for specific IRQ
          let irq_status = ((rx[2] as u16) << 8) | (rx[3] as u16);
          Ok((irq_status & (1 << (irq as u8))) != 0)
        },

        Err(e) => Err(SX1280Error::from(e))
      }
    }

    pub fn clear_irq_status(&mut self, irq: Irq) -> Result<(), SX1280Error> {
      self.enable_chip_select();

      let irq_mask: u16 = 0 | (1 << (irq as u8));
      let tx: [u8; 3] = [
        Command::ClearIrqStatus as u8,
        (irq_mask >> 8) as u8,
        (irq_mask & 0x00FF) as u8
      ];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();
      result.map_err(SX1280Error::from)
    }

    pub fn save_context(&mut self) -> Result<(), SX1280Error> {
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 1] = [Command::SetSaveContext as u8];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();
      result.map_err(SX1280Error::from)
    }

    pub fn set_regulator_mode(&mut self, mode: u8) -> Result<(), SX1280Error> {
      if (mode != 0 && mode != 1) {
        return Err(SX1280Error::InvalidPowerRegulatorMode)
      }

      self.wait();
      self.enable_chip_select();

      let tx: [u8; 2] = [Command::SetRegulatorMode as u8, mode];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();
      result.map_err(SX1280Error::from)
    }

    pub fn set_sleep(mut self, data_retention_mode: u8) -> Result<SX1280<SleepData>, SX1280Error> {
      if (data_retention_mode != 0 && data_retention_mode != 1) {
        return Err(SX1280Error::InvalidDataRetentionMode)
      }
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 2] = [Command::SetSleep as u8, data_retention_mode];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();

      match result {
        Ok(_) => {
          Ok(SX1280 {
            pin_info: self.pin_info,
            irq_info: self.irq_info,
            auto_fs: self.auto_fs,
            state: SleepData {  }
          })
        },

        Err(e) => {
          Err(SX1280Error::from(e))
        }
      }
    }

    pub fn set_fs(mut self) -> Result<SX1280<FsData>, SX1280Error> {
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 1] = [Command::SetFs as u8];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();

      // min 54 us delay depending on regulator mode (i dont need this cuz of wait function?)
      //thread::sleep(Duration::from_micros(60));

      match result {
        Ok(()) => {
          Ok(SX1280 {
            pin_info: self.pin_info,
            irq_info: self.irq_info,
            auto_fs: self.auto_fs,
            state: FsData {  }
          })
        },

        Err(e) => Err(SX1280Error::from(e))
      }
    }

    pub fn enable_auto_fs(&mut self) -> Result<(), SX1280Error> {
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 2] = [Command::SetAutoFS as u8, 1];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();

      match result {
        Ok(()) => {
          self.auto_fs = true;
          Ok(())
        },

        Err(e) => Err(SX1280Error::from(e))
      }
    }

    pub fn disable_auto_fs(&mut self) -> Result<(), SX1280Error> {
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 2] = [Command::SetAutoFS as u8, 0];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();

      match result {
        Ok(()) => {
          self.auto_fs = false;
          Ok(())
        },

        Err(e) => Err(SX1280Error::from(e))
      }
    }

    pub fn set_rx_duty_cycle(mut self, period_base: u8, rx_count: u16, sleep_count: u16) -> SX1280<RxDutyCycleData> {
      self.wait();
      self.enable_chip_select();

      // do any extra math on parameters if needed

      let tx: [u8; 6] = [Command::SetRxDutyCycle as u8,
        period_base,
        (rx_count >> 8) as u8,
        (rx_count & 0x00FF) as u8,
        (sleep_count >> 8) as u8,
        (sleep_count & 0x00FF) as u8
      ];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();
      /* Now the SX1280 is infinitely checking for a packet. It goes into Rx
      mode to check for a packet and if found goes back to Standby. If not found
      it goes to Sleep for some time and then goes back to Rx. The only other
      way to exit if SetStandby is commanded while in Rx mode. This driver
      software cannot go into an infinite loop to check for data because that
      is bad, but in reality that is what is happening in the hardware. There
      will be a function to check if it is in Rx mode and command SetStandby
      to terminate. There will be another function to check if data was
      received by checking the associated interrupt.
       */

       SX1280 {
        pin_info: self.pin_info,
        irq_info: self.irq_info,
        auto_fs: self.auto_fs,
        state: RxDutyCycleData {  }
      }
    }
}

impl SX1280<SleepData> {
  pub fn set_standby(mut self, regulator_mode: u8) -> Result<SX1280<StandbyData>, SX1280Error> {
    // Wake it up
    // Pulse chip select low for min 2 us and wait for busy pin to go low
    self.enable_chip_select();
    thread::sleep(Duration::from_micros(3)); // delay for minimum 2 us
    self.disable_chip_select();

    loop {
      if self.pin_info.busy_pin.digital_read() == Low {
        break;
      }
    }

    // I am awake :)

    let mut driver: SX1280<StandbyData> = SX1280 {
      pin_info: self.pin_info,
      irq_info: self.irq_info,
      auto_fs: self.auto_fs,
      state: StandbyData {  }
    };

    driver.set_regulator_mode(regulator_mode)?;

    Ok(driver)
  }

  pub fn wait(&self) {
    loop {
      if self.pin_info.busy_pin.digital_read() == Low {
        break;
      }
    }
  }

  pub fn enable_chip_select(&mut self) {
    self.pin_info.cs_pin.digital_write(Low); // active low
  }

  pub fn disable_chip_select(&mut self) {
    self.pin_info.cs_pin.digital_write(High); // active low
  }
}

impl SX1280<RxDutyCycleData> {
  pub fn get_status(&mut self) -> Result<(CircuitMode, CommandStatus), SX1280Error> {
    self.enable_chip_select();

    let tx: [u8; 1] = [Command::GetStatus as u8];
    let mut rx: [u8; 1] = [0x00];
    let mut transfer = SpidevTransfer::read_write(&tx, &mut rx);
    let result = self.pin_info.spidev.transfer(&mut transfer);

    self.disable_chip_select();
    match result {
      Ok(()) => {
        let mode = (rx[0] >> 5) & 0b00000111; // get bits 7-5 (the & is unnecessary I think)
        let status = (rx[0] >> 2) & 0b00000111; // get bits 4-2

        let circuit_mode = if mode == 2 {
          CircuitMode::STDBY_RC
        } else if mode == 3 {
          CircuitMode::STDBY_XOSC
        } else if mode == 4 {
          CircuitMode::Fs
        } else if mode == 5 {
          CircuitMode::Rx
        } else {
          CircuitMode::Tx
        };

        let command_status = if status == 1 {
          CommandStatus::SuccessfullyProcessedCommand
        } else if status == 2 {
          CommandStatus::DataAvailableToHost
        } else if status == 3 {
          CommandStatus::CommandTimeOut
        } else if status == 4 {
          CommandStatus::CommandProcessingError
        } else if status == 5 {
          CommandStatus::FailedToExecuteCommand
        } else {
          CommandStatus::CommandTxDone
        };

        Ok((circuit_mode, command_status))
      },

      Err(e) => Err(SX1280Error::from(e))
    }
  }
  // function to terminate by sending SetStandby
  pub fn end_rx_duty_cycle_fs(mut self) -> Result<SX1280<FsData>, SX1280Error> {
    self.wait(); // busy pin is high during sleep and low during other stable states (check this! page 63)

    // do I need to implement another delay?

    let (circuit_mode, _) = self.get_status()?;
    if circuit_mode == CircuitMode::Rx {
      let driver: SX1280<FsData> = SX1280 {
        pin_info: self.pin_info,
        irq_info: self.irq_info,
        auto_fs: self.auto_fs, 
        state: FsData {  }
      };

      driver
    } else {
      // somehow we are in the wrong circuit mode :(
      return Err(SX1280Error::InvalidCircuitMode)
    }
  }

  pub fn end_rx_duty_cycle_standby(mut self) -> Result<SX1280<FsData>, SX1280Error> {
    self.wait(); // busy pin is high during sleep and low during other stable states (check this! page 63)

    // do I need to implement another delay?

    let (circuit_mode, _) = self.get_status()?;
    if circuit_mode == CircuitMode::Rx {
      let driver: SX1280<StandbyData> = SX1280 {
        pin_info: self.pin_info,
        irq_info: self.irq_info,
        auto_fs: self.auto_fs, 
        state: StandbyData {  }
      };

      driver
    } else {
      // somehow we are in the wrong circuit mode :(
      return Err(SX1280Error::InvalidCircuitMode)
    }
  }

  // function to check if packet was received and if so return SW to Standby state to match HW
  pub fn check_packet_received_fs(mut self) -> Result<SX1280<FsData>, SX1280Error> {

  }



  pub fn enable_chip_select(&mut self) {
    self.pin_info.cs_pin.digital_write(Low); // active low
  }

  pub fn disable_chip_select(&mut self) {
    self.pin_info.cs_pin.digital_write(High); // active low
  }

  pub fn wait(&self) {
    loop {
      if self.pin_info.busy_pin.digital_read() == Low {
        break;
      }
    }
  }
}