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

use std::{io, thread, time::Duration};
use crate::metadata::Command;

pub struct SleepData {
  spidev: Spidev,
  cs_pin: Pin,
  busy_pin: Pin,
  reset_pin: Pin,
  dio_1: Pin,
  dio_2: Pin,
  dio_3: Pin
}

pub struct StandbyData {
  spidev: Spidev,
  cs_pin: Pin,
  busy_pin: Pin,
  reset_pin: Pin,
  dio_1: Pin,
  dio_2: Pin,
  dio_3: Pin
}

pub struct FsData {
  spidev: Spidev,
  cs_pin: Pin,
  busy_pin: Pin,
  reset_pin: Pin,
  dio_1: Pin,
  dio_2: Pin,
  dio_3: Pin
}

pub struct RxData {
  spidev: Spidev,
  cs_pin: Pin,
  busy_pin: Pin,
  reset_pin: Pin,
  dio_1: Pin,
  dio_2: Pin,
  dio_3: Pin
}

pub struct TxData {
  spidev: Spidev,
  cs_pin: Pin,
  busy_pin: Pin,
  reset_pin: Pin,
  dio_1: Pin,
  dio_2: Pin,
  dio_3: Pin
}

pub struct SX1280<State> {
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
      reset_pin.mode(Output);
      reset_pin.digital_write(Low); // active low
      thread::sleep(Duration::from_nanos(100));
      reset_pin.digital_write(High);

      busy_pin.mode(Input);
      loop {
          if busy_pin.digital_read() == Low {
            break;
          }
      }

      dio_1.mode(Input);
      dio_2.mode(Input);
      dio_3.mode(Input);

      let driver = SX1280 { 
        state: StandbyData {
          spidev,
          cs_pin,
          busy_pin,
          reset_pin,
          dio_1,
          dio_2,
          dio_3
        }
      };

      // Configuration commands but leave some to user probably

      driver
    }


    pub fn enable_chip_select(&mut self) {
      self.state.cs_pin.digital_write(Low); // active low
    }

    pub fn disable_chip_select(&mut self) {
      self.state.cs_pin.digital_write(High); // active low
    }

    pub fn set_regulator_mode(&mut self, mode: u8) -> io::Result<()> {
      if (mode != 0 && mode != 1) {
        // fuck you
      }
      
      self.enable_chip_select();
      let tx: [u8; 2] = [Command::SetRegulatorMode as u8, mode];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.state.spidev.transfer(&mut transfer);
      self.disable_chip_select();

      result
    }
}

impl SX1280<SleepData> {
  pub fn set_standby(mut self, regulator_mode: u8) -> SX1280<StandbyData> {
    self.enable_chip_select();
    thread::sleep(Duration::from_micros(2)); // delay for minimum 2 us
    self.disable_chip_select();

    loop {
      if self.state.busy_pin.digital_read() == Low {
        break;
      }
    }

    let mut driver = SX1280 { 
      state: StandbyData { 
        spidev: self.state.spidev,
        cs_pin: self.state.cs_pin,
        busy_pin: self.state.busy_pin,
        reset_pin: self.state.reset_pin,
        dio_1: self.state.dio_1,
        dio_2: self.state.dio_2,
        dio_3: self.state.dio_3
      }
    };

    driver.set_regulator_mode(regulator_mode).unwrap();

    driver
  }

  pub fn enable_chip_select(&mut self) {
    self.state.cs_pin.digital_write(Low); // active low
  }

  pub fn disable_chip_select(&mut self) {
    self.state.cs_pin.digital_write(High); // active low
  }
}