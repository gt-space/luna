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

pub struct PinInfo {
  spidev: Spidev,
  cs_pin: Pin,
  busy_pin: Pin,
  reset_pin: Pin,
  dio_1: Pin,
  dio_2: Pin,
  dio_3: Pin,
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



    pub fn save_context(&mut self) -> io::Result<()> {
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 1] = [Command::SetSaveContext as u8];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();

      result
    }

    pub fn set_regulator_mode(&mut self, mode: u8) -> io::Result<()> {
      if (mode != 0 && mode != 1) {
        panic!("Invalid regulator mode provided, only 0 or 1 allowed!")
      }

      self.wait();
      self.enable_chip_select();

      let tx: [u8; 2] = [Command::SetRegulatorMode as u8, mode];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();

      result
    }

    pub fn set_sleep(mut self, data_retention_mode: u8) -> SX1280<SleepData> {
      if (data_retention_mode != 0 && data_retention_mode != 1) {
        panic!("Invalid data retention mode provided, should be 0 or 1!")
      }
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 2] = [Command::SetSleep as u8, data_retention_mode];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();

      SX1280 {
        pin_info: self.pin_info,
        auto_fs: self.auto_fs,
        state: SleepData {  }
      }
    }

    pub fn set_fs(mut self) -> SX1280<FsData> {
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 1] = [Command::SetFs as u8];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();

      // min 54 us delay depending on regulator mode (i dont need this cuz of wait function?)
      //thread::sleep(Duration::from_micros(60));

      SX1280 {
        pin_info: self.pin_info,
        auto_fs: self.auto_fs,
        state: FsData {  }
      }
    }

    pub fn enable_auto_fs(&mut self) {
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 2] = [Command::SetAutoFS as u8, 1];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();
      self.auto_fs = true;
    }

    pub fn disable_auto_fs(&mut self) {
      self.wait();
      self.enable_chip_select();

      let tx: [u8; 2] = [Command::SetAutoFS as u8, 0];
      let mut transfer = SpidevTransfer::write(&tx);
      let result = self.pin_info.spidev.transfer(&mut transfer);

      self.disable_chip_select();
      self.auto_fs = false;
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
        auto_fs: self.auto_fs,
        state: RxDutyCycleData {  }
      }
    }
}

impl SX1280<SleepData> {
  pub fn set_standby(mut self, regulator_mode: u8) -> SX1280<StandbyData> {
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
      auto_fs: self.auto_fs,
      state: StandbyData {  }
    };

    driver.set_regulator_mode(regulator_mode).unwrap();

    driver
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
  // function to terminate by sending SetStandby

  // function to check if packet was received and if so return SW to Standby state to match HW
}