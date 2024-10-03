use jeflog::fail;
use spidev::spidevioctl::SpidevTransfer;
use spidev::Spidev;
use std::sync::Arc;
use std::{thread, time};

use std::collections::HashMap;
use std::rc::Rc;

use crate::gpio::{
  Gpio,
  Pin,
  PinMode::{Input, Output},
  PinValue::{High, Low},
};
use crate::tc::typek_convert;

#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy)]
pub enum Measurement {
  CurrentLoopPt,
  VValve,
  IValve,
  VPower,
  IPower,
  Tc1,
  Tc2,
  DiffSensors,
  Rtd,
}

pub enum ADCEnum {
  ADC(ADC),
  OnboardADC,
}


pub struct ADC {
  pub measurement: Measurement,
  pub spidev: Rc<Spidev>,
  ambient_temp: f64,
  gpio_mappings: Rc<HashMap<Measurement, Pin>>,
  drdy_mappings: Rc<HashMap<Measurement, Pin>>,
}

impl ADC {
  // Constructs a new instance of an Analog-to-Digital Converter
  pub fn new(
    measurement: Measurement,
    spidev: Rc<Spidev>,
    gpio_mappings: Rc<HashMap<Measurement, Pin>>,
    drdy_mappings: Rc<HashMap<Measurement, Pin>>,
  ) -> ADC {
    ADC {
      measurement,
      spidev,
      ambient_temp: 0.0,
      gpio_mappings,
      drdy_mappings,
    }
  }

  pub fn cs_mappings() -> HashMap<Measurement, usize> {
    let mut cs_gpios: HashMap<Measurement, usize> = HashMap::new();
    cs_gpios.insert(Measurement::CurrentLoopPt, 30);
    cs_gpios.insert(Measurement::IValve, 73); // changed
    cs_gpios.insert(Measurement::VValve, 75); // changed
    // cs_gpios.insert(Measurement::VPower, 13);
    // cs_gpios.insert(Measurement::IPower, 15);
    // cs_gpios.insert(Measurement::Tc1, 10);
    // cs_gpios.insert(Measurement::Tc2, 20);
    cs_gpios.insert(Measurement::DiffSensors, 16);
    // cs_gpios.insert(Measurement::Rtd, 11);

    cs_gpios
  }

  // DO NOT USE THIS FUNCTION
  // pub fn init_gpio(&mut self, prev_adc: Option<Measurement>) {
  //   // pull old adc HIGH
  //   if let Some(old_adc) = prev_adc {
  //     if let Some(pin) = self.gpio_mappings.get(&old_adc) {
  //       pin.digital_write(High);
  //     }
  //   }

  //   // pull new adc LOW
  //   if let Some(pin) = self.gpio_mappings.get(&self.measurement) {
  //     pin.digital_write(Low);
  //   }
  // }

  // selects current ADC
  pub fn pull_cs_high_active_low(&mut self) {
    if let Some(pin) = self.gpio_mappings.get(&self.measurement) {
      pin.digital_write(High);
    }
  }

  // deselects current ADC
  pub fn pull_cs_low_active_low(&mut self) {
    if let Some(pin) = self.gpio_mappings.get(&self.measurement) {
      pin.digital_write(Low);
    }
  }

  pub fn poll_data_ready(&mut self) {
    // poll the data ready pin till low (active low)
    let drdy_pin = self.drdy_mappings.get(&self.measurement).unwrap();

    loop {
      let pin_value = drdy_pin.digital_read();
      if pin_value == Low {
        break;
      }
    }
  }

  pub fn init_regs(&mut self) {
    // Read initial registers
    self.read_regs(0, 17);

    // delay for at least 4000*clock period
    // println!("Delaying for 1 second");
    //thread::sleep(time::Duration::from_millis(100));

    // Write to registers
    match self.measurement {
      Measurement::CurrentLoopPt
      | Measurement::VPower
      | Measurement::IPower
      | Measurement::IValve
      | Measurement::VValve => {
        self.write_reg(0x03, 0x00);
        self.write_reg(0x04, 0x1E);
        // self.write_reg(0x08, 0x40);
        // self.write_reg(0x08, 0x00);
        self.write_reg(0x05, 0x0A);
      }

      Measurement::Rtd => {
        self.write_reg(0x03, 0x09);
        self.write_reg(0x04, 0x1E);
        // self.write_reg(0x06, 0x47);
        self.write_reg(0x06, 0x07);
        self.write_reg(0x07, 0x05);
      }

      Measurement::Tc1 | Measurement::Tc2 | Measurement::DiffSensors => {
        self.write_reg(0x03, 0x0D);
        self.write_reg(0x04, 0x1E);
        self.write_reg(0x05, 0x0A);
      }

    }

    // delay for at least 4000*clock period
    // println!("Delaying for 1 second");
    //thread::sleep(time::Duration::from_millis(100));

    // Read registers
    self.read_regs(0, 17);
  }

  pub fn reset_status(&mut self) {
    let tx_buf_reset = [0x06];
    let mut transfer = SpidevTransfer::write(&tx_buf_reset);
    let _status = self.spidev.transfer(&mut transfer);
  }

  pub fn start_conversion(&mut self) {
    let tx_buf_rdata = [0x08];
    let mut rx_buf_rdata = [0x00];
    let mut transfer =
      SpidevTransfer::read_write(&tx_buf_rdata, &mut rx_buf_rdata);
    let _status = self.spidev.transfer(&mut transfer);
    thread::sleep(time::Duration::from_millis(1));
  }

  pub fn self_calibrate(&mut self) {
    let tx_buf_rdata = [0x19];
    let mut rx_buf_rdata = [0x00];
    let mut transfer =
      SpidevTransfer::read_write(&tx_buf_rdata, &mut rx_buf_rdata);
    let _status = self.spidev.transfer(&mut transfer);
    thread::sleep(time::Duration::from_millis(1000));
  }

  pub fn read_regs(&mut self, reg: u8, num_regs: u8) {
    let mut tx_buf_readreg = [0x00; 20];
    let mut rx_buf_readreg = [0x00; 20];
    tx_buf_readreg[0] = 0x20 | reg;
    tx_buf_readreg[1] = num_regs;
    let mut transfer =
      SpidevTransfer::read_write(&tx_buf_readreg, &mut rx_buf_readreg);
    let _status = self.spidev.transfer(&mut transfer);

    println!("{:?} regs: {:?}", self.measurement, rx_buf_readreg);
    if rx_buf_readreg.iter().all(|&byte| byte == 0) {
      fail!("Failed to write and read correct register values");
    }
  }

  pub fn write_reg(&mut self, reg: u8, data: u8) {
    let tx_buf_writereg = [0x40 | reg, 0x00, data];
    let mut rx_buf_writereg = [0x40, 0x00, 0x00];
    let mut transfer =
      SpidevTransfer::read_write(&tx_buf_writereg, &mut rx_buf_writereg);
    let _status = self.spidev.transfer(&mut transfer);
  }

  pub fn get_adc_reading(&mut self, iteration: u64) -> (f64, f64) {
    if self.measurement == Measurement::Rtd
      || self.measurement == Measurement::Tc1
      || self.measurement == Measurement::Tc2
    {
      // can't use data ready for these
      // thread::sleep(time::Duration::from_micros(700));
    } else {
      self.poll_data_ready();
    }
    let val = self.test_read_individual(iteration);

    // let start = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
    // let unix_timestamp = start.as_secs_f64();

    let unix_timestamp = 0.0; // change this!

    (val, unix_timestamp)
  }

  pub fn write_iteration(&mut self, iteration: u64) {
    match self.measurement {
      Measurement::CurrentLoopPt => match iteration % 6 {
        0 => {
          self.write_reg(0x02, 0x0C);
        }
        1 => {
          self.write_reg(0x02, 0x10 | 0x0C);
        }
        2 => {
          self.write_reg(0x02, 0x20 | 0x0C);
        }
        3 => {
          self.write_reg(0x02, 0x30 | 0x0C);
        }
        4 => {
          self.write_reg(0x02, 0x40 | 0x0C);
        }
        5 => {
          self.write_reg(0x02, 0x50 | 0x0C);
        }
        _ => fail!("Failed register write — could not mod iteration"),
      },

      Measurement::IValve | Measurement::VValve => match iteration % 6 {
        0 => {
          self.write_reg(0x02, 0x50 | 0x0C);
        }
        1 => {
          self.write_reg(0x02, 0x40 | 0x0C);
        }
        2 => {
          self.write_reg(0x02, 0x30 | 0x0C);
        }
        3 => {
          self.write_reg(0x02, 0x20 | 0x0C);
        }
        4 => {
          self.write_reg(0x02, 0x10 | 0x0C);
        }
        5 => {
          self.write_reg(0x02, 0x0C);
        }
        _ => fail!("Failed register write — could not mod iteration"),
      },

      Measurement::VPower => match iteration % 5 {
        0 => {
          self.write_reg(0x02, 0x0C);
        }
        1 => {
          self.write_reg(0x02, 0x10 | 0x0C);
        }
        2 => {
          self.write_reg(0x02, 0x20 | 0x0C);
        }
        3 => {
          self.write_reg(0x02, 0x30 | 0x0C);
        }
        4 => {
          self.write_reg(0x02, 0x40 | 0x0C);
        }
        _ => fail!("Failed register write — could not mod iteration"),
      },
      Measurement::IPower => match iteration % 2 {
        0 => {
          self.write_reg(0x02, 0x0C);
        }
        1 => {
          self.write_reg(0x02, 0x10 | 0x0C);
        }
        _ => fail!("Failed register write — could not mod iteration"),
      },
      Measurement::Rtd => match iteration % 2 {
        0 => {
          self.write_reg(0x02, 0x12);
          self.write_reg(0x05, 0x10);
        }
        1 => {
          self.write_reg(0x02, 0x34);
          self.write_reg(0x05, 0x14);
        }
        _ => fail!("Failed register write — could not mod iteration"),
      },

      Measurement::DiffSensors => match iteration % 3 {
        0 => {
          self.write_reg(0x02, 0x54);
        }
        1 => {
          self.write_reg(0x02, 0x32);
        }
        2 => {
          self.write_reg(0x02, 0x10);
        }
        _ => fail!("Failed register write — could not mod iteration"),
      },

      Measurement::Tc1 | Measurement::Tc2 => match iteration % 4 {
        0 => {
          self.write_reg(0x03, 0x08);
          self.write_reg(0x09, 0x40);
        }
        1 => {
          self.write_reg(0x02, 0x50 | 0x04);
        }
        2 => {
          self.write_reg(0x02, 0x30 | 0x02);
        }
        3 => {
          self.write_reg(0x02, 0x10);
        }
        _ => fail!("Failed register write — could not mod iteration"),
      },
      
    }
  }

  pub fn test_read_individual(&mut self, iteration: u64) -> f64 {
    let tx_buf_rdata = [0x12, 0x00, 0x00];
    let mut rx_buf_rdata = [0x00, 0x00, 0x00];
    let mut transfer =
      SpidevTransfer::read_write(&tx_buf_rdata, &mut rx_buf_rdata);
    let _status = self.spidev.transfer(&mut transfer);
    let value: i16 = ((rx_buf_rdata[1] as i16) << 8) | (rx_buf_rdata[2] as i16);

    let mut reading;

    match self.measurement {
      Measurement::CurrentLoopPt | Measurement::IValve => {
        reading = ((value as i32 + 32768) as f64) * (2.5 / ((1 << 15) as f64));
        //println!("valve {:?} I: {:?}", (iteration % 6) + 1, reading);
      }
      Measurement::VPower | Measurement::VValve => {
        reading =
          ((value as i32 + 32768) as f64) * (2.5 / ((1 << 15) as f64)) * 11.0; // 0
                                                                               // ref
                                                                               // println!("{:?}: {:?}", (iteration % 5) + 1, reading);
                                                                               //println!("valve {:?} V: {:?}", (iteration % 6) + 1, reading);
      }
      Measurement::IPower => {
        reading = ((value as i32 + 32768) as f64) * (2.5 / ((1 << 15) as f64)); // 2.5 ref
                                                                                // println!("{:?}: {:?}", (iteration % 2) + 1, reading);
      }
      Measurement::Rtd => {
        reading = (value as f64) * (2.5 / ((1 << 15) as f64)) / 4.0; // 2.5 ref
                                                                     // println!("{:?}: {:?}", (iteration % 2) + 1, reading);
      }
      Measurement::Tc1 | Measurement::Tc2 => {
        if iteration % 4 == 0 {
          // ambient temp
          reading =
            ((value as i32) as f64) * (2.5 / ((1 << 15) as f64)) * 1000.0;
          let ambient = reading * 0.403 - 26.987;
          self.ambient_temp = ambient;
          self.write_reg(0x09, 0x0); // reset sysmon
          self.write_reg(0x03, 0x0D); // reset PGA gain
        } else {
          // convert
          reading = (value as f64) * (2.5 / ((1 << 15) as f64)) / 0.032; // gain of 32
          reading = (typek_convert(self.ambient_temp as f32, reading as f32)
            + 273.15) as f64;
        }
      }
      Measurement::DiffSensors => {
        reading =
          ((value as f64) * (2.5 / ((1 << 15) as f64)) / 0.032) / 1000.0; // gain of 32
                                                                          // println!("{:?}: {:?}", (iteration % 3) + 1, reading);
      }
    }
    reading
  }
}

pub fn open_controllers() -> Vec<Arc<Gpio>> {
  (0..=3).map(Gpio::open).collect()
}

pub fn gpio_controller_mappings( // --> whats on the board
  controllers: &[Arc<Gpio>],
) -> HashMap<Measurement, Pin> {
  // let cl_pin = controllers[0].get_pin(30);
  // cl_pin.mode(Output);

  // let i_valve_pin = controllers[2].get_pin(4);
  // i_valve_pin.mode(Output);

  // let v_valve_pin = controllers[0].get_pin(26);
  // v_valve_pin.mode(Output);

  // let v_power_pin = controllers[2].get_pin(13);
  // v_power_pin.mode(Output);

  // let i_power_pin = controllers[2].get_pin(15);
  // i_power_pin.mode(Output);

  // let tc_1_pin = controllers[0].get_pin(10);
  // tc_1_pin.mode(Output);

  // let tc_2_pin = controllers[0].get_pin(20);
  // tc_2_pin.mode(Output);

  // let diff_pin = controllers[3].get_pin(16);
  // diff_pin.mode(Output);

  // let rtd_pin = controllers[2].get_pin(11);
  // rtd_pin.mode(Output);

    let rtd1_pin = controllers[1].get_pin(28);
    rtd1_pin.mode(Output);

    let rtd2_pin = controllers[2].get_pin(2);
    rtd2_pin.mode(Output);

    let rtd3_pin = controllers[2].get_pin(6);
    rtd3_pin.mode(Output);

    let i_valve_pin = controllers[2].get_pin(9);
    i_valve_pin.mode(Output);

    let v_valve_pin = controllers[2].get_pin(11);
    v_valve_pin.mode(Output);

  HashMap::from([
    //(Measurement::CurrentLoopPt, cl_pin), // dedicated CS pin ?
    (Measurement::IValve, i_valve_pin),
    (Measurement::VValve, v_valve_pin),
    //(Measurement::VPower, v_power_pin),
    //(Measurement::IPower, i_power_pin),
    //(Measurement::Tc1, tc_1_pin),
    //(Measurement::Tc2, tc_2_pin),
    //(Measurement::DiffSensors, diff_pin), // dedicated CS pin ?
    (Measurement::Rtd, rtd1_pin),
    (Measurement::Rtd, rtd2_pin),
    (Measurement::Rtd, rtd3_pin),
  ])
}

pub fn data_ready_mappings(
  controllers: &[Arc<Gpio>],
) -> HashMap<Measurement, Pin> {
  // let cl_pin = controllers[1].get_pin(28);
  // cl_pin.mode(Input);

  // let i_valve_pin = controllers[2].get_pin(3);
  // i_valve_pin.mode(Input);

  // let v_valve_pin = controllers[1].get_pin(12);
  // v_valve_pin.mode(Input);

  // let v_power_pin = controllers[2].get_pin(12);
  // v_power_pin.mode(Input);

  // let i_power_pin = controllers[2].get_pin(14);
  // i_power_pin.mode(Input);

  // let diff_pin = controllers[3].get_pin(15);
  // diff_pin.mode(Input);

  let cl_pin = controllers[0].get_pin(7);
  cl_pin.mode(Input);

  let diff_pin = controllers[2].get_pin(14);
  diff_pin.mode(Input);

  let rtd1_pin = controllers[1].get_pin(18);
  rtd1_pin.mode(Input);

  let rtd2_pin = controllers[2].get_pin(3);
  rtd2_pin.mode(Input);

  let rtd3_pin = controllers[2].get_pin(10);
  rtd3_pin.mode(Input);

  let i_valve_pin = controllers[0].get_pin(14);
  i_valve_pin.mode(Input);

  let v_valve_pin = controllers[2].get_pin(12);
  v_valve_pin.mode(Input);

  HashMap::from([
    (Measurement::CurrentLoopPt, cl_pin),
    (Measurement::DiffSensors, diff_pin),
    (Measurement::Rtd, rtd1_pin),
    (Measurement::Rtd, rtd2_pin),
    (Measurement::Rtd, rtd3_pin),
    (Measurement::IValve, i_valve_pin),
    (Measurement::VValve, v_valve_pin),
    // (Measurement::VPower, v_power_pin),
    // (Measurement::IPower, i_power_pin),
  ])
}

pub fn pull_gpios_high(controllers: &[Arc<Gpio>]) { // --> whats on the board
  let pins = vec![
    // controllers[0].get_pin(30),
    // controllers[2].get_pin(4),
    // controllers[0].get_pin(26),
    // controllers[2].get_pin(13),
    // controllers[2].get_pin(15),
    // controllers[0].get_pin(10),
    // controllers[0].get_pin(20),
    // controllers[3].get_pin(16),
    // controllers[2].get_pin(11),
    // controllers[0].get_pin(5),
    // controllers[0].get_pin(13),
    // controllers[0].get_pin(23),
    // controllers[2].get_pin(23),
    controllers[1].get_pin(28),
    controllers[2].get_pin(2),
    controllers[2].get_pin(6),
    controllers[2].get_pin(9),
    controllers[2].get_pin(11),
  ];

  for pin in pins.iter() {
    pin.mode(Output);
    pin.digital_write(High);
  }
}
