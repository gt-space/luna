use imu::{AdisIMUDriver, DeltaReadData, GenericData, GyroReadData};
use common::comm::gpio::{Gpio, Pin, PinMode, PinMode::*, PinValue::*};
use spidev::Spidev;
use std::sync::Mutex;

use std::time::Duration;
use std::time::SystemTime;
use std::thread::sleep;
use std::env;

const IMU_CS_PIN_LOC : [usize; 2] = [0, 11];
const BAR_CS_PIN_LOC : [usize; 2] = [2, 24];
const MAG_CS_PIN_LOC : [usize; 2] = [1, 14];
const IMU_DR_PIN_LOC : [usize; 2] = [2, 17];
const IMU_NRESET_PIN_LOC : [usize; 2] = [2, 25];


fn main() {
  env::set_var("RUST_BACKTRACE", "1");
  println!("Getting GPIO and pins");
  // Get GPIO handlers
  static mut CONTROLLERS : [Gpio; 4] = [
    Gpio::open_controller(0),
    Gpio::open_controller(1),
    Gpio::open_controller(2),
    Gpio::open_controller(3),
  ];
  

  let mut imu_cs = unsafe { CONTROLLERS[IMU_CS_PIN_LOC[0]].get_pin(IMU_CS_PIN_LOC[1]) };
  imu_cs.mode(Output);
  let mut bar_cs = unsafe { CONTROLLERS[BAR_CS_PIN_LOC[0]].get_pin(BAR_CS_PIN_LOC[1]) };
  bar_cs.mode(Output);
  let mut mag_cs = unsafe { CONTROLLERS[MAG_CS_PIN_LOC[0]].get_pin(MAG_CS_PIN_LOC[1]) };
  mag_cs.mode(Output);
  let mut imu_dr = unsafe { CONTROLLERS[IMU_DR_PIN_LOC[0]].get_pin(IMU_DR_PIN_LOC[1]) };
  imu_dr.mode(Input);
  let mut imu_nreset = unsafe { CONTROLLERS[IMU_NRESET_PIN_LOC[0]].get_pin(IMU_NRESET_PIN_LOC[1]) };
  imu_nreset.mode(Output);
  
  // Ensure all CS are off
  // TODO : check if all boards are CS active low
  println!("writing all chip selects to be off");
  imu_cs.digital_write(High); // IMU, active low
  bar_cs.digital_write(High); // BAR, active low
  mag_cs.digital_write(High); // MAG, active low

  // Get spi
  let spi = Spidev::open("/dev/spidev0.0")
    .expect("Spi initialization should work");

  // Initialize the actual spi handler
  if let Ok(mut driver) = 
    AdisIMUDriver::initialize(spi, imu_dr, imu_nreset, imu_cs) 
  {
    println!("Initialization Success");
    
    println!("Setting Decimation Rate");
    driver
      .write_dec_rate(8)
      .expect("Setting decimation rate failed");

    if driver.validate() {
      println!("Prod ID Validated");
    } else {
      println!("Validation failed");
      return;
    }

    let mut history : Vec<_> = Vec::new();

    for _ in 0..100 {
      let result = driver.burst_read_gyro_16();
      if let Ok(x) = result {
        history.push(x);
      } else {
        println!("ERROR : {}", result.unwrap_err());
      }
      sleep(Duration::from_micros(100));
    }

    for (general, read) in history {
      println!("------\n{} | {}", general.data_counter, read);
    }
    
    return;
  } else {
    println!("Initialization Failure!");
  }
  println!("End of test");
}