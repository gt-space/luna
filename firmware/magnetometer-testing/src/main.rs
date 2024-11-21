use magnetometer::{LIS3MDLDriver, MagnetometerData};
use common::comm::gpio::{Gpio, Pin, PinMode, PinMode::*, PinValue::*};
use spidev::Spidev;

use std::time::Duration;
use std::time::SystemTime;
use std::thread::sleep;
use std::env;

//why how what is this fr
const IMU_CS_PIN_LOC : [usize; 2] = [0, 11];
const BAR_CS_PIN_LOC : [usize; 2] = [2, 24];
const MAG_CS_PIN_LOC : [usize; 2] = [1, 14];
const MAG_DR_PIN_LOC : [usize; 2] = [2, 1]; 
const MAG_INT_PIN_LOC : [usize; 2] = [1, 29];

fn main() {
  env::set_var("RUST_BACKTRACE", "1");
  println!("Getting GPIO and pins");
  // Get GPIO handlers
  let mut controllers : Vec<Gpio>= (0..=3).map(Gpio::open_controller).collect();

  let mut imu_cs = controllers[IMU_CS_PIN_LOC[0]].get_pin(IMU_CS_PIN_LOC[1]);
  imu_cs.mode(Output);
  let mut bar_cs = controllers[BAR_CS_PIN_LOC[0]].get_pin(BAR_CS_PIN_LOC[1]);
  bar_cs.mode(Output);
  let mut mag_cs = controllers[MAG_CS_PIN_LOC[0]].get_pin(MAG_CS_PIN_LOC[1]);
  mag_cs.mode(Output);
  let mut mag_dr = controllers[MAG_DR_PIN_LOC[0]].get_pin(MAG_DR_PIN_LOC[1]); // not done 
  mag_dr.mode(Input);
  let mut mag_int = controllers[MAG_INT_PIN_LOC[0]].get_pin(MAG_INT_PIN_LOC[1]); // not done 
  mag_int.mode(Input);
  // let mut imu_dr = controllers[IMU_DR_PIN_LOC[0]].get_pin(IMU_DR_PIN_LOC[1]);
  // imu_dr.mode(Input);
  // let mut imu_nreset = controllers[IMU_NRESET_PIN_LOC[0]].get_pin(IMU_NRESET_PIN_LOC[1]);
  // imu_nreset.mode(Output);

  
  // Ensure all CS are off
  // TODO : check if all boards are CS active low
  println!("writing all chip selects to be off");
  imu_cs.digital_write(High); // IMU, active low
  bar_cs.digital_write(High); // BAR, active low
  mag_cs.digital_write(High); // MAG, active low

  // Get spi
  let spi = Spidev::open("/dev/spidev0.0")
    .expect("Spi initialization should work");
  println!("spi initialized");

  // Initialize the actual spi handler
  if let Ok(mut driver) = LIS3MDLDriver::initialize(spi, mag_dr, mag_cs, mag_int) {
    println!("Initialization Success");

    // if driver.validate() {
    //   println!("Prod ID Validated");
    // } else {
    //   println!("Validation failed");
    //   return;
    // }

    // verify who am i register 
    let whoami = driver.read_8_bit(magnetometer::Registers::WHO_AM_I_MAG)?;
    println!("WHO AM I: {}", whoami);
    // should be 0x3D

    let mut history : Vec<_> = Vec::new();

    for _ in 0..100 {
      let result = driver.read_magnetic_field();
      if let Ok(x) = result {
        history.push(x);
      } else {
        println!("ERROR : {}", result.unwrap_err());
      }
      sleep(Duration::from_micros(100));
    }

    for data in history {
      println!("------\n{}", data);
    }
    
    return;
  } else {
    println!("Initialization Failure!");
  }
  println!("End of test");
}