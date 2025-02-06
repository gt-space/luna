use lis3mdl::{LIS3MDL, Result};
use common::comm::gpio::{Gpio, PinMode, PinValue};
use once_cell::sync::Lazy;

const IMU_CS_PIN_LOC : [usize; 2] = [0, 11];
const BAR_CS_PIN_LOC : [usize; 2] = [2, 24];
const MAG_CS_PIN_LOC : [usize; 2] = [1, 14];
const MAG_DR_PIN_LOC : [usize; 2] = [2, 1];
// const MAG_INT_PIN_LOC : [usize; 2] = [1, 29];

pub static GPIO_CONTROLLERS: Lazy<Vec<Gpio>> = Lazy::new(|| open_controllers());

pub fn open_controllers() -> Vec<Gpio> {
  (0..=3).map(Gpio::open_controller).collect()
}

fn main() -> Result<()> {
  println!("Getting GPIO and pins");
  // Get GPIO handlers
  let mut imu_cs =
    GPIO_CONTROLLERS[IMU_CS_PIN_LOC[0]].get_pin(IMU_CS_PIN_LOC[1]);
  imu_cs.mode(PinMode::Output);
  let mut bar_cs =
    GPIO_CONTROLLERS[BAR_CS_PIN_LOC[0]].get_pin(BAR_CS_PIN_LOC[1]);
  bar_cs.mode(PinMode::Output);
  let mut mag_cs =
    GPIO_CONTROLLERS[MAG_CS_PIN_LOC[0]].get_pin(MAG_CS_PIN_LOC[1]);
  mag_cs.mode(PinMode::Output);
  let mut mag_dr =
    GPIO_CONTROLLERS[MAG_DR_PIN_LOC[0]].get_pin(MAG_DR_PIN_LOC[1]);
  mag_dr.mode(PinMode::Input);

  // Ensure all CS are off
  println!("writing all chip selects to be off");
  imu_cs.digital_write(PinValue::High); // IMU, active low
  bar_cs.digital_write(PinValue::High); // BAR, active low
  mag_cs.digital_write(PinValue::High); // MAG, active low

  // Get spi
  let bus = "/dev/spidev0.0";

  // Initialize the actual spi handler
  let mut _driver = LIS3MDL::new(bus, Some(mag_cs), mag_dr)?;
  println!("End of test");
  Ok(())

  // if let Ok(mut driver) = LIS3MDLDriver::new(bus, mag_dr, mag_cs) {
  //   println!("Initialization Success");

  //   // let mut history : Vec<_> = Vec::new();

  //   // for _ in 0..100 {
  //   //   let result = driver.read_magnetic_field();
  //   //   if let Ok(x) = result {
  //   //     history.push(x);
  //   //   } else {
  //   //     println!("ERROR : {}", result.unwrap_err());
  //   //   }
  //   //   sleep(Duration::from_micros(100));
  //   // }

  //   // for data in history {
  //   //   println!("------\n{}", data);
  //   // }
    
  //   return;
  // } else {
  //   println!("Initialization Failure!");
  // }
}
