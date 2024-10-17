use imu::{AdisIMUDriver, DeltaReadData, GenericData, GyroReadData};
use common::comm::gpio::{Gpio, Pin, PinMode, PinMode::*, PinValue::*};
use spidev::Spidev;

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
  let mut controllers : Vec<Gpio>= (0..=3).map(Gpio::open_controller).collect();

  let mut imu_cs = controllers[IMU_CS_PIN_LOC[0]].get_pin(IMU_CS_PIN_LOC[1]);
  imu_cs.mode(Output);
  let mut bar_cs = controllers[BAR_CS_PIN_LOC[0]].get_pin(BAR_CS_PIN_LOC[1]);
  bar_cs.mode(Output);
  let mut mag_cs = controllers[MAG_CS_PIN_LOC[0]].get_pin(MAG_CS_PIN_LOC[1]);
  mag_cs.mode(Output);
  let mut imu_dr = controllers[IMU_DR_PIN_LOC[0]].get_pin(IMU_DR_PIN_LOC[1]);
  imu_dr.mode(Input);
  let mut imu_nreset = controllers[IMU_NRESET_PIN_LOC[0]].get_pin(IMU_NRESET_PIN_LOC[1]);
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
    AdisIMUDriver::initialize(spi, imu_dr, imu_nreset, imu_cs, false) {
    println!("Initialization Success");
    let mut bazinga : Vec<DeltaReadData> = Vec::with_capacity(1000);
    let mut badonga : Vec<i16> = Vec::with_capacity(1000);
    let mut curr : DeltaReadData = DeltaReadData {
      delta_angle : [0; 3],
      delta_velocity : [0; 3],
    };
    let mut old_counter;
    driver
      .write_dec_rate(8)
      .expect("Setting decimation rate failed");
    println!("Prod ID : {:04x}", driver.read_prod_id().unwrap_or(-1));
    println!("MSC_CTRL pre-delta : {:04x}", driver.read_msc_ctrl().unwrap_or(-1));
    let _ = driver.burst_read_delta_16();
    println!("MSC_CTRL post-delta : {:04x}", driver.read_msc_ctrl().unwrap_or(-1));
    let _ = driver.burst_read_gyro_16();
    println!("MSC_CTRL post-gyro : {:04x}", driver.read_msc_ctrl().unwrap_or(-1));
    {
      let result = driver.burst_read_delta_16();
      if result.is_ok() {
        //bazinga.push(result.unwrap());
        let (generic, delta) = result.unwrap();
        old_counter = generic.data_counter;       
      } else {
        println!(
          "Failed to read prod id at iteration {} cause {:?}", 
          0, 
          result.unwrap_err()
        );
        return;
      }
    }
    let start_time = SystemTime::now();
    for i in 0..100 {
      let result = driver.burst_read_gyro_16();
      if result.is_ok() {
        //bazinga.push(result.unwrap());
        let (generic, delta) = result.unwrap();

        badonga.push(generic.data_counter);
        
        // curr.add(delta, (generic.data_counter - old_counter).into()); 
        
        println!("------\n{:04} | {}", generic.data_counter, delta.clone());
        old_counter = generic.data_counter;

        // bazinga.push(curr.clone());
      } else {
        println!(
          "Failed to read prod id at iteration {} cause {:?}", 
          0, 
          result.unwrap_err()
        );
      }
      //sleep(Duration::from_millis(100));
    }
    let duration = start_time.elapsed().expect("Should work");
    println!("{} sec", duration.as_millis() as f32 / 1000.0);
    return;
    for (index, item) in bazinga.iter().enumerate() {
      if index % 10 != 0 { continue; }
      println!("--------");
      println!("{}", item.clone().divide(250));
    }
  } else {
    println!("Initialization Failure!");
  }
  println!("End of test");
}