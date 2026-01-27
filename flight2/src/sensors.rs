use ads124s06::ADC;
use common::comm::gpio::RpiPin;
use common::comm::{ADCKind, FlightComputerADC};
use imu::AdisIMUDriver;
use lis2mdl::LIS2MDL;
use ms5611::MS5611;
use spidev::Spidev;
use std::sync::mpsc;
use std::thread;

// mag+baro
pub fn spawn_mag_bar_worker() -> Result<(), Box<dyn std::error::Error>> {
  let barometer = MS5611::new_with_gpio_pin(
    "/dev/spidev0.0",
    Some(Box::new(RpiPin::new(8)?)),
    4096,
  );
  let magnetometer = LIS2MDL::new_with_gpio_pin(
    "/dev/spidev0.1",
    Some(Box::new(RpiPin::new(7)?)),
  );
  let imu = AdisIMUDriver::initialize_with_gpio_pins(
    Spidev::open("/dev/spidev5.0")?,
    Box::new(RpiPin::new(22)?),
    Box::new(RpiPin::new(23)?),
    Box::new(RpiPin::new(12)?),
  )
  .unwrap();

  let adc = ADC::new_with_gpio_pins(
    "/dev/spidev5.1",
    Some(Box::new(RpiPin::new(27)?)),
    Some(Box::new(RpiPin::new(26)?)),
    ADCKind::FlightComputer(FlightComputerADC::Power),
  )
  .unwrap();

  Ok(())
}

// imu+adc
