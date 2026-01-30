use ads124s06::ADC;
use common::comm::gpio::RpiPin;
use common::comm::{ADCKind, FlightComputerADC};
use imu::AdisIMUDriver;
use lis2mdl::{MagnetometerData, LIS2MDL};
use ms5611::MS5611;
use spidev::Spidev;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;

pub struct SensorHandle<T> {
  running: Arc<AtomicBool>,
  thread: thread::JoinHandle<()>,
  rx: mpsc::Receiver<T>,
}

impl<T: Send + 'static> SensorHandle<T> {
  pub fn new<F>(mut read: F) -> Self
  where
    F: FnMut(&mpsc::Sender<T>) -> () + Send + 'static,
  {
    let running = Arc::new(AtomicBool::new(true));
    let (tx, rx) = mpsc::channel();
    let thread = {
      let running = running.clone();
      thread::spawn(move || {
        while running.load(Ordering::Relaxed) {
          read(&tx);
        }
      })
    };
    SensorHandle {
      running,
      thread,
      rx,
    }
  }

  pub fn try_read(&self) -> Result<T, mpsc::TryRecvError> {
    self.rx.try_recv()
  }

  pub fn stop(self) {
    self.running.store(false, Ordering::Relaxed);
    self.thread.join().expect("sensor worker thread panicked");
  }
}

pub struct BarometerData {
  /// Pressure in Pa
  pub pressure: f64,
  /// Temperature in degrees Celsius
  pub temperature: f64,
}

pub fn spawn_mag_bar_worker() -> Result<
  SensorHandle<(MagnetometerData, BarometerData)>,
  Box<dyn std::error::Error>,
> {
  let mut barometer = MS5611::new_with_gpio_pin(
    "/dev/spidev0.0",
    Some(Box::new(RpiPin::new(8)?)),
    4096,
  )
  .unwrap();
  let mut magnetometer = LIS2MDL::new_with_gpio_pin(
    "/dev/spidev0.1",
    Some(Box::new(RpiPin::new(7)?)),
  )
  .unwrap();

  Ok(SensorHandle::new(move |tx| {
    let magnetometer_data = magnetometer.read().unwrap();
    let barometer_data = BarometerData {
      pressure: barometer.read_pressure().unwrap(),
      temperature: barometer.read_temperature().unwrap(),
    };
    tx.send((magnetometer_data, barometer_data)).unwrap();
  }))
}

pub fn spawn_imu_adc_worker() -> Result<(), Box<dyn std::error::Error>> {
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
