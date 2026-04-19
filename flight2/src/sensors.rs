use crate::imu_logger::{
  FileLogger as ImuFileLogger, LoggerConfig as ImuLoggerConfig,
  LoggerError as ImuLoggerError,
};
use ads124s06::ADC;
use common::comm::{
  bms::Rail,
  fc_sensors::{AdcData, Imu, Vector},
  gpio::{GpioPin, PinMode, PinValue, RpiGpioController},
  ADCError, ADCFamily,
  ADCKind::FlightComputer,
  FlightComputerADC,
};
use imu::{
  bit_mappings::{DriverResult, ImuDriverError},
  AdisIMUDriver,
};
use lis2mdl::{MagnetometerData, LIS2MDL};
use ms5611::MS5611;
use spidev::Spidev;
use std::{
  error::Error,
  fmt,
  sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::Sender,
    Arc,
  },
  thread,
  time::{Duration, Instant},
};
use std::{fs, sync::mpsc};

use std::sync::OnceLock;

const CURRENT_3V3_SCALE: f64 = 1.0;
const VOLTAGE_3V3_SCALE: f64 = 2.0;
const CURRENT_5V_SCALE: f64 = 5.0 / 3.0;
const VOLTAGE_5V_SCALE: f64 = 3.0;
const CURRENT_LOOP_PT_SCALE: f64 = 2.0;

/// Global Raspberry Pi GPIO controller that isopened once and shared safely.
fn gpio_controller() -> &'static RpiGpioController {
  static CONTROLLER: OnceLock<RpiGpioController> = OnceLock::new();
  CONTROLLER.get_or_init(|| {
    RpiGpioController::open_controller()
      .expect("Failed to open Raspberry Pi GPIO controller")
  })
}

pub struct SensorHandle<T> {
  running: Arc<AtomicBool>,
  thread: Option<thread::JoinHandle<()>>,
  rx: mpsc::Receiver<T>,
}

impl<T: Send + 'static> SensorHandle<T> {
  pub fn new<F>(mut read: F) -> Self
  where
    F: FnMut(&mpsc::Sender<T>) + Send + 'static,
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
      thread: Some(thread),
      rx,
    }
  }

  /// Non-blocking attempt to receive the latest sample from the worker
  pub fn try_read(&self) -> Result<T, mpsc::TryRecvError> {
    self.rx.try_recv()
  }
}

impl<T> SensorHandle<T> {
  pub fn stop(&mut self) {
    self.running.store(false, Ordering::Relaxed);
    if let Some(handle) = self.thread.take() {
      handle.join().expect("sensor worker thread panicked");
    }
  }
}

impl<T> Drop for SensorHandle<T> {
  fn drop(&mut self) {
    self.stop();
  }
}

pub struct BarometerData {
  /// Pressure in Pa
  pub pressure: f64,
  /// Temperature in degrees Celsius
  pub temperature: f64,
}

pub struct MagBarSample {
  pub magnetometer: Option<MagnetometerData>,
  pub barometer: Option<BarometerData>,
}

#[derive(Debug)]
pub enum MagBarError {
  Magnetometer(lis2mdl::Error),
  Barometer(ms5611::Error),
}

impl fmt::Display for MagBarError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      MagBarError::Magnetometer(error) => {
        write!(f, "LIS2MDL magnetometer error: {error}")
      }
      MagBarError::Barometer(error) => {
        write!(f, "MS5611 barometer error: {error}")
      }
    }
  }
}

impl Error for MagBarError {}

impl From<lis2mdl::Error> for MagBarError {
  fn from(error: lis2mdl::Error) -> Self {
    MagBarError::Magnetometer(error)
  }
}

impl From<ms5611::Error> for MagBarError {
  fn from(error: ms5611::Error) -> Self {
    MagBarError::Barometer(error)
  }
}

pub fn spawn_mag_bar_worker(
  enable_magnetometer: bool,
  enable_barometer: bool,
) -> Result<SensorHandle<MagBarSample>, MagBarError> {
  let controller = gpio_controller();

  let mut magnetometer = if enable_magnetometer {
    Some(LIS2MDL::new_with_gpio_pin(
      "/dev/spidev0.1",
      Some(Box::new(controller.get_pin(7))),
    )?)
  } else {
    None
  };
  let mut barometer = if enable_barometer {
    Some(MS5611::new_with_gpio_pin(
      "/dev/spidev0.0",
      Some(Box::new(controller.get_pin(8))),
      4096,
    )?)
  } else {
    None
  };

  Ok(SensorHandle::new(move |tx| {
    let magnetometer_data = if let Some(magnetometer) = magnetometer.as_mut() {
      match magnetometer.read() {
        Ok(data) => Some(data),
        Err(error) => {
          eprintln!("Failed to read magnetometer sensor data: {error:?}");
          None
        }
      }
    } else {
      None
    };

    let barometer_data = if let Some(barometer) = barometer.as_mut() {
      match (barometer.read_pressure(), barometer.read_temperature()) {
        (Ok(pressure), Ok(temperature)) => Some(BarometerData {
          pressure,
          temperature,
        }),
        (pressure, temp) => {
          eprintln!("Failed to read barometer sensor data:");
          eprintln!("- Barometer pressure: {pressure:?}");
          eprintln!("- Barometer temperature: {temp:?}");
          None
        }
      }
    } else {
      None
    };

    if tx
      .send(MagBarSample {
        magnetometer: magnetometer_data,
        barometer: barometer_data,
      })
      .is_err()
    {
      eprintln!("Cannot send mag/bar sensor data to closed channel");
    }
  }))
}

/// Combined sample from the IMU and all rail channels.
pub struct ImuAdcSample {
  /// IMU state (accelerometer + gyroscope), using the shared `Imu` type.
  pub imu: Imu,
  /// Sampled ADC data.
  pub adc: AdcData,
}

/// Errors that can occur while starting the IMU+ADC worker.
#[derive(Debug)]
pub enum ImuAdcWorkerError {
  Imu(ImuDriverError),
  Adc(ADCError),
}

impl fmt::Display for ImuAdcWorkerError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::Imu(e) => write!(f, "IMU error: {e}"),
      Self::Adc(e) => write!(f, "ADC error: {e:?}"),
    }
  }
}

impl Error for ImuAdcWorkerError {}

impl From<ImuDriverError> for ImuAdcWorkerError {
  fn from(err: ImuDriverError) -> Self {
    ImuAdcWorkerError::Imu(err)
  }
}

impl From<ADCError> for ImuAdcWorkerError {
  fn from(err: ADCError) -> Self {
    ImuAdcWorkerError::Adc(err)
  }
}

/// Pins used on IMU (GPIO numbers)
pub struct IMUPins {
  pub cs: u8,
  pub dr: u8,
  pub nreset: u8,
}

const IMU_PINS: IMUPins = IMUPins {
  cs: 12,
  dr: 22,
  nreset: 23,
};

/// Initializes the IMU driver and returns it.
fn init_imu() -> DriverResult<AdisIMUDriver> {
  let controller = gpio_controller();

  // Chip select is active low, so set it to high to disable
  let mut cs = controller.get_pin(IMU_PINS.cs);
  cs.mode(PinMode::Output);
  cs.digital_write(PinValue::High);

  // Set data ready as input
  let mut dr = controller.get_pin(IMU_PINS.dr);
  dr.mode(PinMode::Input);

  // Reset is active low, so set it to high to disable
  let mut nreset = controller.get_pin(IMU_PINS.nreset);
  nreset.mode(PinMode::Output);

  let spi = Spidev::open("/dev/spidev5.0")?;
  let mut imu_driver = AdisIMUDriver::initialize_with_gpio_pins(
    spi,
    Box::new(dr),
    Box::new(nreset),
    Box::new(cs),
  )?;

  imu_driver.write_dec_rate(8)?;
  imu_driver.validate()?;

  Ok(imu_driver)
}

fn init_adc_regs(adc: &mut ADC) -> Result<(), ADCError> {
  adc.set_negative_input_channel_to_aincom()?; // aincom is grounded
  adc.set_positive_input_channel(0)?;

  // pga register
  adc.set_programmable_conversion_delay(14)?;
  adc.disable_pga()?;

  // datarate register
  adc.disable_global_chop()?;
  adc.enable_internal_clock_disable_external()?;
  adc.enable_continious_conversion_mode()?;
  adc.enable_low_latency_filter()?;
  adc.set_data_rate(4000.0)?; // max sampling mode

  // ref register
  adc.disable_reference_monitor()?;
  adc.enable_positive_reference_buffer()?;
  adc.enable_negative_reference_buffer()?;
  //adc.disable_negative_reference_buffer();
  adc.set_ref_input_internal_2v5_ref()?;
  adc.enable_internal_voltage_reference_on_pwr_down()?;

  // idacmag register
  adc.disable_pga_output_monitoring()?;
  adc.open_low_side_pwr_switch()?;
  adc.set_idac_magnitude(0)?;

  // idacmux register
  adc.disable_idac1()?;
  adc.disable_idac2()?;

  // vbias register
  adc.disable_vbias()?;

  // system monitor register
  adc.disable_system_monitoring()?;
  adc.disable_spi_timeout()?;
  adc.disable_crc_byte()?;
  adc.disable_status_byte()?;

  Ok(())
}

fn init_adc() -> Result<ADC, ADCError> {
  let controller = gpio_controller();

  // Set data ready to input mode
  let mut adc_dr = Box::new(controller.get_pin(27));
  adc_dr.mode(PinMode::Input);

  // Chip select is active low, so set it to high to disable
  let mut adc_cs = Box::new(controller.get_pin(26));
  adc_cs.mode(PinMode::Output);
  adc_cs.digital_write(PinValue::High);

  // Initialize ADC
  let mut adc = ADC::new_with_gpio_pins(
    "/dev/spidev5.1",
    Some(adc_dr),
    Some(adc_cs),
    FlightComputer(FlightComputerADC::Power),
  )
  .map_err(|e| {
    eprintln!("Failed to initialize ADC: {e:?}");
    e
  })?;

  // Initialize ADC registers
  init_adc_regs(&mut adc)?;

  // Start continuous conversion on adc
  adc.spi_start_conversion()?;

  Ok(adc)
}

/// Wait for the ADC's DRDY pin to indicate data ready, or time out.
///
/// Returns `true` if it's OK to attempt a conversion read for this channel,
/// `false` if we timed out and should skip this channel.
fn wait_for_adc_drdy(adc: &mut ADC, timeout: Duration) -> bool {
  let start = Instant::now();
  loop {
    match adc.check_drdy() {
      Some(PinValue::Low) => {
        return true;
      }
      Some(_) => {
        if Instant::now().duration_since(start) > timeout {
          return false;
        }
      }
      None => {
        thread::sleep(Duration::from_micros(700));
        return true;
      }
    }
  }
}

/// Read a single ADC channel and return a physical measurement (e.g. volts).
/// On success returns `Some(value)`. On error, logs and returns `None`.
fn read_adc_measurement(adc: &mut ADC) -> Option<f64> {
  match adc.read_counts() {
    Ok(raw) => Some(adc.calc_diff_measurement(raw)),
    Err(e) => {
      eprintln!("ADC read failed: {e:?}");
      None
    }
  }
}

/// ADC input channel assignments for the flight computer ADC.
#[derive(Clone, Copy)]
#[repr(u8)]
enum AdcChannel {
  Rail3v3Current,
  Rail3v3Voltage,
  Rail5vCurrent,
  Rail5vVoltage,
  CurrentLoopPt,
}

// Allows us to iterate over channels in sequential order.
// Rail3v3Current = 0, Rail3v3Voltage = 1, etc
impl AdcChannel {
  const ALL: [Self; 5] = [
    Self::Rail3v3Current,
    Self::Rail3v3Voltage,
    Self::Rail5vCurrent,
    Self::Rail5vVoltage,
    Self::CurrentLoopPt,
  ];
}

/// Sample all configured ADC input channels once and return the collected data.
fn sample_adc_channels(adc: &mut ADC, timeout: Duration) -> AdcData {
  let mut data = AdcData {
    rail_3v3: Rail {
      voltage: 0.0,
      current: 0.0,
    },
    rail_5v: Rail {
      voltage: 0.0,
      current: 0.0,
    },
    current_loop_pt: 0.0,
  };

  for ch in AdcChannel::ALL {
    if !wait_for_adc_drdy(adc, timeout) {
      eprintln!("ADC DRDY timeout on channel {}; skipping", ch as u8);
      continue;
    }

    if let Some(value) = read_adc_measurement(adc) {
      match ch {
        AdcChannel::Rail3v3Current => {
          data.rail_3v3.current = value * CURRENT_3V3_SCALE
        }
        AdcChannel::Rail3v3Voltage => {
          data.rail_3v3.voltage = value * VOLTAGE_3V3_SCALE
        }
        AdcChannel::Rail5vCurrent => {
          data.rail_5v.current = value * CURRENT_5V_SCALE
        }
        AdcChannel::Rail5vVoltage => {
          data.rail_5v.voltage = value * VOLTAGE_5V_SCALE
        }
        AdcChannel::CurrentLoopPt => {
          data.current_loop_pt = value * CURRENT_LOOP_PT_SCALE
        }
      }

      let next = (ch as u8 + 1) % AdcChannel::ALL.len() as u8;
      if let Err(e) = adc.set_positive_input_channel(next) {
        eprintln!("Failed to set ADC positive input channel to {next}: {e:?}",);
      }
    }
  }

  data
}

/// Reads a sample from the IMU and returns an `Imu` instance if successful,
/// otherwise returns `None`.
fn read_imu_sample(
  imu: &mut AdisIMUDriver,
  last_data_counter: &mut Option<i16>,
) -> Option<Imu> {
  let (generic_data, imu_data) = match imu.burst_read_gyro_16() {
    Ok(result) => result,
    Err(e) => {
      eprintln!("IMU read failed: {e}");
      return None;
    }
  };

  if let Some(last_counter) = *last_data_counter {
    if generic_data.data_counter == last_counter {
      return None;
    }
  }
  *last_data_counter = Some(generic_data.data_counter);

  let accel_raw = imu_data.get_accel_float();
  let gyro_raw = imu_data.get_gyro_float();

  let imu_sample = Imu {
    accelerometer: Vector {
      x: accel_raw[0] as f64,
      y: accel_raw[1] as f64,
      z: accel_raw[2] as f64,
    },
    gyroscope: Vector {
      x: gyro_raw[0] as f64,
      y: gyro_raw[1] as f64,
      z: gyro_raw[2] as f64,
    },
  };

  Some(imu_sample)
}

/// Spawns a worker thread that samples the IMU and ADC and sends the samples to
/// a channel.
pub fn spawn_imu_adc_worker(
  enable_imu: bool,
) -> Result<SensorHandle<ImuAdcSample>, ImuAdcWorkerError> {
  let mut imu = if enable_imu {
    Some(init_imu().map_err(|e| {
      eprintln!("IMU initialization failed: {e}");
      e
    })?)
  } else {
    None
  };

  let mut adc = init_adc().map_err(|e| {
    eprintln!("ADC initialization failed: {e:?}");
    e
  })?;

  // Initialize IMU file logger
  let imu_logger_config = ImuLoggerConfig {
    ..ImuLoggerConfig::default()
  };

  let imu_logger: Option<Arc<ImuFileLogger>> =
    match ImuFileLogger::new(imu_logger_config) {
      Ok(logger) => Some(Arc::new(logger)),
      Err(e) => {
        eprintln!(
        "Failed to initialize IMU file logger (continuing without logging): {}",
        e
      );
        None
      }
    };

  let mut last_data_counter: Option<i16> = None;
  let mut current_imu_sample = Imu::default();
  const ADC_DRDY_TIMEOUT: Duration = Duration::from_micros(1000);
  let imu_logger_for_thread = imu_logger.clone();

  Ok(SensorHandle::new(move |tx: &Sender<ImuAdcSample>| {
    // Sample IMU
    if let Some(imu) = imu.as_mut() {
      if let Some(new_imu) = read_imu_sample(imu, &mut last_data_counter) {
        current_imu_sample = new_imu;

        // Log IMU sample to disk if logger is available
        if let Some(ref imu_logger) = imu_logger_for_thread {
          match imu_logger.log(current_imu_sample) {
            Err(ImuLoggerError::ChannelFull) => {
              // Channel full is expected under heavy load - rate-limit warning.
              static mut LAST_WARN: Option<Instant> = None;
              unsafe {
                let now = Instant::now();
                let should_warn = LAST_WARN
                  .map(|last| now.duration_since(last).as_secs() >= 5)
                  .unwrap_or(true);
                if should_warn {
                  eprintln!(
                    "IMU logging channel full (disk I/O cannot keep up). Some data may be dropped."
                  );
                  LAST_WARN = Some(now);
                }
              }
            }
            Err(ImuLoggerError::ChannelDisconnected) => {
              // Writer thread died – this is fatal.
              panic!("IMU logging channel disconnected (writer thread may have crashed)");
            }
            Err(e) => {
              // Other errors (IO, serialization) – treat as fatal for now.
              panic!("Failed to log IMU data to disk: {e}");
            }
            Ok(()) => {}
          }
        }
      }
    }

    // Sample ADC rails
    let adc_data = sample_adc_channels(&mut adc, ADC_DRDY_TIMEOUT);
    let sample = ImuAdcSample {
      imu: current_imu_sample,
      adc: adc_data,
    };

    if tx.send(sample).is_err() {
      eprintln!("Cannot send IMU/ADC sample to closed channel");
    }
  }))
}

/// Spawns a worker thread that reads onboard Pi temperature and sends the samples to a channel.
pub fn spawn_pi_temperature_worker() -> SensorHandle<f64> {
  SensorHandle::new(move |tx| {
    match fs::read_to_string("/sys/class/thermal/thermal_zone0/temp") {
      Ok(temp) => {
        let temp: f64 = temp.parse().expect("failed to parse Pi temperature");
        let temp = temp / 1000.0; // convert mC to C
        if tx.send(temp).is_err() {
          eprintln!("Cannot send Pi temperature to closed channel");
        }
      }
      Err(e) => {
        eprintln!("Failed to read Pi temperature: {e}");
      }
    }
  })
}
