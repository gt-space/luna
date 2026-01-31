use crate::imu_logger::{
  FileLogger as ImuFileLogger, LoggerConfig as ImuLoggerConfig,
  LoggerError as ImuLoggerError,
};
use ads124s06::ADC;
use common::comm::{
  ahrs::{Imu, Vector},
  gpio::{GpioPin, PinMode, PinValue, RpiPin},
  ADCError, ADCFamily,
  ADCKind::FlightComputer,
  FlightComputerADC,
};
use imu::AdisIMUDriver;
use lis2mdl::{MagnetometerData, LIS2MDL};
use ms5611::MS5611;
use spidev::Spidev;
use std::{
  error::Error,
  fmt, io,
  sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::{self, Receiver, Sender},
    Arc,
  },
  thread,
  time::{Duration, Instant},
};

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
) -> Result<SensorHandle<(MagnetometerData, BarometerData)>, MagBarError> {
  let mut magnetometer = LIS2MDL::new_with_gpio_pin(
    "/dev/spidev0.1",
    Some(Box::new(RpiPin::new(7))),
  )?;
  let mut barometer = MS5611::new_with_gpio_pin(
    "/dev/spidev0.0",
    Some(Box::new(RpiPin::new(8))),
    4096,
  )?;

  Ok(SensorHandle::new(move |tx| {
    match (
      magnetometer.read(),
      barometer.read_pressure(),
      barometer.read_temperature(),
    ) {
      (Ok(magnetometer_data), Ok(pressure), Ok(temperature)) => {
        if let Err(_) = tx.send((
          magnetometer_data,
          BarometerData {
            pressure,
            temperature,
          },
        )) {
          eprintln!("Cannot send mag/bar sensor data to closed channel");
        }
      }
      (mag, pressure, temp) => {
        eprintln!("Failed to read mag/bar sensor data:");
        eprintln!("- Magnetometer: {mag:?}");
        eprintln!("- Barometer pressure: {pressure:?}");
        eprintln!("- Barometer temperature: {temp:?}");
      }
    };
  }))
}

/// Logical channels on the flight-computer ADS124S06.
#[derive(Debug, Clone, Copy)]
pub enum FlightRailChannel {
  /// 3V3 rail current (ADC channel 0).
  Rail3v3Current,
  /// 3V3 rail voltage (ADC channel 1).
  Rail3v3Voltage,
  /// 5V rail current (ADC channel 2).
  Rail5vCurrent,
  /// 5V rail voltage (ADC channel 3).
  Rail5vVoltage,
}

/// Single ADC rail measurement.
pub struct RailSample {
  /// Which rail / quantity this sample represents.
  pub channel: FlightRailChannel,
  /// Measured value from the ADC (currently differential voltage).
  pub value: f64,
}

/// Combined sample from the IMU and all rail channels.
pub struct ImuAdcSample {
  /// IMU state (accelerometer + gyroscope), using the shared `Imu` type.
  pub imu: Imu,
  /// All rail measurements collected in this iteration.
  pub rails: Vec<RailSample>,
}

/// Errors that can occur while initializing the IMU.
#[derive(Debug)]
pub enum ImuInitError {
  /// Opening the SPI device for the IMU failed.
  SpiOpen(io::Error),
  /// Initializing the IMU driver over SPI/GPIO failed.
  DriverInit(String),
  /// Failed to write the desired decimation rate.
  DecimationWrite,
  /// IMU PROD_ID validation failed.
  ProdIdValidateFailed,
}

impl fmt::Display for ImuInitError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::SpiOpen(e) => write!(f, "failed to open IMU SPI device: {e}"),
      Self::DriverInit(e) => write!(f, "failed to initialize IMU driver: {e}"),
      Self::DecimationWrite => write!(f, "failed to set IMU decimation rate"),
      Self::ProdIdValidateFailed => write!(f, "failed to validate IMU PROD_ID"),
    }
  }
}

impl Error for ImuInitError {
  fn source(&self) -> Option<&(dyn Error + 'static)> {
    match self {
      Self::SpiOpen(e) => Some(e),
      Self::DriverInit(_)
      | Self::DecimationWrite
      | Self::ProdIdValidateFailed => None,
    }
  }
}

/// Errors that can occur while starting the IMU+ADC worker.
#[derive(Debug)]
pub enum ImuAdcWorkerError {
  /// Failed to initialize the IMU (SPI, GPIO, or driver).
  ImuInit(ImuInitError),
  /// Failed to initialize the ADC.
  AdcInit(ADCError),
}

impl fmt::Display for ImuAdcWorkerError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::ImuInit(e) => write!(f, "{e}"),
      Self::AdcInit(e) => write!(f, "failed to initialize ADC: {e:?}"),
    }
  }
}

impl Error for ImuAdcWorkerError {
  fn source(&self) -> Option<&(dyn Error + 'static)> {
    match self {
      Self::ImuInit(e) => Some(e),
      Self::AdcInit(e) => Some(e),
    }
  }
}

/// IMU type that contains pin numbers and the driver
pub struct IMUInfo {
  pub pins: IMUPins,
  pub driver: AdisIMUDriver,
}

/// Pins used on IMU (GPIO numbers)
pub struct IMUPins {
  pub cs: u8,
  pub dr: u8,
  pub nreset: u8,
}

/// Initializes the IMU driver and returns an IMU instance.
fn init_imu() -> Result<IMUInfo, ImuInitError> {
  // GPIO indices for IMU pins
  let imu_pins = IMUPins {
    cs: 12,
    dr: 22,
    nreset: 23,
  };

  // Chip select is active low, so set it to high to disable
  let mut cs = RpiPin::new(imu_pins.cs);
  cs.mode(PinMode::Output);
  cs.digital_write(PinValue::High);

  // Set data ready as input
  let mut dr = RpiPin::new(imu_pins.dr);
  dr.mode(PinMode::Input);

  // Reset is active low, so set it to high to disable
  let mut nreset = RpiPin::new(imu_pins.nreset);
  nreset.mode(PinMode::Output);

  // Initialize driver
  let spi = Spidev::open("/dev/spidev5.0").map_err(ImuInitError::SpiOpen)?;
  let mut imu_driver = AdisIMUDriver::initialize_with_gpio_pins(
    spi,
    Box::new(dr),
    Box::new(nreset),
    Box::new(cs),
  )
  .map_err(|e| ImuInitError::DriverInit(format!("{e}")))?;

  imu_driver
    .write_dec_rate(8)
    .map_err(|_| ImuInitError::DecimationWrite)?;
  if !imu_driver.validate() {
    return Err(ImuInitError::ProdIdValidateFailed);
  }

  Ok(IMUInfo {
    pins: imu_pins,
    driver: imu_driver,
  })
}

fn init_adc_regs(adc: &mut ADC) -> Result<(), ADCError> {
  // mux register
  adc.set_positive_input_channel(0)?;
  adc.set_negative_input_channel_to_aincom(); // aincom is grounded

  // pga register
  adc.set_programmable_conversion_delay(14);
  adc.disable_pga();

  // datarate register
  adc.disable_global_chop();
  adc.enable_internal_clock_disable_external();
  adc.enable_continious_conversion_mode();
  adc.enable_low_latency_filter();
  adc.set_data_rate(4000.0); // max sampling mode

  // ref register
  adc.disable_reference_monitor();
  adc.enable_positive_reference_buffer();
  adc.enable_negative_reference_buffer();
  //adc.disable_negative_reference_buffer();
  adc.set_ref_input_internal_2v5_ref();
  adc.enable_internal_voltage_reference_on_pwr_down();

  // idacmag register
  adc.disable_pga_output_monitoring();
  adc.open_low_side_pwr_switch();
  adc.set_idac_magnitude(0);

  // idacmux register
  adc.disable_idac1();
  adc.disable_idac2();

  // vbias register
  adc.disable_vbias();

  // system monitor register
  adc.disable_system_monitoring();
  adc.disable_spi_timeout();
  adc.disable_crc_byte();
  adc.disable_status_byte();

  Ok(())
}

fn init_adc() -> Result<ADC, ADCError> {
  // Set data ready to input mode
  let mut adc_dr = Box::new(RpiPin::new(27));
  adc_dr.mode(PinMode::Input);

  // Chip select is active low, so set it to high to disable
  let mut adc_cs = Box::new(RpiPin::new(26));
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
  adc.spi_start_conversion();

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

/// Sample all configured ADC input channels once, returning a vector of
/// per-channel rail measurements.
fn sample_adc_channels(
  adc: &mut ADC,
  timeout: Duration,
  num_channels: usize,
) -> Vec<RailSample> {
  let mut samples = Vec::with_capacity(num_channels);

  for channel in 0..num_channels {
    if !wait_for_adc_drdy(adc, timeout) {
      eprintln!("ADC DRDY timeout on channel {channel}; skipping this channel");
      continue;
    }

    if let Some(value) = read_adc_measurement(adc) {
      let rail_channel = match channel {
        0 => FlightRailChannel::Rail3v3Current,
        1 => FlightRailChannel::Rail3v3Voltage,
        2 => FlightRailChannel::Rail5vCurrent,
        3 => FlightRailChannel::Rail5vVoltage,
        _ => continue,
      };

      samples.push(RailSample {
        channel: rail_channel,
        value,
      });

      let next = ((channel + 1) % num_channels) as u8;
      if let Err(e) = adc.set_positive_input_channel(next) {
        eprintln!(
          "Failed to set ADC positive input channel to {}: {e:?}",
          next
        );
      }
    }
  }

  samples
}

/// Reads a sample from the IMU and returns an `Imu` instance if successful,
/// otherwise returns `None`.
fn read_imu_sample(
  imu: &mut IMUInfo,
  last_data_counter: &mut Option<i16>,
) -> Option<Imu> {
  let (generic_data, imu_data) = match imu.driver.burst_read_gyro_16() {
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

/// Spawns a worker thread that samples the IMU and ADC and sends the samples to a channel.
pub fn spawn_imu_adc_worker() -> Result<
  (
    thread::JoinHandle<()>,
    Arc<AtomicBool>,
    Receiver<ImuAdcSample>,
  ),
  ImuAdcWorkerError,
> {
  // Initialize IMU
  let mut imu = init_imu().map_err(ImuAdcWorkerError::ImuInit)?;

  // Initialize ADC
  let mut adc = init_adc().map_err(|e| {
    eprintln!("Failed to initialize ADC: {e:?}");
    ImuAdcWorkerError::AdcInit(e)
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

  let running = Arc::new(AtomicBool::new(true));
  let (tx, rx): (Sender<ImuAdcSample>, Receiver<ImuAdcSample>) =
    mpsc::channel();
  let thread_running = running.clone();
  let imu_logger_for_thread = imu_logger.clone();

  let handle = thread::spawn(move || {
    let mut last_data_counter: Option<i16> = None;
    let mut current_imu_sample = Imu::default();
    const ADC_DRDY_TIMEOUT: Duration = Duration::from_micros(1000);
    const ADC_NUM_INPUT_CHANNELS: usize = 4;

    loop {
      if !thread_running.load(Ordering::SeqCst) {
        break;
      }

      // Sample IMU
      if let Some(new_imu) = read_imu_sample(&mut imu, &mut last_data_counter) {
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

      // Sample ADC rails
      let rails =
        sample_adc_channels(&mut adc, ADC_DRDY_TIMEOUT, ADC_NUM_INPUT_CHANNELS);
      let sample = ImuAdcSample {
        imu: current_imu_sample,
        rails,
      };

      if tx.send(sample).is_err() {
        break;
      }
    }
  });

  Ok((handle, running, rx))
}
