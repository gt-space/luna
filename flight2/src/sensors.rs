use std::{
    error::Error,
    fmt,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Sender},
        Arc, OnceLock,
    },
    thread,
    time::{Duration, Instant},
};

use ads124s06::ADC;
use asm330lhbg1::{Asm330lhbg1, Sample as ImuSample};
use common::comm::{
    bms::Rail,
    fc_sensors::{AdcData, Imu, Vector},
    gpio::{GpioPin, PinMode, PinValue, RpiGpioController},
    ADCError, ADCFamily,
    ADCKind::FlightComputer,
    FlightComputerADC,
};
use lis2mdl::{MagnetometerData, LIS2MDL};
use ms5611::MS5611;

use crate::imu_logger::{
    FileLogger as ImuFileLogger, LoggerConfig as ImuLoggerConfig, LoggerError as ImuLoggerError,
};

const CURRENT_3V3_SCALE: f64 = 1.0;
const VOLTAGE_3V3_SCALE: f64 = 2.0;
const CURRENT_5V_SCALE: f64 = 5.0 / 3.0;
const VOLTAGE_5V_SCALE: f64 = 3.0;
const CURRENT_LOOP_PT_SCALE: f64 = 2.0;

/// Global Raspberry Pi GPIO controller that isopened once and shared safely.
fn gpio_controller() -> &'static RpiGpioController {
    static CONTROLLER: OnceLock<RpiGpioController> = OnceLock::new();
    CONTROLLER.get_or_init(|| {
        RpiGpioController::open_controller().expect("Failed to open Raspberry Pi GPIO controller")
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

/// Rail measurements and any fresh IMU reading from the same worker iteration.
pub struct ImuAdcSample {
    pub imu: Option<Imu>,
    /// Sampled ADC data.
    pub adc: AdcData,
}

const IMU_SPI_PATH: &str = "/dev/spidev5.0";
const ADC_SPI_PATH: &str = "/dev/spidev5.1";
const IMU_STALE_TIMEOUT: Duration = Duration::from_millis(250);
const IMU_WARNING_INTERVAL: Duration = Duration::from_secs(5);

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

    // The Pi device tree assigns GPIO26 as SPI5 CS1 and Linux drives it.
    let mut adc = ADC::new_with_gpio_pins(
        ADC_SPI_PATH,
        Some(adc_dr),
        None,
        FlightComputer(FlightComputerADC::Power),
    )?;

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
                AdcChannel::Rail3v3Current => data.rail_3v3.current = value * CURRENT_3V3_SCALE,
                AdcChannel::Rail3v3Voltage => data.rail_3v3.voltage = value * VOLTAGE_3V3_SCALE,
                AdcChannel::Rail5vCurrent => data.rail_5v.current = value * CURRENT_5V_SCALE,
                AdcChannel::Rail5vVoltage => data.rail_5v.voltage = value * VOLTAGE_5V_SCALE,
                AdcChannel::CurrentLoopPt => data.current_loop_pt = value * CURRENT_LOOP_PT_SCALE,
            }

            let next = (ch as u8 + 1) % AdcChannel::ALL.len() as u8;
            if let Err(e) = adc.set_positive_input_channel(next) {
                eprintln!("Failed to set ADC positive input channel to {next}: {e:?}",);
            }
        }
    }

    data
}

fn read_imu_sample(
    imu: &mut Asm330lhbg1,
    last_sample_at: &mut Instant,
    last_warning_at: &mut Option<Instant>,
) -> Option<Imu> {
    let result = imu.try_read_sample();
    let now = Instant::now();
    let error = match result {
        Ok(Some(sample)) => {
            *last_sample_at = now;
            return Some(imu_to_telemetry(sample));
        }
        Ok(None) if now.duration_since(*last_sample_at) < IMU_STALE_TIMEOUT => return None,
        Ok(None) => None,
        Err(error) => Some(error),
    };
    if last_warning_at.is_none_or(|last| now.duration_since(last) >= IMU_WARNING_INTERVAL) {
        match error {
            Some(error) => eprintln!("ASM330LHBG1 read failed: {error}"),
            None => {
                eprintln!("ASM330LHBG1 has not produced a fresh sample in {IMU_STALE_TIMEOUT:?}")
            }
        }
        *last_warning_at = Some(now);
    }
    None
}

fn imu_to_telemetry(sample: ImuSample) -> Imu {
    // Publish sensor axes until the installed CTV body-frame mapping is
    // verified.
    let [ax, ay, az] = sample.acceleration_mps2;
    let [gx, gy, gz] = sample.angular_rate_dps;
    Imu {
        accelerometer: Vector {
            x: ax,
            y: ay,
            z: az,
        },
        gyroscope: Vector {
            x: gx,
            y: gy,
            z: gz,
        },
    }
}

/// Spawns a worker thread that samples the IMU and ADC and sends the samples to
/// a channel.
pub fn spawn_imu_adc_worker(enable_imu: bool) -> Result<SensorHandle<ImuAdcSample>, ADCError> {
    let mut adc = init_adc()?;
    let mut imu = if enable_imu {
        match Asm330lhbg1::open(IMU_SPI_PATH) {
            Ok(imu) => {
                println!("ASM330LHBG1 initialized on {IMU_SPI_PATH}");
                Some(imu)
            }
            Err(error) => {
                eprintln!(
                    "ASM330LHBG1 initialization failed on {IMU_SPI_PATH}: {error}. \
                     Continuing with ADC only."
                );
                None
            }
        }
    } else {
        None
    };

    let mut imu_logger =
        imu.as_ref()
            .and_then(|_| match ImuFileLogger::new(ImuLoggerConfig::default()) {
                Ok(logger) => Some(logger),
                Err(error) => {
                    eprintln!("Failed to initialize IMU file logger: {error}");
                    None
                }
            });

    let mut last_imu_sample_at = Instant::now();
    let mut last_imu_warning_at = None;
    let mut last_log_warning_at = None;
    const ADC_DRDY_TIMEOUT: Duration = Duration::from_micros(1000);

    Ok(SensorHandle::new(move |tx: &Sender<ImuAdcSample>| {
        let imu_sample = imu.as_mut().and_then(|imu| {
            read_imu_sample(imu, &mut last_imu_sample_at, &mut last_imu_warning_at)
        });
        if let (Some(sample), Some(logger)) = (imu_sample, imu_logger.as_ref()) {
            match logger.log(sample) {
                Err(ImuLoggerError::ChannelFull) => {
                    let now = Instant::now();
                    if last_log_warning_at
                        .is_none_or(|last| now.duration_since(last) >= IMU_WARNING_INTERVAL)
                    {
                        eprintln!("IMU logging channel full. Dropping samples from the log.");
                        last_log_warning_at = Some(now);
                    }
                }
                Err(error) => {
                    eprintln!("IMU file logging stopped: {error}");
                    imu_logger = None;
                }
                Ok(()) => {}
            }
        }

        let sample = ImuAdcSample {
            imu: imu_sample,
            adc: sample_adc_channels(&mut adc, ADC_DRDY_TIMEOUT),
        };

        if tx.send(sample).is_err() {
            eprintln!("Cannot send IMU/ADC sample to closed channel");
        }
    }))
}
