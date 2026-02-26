use std::{
  net::{SocketAddr, UdpSocket},
  sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
  },
  thread,
  time::{Instant, SystemTime},
};

use crate::{
  adc::{read_3v3_rail, read_5v_rail},
  communication::{
    check_and_execute, check_heartbeat, establish_flight_computer_connection,
    send_data,
  },
  driver::{init_barometer, init_gpio, init_imu, init_magnetometer},
  file_logger::{FileLogger, LoggerConfig},
  pins::config_pins,
};
use common::comm::ahrs::{Ahrs, Barometer, DataPoint, Imu, Vector};
use jeflog::{fail, warn};
use lis2mdl::LIS2MDL;
use ms5611::MS5611;
use tokio::sync::watch;

pub enum State {
  Init(InitData),
  MainLoop(MainLoopData),
  Abort(AbortData),
}

pub struct InitData {
  pub imu_logger_config: LoggerConfig,
}

pub struct MainLoopData {
  pub my_data_socket: UdpSocket,
  pub my_command_socket: UdpSocket,
  pub fc_address: SocketAddr,
  pub then: Instant,
  pub imu_logger_config: LoggerConfig,
  pub imu_thread: (
    thread::JoinHandle<()>,
    Arc<AtomicBool>,
    watch::Receiver<Imu>,
    Option<Arc<FileLogger>>,
  ),
  pub barometer: MS5611,
  pub magnetometer: LIS2MDL,
}

pub struct AbortData {
  pub imu_logger_config: LoggerConfig,
}

impl State {
  pub fn next(self) -> Self {
    match self {
      State::Init(data) => init(data),
      State::MainLoop(data) => main_loop(data),
      State::Abort(data) => abort(data),
    }
  }
}

fn init(data: InitData) -> State {
  config_pins(); // through linux calls to 'config-pin' script, change pins to GPIO
  init_gpio(); // pull all chip selects high

  println!("Initializing drivers");
  let mut imu = init_imu().expect("failed to initialize IMU driver");
  let barometer =
    init_barometer().expect("failed to initialize barometer driver");
  let magnetometer =
    init_magnetometer().expect("failed to initialize magnetometer driver");

  println!("Connecting to flight computer");
  let (data_socket, command_socket, fc_address) =
    establish_flight_computer_connection();
  println!("Connected to: {}", fc_address);

  println!(
    "Creating file logger for IMU data with config: {:?}",
    data.imu_logger_config
  );

  let imu_logger = match FileLogger::new(data.imu_logger_config.clone()) {
    Ok(logger) => {
      if data.imu_logger_config.enabled {
        println!(
          "File logging enabled. Log directory: {:?}",
          data.imu_logger_config.log_dir
        );
      }
      Some(Arc::new(logger))
    }
    Err(e) => {
      warn!("Failed to initialize file logger: {}. Continuing without file logging.", e);
      None
    }
  };

  let running = Arc::new(AtomicBool::new(true));
  let (imu_main_tx, imu_rx) = watch::channel(Imu::default());
  let imu_thread_handle = {
    let running = running.clone();
    let imu_logger = imu_logger.clone();
    thread::spawn(move || {
      let mut last_data_counter: Option<i16> = None;
      while running.load(Ordering::SeqCst) {
        match imu.burst_read_gyro_16() {
          Ok((generic_data, imu_data)) => {
            // Check if we have new data by comparing the data_counter
            // The IMU increments this counter each time it has new data available
            // If the counter hasn't changed, we're reading duplicate data
            if let Some(last_counter) = last_data_counter {
              if generic_data.data_counter == last_counter {
                // Same data as last read, skip logging to avoid duplicates
                continue;
              }
            }
            last_data_counter = Some(generic_data.data_counter);
            
            let (accel, gyro) =
              (imu_data.get_accel_float(), imu_data.get_gyro_float());
            let imu_data = Imu {
              accelerometer: Vector {
                x: accel[0] as f64,
                y: accel[1] as f64,
                z: accel[2] as f64,
              },
              gyroscope: Vector {
                x: gyro[0] as f64,
                y: gyro[1] as f64,
                z: gyro[2] as f64,
              },
            };
            imu_main_tx
              .send(imu_data)
              .expect("main thread has already exited");
            if let Some(imu_logger) = &imu_logger {
              match imu_logger.log(imu_data) {
                Err(crate::file_logger::LoggerError::ChannelFull) => {
                  // Channel full is expected under heavy load - just warn occasionally
                  // to avoid spamming logs (rate-limited to once per 5 seconds)
                  static mut LAST_WARN: Option<Instant> = None;
                  unsafe {
                    let now = Instant::now();
                    let should_warn = LAST_WARN
                      .map(|last| now.duration_since(last).as_secs() >= 5)
                      .unwrap_or(true);
                    if should_warn {
                      warn!("IMU logging channel full (disk I/O cannot keep up). Some data may be dropped.");
                      LAST_WARN = Some(now);
                    }
                  }
                }
                Err(crate::file_logger::LoggerError::ChannelDisconnected) => {
                  // Channel disconnected means writer thread died - this is fatal
                  fail!("IMU logging channel disconnected (writer thread may have crashed)");
                }
                Err(e) => {
                  // Other errors (IO, serialization)
                  fail!("Failed to log IMU data to disk: {e}");
                }
                Ok(()) => {
                  // Success - no action needed
                }
              }
            }
          }
          Err(e) => fail!("Failed to read IMU data: {e}"),
        };
      }
    })
  };

  State::MainLoop(MainLoopData {
    my_data_socket: data_socket,
    my_command_socket: command_socket,
    fc_address,
    then: Instant::now(),
    imu_logger_config: data.imu_logger_config,
    imu_thread: (imu_thread_handle, running, imu_rx, imu_logger),
    barometer,
    magnetometer,
  })
}

fn main_loop(mut data: MainLoopData) -> State {
  check_and_execute(&data.my_command_socket);
  let (updated_time, abort_status) =
    check_heartbeat(&data.my_data_socket, data.then);
  data.then = updated_time;

  if abort_status {
    let (handle, running, _, imu_logger) = data.imu_thread;
    println!("Stopping IMU thread");
    running.store(false, Ordering::SeqCst);
    handle.join().expect("IMU thread panicked"); // ensure IMU thread exits to re-acquire the logger
    println!("Shutting down IMU file logger");
    // Once IMU thread is joined, there should be no other strong references
    // If the thread panics, unwinding would drop the other references
    if let Some(imu_logger) = imu_logger.and_then(Arc::into_inner) {
      imu_logger
        .shutdown()
        .expect("failed to shutdown IMU file logger");
    }
    return State::Abort(AbortData {
      imu_logger_config: data.imu_logger_config,
    });
  }

  let (_, _, imu_rx, _) = &data.imu_thread;
  let imu = *imu_rx.borrow();

  let barometer = match (
    data.barometer.read_temperature(),
    data.barometer.read_pressure(),
  ) {
    (Ok(temperature), Ok(pressure)) => Barometer {
      temperature,
      pressure,
    },
    (a, b) => {
      fail!(
        "Failed to read barometer data\n- Temperature: {a:?}\n- Pressure: {b:?}"
      );
      return State::MainLoop(data);
    }
  };

  let magnetometer = match data.magnetometer.read() {
    Ok(mag) => Vector {
      x: mag.x as f64,
      y: mag.y as f64,
      z: mag.z as f64,
    },
    Err(e) => {
      fail!("Failed to read magnetometer data: {e}");
      return State::MainLoop(data);
    }
  };

  let datapoint = DataPoint {
    state: Ahrs {
      rail_3v3: read_3v3_rail(),
      rail_5v: read_5v_rail(),
      imu,
      magnetometer,
      barometer,
    },
    timestamp: get_timestamp(),
  };

  send_data(&data.my_data_socket, &data.fc_address, datapoint);

  State::MainLoop(data)
}

fn abort(data: AbortData) -> State {
  fail!("Aborting goodbye!");

  init_gpio(); // pull all chip selects high

  State::Init(InitData {
    imu_logger_config: data.imu_logger_config,
  })
}

/// Current Unix timestamp in seconds with nanosecond precision
fn get_timestamp() -> f64 {
  SystemTime::now()
    .duration_since(SystemTime::UNIX_EPOCH)
    .expect("failed to get timestamp")
    .as_secs_f64()
}
