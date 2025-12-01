use std::net::{SocketAddr, UdpSocket};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

use crate::adc::{read_3v3_rail, read_5v_rail};
use crate::communication::{
  check_and_execute, check_heartbeat, establish_flight_computer_connection,
  send_data,
};
use crate::driver::{init_barometer, init_gpio, init_imu, init_magnetometer};
use crate::pins::config_pins;
use common::comm::ahrs::{Ahrs, Barometer, DataPoint, Imu, Vector};
use imu::AdisIMUDriver;
use jeflog::fail;
use lis2mdl::LIS2MDL;
use ms5611::MS5611;

const FREQUENCY: f64 = 300.0;
const INTERVAL: f64 = 1.0 / FREQUENCY;

pub enum State {
  Init,
  Connect(ConnectData),
  MainLoop(MainLoopData),
  Abort(AbortData),
}

pub struct ConnectData {
  imu: AdisIMUDriver,
  barometer: MS5611,
  magnetometer: LIS2MDL,
}

pub struct MainLoopData {
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr,
  then: Instant,
  imu: AdisIMUDriver,
  barometer: MS5611,
  magnetometer: LIS2MDL,
}

pub struct AbortData {
  reconnect_data: ConnectData,
}

impl State {
  pub fn next(self) -> Self {
    match self {
      State::Init => init(),

      State::Connect(data) => connect(data),

      State::MainLoop(data) => main_loop(data),

      State::Abort(data) => abort(data),
    }
  }
}

fn init() -> State {
  config_pins(); // through linux calls to 'config-pin' script, change pins to GPIO
  init_gpio(); // pull all chip selects high

  println!("Initializing drivers");

  State::Connect(ConnectData {
    imu: init_imu().expect("failed to initialize IMU driver"),
    barometer: init_barometer().expect("failed to initialize barometer driver"),
    magnetometer: init_magnetometer()
      .expect("failed to initialize magnetometer driver"),
  })
}

fn connect(data: ConnectData) -> State {
  println!("Connecting to flight computer");

  let (data_socket, command_socket, fc_address) =
    establish_flight_computer_connection();

  println!("Connected to: {}", fc_address);

  State::MainLoop(MainLoopData {
    my_data_socket: data_socket,
    my_command_socket: command_socket,
    fc_address,
    then: Instant::now(),
    imu: data.imu,
    barometer: data.barometer,
    magnetometer: data.magnetometer,
  })
}

fn main_loop(mut data: MainLoopData) -> State {
  check_and_execute(&data.my_command_socket);
  let (updated_time, abort_status) =
    check_heartbeat(&data.my_data_socket, data.then);
  data.then = updated_time;
  let loop_start = Instant::now();

  if abort_status {
    return State::Abort(AbortData {
      reconnect_data: ConnectData {
        imu: data.imu,
        barometer: data.barometer,
        magnetometer: data.magnetometer,
      },
    });
  }

  let imu = match data.imu.burst_read_gyro_16() {
    Ok((_, imu_data)) => {
      let (accel, gyro) =
        (imu_data.get_accel_float(), imu_data.get_gyro_float());
      Imu {
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
      }
    }
    Err(e) => {
      fail!("Failed to read IMU data: {e}");
      return State::MainLoop(data);
    }
  };

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
      barometer,
      magnetometer,
    },
    timestamp: SystemTime::now()
      .duration_since(SystemTime::UNIX_EPOCH)
      .unwrap()
      .as_secs_f64(),
  };

  send_data(&data.my_data_socket, &data.fc_address, datapoint);

  thread::sleep(
    Duration::from_secs_f64(INTERVAL)
      .saturating_sub(Instant::now().duration_since(loop_start)),
  );

  State::MainLoop(data)
}

fn abort(data: AbortData) -> State {
  fail!("Aborting goodbye!");

  init_gpio(); // pull all chip selects high

  State::Connect(data.reconnect_data)
}
