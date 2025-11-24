use std::net::{SocketAddr, UdpSocket};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

use crate::adc::{read_3v3_rail, read_5v_rail};
use crate::command::{init_drivers, init_gpio, Drivers};
use crate::communication::{
  check_and_execute, check_heartbeat, establish_flight_computer_connection,
  send_data,
};
use crate::pins::config_pins;
use common::comm::ahrs::{Ahrs, Barometer, DataPoint, Imu, Vector};
use jeflog::fail;

const FREQUENCY: f64 = 300.0;
const INTERVAL: f64 = 1.0 / FREQUENCY;

pub enum State {
  Init,
  Connect(ConnectData),
  MainLoop(MainLoopData),
  Abort(AbortData),
}

pub struct ConnectData {
  drivers: Drivers,
}

pub struct MainLoopData {
  my_data_socket: UdpSocket,
  my_command_socket: UdpSocket,
  fc_address: SocketAddr,
  then: Instant,
  drivers: Drivers,
}

pub struct AbortData {
  drivers: Drivers,
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

  // IMU, barometer, magnetometer
  match init_drivers() {
    Ok(drivers) => State::Connect(ConnectData { drivers }),
    Err(e) => panic!("Failed to initialize drivers: {e}"),
  }
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
    drivers: data.drivers,
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
      drivers: data.drivers,
    });
  }

  let imu = {
    let Ok((_, imu_data)) = data.drivers.imu.burst_read_gyro_16() else {
      fail!("Failed to read IMU data");
      return State::MainLoop(data);
    };
    let (accel, gyro) = (imu_data.get_accel_float(), imu_data.get_gyro_float());
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
  };

  let barometer = {
    let (Ok(temperature), Ok(pressure)) = (
      data.drivers.barometer.read_temperature(),
      data.drivers.barometer.read_pressure(),
    ) else {
      fail!("Failed to read barometer data");
      return State::MainLoop(data);
    };

    Barometer {
      temperature,
      pressure,
    }
  };

  let magnetometer = {
    let Ok(mag) = data.drivers.magnetometer.read() else {
      fail!("Failed to read magnetometer data");
      return State::MainLoop(data);
    };
    Vector {
      x: mag.x as f64,
      y: mag.y as f64,
      z: mag.z as f64,
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

  State::Connect(ConnectData {
    drivers: data.drivers,
  })
}
