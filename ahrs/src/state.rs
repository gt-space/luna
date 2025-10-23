use std::net::{SocketAddr, UdpSocket};
use std::time::{Instant, SystemTime};

use crate::command::{init_drivers, init_gpio, Drivers};
use crate::communication::{
  check_and_execute, check_heartbeat, establish_flight_computer_connection,
  send_data,
};
use crate::pins::config_pins;
use common::comm::ahrs::{Ahrs, DataPoint, Imu, Vector};
use jeflog::fail;

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
  init_gpio(); // disable all chip selects

  // IMU, barometer, magnetometer
  match init_drivers() {
    Ok(drivers) => State::Connect(ConnectData { drivers }),
    Err(e) => panic!("Failed to initialize drivers: {e}"),
  }
}

fn connect(data: ConnectData) -> State {
  let (data_socket, command_socket, fc_address) =
    establish_flight_computer_connection();

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

  if abort_status {
    return State::Abort(AbortData {
      drivers: data.drivers,
    });
  }

  let Ok((_, imu_data)) = data.drivers.imu.burst_read_gyro_16() else {
    fail!("Failed to read IMU data");
    return State::MainLoop(data);
  };
  let (accel, gyro) = (imu_data.get_accel_float(), imu_data.get_gyro_float());

  let datapoint = DataPoint {
    state: Ahrs {
      imu: Imu {
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
      },
      ..Default::default() // TODO
    },
    timestamp: SystemTime::now()
      .duration_since(SystemTime::UNIX_EPOCH)
      .unwrap()
      .as_secs_f64(),
  };

  send_data(&data.my_data_socket, &data.fc_address, datapoint);

  State::MainLoop(data)
}

fn abort(data: AbortData) -> State {
  fail!("Aborting goodbye!");

  init_gpio(); // pull all chip selects high

  State::Connect(ConnectData {
    drivers: data.drivers,
  })
}
