pub mod command;
pub mod protocol;

use std::{net::{UdpSocket, ToSocketAddrs}, process::exit, time::{Duration, Instant}};
use command::execute;
use common::comm::{DataMessage, DataPoint, Gpio, SamControlMessage, ADCKind::{VBatUmbCharge, SamAnd5V}};
use jeflog::{warn, fail, pass};
use postcard;
use protocol::init_gpio;
use ads114s06::ADC;

const FC_ADDR: &str = "server-01";
const BMS_ID: &str = "bms-01";
const COMMAND_PORT: u16 = 8378;
const IDENTITY_WAIT_PERIOD: Duration = Duration::from_millis(50);
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(250);

fn main() {
  let gpio_controllers: Vec<Gpio> = open_controllers();
  init_gpio(&gpio_controllers);
  let (data_socket, command_socket) = establish_flight_computer_connection();

  // VBatUmbCharge
  let mut adc1: ADC = ADC::new(
    "/dev/spidev0.0",
    gpio_controllers[1].get_pin(28),
    gpio_controllers[0].get_pin(30),
    VBatUmbCharge
  ).expect("Failed to initialize VBatUmbCharge ADC");

  // SamAnd5V
  let mut adc2: ADC = ADC::new(
    "/dev/spidev0.1",
    gpio_controllers[1].get_pin(18),
    gpio_controllers[0].get_pin(31),
    SamAnd5V
  ).expect("Failed to initialize the SamAnd5V ADC");
  
  let mut then = Instant::now();
  loop {
    check_and_execute(&gpio_controllers, &command_socket);
    then = check_heartbeat(&data_socket, then, &gpio_controllers);
    let mut datapoints = Vec::with_capacity(9);

    for i in 0..6 {
      if let Some(datapoint) = adc1.poll(i) {
        datapoints.push(datapoint);
      }

      if let Some(datapoint) = adc2.poll(i) {
        datapoints.push(datapoint);
      }
    }

    send_data(&data_socket, datapoints);
  }
}

// make sure you keep track of these UdpSockets, and pass them into the correct
// functions. Left is data, right is command.
fn establish_flight_computer_connection() -> (UdpSocket, UdpSocket) {
  let mut buf: [u8; 10240] = [0; 10240];
  let data_socket = UdpSocket::bind(("0.0.0.0", 4573))
    .expect("Could not open data socket.");
  let command_socket = UdpSocket::bind(("0.0.0.0", COMMAND_PORT))
    .expect("Could not open command socket.");
  let _ = command_socket.set_nonblocking(true)
    .expect("Could not set command socket to unblocking");
  let _ = data_socket.set_read_timeout(Some(IDENTITY_WAIT_PERIOD))
    .expect("Could not set read timeout for data_socket");

  let address = format!("{}.local:4573", FC_ADDR)
          .to_socket_addrs()
          .ok()
          .and_then(|mut addrs| addrs.find(|addr| addr.is_ipv4()));

  let fc_address = address.expect("Flight Computer address could not be found!");

  pass!(
    "Target \x1b[1m{}\x1b[0m located at \x1b[1m{}\x1b[0m.",
    FC_ADDR,
    fc_address.ip()
  );

  data_socket.connect(fc_address).expect("Could not connect the socket to FC address.");
  
  let identity = DataMessage::Identity(BMS_ID.to_string());
  let packet = postcard::to_allocvec(&identity)
    .expect("Could not create identity message send buffer");

  loop {
    let _ = data_socket.send(&packet)
      .expect("Could not send Identity message");

    let result = match data_socket.recv(&mut buf) {
      Ok(size) =>
        postcard::from_bytes::<DataMessage>(&buf[..size])
          .expect("Could not deserialized recieved message"),
      Err(e) => {
        println!("Failed to recieve data: {e}.");
        continue;
      }
    };

    match result {
      DataMessage::Identity(id) => {
        println!("Connection established with FC ({id})");
        data_socket.set_nonblocking(true)
          .expect("Could not set socket to non-blocking mode.");
        
        return (data_socket, command_socket)
      },
      DataMessage::FlightHeartbeat => {
        println!("Recieved heartbeat from FC despite no identity.");
        continue;
      },
      _ => {
        println!("Recieved nonsenical data from FC.");
        continue;
      }
    }
  }
}

fn send_data(socket: &UdpSocket, datapoints: Vec<DataPoint>) {
  let mut buffer: [u8; 65536] = [0; 65536];

  let seralized = match postcard::to_slice(&datapoints, &mut buffer) {
    Ok(slice) => slice,
    Err(e) => {
      warn!("Could not serialize buffer ({e}), continuing...");
      return;
    }
  };
  
  match socket.send(seralized) {
    Ok(size) => {
      pass!("Successfully sent {size} bytes of data...");
    },
    Err(e) => {
      warn!("Could not send data ({e}), continuing...");
    }
  };
}

// Make sure you keep track of the timer that is returned, and pass it in on the next loop
fn check_heartbeat(socket: &UdpSocket, timer: Instant, gpio_controllers: &[Gpio]) -> Instant {
  let mut buffer: [u8; 256] = [0; 256];

  let delta = Instant::now() - timer;
  if delta > HEARTBEAT_TIME_LIMIT {
    abort(gpio_controllers);
  }

  let size = match socket.recv(&mut buffer) {
    Ok(size) => size,
    Err(_) => {
      return timer;
    }
  };

  let message = match postcard::from_bytes::<DataMessage>(&buffer[..size]) {
    Ok(message) => message,
    Err(e) => {
      warn!("Could not deserialize data from FC ({e}), continuing...");
      return timer;
    }
  };

  match message {
    DataMessage::FlightHeartbeat => Instant::now(),
    _ => {
      warn!("Expected Flight Heartbeat was not detected.");
      timer
    }
  }
}

fn check_and_execute(gpio_controllers: &[Gpio], command_socket: &UdpSocket) {
  let mut buf: [u8; 10240] = [0; 10240];

  let size = match command_socket.recv_from(&mut buf) {
    Ok((size, _)) => size,
    Err(_) => return,
  };

  let command = match postcard::from_bytes::<SamControlMessage>(&buf[..size]) {
    Ok(command) => command,
    Err(e) => {
      fail!("Command was recieved but could not be deserialized ({e}).");
      return;
    }
  };

  pass!("Executing command...");
  execute(gpio_controllers, command);
}

fn abort(gpio_controllers: &[Gpio]) {
  fail!("Aborting...");
  protocol::init_gpio(gpio_controllers);
  exit(1);
}

pub fn open_controllers() -> Vec<Gpio> {
  (0..=3).map(Gpio::open_controller).collect()
}