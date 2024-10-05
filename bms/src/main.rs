pub mod gpio;
pub mod adc;
pub mod command;
pub mod state;
pub mod protocol;

use std::{net::{SocketAddr, UdpSocket}, process::exit, thread, time::{Duration, Instant}};
use common::comm::{DataMessage, DataPoint};
use jeflog::{warn, task, fail, pass};
use postcard;

const FC_ADDR: &str = "server-01";
const BMS_ID: &str = "bms-01";
const IDENTITY_WAIT_PERIOD: Duration = Duration::from_millis(50);
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(100);

fn main() {

}

fn init() {
  init_gpio(gpio_controllers);
  let cs_mappings = get_cs_mappings(gpio_controllers);
  let drdy_mappings = get_drdy_mappings(gpio_controllers);
  let spi0 = create_spi("/dev/spidev0.0").unwrap();

  let adc1: ADC = ADC::new(
    spi0,
    drdy_mappings.get(&ADCKind::VBatUmbCharge).unwrap(),
    cs_mappings.get(&ADCKind::VBatUmbCharge).unwrap(),
    VBatUmbCharge
  );

  let adc2: ADC = ADC::new(
    spi0,
    drdy_mappings.get(&ADC::SamAnd5V).unwrap(),
    cs_mappings.get(&ADCKind::SamAnd5V).unwrap(),
    SamAnd5V
  );

  let adcs = vec![adc1, adc2];
}

fn establish_flight_computer_connection() -> UdpSocket {
  let mut buf: [u8; 10240] = [0; 10240];
  let socket = UdpSocket::bind(("0.0.0.0", 4573))
    .expect("Could not open socket.");
  socket.set_read_timeout(Some(IDENTITY_WAIT_PERIOD));

  let address: Option<SocketAddr> = format!("{}.local:4573", FC_ADDR)
          .to_socket_addrs()
          .ok()
          .and_then(|mut addrs| addrs.find(|addr| addr.is_ipv4()));

  let Some(fc_address) = address else {
    fail!("Target \x1b[1m{}\x1b[0m could not be located.", FC_ADDR);
  };

  pass!(
    "Target \x1b[1m{}\x1b[0m located at \x1b[1m{}\x1b[0m.",
    FC_ADDR,
    fc_address.ip()
  );

  socket.connect(fc_address).expect("Could not connect the socket to FC address.");
  
  let identity = DataMessage::Identity(BMS_ID.to_string());
  let packet = postcard::to_allocvec(&identity);
  packet.expect("Could not create identity message send buffer");

  loop {
    let count = socket.send(packet)
      .expect("Could not send Identity message");

    let result = match socket.recv(&buf) {
      Ok((size, socket)) =>
        postcard::from_bytes::<DataMessage>(&buf[..size])
          .expect("Could not deserialized recieved message"),
      Err(e) => {
        println("Failed to recieve data: {e}.");
        continue;
      }
    };

    match result {
      DataMessage::Identity(id) => {
        println!("Connection established with FC ({id})");
        socket.set_nonblocking(true)
          .expect("Could not set socket to non-blocking mode.")
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

fn send_data(socket: UdpSocket, datapoints: Vec<DataPoint>) {
  let buffer: [u8; 65536] = [0; 65536];

  let seralized = match postcard::to_slice(&datapoints, &buffer) {
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

fn check_heartbeat(socket: UdpSocket, timer: Instant, gpio_controllers: &[Gpio]) -> Instant {
  let buffer: [u8; 256] = [0; 256];

  if let Err(e) = socket.recv(&buffer) {
    warn!("Could not recieve data from FC ({e}), continuing...");
    return;
  }

  let message = match postcard::from_bytes<DataMessage>(&buffer) {
    Ok(message) => message,
    Err(e) => {
      warn!("Could not deserialize data from FC ({e}), continuing...");
    }
  };

  match message {
    DataMessage::FlightHeartbeat => Instant::now(),
    _ => {
      let delta = Instant::now() - timer;
      if delta > HEARTBEAT_TIME_LIMIT {
        abort(gpio_controllers);
        
      }
      delta
    }
  }
}

fn abort(gpio_controllers: &[Gpio]) {
  fail!("Aborting...");
  protocol::init_gpio(gpio_controllers);
  exit(1);
}

fn init_adcs() {

}

fn poll_adcs() {
  
}