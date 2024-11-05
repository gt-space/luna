pub mod command;
pub mod protocol;
pub mod adc;

use std::{borrow::Cow, net::{SocketAddr, ToSocketAddrs, UdpSocket}, process::exit, thread, time::{Duration, Instant}};
use command::execute;
use common::comm::{ChannelType, DataMessage, DataPoint, Gpio, PinValue::{Low, High}, SamControlMessage, ADCKind, ADCKind::{VBatUmbCharge, SamAnd5V}};
use jeflog::{warn, fail, pass};
use protocol::init_gpio;
use ads114s06::ADC;
use adc::{init_adcs, poll_adcs};

const FC_ADDR: &str = "server-01";
const BMS_ID: &str = "bms-01";
const COMMAND_PORT: u16 = 8378;
const HEARTBEAT_TIME_LIMIT: Duration = Duration::from_millis(250);

fn main() {
  let gpio_controllers: Vec<Gpio> = open_controllers();
  init_gpio(&gpio_controllers);
  
  // VBatUmbCharge
  let mut adc1: ADC = ADC::new(
    "/dev/spidev0.0",
    gpio_controllers[1].get_pin(28),
    Some(gpio_controllers[0].get_pin(30)),
    VBatUmbCharge
  ).expect("Failed to initialize VBatUmbCharge ADC");

  // SamAnd5V
  let mut adc2: ADC = ADC::new(
    "/dev/spidev0.0",
    gpio_controllers[1].get_pin(18),
    Some(gpio_controllers[0].get_pin(31)),
    SamAnd5V
  ).expect("Failed to initialize the SamAnd5V ADC");

  thread::sleep(Duration::from_millis(100));

  println!("ADC 1 regs (before init)");
  for (reg, reg_value) in adc1.spi_read_all_regs().unwrap().into_iter().enumerate() {
    println!("Reg {:x}: {:08b}", reg, reg_value);
  }
  println!("");
  println!("ADC 2 regs (before init)");
  for (reg, reg_value) in adc2.spi_read_all_regs().unwrap().into_iter().enumerate() {
    println!("Reg {:x}: {:08b}", reg, reg_value);
  }


  let mut adcs: Vec<ADC> = vec![adc1, adc2];
  init_adcs(&mut adcs);

  // begin FC communication
  let (data_socket, command_socket, fc_address) = establish_flight_computer_connection();
  
  let mut then = Instant::now();
  loop {
    println!("Checking for commands...");

    // check if commands were sent from the FC. if so, execute them
    check_and_execute(&gpio_controllers, &command_socket);
    println!("Checking heartbeat...");

    // check if the FC is sending a heartbeat. if so, reset the timer
    then = check_heartbeat(&data_socket, then, &gpio_controllers);
    
    let datapoints = poll_adcs(&mut adcs);

    println!("Sending data...");

    // send the adc data to the FC
    send_data(&data_socket, &fc_address, datapoints);
  }
}

// make sure you keep track of these UdpSockets, and pass them into the correct
// functions. Left is data, right is command.
fn establish_flight_computer_connection() -> (UdpSocket, UdpSocket, SocketAddr) {
  // area in memory where the flight computer handshake response should be stored
  let mut buf: [u8; 10240] = [0; 10240];

  // create the socket where all the data is send from
  let data_socket = UdpSocket::bind(("0.0.0.0", 4573))
    .expect("Could not open data socket.");

  // create the socket where all the commands are recieved from
  let command_socket = UdpSocket::bind(("0.0.0.0", COMMAND_PORT))
    .expect("Could not open command socket.");

  // make it so the CPU doesn't wait for messages to be recieved
  command_socket.set_nonblocking(true)
    .expect("Could not set command socket to nonblocking");

  // look for the flight computer based on it's dynamic IP
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
  
  // Create the BMS handshake message
  // It lets the flight computer know know what board type and number this device is.
  let identity = DataMessage::Identity(BMS_ID.to_string());

  // Allocate memory to store the BMS handshake message in 
  let packet = postcard::to_allocvec(&identity)
    .expect("Could not create identity message send buffer");

  loop {
    // Try to send the BMS handshake to the flight computer.
    // If this panics, it means that the BMS couldn't send the handshake at all.
    let size = data_socket.send_to(&packet, fc_address)
      .expect("Could not send Identity message");

    println!("Sent identity of size {size}");

    // Check if the FC has responded with its own handshake message. If so,
    // convert it from raw bytes to a DataMessage enum
    let result = match data_socket.recv_from(&mut buf) {
      Ok((size, _)) =>
        postcard::from_bytes::<DataMessage>(&buf[..size])
          .expect("Could not deserialize recieved message"),
      Err(e) => {
        println!("Failed to recieve FC heartbeat: {e}. Retrying...");
        continue;
      }
    };

    match result {
      // If the Identity message was recieved correctly.
      DataMessage::Identity(id) => {
        println!("Connection established with FC ({id})");

        data_socket.set_nonblocking(true)
          .expect("Could not set data socket to nonblocking");
        
        return (data_socket, command_socket, fc_address)
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

fn send_data(socket: &UdpSocket, address: &SocketAddr, datapoints: Vec<DataPoint>) {
  // create a buffer to store the data to send in
  let mut buffer: [u8; 65536] = [0; 65536];

  // get the data and store it in the buffer
  let data = DataMessage::Bms(BMS_ID.to_string(), Cow::Owned(datapoints));
  let seralized = match postcard::to_slice(&data, &mut buffer) {
    Ok(slice) => {
      pass!("Sliced data.");
      slice
    },
    Err(e) => {
      warn!("Could not serialize buffer ({e}), continuing...");
      return;
    }
  };
  
  // send the data to the FC
  match socket.send_to(seralized, address) {
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
  // create a location to store the heartbeat recieved from the FC
  let mut buffer: [u8; 256] = [0; 256];

  // check if we have exceeded the heartbeat timer
  let delta = Instant::now() - timer;
  if delta > HEARTBEAT_TIME_LIMIT {
    abort(gpio_controllers);
  }

  // get data from the socket and insert into buffer
  let size = match socket.recv_from(&mut buffer) {
    Ok((size, _)) => size,
    Err(_) => {
      return timer;
    }
  };

  // convert the recieved data into a DataMessage
  let message = match postcard::from_bytes::<DataMessage>(&buffer[..size]) {
    Ok(message) => message,
    Err(e) => {
      warn!("Could not deserialize data from FC ({e}), continuing...");
      return timer;
    }
  };

  match message {
    // if the message was a Heartbeat, reset the timer
    DataMessage::FlightHeartbeat => Instant::now(),
    _ => {
      // if not, keep the timer going
      warn!("Expected Flight Heartbeat was not detected.");
      timer
    }
  }
}

fn check_and_execute(gpio_controllers: &[Gpio], command_socket: &UdpSocket) {
  // where to store the command recieved from the FC
  let mut buf: [u8; 10240] = [0; 10240];

  // check if we got a command from the FC
  let size = match command_socket.recv_from(&mut buf) {
    Ok((size, _)) => size,
    Err(_) => return,
  };

  // Convert the recieved data into a SamControlMessage
  let command = match postcard::from_bytes::<SamControlMessage>(&buf[..size]) {
    Ok(command) => command,
    Err(e) => {
      fail!("Command was recieved but could not be deserialized ({e}).");
      return;
    }
  };

  pass!("Executing command...");
  
  // execute the command
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