use crate::gpio::{PinMode::Output, PinValue::Low};
use crate::{
  adc::{
    self,
    data_ready_mappings,
    gpio_controller_mappings,
    pull_gpios_high,
    ADCEnum,
    ADC,
  },
  data::{generate_data_point, serialize_data},
  gpio::Gpio,
};
use common::comm::{DataMessage, DataPoint};
use hostname;
use jeflog::{fail, pass, task, warn};
use spidev::{SpiModeFlags, Spidev, SpidevOptions};
use std::net::ToSocketAddrs;
use std::rc::Rc;
use std::{
  net::{SocketAddr, UdpSocket},
  sync::Arc,
  thread,
  time::Instant,
};
use std::{io, fs};

const FC_ADDR: &str = "server-01";

const FC_HEARTBEAT_TIMEOUT: u128 = 500;

const PATH_3V3: &str = r"/sys/bus/iio/devices/iio:device0/in_voltage0_raw";
const PATH_5V: &str = r"/sys/bus/iio/devices/iio:device0/in_voltage1_raw";
const PATH_5I: &str = r"/sys/bus/iio/devices/iio:device0/in_voltage2_raw";
const PATH_24V: &str = r"/sys/bus/iio/devices/iio:device0/in_voltage3_raw";
const PATH_24I: &str = r"/sys/bus/iio/devices/iio:device0/in_voltage4_raw";
const RAIL_PATHS: [&str; 5] = [PATH_3V3, PATH_5V, PATH_5I, PATH_24V, PATH_24I];


pub struct Data {
  pub data_socket: UdpSocket,
  flight_computer: Option<SocketAddr>,
  adcs: Option<Vec<adc::ADCEnum>>,
  state_num: u32,
  curr_measurement: Option<adc::Measurement>,
  data_points: Vec<DataPoint>,
  board_id: Option<String>,
  gpio_controllers: Vec<Arc<Gpio>>,
}

impl Data {
  pub fn new(gpio_controllers: Vec<Arc<Gpio>>) -> Data {
    Data {
      data_socket: UdpSocket::bind(("0.0.0.0", 4573))
        .expect("Could not bind client socket"),
      flight_computer: None,
      adcs: None,
      state_num: 0,
      curr_measurement: None,
      data_points: Vec::with_capacity(60),
      board_id: None,
      gpio_controllers,
    }
  }
}

#[derive(PartialEq, Debug)]
pub enum State {
  Init,
  DeviceDiscovery,
  Identity,
  InitAdcs,
  PollAdcs,
}

impl State {
  pub fn next(self, data: &mut Data) -> State {
    if data.state_num % 100000 == 0 {
      println!("{:?} {}", self, data.state_num);
    }
    data.state_num += 1;

    match self {
      State::Init => {
        /* Create a spidev wrapper to work with
        you call this wrapper to handle and all transfers */
        // let mut spidev = Spidev::open("/dev/spidev0.0").unwrap();

        // let options = SpidevOptions::new()
        //   .bits_per_word(8)
        //   .max_speed_hz(10_000_000)
        //   .lsb_first(false)
        //   .mode(SpiModeFlags::SPI_MODE_1)
        //   .build();
        // spidev.configure(&options).unwrap();
        let spi0 = create_spi("/dev/spidev0.0").unwrap();
        let spi1 = create_spi("/dev/spidev1.0").unwrap();

        //let ref_spidev: Rc<_> = Rc::new(spidev);
        let ref_spi0: Rc<_> = Rc::new(spi0);
        let ref_spi1: Rc<_> = Rc::new(spi1);
        let ref_controllers =
          Rc::new(gpio_controller_mappings(&data.gpio_controllers));
        let ref_drdy = Rc::new(data_ready_mappings(&data.gpio_controllers));

        // Instantiate all measurement types
        // spi1 = current loops, differenital sensors
        // spi2 = valve voltage, valve current, rtd
        let ds = ADCEnum::ADC(ADC::new(
            adc::Measurement::DiffSensors,
            ref_spi1.clone(),
            ref_controllers.clone(),
            ref_drdy.clone(),
        ));
        let cl = ADCEnum::ADC(ADC::new(
          adc::Measurement::CurrentLoopPt,
          ref_spi1.clone(),
          ref_controllers.clone(),
          ref_drdy.clone(),
        ));
        let vvalve = ADCEnum::ADC(ADC::new(
          adc::Measurement::VValve,
          ref_spi0.clone(),
          ref_controllers.clone(),
          ref_drdy.clone(),
        ));
        let ivalve = ADCEnum::ADC(ADC::new(
          adc::Measurement::IValve,
          ref_spi0.clone(),
          ref_controllers.clone(),
          ref_drdy.clone(),
        ));
        let rtd1 = ADCEnum::ADC(ADC::new(
          adc::Measurement::Rtd,
          ref_spi0.clone(),
          ref_controllers.clone(),
          ref_drdy.clone(),
        ));
        let rtd2 = ADCEnum::ADC(ADC::new(
          adc::Measurement::Rtd,
          ref_spi0.clone(),
          ref_controllers.clone(),
          ref_drdy.clone(),
        ));
        let rtd3 = ADCEnum::ADC(ADC::new(
          adc::Measurement::Rtd,
          ref_spi0.clone(),
          ref_controllers.clone(),
          ref_drdy.clone(),
        ));

        let pwr = ADCEnum::OnboardADC;
 
        pull_gpios_high(&data.gpio_controllers);

        data.adcs = Some(vec![
          ds,
          cl,
          // vvalve,
          ivalve,
          pwr
        ]);

        data
          .data_socket
          .set_nonblocking(true)
          .expect("set_nonblocking call failed");
        data.board_id = get_board_id();

        State::DeviceDiscovery
      }

      State::DeviceDiscovery => {
        task!("Locating the flight computer.");

        let address = format!("{}.local:4573", FC_ADDR)
          .to_socket_addrs()
          .ok()
          .and_then(|mut addrs| addrs.find(|addr| addr.is_ipv4()));

        let Some(address) = address else {
          fail!("Target \x1b[1m{}\x1b[0m could not be located.", FC_ADDR);
          return State::DeviceDiscovery;
        };

        pass!(
          "Target \x1b[1m{}\x1b[0m located at \x1b[1m{}\x1b[0m.",
          FC_ADDR,
          address.ip()
        );
        data.flight_computer = Some(address);

        State::InitAdcs
      }

      State::Identity => {
        let mut buf = [0; 65536];

        if let Some(board_id) = data.board_id.clone() {
          let identity = DataMessage::Identity(board_id);
          let data_serialized = postcard::to_allocvec(&identity);

          if let Some(socket_addr) = data.flight_computer {
            data
              .data_socket
              .send_to(&data_serialized.unwrap(), socket_addr)
              .expect("Could not send Identity message.");
          } else {
            fail!("Could not send Identity message.");
          }
        } else {
          fail!("Could not send Identity message, invalid board information.");
        }

        if let Ok((num_bytes, _)) = data.data_socket.recv_from(&mut buf) {
          let deserialized_result =
            postcard::from_bytes::<DataMessage>(&buf[..num_bytes]);
          println!("{:#?}", deserialized_result);
          match deserialized_result {
            Ok(message) => {
              match message {
                // FC sends identity back
                DataMessage::Identity(_) => {
                  pass!("Received Identity message from the flight computer, monitoring heartbeat");

                  let socket_copy = data.data_socket.try_clone();
                  let controllers = data.gpio_controllers.clone();

                  // Spawn heartbeat thread
                  thread::spawn(move || {
                    monitor_heartbeat(socket_copy.ok().unwrap(), &controllers);
                  });

                  return State::PollAdcs;
                }
                _ => {
                  warn!("Received unexpected message from the flight computer");
                  return State::Identity;
                }
              }
            }
            Err(_error) => {
              fail!("Bad message from flight computer");
              return State::Identity;
            }
          };
        };

        State::Identity
      }

      State::InitAdcs => {
        for adc_enum in data.adcs.as_mut().unwrap() {
          match adc_enum {
            ADCEnum::ADC(adc) => {
              adc.pull_cs_low_active_low(); // select current ADC
              // Does below line do anything?
              data.curr_measurement = Some(adc.measurement); // see if we can remove option
              adc.reset_status(); // reset registers
              adc.init_regs(); // measurement specific register initialization
              adc.start_conversion(); // begin converting in single shot mode
              adc.write_iteration(0); // pin mux to be on first channel
              adc.pull_cs_high_active_low(); // deselect current ADC
            },

            ADCEnum::OnboardADC => {
              // nothing to initialize because it is on Beaglebone
            }
          }
        }

        pass!("Initialized ADCs");
        State::Identity
      }

      State::PollAdcs => {
        /*
        For each iteration of PollAdcs the the data_points vector will hold
        one value from each channel of each ADC, thus we clear it at the start
        to just have data from one iteration
         */
        data.data_points.clear();
        /*
        Going from 0 to 5 inclusive is the maximum number of channels or
        readings we can get from an ADC. If the current ADC has less, we simply
        skip that channel and go to the next ADC
         */
        for i in 0..6 {
          for adc_enum in data.adcs.as_mut().unwrap() {
            let (raw_value, unix_timestamp, measurement) = match adc_enum {
              ADCEnum::ADC(adc) => {
                let diff_reached_max_channel = i > 2 && adc.measurement == adc::Measurement::DiffSensors;
                let rtd_reached_max_channel = i > 1 && adc.measurement == adc::Measurement::Rtd;
                // skip to next ADC logic
                if diff_reached_max_channel || rtd_reached_max_channel {
                  continue;
                }

                adc.pull_cs_low_active_low(); // select current ADC
                data.curr_measurement = Some(adc.measurement); // set measurement of current data struct
                let (val, time) = adc.get_adc_reading(i); // get data and time
                adc.write_iteration(i + 1); // perform pin mux to next channel or reading
                adc.pull_cs_high_active_low(); // deselect current ADC
                (val, time, adc.measurement)
              },

              ADCEnum::OnboardADC => {
                if i > 4 {
                  continue;
                }

                let (val, rail_measurement) = read_onboard_adc(i);
                data.curr_measurement = Some(rail_measurement);
                (val, 0.0, rail_measurement)
              }
            };

            let data_point = generate_data_point(raw_value, unix_timestamp, i, measurement);
            data.data_points.push(data_point);
          }
        }

        // this block of code sends data to flight computer
        if let Some(board_id) = data.board_id.clone() {
          let serialized = serialize_data(board_id, &data.data_points);

          if let Some(socket_addr) = data.flight_computer {
            data
              .data_socket
              .send_to(&serialized.unwrap(), socket_addr)
              .expect("couldn't send data to flight computer");
          }
        }
        State::PollAdcs
      }
    }
  }
}

fn monitor_heartbeat(socket: UdpSocket, gpio_controllers: &[Arc<Gpio>]) {
  let mut buf = [0; 65536];
  let mut last_heartbeat = Instant::now();

  loop {
    let curr_time = Instant::now();
    let time_elapsed = curr_time.duration_since(last_heartbeat).as_millis();

    if time_elapsed > FC_HEARTBEAT_TIMEOUT {
      // Abort system if loss of comms detected
      break;
    }

    if let Ok((num_bytes, _)) = socket.recv_from(&mut buf) {
      let deserialized_result =
        postcard::from_bytes::<DataMessage>(&buf[..num_bytes]);

      if let Ok(message) = deserialized_result {
        if message == DataMessage::FlightHeartbeat {
          last_heartbeat = Instant::now();
        }
      } else {
        fail!("Failed to deserialize DataMessage from flight computer.");
      }
    }
  }

  abort(gpio_controllers);
}

fn abort(controllers: &[Arc<Gpio>]) {
  fail!("Aborting the SAM Board.");
  warn!("You must manually restart SAM software.");

  let pins = [
    controllers[0].get_pin(8),  // valve 1
    controllers[2].get_pin(16), // valve 2
    controllers[2].get_pin(17), // valve 3
    controllers[2].get_pin(25), // valve 4
    controllers[2].get_pin(1),  // valve 5
    controllers[1].get_pin(14), // valve 6
  ];

  for pin in pins.iter() {
    pin.mode(Output);
    pin.digital_write(Low);
  }
}

fn get_board_id() -> Option<String> {
  match hostname::get() {
    Ok(hostname) => {
      let name = hostname.to_string_lossy().to_string();
      Some(name)
    }
    Err(e) => {
      fail!("Error getting board ID for Establish message: {}", e);
      None
    }
  }
}

/// Creates and instance of the Spidev SPI Wrapper
/// 
/// 'bus' - A string that tells the spidev device the provided path
/// to open
/// 
/// Typically, the path will be something like `"/dev/spidev0.0"`
/// where the first number if the bus and the second number
/// is the chip select on that bus for the device being targeted.
fn create_spi(bus: &str) -> io::Result<Spidev> {
  let mut spi = Spidev::open(bus)?;
  let options = SpidevOptions::new()
      .bits_per_word(8)
      .max_speed_hz(10_000_000)
      .lsb_first(false)
      .mode(SpiModeFlags::SPI_MODE_1)
      .build();
  spi.configure(&options)?;
  Ok(spi)
}

// pinout is for flight version
pub fn read_onboard_adc(channel: u64) -> (f64, adc::Measurement) {
  let data = match fs::read_to_string(RAIL_PATHS[channel as usize]) {
    Ok(output) => output,
    Err(_e) => {
      eprintln!("Fail to read {}", RAIL_PATHS[channel as usize]);
      if channel == 0 || channel == 1 || channel == 3 {
        return (-1.0, adc::Measurement::VPower)
      } else {
        return (-1.0, adc::Measurement::IPower)
      }
    }
  };

  if data.is_empty() {
    eprintln!("Empty data for on board ADC channel {}", channel);
    if channel == 0 || channel == 1 || channel == 3 {
      return (-1.0, adc::Measurement::VPower)
    } else {
      return (-1.0, adc::Measurement::IPower)
    }
  }

  match data.trim().parse::<f64>() {
    Ok(data) => {
      let voltage = 1.8 * (data as f64) / ((1 << 12) as f64);
      if channel == 0 || channel == 1 || channel == 3 {
        ((voltage * (4700.0 + 100000.0) / 4700.0), adc::Measurement::VPower)
      } else {
        (voltage, adc::Measurement::VPower)
      }
    },

    Err(_e) => {
      eprintln!("Fail to convert from String to f64 for onboard ADC channel {}", channel);
      if channel == 0 || channel == 1 || channel == 3 {
        (-1.0, adc::Measurement::VPower)
      } else {
        (-1.0, adc::Measurement::IPower)
      }
    }
  }
}
