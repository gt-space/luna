pub mod gpio;
pub mod adc;
pub mod command;
pub mod state;
pub mod protocol;

use std::net::{UdpSocket, SocketAddr};
use jeflog;

const FC_ADDR: &str = "server-01";

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

fn establish_flight_computer_connection() {
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
    fc_addressip()
  );
  
  let socket = UdpSocket::bind()
}

fn init_adcs() {

}

fn poll_adcs() {
  
}