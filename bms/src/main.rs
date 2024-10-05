pub mod gpio;
pub mod adc;
pub mod command;
pub mod state;
pub mod protocol;

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

}

fn init_adcs() {

}

fn poll_adcs() {
  
}