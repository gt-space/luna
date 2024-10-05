pub fn init_gpio(gpio_controllers: &[Gpio]) {
  // set battery enable low
  // set sam enable low (disable)
  // set charge enable low (disable)
  // set estop reset low
  command::disable_battery_power(gpio_controllers);
  command::disable_sam_power(gpio_controllers);
  command::disable_charger(gpio_controllers);
  command::set_estop_low(gpio_controllers);

  for chip_select_pin in get_chip_select_mappings(gpio_controllers).values_mut() {
    chip_select_pin.digital_write(High); // active low
  }
}

pub fn get_cs_mappings(gpio_controllers: &[Gpio]) -> HashMap<ADCKind, Pin> {
  let mut vbat_umb_charge_chip_select: Pin = gpio_controllers[0].get_pin(30);
  vbat_umb_charge_chip_select.mode(Output);
  let mut sam_and_5v_chip_select: Pin = gpio_controllers[0].get_pin(31);
  sam_and_5v_chip_select.mode(Output);

  HashMap::from([(ADCKind::VBatUmbCharge, vbat_umb_charge_chip_select),
  (ADCKind::SamAnd5V, sam_and_5v_chip_select)])
}

pub fn get_drdy_mappings(gpio_controllers: &[Gpio]) -> HashMap<ADCKind, Pin> {
  let mut vbat_umb_charge_drdy: Pin = gpio_controllers[1].get_pin(28);
  vbat_umb_charge_drdy.mode(Input);
  let mut sam_and_5v_drdy: Pin = gpio_controllers[1].get_pin(18);
  sam_and_5v_drdy.mode(Input);

  HashMap::from([(ADCKind::VBatUmbCharge, vbat_umb_charge_drdy), 
  (ADCKind::SamAnd5V, sam_and_5v_drdy)])
}

/*
Creates an instance of the Spidev SPI Wrapper.
'bus' - A string that tells the spidev devices the provided path to open.
Typically, the path will be something like "/dev/spidev0.0" where the first
number is the SPI bus as seen on the schematic, SPI(X), and the second number
is the chip select number of that SPI line
 */
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