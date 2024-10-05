use std::collections::HashMap;
use crate::command;
use spidev::{SpiModeFlags, Spidev, SpidevOptions};
use ads114s06::{ADC, Channel};
use common::comm::ADCKind;

use common::comm::{
  Gpio,
  Pin,
  PinMode::{Input, Output},
  PinValue::{High, Low},
};
use common::comm::ADCKind::{VBatUmbCharge, SamAnd5V};

pub fn init_gpio(gpio_controllers: &[Gpio]) {
  // set battery enable low
  // set sam enable low (disable)
  // set charge enable low (disable)
  // set estop reset low
  command::disable_battery_power(gpio_controllers);
  command::disable_sam_power(gpio_controllers);
  command::disable_charger(gpio_controllers);
  command::estop_init(gpio_controllers);

  for chip_select_pin in get_chip_select_mappings(gpio_controllers).values_mut() {
    chip_select_pin.digital_write(High); // active low
  }
}

fn init_adcs(gpio_controllers: &[Gpio]) {
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

  for adc in adcs {
    adc.cs_pin.digital_write(Low); // select ADC (active low)

    // positive input channel initial mux
    match adc.kind {
      VBatUmbCharge => adc.set_positive_input_channel(Channel::AIN0);

      SamAnd5V => adc.set_positive_input_channel(Channel::AIN2);
    }
    // negative channel input mux (does not change)
    adc.set_negative_input_channel(Channel::AINCOM);

    // pga register (same as SAM)
    adc.set_programmable_conversion_delay(14);
    adc.set_pga_gain(1);
    adc.disable_pga();
    // datarate register (same as SAM)
    adc.disable_global_chop();
    adc.enable_internal_clock_disable_external();
    adc.enable_continious_conversion_mode();
    adc.enable_low_latency_filter();
    adc.set_data_rate(4000);
    // ref register (same as SAM)
    adc.enable_positive_reference_buffer();
    adc.disable_negative_reference_buffer();
    adc.set_ref_input_internal_2v5_ref();
    adc.enable_internal_voltage_reference_on_pwr_down();
    // idacmag register
    adc.open_low_side_pwr_switch();
    adc.set_idac_magnitude(0);
    // idacmux register
    adc.disable_idac1();
    adc.disable_idac2();
    // vbias register
    adc.disable_vbias();
    // system monitor register
    adc.disable_system_monitoring();
    // initiate single shot mode
    adc.spi_start_conversion();

    adc.cs_pin.digital_write(High); // deselect ADC (active low)
  }
}

fn poll_adcs(adcs: Vec<ADC>) {
  for i in 0..6 {
    for adc in adcs {
      let reached_max_vbat_umb_charge = adc.kind == VBatUmbCharge && i > 4;
      let reached_max_sam_and_5v = adc.kind == SamAnd5V && i < 2;
      if (reached_max_vbat_umb_charge || reached_max_sam_and_5v) return;
      adc.cs_pin.digital_write(Low); // active Low

      // poll for data ready
      loop {
        let drdy_val = adc.drdy_pin.digital_read();
        if drdy_val == Low {
          break;
        }
      }

      // call spi read data
      let raw_code = match adc.spi_read_data() {
        Ok(data) => data,
        Err(e) => {
          eprintln!("Err Reading ADC data on channel {}: {:#?}", i, e);
          i16::MIN // gotta double check this
        }
      }
      // do shit with data
      data = adc.calculate_differential_measurement(code)

      // pin mux
      match adc.kind {
        
      }

      adc.cs_pin.digital_write(High); // active Low
    }
  }
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