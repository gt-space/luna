// modifying copy of Vespula BMS code

use common::comm::{ChannelType, DataPoint, ADCKind, ADCKind::{VBatUmbCharge, SamAnd5V}, Gpio, PinValue::{Low, High}, PinMode::{Input, Output}};
use ads114s06::ADC;

pub fn init_adcs(adcs: &mut Vec<ADC>) {
  for (i, adc) in adcs.into_iter().enumerate() {
    adc.spi_reset(); // initalize registers to default values first

    // positive input channel initial mux
    adc.set_positive_input_channel(0);

    // negative channel input mux (does not change)
    adc.set_negative_input_channel_to_aincom();

    // pga register (same as SAM)
    adc.set_programmable_conversion_delay(14);
    adc.set_pga_gain(1);
    adc.disable_pga();
    // datarate register (same as SAM)
    adc.disable_global_chop();
    adc.enable_internal_clock_disable_external();
    adc.enable_continious_conversion_mode();
    adc.enable_low_latency_filter();
    adc.set_data_rate(4000.0);
    // ref register (same as SAM)
    adc.disable_reference_monitor();
    adc.enable_positive_reference_buffer();
    adc.disable_negative_reference_buffer();
    adc.set_ref_input_internal_2v5_ref();
    adc.enable_internal_voltage_reference_on_pwr_down();
    // idacmag register
    adc.disable_pga_output_monitoring();
    adc.open_low_side_pwr_switch();
    adc.set_idac_magnitude(0);
    // idacmux register
    adc.disable_idac1();
    adc.disable_idac2();
    // vbias register
    adc.disable_vbias();
    // system monitor register
    adc.disable_system_monitoring();
    adc.disable_spi_timeout();
    adc.disable_crc_byte();
    adc.disable_status_byte();

    if adc.kind == VBatUmbCharge {
      adc.set_gpio_mode(0, Output);
      adc.gpio_digital_write(0, Low);
      adc.set_gpio_mode(1, Input);
    }

    println!("ADC{} regs (after init)", i + 1);
    for (reg, reg_value) in adc.spi_read_all_regs().unwrap().into_iter().enumerate() {
      println!("Reg {:x}: {:08b}", reg, reg_value);
    }

    // initiate continious conversion mode
    adc.spi_start_conversion();

  }
}

pub fn poll_adcs(adcs: &mut Vec<ADC>) -> Vec<DataPoint> {
  // sampling 4 channels
  let mut datapoints = Vec::with_capacity(4);
  for channel in 0..6 {
    for (i, adc) in adcs.iter_mut().enumerate() {
      // channels 1 and 2 are not used
      if adc.kind == VBatUmbCharge && (channel == 1 || channel == 2) {
        continue;
      }

      // poll for data ready
      loop {
        if adc.check_drdy() == Low {
          break;
        }
      }

      let raw_code = match adc.spi_read_data() {
        Ok(data) => data,
        Err(e) => {
          eprintln!("Err Reading ADC data on channel {}: {:#?}", channel, e);
          -999
        }
      };

      // Converting ADC code to actual value based on BMS schematic

      let mut data = adc.calculate_differential_measurement(raw_code);

      // invert voltage divider
      if adc.kind == VBatUmbCharge && (channel == 0 || channel == 5) {
        data *= 14.0;
      }

      // High side drive current sense
      if adc.kind == VBatUmbCharge && channel == 3 {
        data /= 750.0; // shunt resistor connected to SNS pin
        data *= 1200.0; // ratio of Isns / Iload = 1 / 1200
      }

      // invert shunt resistor and current sense amplifier
      if adc.kind == VBatUmbCharge && channel == 4 {
        data *= 2.0;
      }

      // Next channel logic and checking load switch fault status
      if adc.kind == VBatUmbCharge {
        adc.set_positive_input_channel((channel + 1) % 5).ok();
        if adc.gpio_digital_read(0) == High {
          adc.gpio_digital_write(1, High);
          adc.gpio_digital_write(1, Low);
        }
      }

      let data_point: DataPoint = generate_data_point(data, 0.0, channel, adc.kind);
      datapoints.push(data_point);
    }
  }
  
  datapoints
}

pub fn generate_data_point(data: f64, timestamp: f64, iteration: u8, kind: ADCKind) -> DataPoint {
  DataPoint {
    value: data,
    timestamp: timestamp,
    channel: (iteration + 1) as u32,
    channel_type: {
      if kind == VBatUmbCharge {
        ChannelType::RailVoltage
      } else {
        ChannelType::CurrentLoop
      }
    }
    // channel_type: ChannelType::RailVoltage,
  }
}