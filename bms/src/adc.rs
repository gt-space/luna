use common::comm::{ChannelType, DataPoint, ADCKind, ADCKind::{VBatUmbCharge, SamAnd5V}, Gpio, PinValue::{Low, High}};
use ads114s06::ADC;

pub fn init_adcs(adcs: &mut Vec<ADC>) {
  for adc in adcs {
    adc.cs_pin.digital_write(Low); // select ADC (active low)

    // positive input channel initial mux

    if adc.kind == VBatUmbCharge {
      adc.set_positive_input_channel(0);
    } else {
      adc.set_positive_input_channel(2);
    }

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
    // initiate continious conversion mode
    adc.spi_start_conversion();

    adc.cs_pin.digital_write(High); // deselect ADC (active low)
  }
}

pub fn poll_adcs(adcs: &mut Vec<ADC>) -> Vec<DataPoint> {
  let mut datapoints = Vec::with_capacity(9);
  for channel in 0..6 {
    for adc in adcs.iter_mut() {
      let reached_max_vbat_umb_charge = adc.kind == VBatUmbCharge && channel > 4;
      let reached_max_sam_and_5v = adc.kind == SamAnd5V && channel < 2;
      if reached_max_vbat_umb_charge || reached_max_sam_and_5v {
        continue;
      }

      adc.cs_pin.digital_write(Low); // active Low

      // poll for data ready
      loop {
        let drdy_val = adc.drdy_pin.digital_read();
        if drdy_val == Low {
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

      // do shit with data
      let data = adc.calculate_differential_measurement(raw_code);

      if adc.kind == VBatUmbCharge {
        adc.set_positive_input_channel((channel + 1) % 5).ok();
      }

      if adc.kind == SamAnd5V {
        if channel == 5 {
          adc.set_positive_input_channel(0).ok();
        } else {
          adc.set_positive_input_channel(channel + 1).ok();
        }
      }

      // deselect ADC
      adc.cs_pin.digital_write(High); // active Low

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
        ChannelType::ValveVoltage
      }
    }
    // channel_type: ChannelType::RailVoltage,
  }
}