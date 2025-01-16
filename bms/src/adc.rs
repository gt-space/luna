use std::time::{Instant, Duration};
use common::comm::{bms::{Bms, DataPoint}, ADCKind::{VBatUmbCharge, SamAnd5V}, gpio::PinValue::Low};
use ads114s06::ADC;
use std::f64::NAN;

pub fn init_adcs(adcs: &mut Vec<ADC>) {
  for (i, adc) in adcs.into_iter().enumerate() {
    adc.spi_reset(); // initalize registers to default values first

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

    println!("ADC{} regs (after init)", i + 1);
    for (reg, reg_value) in
      adc.spi_read_all_regs().unwrap().into_iter().enumerate()
    {
      println!("Reg {:x}: {:08b}", reg, reg_value);
    }

    // initiate continious conversion mode
    adc.spi_start_conversion();
  }
}

pub fn poll_adcs(adcs: &mut Vec<ADC>) -> DataPoint {
  let mut bms_data = Bms::default();
  for channel in 0..6 {
    for (i, adc) in adcs.iter_mut().enumerate() {
      let reached_max_vbat_umb_charge =
        adc.kind == VBatUmbCharge && channel > 4;
      let reached_max_sam_and_5v = adc.kind == SamAnd5V && channel < 2;
      if reached_max_vbat_umb_charge || reached_max_sam_and_5v {
        continue;
      }

      // poll for data ready
      let time = Instant::now();
      let mut go_to_next_adc: bool = false;
      loop {
        if adc.check_drdy() == Low {
          break;
        } else if Instant::now() - time > Duration::from_millis(250) {
          eprintln!("ADC {} drdy not pulled low... going to next ADC", i);
          go_to_next_adc = true;
          break;
        }
      }

      if go_to_next_adc {
        continue;
      }

      let data = match adc.spi_read_data() {
        Ok(raw_code) => {
          adc.calculate_differential_measurement(raw_code)
        },

        Err(e) => {
          eprintln!("Err reading data on ADC {} channel {}: {:#?}", i, channel, e);
          NAN
        }
      };

      match adc.kind {
        VBatUmbCharge => {
          if channel == 0 {
            bms_data.battery_bus.current = data * 2.0;
          } else if channel == 1 {
            bms_data.battery_bus.voltage = data * 22.5;
          } else if channel == 2 {
            bms_data.umbilical_bus.current = data * 2.0;
          } else if channel == 3 {
            bms_data.umbilical_bus.voltage = data * 22.5;
          } else if channel == 4 {
            // charger current sense
            bms_data.charger = (data - 0.25) / 0.15;
          }

          // muxing logic
          adc.set_positive_input_channel((channel + 1) % 5).unwrap();
        },

        SamAnd5V => {
          if channel == 2 {
            bms_data.sam_power_bus.current = data * 2.0;
          } else if channel == 3 {
            bms_data.sam_power_bus.voltage = data * 22.5;
          } else if channel == 4 {
            bms_data.five_volt_rail.voltage = data * 22.5;
          } else if channel == 5 {
            bms_data.five_volt_rail.current = data * 2.0;
          }

          // muxing logic
          if channel == 5 {
            adc.set_positive_input_channel(0).unwrap();
          } else {
            adc.set_positive_input_channel(channel + 1).unwrap();
          }
        },

        _ => panic!("Imposter ADC among us!")
      }
    }
  }

  DataPoint {
    state: bms_data,
    timestamp: 0.0,
  }
}
