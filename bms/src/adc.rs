use ads114s06::ADC;
use common::comm::{
  bms::{Bms, DataPoint},
  gpio::PinValue::Low,
  ADCKind::VespulaBms,
  VespulaBmsADC,
};
use jeflog::warn;
use std::f64::NAN;
use std::{
  thread::sleep,
  time::{Duration, Instant},
};

const ADC_DRDY_TIMEOUT: Duration = Duration::from_micros(1000);

pub fn init_adcs(adcs: &mut Vec<ADC>) {
  for adc in adcs.iter_mut() {
    print!("ADC {:?} regs (before init): [", adc.kind);
    for reg_value in adc.spi_read_all_regs().unwrap().iter() {
      print!("{:x} ", reg_value);
    }
    print!("]\n");

    // positive input channel initial mux
    if adc.kind == VespulaBms(VespulaBmsADC::VBatUmbCharge) {
      adc.set_positive_input_channel(0);
    } else if adc.kind == VespulaBms(VespulaBmsADC::SamAnd5V) {
      adc.set_positive_input_channel(2);
    } else {
      panic!("Imposter ADC among us!")
    }

    // negative channel input mux (does not change)
    adc.set_negative_input_channel_to_aincom();

    // pga register
    adc.set_programmable_conversion_delay(14);
    adc.set_pga_gain(1);
    adc.disable_pga();
    // datarate register
    adc.disable_global_chop();
    adc.enable_internal_clock_disable_external();
    adc.enable_continious_conversion_mode();
    adc.enable_low_latency_filter();
    adc.set_data_rate(4000.0);
    // ref register
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

    print!("ADC {:?} regs (after init): [", adc.kind);
    for reg_value in adc.spi_read_all_regs().unwrap().iter() {
      print!("{:x} ", reg_value);
    }
    print!("]\n");
  }
}

pub fn start_adcs(adcs: &mut Vec<ADC>) {
  for adc in adcs.iter_mut() {
    adc.spi_start_conversion(); // start continiously collecting data
  }
}

pub fn reset_adcs(adcs: &mut Vec<ADC>) {
  for adc in adcs.iter_mut() {
    adc.spi_stop_conversion(); // stop collecting data

    // reset back to first channel for when data collection resumes
    match adc.kind {
      VespulaBms(vespula_bms_adc) => match vespula_bms_adc {
        VespulaBmsADC::VBatUmbCharge => {
          adc.set_positive_input_channel(0);
        }

        VespulaBmsADC::SamAnd5V => {
          adc.set_positive_input_channel(2);
        }
      },

      _ => panic!("Imposter ADC among us!"),
    }
  }
}

pub fn poll_adcs(adcs: &mut Vec<ADC>) -> DataPoint {
  let mut bms_data = Bms::default();
  for channel in 0..6 {
    for (i, adc) in adcs.iter_mut().enumerate() {
      let reached_max_vbat_umb_charge =
        adc.kind == VespulaBms(VespulaBmsADC::VBatUmbCharge) && channel > 4;
      let reached_max_sam_and_5v =
        adc.kind == VespulaBms(VespulaBmsADC::SamAnd5V) && channel < 2;
      if reached_max_vbat_umb_charge || reached_max_sam_and_5v {
        continue;
      }

      // poll for data ready
      let time = Instant::now();
      let mut go_to_next_adc: bool = false;

      // make sure that this new version works
      loop {
        if let Some(pin_val) = adc.check_drdy() {
          if pin_val == Low {
            break;
          } else if Instant::now() - time > ADC_DRDY_TIMEOUT {
            warn!(
              "ADC {:?} drdy not pulled low... going to next ADC",
              adc.kind
            );
            go_to_next_adc = true;
            break;
          }
        } else {
          sleep(Duration::from_micros(700));
          break;
        }
      }

      if go_to_next_adc {
        continue; // cannot communicate with current ADC
      }

      let data = match adc.spi_read_data() {
        Ok(raw_code) => adc.calc_diff_measurement(raw_code),

        Err(e) => {
          eprintln!(
            "Err reading data on ADC {} channel {}: {:#?}",
            i, channel, e
          );
          NAN
        }
      };

      match adc.kind {
        VespulaBms(vespula_bms_adc) => {
          match vespula_bms_adc {
            VespulaBmsADC::VBatUmbCharge => {
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
              adc.set_positive_input_channel((channel + 1) % 5);
            }

            VespulaBmsADC::SamAnd5V => {
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
                adc.set_positive_input_channel(0);
              } else {
                adc.set_positive_input_channel(channel + 1);
              }
            }
          }
        }

        _ => panic!("Imposter ADC among us!"),
      }
    }
  }

  DataPoint {
    state: bms_data,
    timestamp: 0.0,
  }
}
