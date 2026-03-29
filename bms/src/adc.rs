use common::comm::{
  bms::{Bms, DataPoint},
  gpio::PinValue::Low,
  ADCKind::VespulaBms,
  VespulaBmsADC,
  ADCFamily
};
use crate::BMS_VERSION;
use crate::BmsVersion;
use jeflog::warn;
use std::{
  thread::sleep,
  time::{Duration, Instant},
};

const ADC_DRDY_TIMEOUT: Duration = Duration::from_micros(1000);

pub fn init_adcs(adcs: &mut [Box<dyn ADCFamily>]) {
  for adc in adcs.iter_mut() {
    print!("ADC {:?} regs (before init): [", adc.kind());
    for reg_value in adc.spi_read_all_regs().unwrap().iter() {
      print!("{:x} ", reg_value);
    }
    println!("]");

    // negative channel input mux (does not change)
    adc.set_negative_input_channel_to_aincom().expect("Failed to set negative input channel to aincom");

    // positive input channel initial mux
    match *BMS_VERSION {
      BmsVersion::Rev2 | BmsVersion::Rev3 => {
        if adc.kind() == VespulaBms(VespulaBmsADC::VBatUmbCharge) {
          adc.set_positive_input_channel(0).expect("Failed to set positive input channel to 0");
        } else if adc.kind() == VespulaBms(VespulaBmsADC::SamAnd5V) {
          adc.set_positive_input_channel(2).expect("Failed to set positive input channel to 2");
        } else {
          panic!(
            "unexpected ADC on BMS init (BMS {:?}): {:?}",
            *BMS_VERSION,
            adc.kind()
          )
        }
      }
      BmsVersion::Rev4 => {
        if adc.kind() == VespulaBms(VespulaBmsADC::VBatUmbCharge) 
          || adc.kind() == VespulaBms(VespulaBmsADC::RecoTelFCB) 
          || adc.kind() == VespulaBms(VespulaBmsADC::SamAnd5V) {
          adc.set_positive_input_channel(0).expect("Failed to set positive input channel to 0");
        } else {
          panic!(
            "unexpected ADC on BMS init (BMS {:?}): {:?}",
            *BMS_VERSION,
            adc.kind()
          )
        }
      }
    }

    // pga register
    adc.set_programmable_conversion_delay(14).expect("Failed to set programmable conversion delay to 14");
    adc.set_pga_gain(1).expect("Failed to set pga gain to 1");
    adc.disable_pga().expect("Failed to disable pga");
    // datarate register
    adc.disable_global_chop().expect("Failed to disable global chop");
    adc.enable_internal_clock_disable_external().expect("Failed to enable internal clock disable external");
    adc.enable_continious_conversion_mode().expect("Failed to enable continious conversion mode");
    adc.enable_low_latency_filter().expect("Failed to enable low latency filter");
    adc.set_data_rate(4000.0).expect("Failed to set data rate to 4000.0");
    // ref register
    adc.disable_reference_monitor().expect("Failed to disable reference monitor");
    adc.enable_positive_reference_buffer().expect("Failed to enable positive reference buffer");
    adc.disable_negative_reference_buffer().expect("Failed to disable negative reference buffer");
    adc.set_ref_input_internal_2v5_ref().expect("Failed to set ref input internal 2v5 ref");
    adc.enable_internal_voltage_reference_on_pwr_down().expect("Failed to enable internal voltage reference on pwr down");
    // idacmag register
    adc.disable_pga_output_monitoring().expect("Failed to disable pga output monitoring");
    adc.open_low_side_pwr_switch().expect("Failed to open low side pwr switch");
    adc.set_idac_magnitude(0).expect("Failed to set idac magnitude to 0");
    // idacmux register
    adc.disable_idac1().expect("Failed to disable idac1");
    adc.disable_idac2().expect("Failed to disable idac2");
    // vbias register
    adc.disable_vbias().expect("Failed to disable vbias");
    // system monitor register
    adc.disable_system_monitoring().expect("Failed to disable system monitoring");
    adc.disable_spi_timeout().expect("Failed to disable spi timeout");
    adc.disable_crc_byte().expect("Failed to disable crc byte");
    adc.disable_status_byte().expect("Failed to disable status byte");

    print!("ADC {:?} regs (after init): [", adc.kind());
    for reg_value in adc.spi_read_all_regs().unwrap().iter() {
      print!("{:x} ", reg_value);
    }
    println!("]");
  }
}

pub fn start_adcs(adcs: &mut [Box<dyn ADCFamily>]) {
  for adc in adcs.iter_mut() {
    let _ = adc.spi_start_conversion(); // start continiously collecting data
  }
}

pub fn reset_adcs(adcs: &mut [Box<dyn ADCFamily>]) {
  for adc in adcs.iter_mut() {
    let _ = adc.spi_stop_conversion(); // stop collecting data

    // reset back to first channel for when data collection resumes
    match *BMS_VERSION {
      BmsVersion::Rev2 | BmsVersion::Rev3 => {
        match adc.kind() {
          VespulaBms(vespula_bms_adc) => match vespula_bms_adc {
            VespulaBmsADC::VBatUmbCharge => {
              let _ = adc.set_positive_input_channel(0);
            },

            VespulaBmsADC::SamAnd5V => {
              let _ = adc.set_positive_input_channel(2);
            },

            unexpected => panic!(
              "unexpected VespulaBms ADC (BMS {:?}): {unexpected}",
              *BMS_VERSION
            ),
          },

          kind => panic!(
            "unexpected ADC kind on BMS reset (expected VespulaBms) (BMS {:?}): {kind:?}",
            *BMS_VERSION
          ),
        }
      },

      BmsVersion::Rev4 => {
        // all input channels are used on rev4
        let _ = adc.set_positive_input_channel(0);
      }
    }
  }
}

pub fn poll_adcs(adcs: &mut [Box<dyn ADCFamily>]) -> DataPoint {
  let mut bms_data = Bms::default();
  for channel in 0..6 {
    for (i, adc) in adcs.iter_mut().enumerate() {
      let reached_max_vbat_umb_charge = 
        match *BMS_VERSION {
          BmsVersion::Rev2 => {
            adc.kind() == VespulaBms(VespulaBmsADC::VBatUmbCharge) && channel > 4
          },
          // all input channels are used on rev3 and rev4
          BmsVersion::Rev3 | BmsVersion::Rev4 => {
            false
          },
        };

      let reached_max_sam_and_5v =
        match *BMS_VERSION {
          BmsVersion::Rev2 | BmsVersion::Rev3 => {
            adc.kind() == VespulaBms(VespulaBmsADC::SamAnd5V) && channel < 2
          },
          // all input channels are used on rev4
          BmsVersion::Rev4 => {
            false
          },
        };
      
      // not used on rev2 or rev3. on the rev4, all input channels are used.
      let reached_max_reco_tel_fcb = false;
        
      if reached_max_vbat_umb_charge 
        || reached_max_sam_and_5v 
        || reached_max_reco_tel_fcb {
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
              adc.kind()
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

      let data = match adc.read_counts() {
        Ok(raw_code) => adc.calc_diff_measurement(raw_code),

        Err(e) => {
          eprintln!(
            "Err reading data on ADC {} channel {}: {:#?}",
            i, channel, e
          );
          f64::NAN
        }
      };

      match *BMS_VERSION {
        BmsVersion::Rev2 | BmsVersion::Rev3 => {
          match adc.kind() {
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
                    bms_data.charger = (data - 0.25) / 0.30;
                  } else if channel == 5 {
                    bms_data.chassis = data * 22.5;
                  }

                  // muxing logic
                  let _ =adc.set_positive_input_channel((channel + 1) % 5);
                },

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
                    let _ = adc.set_positive_input_channel(0);
                  } else {
                    let _ =adc.set_positive_input_channel(channel + 1);
                  }
                },

                unexpected => panic!(
                  "unexpected VespulaBms ADC (BMS {:?}): {unexpected}",
                  *BMS_VERSION
                ),
              }
            },
            
            kind => panic!(
              "unexpected ADC kind on BMS poll (expected VespulaBms) (BMS {:?}): {kind:?}",
              *BMS_VERSION
            ),
          }
        },

        BmsVersion::Rev4 => {
          match adc.kind() {
            VespulaBms(vespula_bms_adc) => {
              match vespula_bms_adc {
                VespulaBmsADC::VBatUmbCharge => {
                  if channel == 0 {
                    bms_data.umbilical_bus.current = data / 0.5;
                  } else if channel == 1 {
                    bms_data.umbilical_bus.voltage = data * 22.5;
                  } else if channel == 2 {
                    bms_data.battery_bus.current = data / 0.5;
                  } else if channel == 3 {
                    bms_data.battery_bus.voltage = data * 22.5;
                  } else if channel == 4 {
                    bms_data.ethernet_bus.current = data / 0.9;
                  } else if channel == 5 {
                    bms_data.ethernet_bus.voltage = data * 22.5;
                  }

                  // muxing logic
                  let _ = adc.set_positive_input_channel((channel + 1) % 6);
                },

                VespulaBmsADC::RecoTelFCB => {
                  if channel == 0 {
                    bms_data.reco_load_switch_1 = data * 22.5;
                  } else if channel == 1 {
                    bms_data.reco_load_switch_2 = data * 22.5;
                  } else if channel == 2 {
                    bms_data.tel_bus.current = data / 0.9;
                  } else if channel == 3 {
                    bms_data.tel_bus.voltage = data * 22.5;
                  } else if channel == 4 {
                    bms_data.fcb_bus.current = data / 0.9;
                  } else if channel == 5 {
                    bms_data.fcb_bus.voltage = data * 22.5;
                  }

                  // muxing logic
                  let _ =adc.set_positive_input_channel((channel + 1) % 6);
                },

                VespulaBmsADC::SamAnd5V => {
                  if channel == 0 {
                    bms_data.sam_power_bus.current = data / 0.9;
                  } else if channel == 1 {
                    bms_data.sam_power_bus.voltage = data * 22.5;
                  } else if channel == 2 {
                    bms_data.five_volt_rail.current = data / 0.5;
                  } else if channel == 3 {
                    bms_data.five_volt_rail.voltage = data * 22.5;
                  } else if channel == 4 {
                    bms_data.charger = (data - 0.25) / 0.30;
                  } else if channel == 5 {
                    bms_data.chassis = data * 22.5;
                  }

                  // muxing logic
                  let _ = adc.set_positive_input_channel((channel + 1) % 6);
                },
              }
            },
            
            kind => panic!(
              "unexpected ADC kind on BMS poll (expected VespulaBms) (BMS {:?}): {kind:?}",
              *BMS_VERSION
            ),
          }
        }
      }
    }
  }

  DataPoint {
    state: bms_data,
    timestamp: 0.0,
  }
}
