use ads114s06::ADC as ADC;
use common::comm::{
  igniter::{Igniter, DataPoint},
  ADCKind::IgniterRev1,
  IgniterRev1ADC,
  gpio::PinValue::Low,
  ADCFamily,
};
use jeflog::warn;
use std::{
  thread::sleep,
  time::{Duration, Instant},
};

const ADC_DRDY_TIMEOUT: Duration = Duration::from_micros(1000);

/// initialize adc registers for each ADC
pub fn init_adcs(adcs: &mut [ADC]) {
  for adc in adcs.iter_mut() {
    print!("ADC {:?} regs (before init): [", adc.kind());
    for reg_value in adc.spi_read_all_regs().unwrap().iter() {
      print!("{:x} ", reg_value);
    }
    println!("]");

    // negative channel input mux (does not change)
    adc.set_negative_input_channel_to_aincom().expect("Failed to set negative input channel to aincom");

    // all adcs have ain0 connected
    adc.set_positive_input_channel(0).expect("Failed to set positive input channel to ain0");

    // pga register
    adc.set_programmable_conversion_delay(14).expect("Failed to set programmable conversion delay");
    adc.set_pga_gain(1).expect("Failed to set pga gain");
    adc.disable_pga().expect("Failed to disable pga");
    // datarate register
    adc.disable_global_chop().expect("Failed to disable global chop");
    adc.enable_internal_clock_disable_external().expect("Failed to enable internal clock disable external");
    adc.enable_continious_conversion_mode().expect("Failed to enable continious conversion mode");
    adc.enable_low_latency_filter().expect("Failed to enable low latency filter");
    adc.set_data_rate(4000.0).expect("Failed to set data rate");
    // ref register
    adc.disable_reference_monitor().expect("Failed to disable reference monitor");
    adc.enable_positive_reference_buffer().expect("Failed to enable positive reference buffer");
    adc.disable_negative_reference_buffer().expect("Failed to disable negative reference buffer");
    adc.set_ref_input_internal_2v5_ref().expect("Failed to set ref input internal 2v5 ref");
    adc.enable_internal_voltage_reference_on_pwr_down().expect("Failed to enable internal voltage reference on pwr down");
    // idacmag register
    adc.disable_pga_output_monitoring().expect("Failed to disable pga output monitoring");
    adc.open_low_side_pwr_switch().expect("Failed to open low side pwr switch");
    adc.set_idac_magnitude(0).expect("Failed to set idac magnitude");
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

/// Start continuous data collection for each ADC
pub fn start_adcs(adcs: &mut [ADC]) {
  for adc in adcs.iter_mut() {
    let _ = adc.spi_start_conversion(); // start continiously collecting data
  }
}

/// Stop continuous data collection and reset each ADC to their initial state
pub fn reset_adcs(adcs: &mut [ADC]) {
  for adc in adcs.iter_mut() {
    let _ = adc.spi_stop_conversion(); // stop collecting data

    // reset back to first channel for when data collection resumes
    let _ = adc.set_positive_input_channel(0);
  }
}

// TODO: Update scaling factors for all channels on all adcs
/// Poll each ADC on connected channels and return collected data 
/// If the drdy isn't pulled low on an adc for ADC_DRDY_TIMEOUT, 
/// we skip that adc and move onto the next adc.
pub fn poll_adcs(adcs: &mut [ADC]) -> DataPoint {
  let mut igniter_data = Igniter::default();
  for channel in 0..6 {
    for (i, adc) in adcs.iter_mut().enumerate() {
      // poll for data ready, if not low (active low) for ADC_DRDY_TIMEOUT
      // move onto the next adc
      let time = Instant::now();
      let mut go_to_next_adc: bool = false;
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

      match adc.kind() {
        IgniterRev1(igniter_adc) => {
          match igniter_adc {
            IgniterRev1ADC::ConstantVoltage => {
              // constant voltage channels are 0-2, we separate cc and cv 
              // when collecting data. every constant voltage channel has 
              // data for voltage and current.
              match channel {
                0 => igniter_data.channels[0].voltage = data,
                1 => igniter_data.channels[1].voltage = data,
                2 => igniter_data.channels[2].voltage = data,
                3 => igniter_data.channels[0].current = data,
                4 => igniter_data.channels[1].current = data,
                5 => igniter_data.channels[2].current = data,
                _ => panic!("Invalid channel for ConstantVoltage"),
              }

              // muxing logic
              let _ = adc.set_positive_input_channel((channel + 1) % 6);
            },
            IgniterRev1ADC::ConstantCurrent => {
              // constant current channels are 3-5, we separate cc and cv 
              // when collecting data. every constant current channel has 
              // data for voltage and current.
              match channel {
                0 => igniter_data.channels[3].voltage = data,
                1 => igniter_data.channels[4].voltage = data,
                2 => igniter_data.channels[5].voltage = data,
                3 => igniter_data.channels[3].current = data,
                4 => igniter_data.channels[4].current = data,
                5 => igniter_data.channels[5].current = data,
                _ => panic!("Invalid channel for ConstantCurrent"),
              }

              // muxing logic
              let _ = adc.set_positive_input_channel((channel + 1) % 6);
            },
            IgniterRev1ADC::Continuity => {
              igniter_data.continuity[channel as usize] = data;

              // muxing logic
              let _ = adc.set_positive_input_channel((channel + 1) % 6);
            },
            IgniterRev1ADC::PowerMonitoring => {
              match channel {
                0 => igniter_data.p5v0_rail.voltage = data,
                1 => igniter_data.p5v0_rail.current = data,
                2 => igniter_data.config_rail.voltage = data,
                3 => igniter_data.config_rail.current = data,
                4 => igniter_data.p24v0_rail.voltage = data,
                5 => igniter_data.p24v0_rail.current = data,
                _ => panic!("Invalid channel for ConstantCurrent"),
              }

              // muxing logic
              let _ = adc.set_positive_input_channel((channel + 1) % 6);
            },
          }
        }

        _ => panic!("Imposter ADC among us!"),
      }
    }
  }

  DataPoint {
    state: igniter_data,
    timestamp: 0.0, // filled in right before we send data to FC
  }
}
