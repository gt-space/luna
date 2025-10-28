use ads114s06::ADC;
use common::comm::{
    tcmod::{Tc, DataPoint},
    gpio::PinValue::Low,
};
use jeflog::warn;
use std::{thread::sleep, time::{Duration, Instant}};

const ADC_DRDY_TIMEOUT: Duration = Duration::from_micros(1000);

pub fn init_adcs(adcs: &mut [ADC]) {
  for adc in adcs.iter_mut() {
    print!("ADC {:?} regs (before init): [", adc.kind);
    for reg_value in adc.spi_read_all_regs().unwrap().iter() {
      print!("{:x} ", reg_value);
    }
    println!("]");
    

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
    println!("]");
  }
}

pub fn start_adcs(adcs: &mut [ADC]) {
    for adc in adcs.iter_mut() {
        adc.spi_start_conversion();
    }
}

pub fn reset_adcs(adcs: &mut [ADC]) {
    for adc in adcs.iter_mut() {
        adc.spi_stop_conversion();
        adc.set_positive_input_channel(0);
    }
}

pub fn poll_tc_adcs(adcs: &mut [ADC]) -> DataPoint {
    let mut tc_data = Tc::default();

    for iteration in 0..2 {
        //iterate banks 0–2
        for (bank_idx, adc) in adcs.iter_mut().enumerate() {
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
                continue;
            }

            // read/convert raw sample to temperature
            let data = match adc.spi_read_data() {
                Ok(code) => adc.calc_diff_measurement(code),
                Err(e) => {
                    eprintln!(
                        "Error reading ADC bank {} ch {}: {:#?}",
                        bank_idx, channel, e
                    );
                    f64::NAN
                }
            };

            tc_data.temperatures[bank_idx][channel] = data;

            let next_ch = if channel == 2 { 0 } else { channel + 1 };
            adc.set_positive_input_channel(next_ch);
        }
    }

    DataPoint {
        state: tc_data,
        timestamp: 0.0, 
    }
}
