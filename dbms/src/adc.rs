use ads114s06::ADC;
use common::comm::{
  dbms::{Dbms, DataPoint},
  gpio::PinValue::Low,
};
use jeflog::warn;
use std::{
  thread::sleep,
  time::{Duration, Instant},
};

const ADC_DRDY_TIMEOUT: Duration = Duration::from_micros(1000);

pub fn init_adcs(adcs: &mut [ADC]) {
  for adc in adcs.iter_mut() {
    print!("ADC {:?} regs (before init): [", adc.kind);
    for reg_value in adc.spi_read_all_regs().unwrap().iter() {
      print!("{:x} ", reg_value);
    }
    println!("]");

    // positive input channel initial mux
    adc.set_positive_input_channel(2);

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
    adc.spi_start_conversion(); // start continiously collecting data
  }
}

pub fn reset_adcs(adcs: &mut [ADC]) {
  for adc in adcs.iter_mut() {
    adc.spi_stop_conversion(); // stop collecting data
    adc.set_positive_input_channel(2); // reset to first channel of adc
  }
}

pub fn poll_adcs(adcs: &mut [ADC]) -> DataPoint {
  let mut dbms_data = Dbms::default();

  for channel in 0..6 {
    for (i, adc) in adcs.iter_mut().enumerate() {
      if channel < 2 {
        continue; // skip channels 0 and 1
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
          f64::NAN
        }
      };

      match channel {
        2 => {
          // translate 28 V/V
          dbms_data.battery_bus.voltage = data * 28.0;
          println!("Battery bus voltage: {}", dbms_data.battery_bus.voltage);
        }
        3 => {
          // reverse 0.5 gain
          dbms_data.battery_bus.current = data * 2.0;
          println!("Battery bus current: {}", dbms_data.battery_bus.current);
        }
        4 => {
          // translate 11 V/V
          dbms_data.five_volt_rail.voltage = data * 11.0;
          println!("Five volt voltage: {}", dbms_data.five_volt_rail.voltage);
        }
        5 => {
          // reverse 0.5 gain
          dbms_data.five_volt_rail.current = data * 2.0;
          println!("Five volt current: {}", dbms_data.five_volt_rail.current);
        }
        _ => panic!("Invalid channel"),
      }

      // muxing logic
      if channel == 5 {
        adc.set_positive_input_channel(0); // go to channel 0 from 5
      } else {
        adc.set_positive_input_channel(channel + 1); // next channel
      }
    }
  }

  DataPoint {
    state: dbms_data,
    timestamp: 0.0,
  }
}
