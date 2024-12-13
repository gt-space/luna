use std::{thread, time::{Duration, Instant}};
use common::comm::{gpio::PinValue::Low, sam::{ChannelType, DataPoint}, ADCKind::{self, SamRev3, SamRev4Gnd, SamRev4Flight}, SamRev3ADC, SamRev4GndADC, SamRev4FlightADC};
use ads114s06::ADC;

use crate::{SAM_VERSION, SamVersion};

pub fn init_adcs(adcs: &mut Vec<ADC>) {
  for (i, adc) in adcs.iter_mut().enumerate() {

    // mux register
    adc.set_positive_input_channel(0); // change where needed
    adc.set_negative_input_channel_to_aincom(); // change where needed

    // pga register (same as SAM)
    adc.set_programmable_conversion_delay(14);
    adc.set_pga_gain(1); // change where needed
    adc.disable_pga(); // change where needed

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
    adc.set_ref_input_internal_2v5_ref(); // change for RTDs
    adc.enable_internal_voltage_reference_on_pwr_down();

    // idacmag register
    adc.disable_pga_output_monitoring();
    adc.open_low_side_pwr_switch();
    adc.set_idac_magnitude(0); // change for RTDs

    // idacmux register
    adc.disable_idac1(); // change for RTD
    adc.disable_idac2(); // change for RTD

    // vbias register
    adc.disable_vbias();

    // system monitor register
    adc.disable_system_monitoring();
    adc.disable_spi_timeout();
    adc.disable_crc_byte();
    adc.disable_status_byte();

    match adc.kind {
      SamRev3(rev3_adc) => {
        match rev3_adc {
          SamRev3ADC::DiffSensors => {
            adc.enable_pga();
            adc.set_pga_gain(32);
            adc.set_positive_input_channel(1);
            adc.set_negative_input_channel(0);
          },

          SamRev3ADC::IValve => {
            adc.set_positive_input_channel(5);
          },

          SamRev3ADC::VValve => {
            adc.set_positive_input_channel(5);
          },

          SamRev3ADC::Tc1 | SamRev3ADC::Tc2 => {
            adc.enable_pga();
            adc.set_pga_gain(32);
            adc.set_positive_input_channel(1);
            adc.set_negative_input_channel(0);
          }

          _ => {} // no other changes needed for other ADCs
        }
      },

      SamRev4Gnd(rev4_gnd_adc) => {
        match rev4_gnd_adc {
          SamRev4GndADC::DiffSensors => {
            adc.enable_pga();
            adc.set_pga_gain(32);
            adc.set_positive_input_channel(0);
            adc.set_negative_input_channel(1);
          },

          SamRev4GndADC::IValve => {
            adc.set_positive_input_channel(2);
          },

          SamRev4GndADC::VValve => {
            adc.set_positive_input_channel(5);
          },

          SamRev4GndADC::Rtd1 | SamRev4GndADC::Rtd2 | SamRev4GndADC::Rtd3 => {
            adc.set_idac_magnitude(1000); // 1000 uA or 1 mA
            adc.enable_idac1_output_channel(0);
            adc.enable_idac2_output_channel(5);
            adc.set_positive_input_channel(1);
            adc.set_negative_input_channel(2);
            adc.set_ref_input_ref0();
          },

          _ => {} // no other changes needed for other ADCs
        }
      },

      SamRev4Flight(rev4_flight_adc) => {
        match rev4_flight_adc {
          SamRev4FlightADC::DiffSensors => {
            adc.enable_pga();
            adc.set_pga_gain(32);
            adc.set_positive_input_channel(0);
            adc.set_negative_input_channel(1);
          },

          SamRev4FlightADC::Rtd1 | SamRev4FlightADC::Rtd2 | SamRev4FlightADC::Rtd3 => {
            adc.set_idac_magnitude(1000); // 1000 uA or 1 mA
            adc.enable_idac1_output_channel(0);
            adc.enable_idac2_output_channel(5);
            adc.set_positive_input_channel(1);
            adc.set_negative_input_channel(2);
            adc.set_ref_input_ref0();
          },

          _ => {} // no other changes needed for other ADCs
        }
      },

      _ => panic!("Imposter ADC among us!")
    }

  }
}

pub fn poll_adcs(adcs: &mut Vec<ADC>) -> Vec<DataPoint> {
  data_points = vec![];

  for channel in 0..6 {
    for (i, adc) in adcs.iter_mut().enumerate() {

      let reached_max_rev3 = adc.kind == SamRev3(SamRev3ADC::DiffSensors) && channel > 1
        || adc.kind == SamRev3(SamRev3ADC::IPower) && channel > 1
        || adc.kind == SamRev3(SamRev3ADC::VPower) && channel > 4
        || adc.kind == SamRev3(SamRev3ADC::Tc1) && channel > 2
        || adc.kind == SamRev3(SamRev3ADC::Tc2) && channel > 2;

      // same for rev4 flight and ground channel wise
      let reached_max_rev4_gnd = adc.kind == SamRev4Gnd(SamRev4GndADC::DiffSensors) && channel > 1
        || adc.kind == SamRev4Gnd(SamRev4GndADC::Rtd1) && channel > 1
        || adc.kind == SamRev4Gnd(SamRev4GndADC::Rtd2) && channel > 1
        || adc.kind == SamRev4Gnd(SamRev4GndADC::Rtd3) && channel > 1;

      let reached_max_rev4_flight = adc.kind == SamRev4Flight(SamRev4FlightADC::DiffSensors) && channel > 1
        || adc.kind == SamRev4Flight(SamRev4FlightADC::Rtd1) && channel > 1
        || adc.kind == SamRev4Flight(SamRev4FlightADC::Rtd2) && channel > 1
        || adc.kind == SamRev4Flight(SamRev4FlightADC::Rtd3) && channel > 1;
      
      if reached_max_rev3 || reached_max_rev4_gnd || reached_max_rev4_flight {
        continue;
      }

      let time = Instant::now();
      let mut go_to_next_adc: bool = false;
      if let Some(_) = adc.drdy_pin {
        loop {
          if adc.check_drdy() == Low {
            break;
          } else if Instant::now() - time > Duration::from_millis(250) { // research what this value should be
            go_to_next_adc = true;
            break;
          }
        }
      } else {
        thread::sleep(Duration::from_micros(700)); // delay for TCs
      }

      if go_to_next_adc {
        continue; // cannot communicate with current ADC
      }

      let make_nan = false;
      let raw_data = match adc.spi_read_data() {
        Ok(data) => data,
        Err(e) => {
          eprintln!("{:?}: Error reading from {:?} channel {}", SAM_VERSION, adc.kind, channel);
          make_nan = true;
        }
      };

      // handle math, pin muxing
      match adc.kind {
        SamRev3(rev3_adc) => {
          match rev3_adc {
            SamRev3ADC::CurrentLoopPt => {

            },

          }
        },

        SamRev4Gnd(rev4_gnd_adc) => {
          match rev4_gnd_adc {
            SamRev4GndADC::CurrentLoopPt => {

            }
          }
        },

        SamRev4Flight(rev4_flight_adc) => {
          match rev4_flight_adc {
            SamRev4FlightADC::CurrentLoopPt => {

            }
          }
        }
      }

      // generate data point and push onto vec
    }
  }
  vec![DataPoint {value: 0.0, timestamp: 0.0, channel: 0, channel_type: ChannelType::CurrentLoop}]
}
