use std::{os::raw, thread, time::{Duration, Instant}};
use common::comm::{gpio::PinValue::Low, sam::{ChannelType, DataPoint}, ADCKind::{self, SamRev3, SamRev4Gnd, SamRev4Flight}, SamRev3ADC, SamRev4GndADC, SamRev4FlightADC};
use ads114s06::ADC;
use std::f64::NAN;
use std::{io, fs};
use crate::{data::generate_data_point, tc::typek_convert};

use crate::{SAM_VERSION, SamVersion};

pub fn init_adcs(adcs: &mut Vec<ADC>) {
  for (i, adc) in adcs.iter_mut().enumerate() {

    // mux register
    adc.set_positive_input_channel(0); // change where needed
    adc.set_negative_input_channel_to_aincom(); // change where needed

    // pga register
    adc.set_programmable_conversion_delay(14);
    adc.set_pga_gain(1); // change where needed
    adc.disable_pga(); // change where needed

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
    adc.disable_system_monitoring(); // change for TCs
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
            // set up for initial ambient read
            // handles enabling and setting PGA Gain
            adc.enable_internal_temp_sensor(1);

            // sets up for after initial ambient read
            adc.set_positive_input_channel(5);
            adc.set_negative_input_channel(4);
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

pub fn poll_adcs(adcs: &mut Vec<ADC>, ambient_temps: &mut Option<Vec<f64>>) -> Vec<DataPoint> {
  let mut datapoints: Vec<DataPoint> = Vec::new();

  for iteration in 0..6 { // max number of channels on ADC
    for (i, adc) in adcs.iter_mut().enumerate() {

      let reached_max_rev3 = adc.kind == SamRev3(SamRev3ADC::DiffSensors) && iteration > 2
        || adc.kind == SamRev3(SamRev3ADC::IPower) && iteration > 1
        || adc.kind == SamRev3(SamRev3ADC::VPower) && iteration > 4
        || adc.kind == SamRev3(SamRev3ADC::Tc1) && iteration > 3 // extra reading for PCB temp
        || adc.kind == SamRev3(SamRev3ADC::Tc2) && iteration > 3; // extra reading for PCB temp

      // same for rev4 flight and ground channel wise
      let reached_max_rev4_gnd = adc.kind == SamRev4Gnd(SamRev4GndADC::DiffSensors) && iteration > 1
        || adc.kind == SamRev4Gnd(SamRev4GndADC::Rtd1) && iteration > 1
        || adc.kind == SamRev4Gnd(SamRev4GndADC::Rtd2) && iteration > 1
        || adc.kind == SamRev4Gnd(SamRev4GndADC::Rtd3) && iteration > 1;

      let reached_max_rev4_flight = adc.kind == SamRev4Flight(SamRev4FlightADC::DiffSensors) && iteration > 1
        || adc.kind == SamRev4Flight(SamRev4FlightADC::Rtd1) && iteration > 1
        || adc.kind == SamRev4Flight(SamRev4FlightADC::Rtd2) && iteration > 1
        || adc.kind == SamRev4Flight(SamRev4FlightADC::Rtd3) && iteration > 1;
      
      if reached_max_rev3 || reached_max_rev4_gnd || reached_max_rev4_flight {
        continue;
      }

      let time = Instant::now();
      let mut go_to_next_adc: bool = false;
      if let Some(_) = adc.drdy_pin {
        loop {
          if adc.check_drdy() == Low {
            break;
          } else if Instant::now() - time > Duration::from_millis(250) {
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

      // math and pin muxing logic
      let calc_data: f64 = match adc.spi_read_data() {
        Ok(raw_data) => {
          match adc.kind {
            SamRev3(rev3_adc) => {
              match rev3_adc {
                SamRev3ADC::CurrentLoopPt => {
                  let data = adc.calc_diff_measurement_offset(raw_data);
                  adc.set_positive_input_channel((iteration + 1) % 6);

                  data
                },

                SamRev3ADC::IValve => {
                  let data = adc.calc_diff_measurement_offset(raw_data);
                  adc.set_positive_input_channel(5 - ((iteration + 1) % 6));

                  data
                },

                SamRev3ADC::VValve => {
                  let data = adc.calc_diff_measurement_offset(raw_data) * 11.0;
                  adc.set_positive_input_channel(5 - ((iteration + 1) % 6));

                  data
                },

                SamRev3ADC::IPower => {
                  let data = adc.calc_diff_measurement_offset(raw_data);
                  adc.set_positive_input_channel((iteration + 1) % 2);

                  data
                },

                SamRev3ADC::VPower => {
                  let data = adc.calc_diff_measurement_offset(raw_data) * 11.0;
                  adc.set_positive_input_channel((iteration + 1) % 5);

                  data
                }

                SamRev3ADC::DiffSensors => {
                  let data = adc.calc_diff_measurement(raw_data) / 1000.0;
                  
                  if iteration == 0 {
                    adc.set_positive_input_channel(3);
                    adc.set_negative_input_channel(2);
                  } else if iteration == 1 {
                    adc.set_positive_input_channel(5);
                    adc.set_negative_input_channel(4);
                  } else if iteration == 2 {
                    adc.set_positive_input_channel(1);
                    adc.set_negative_input_channel(0);
                  }

                  data
                },

                SamRev3ADC::Tc1 => {
                  if iteration == 0 { // ambient temp
                    let data = adc.calc_diff_measurement(raw_data) * 1000.0;
                    let ambient_temp = data * 0.403 - 26.987;
                    ambient_temps.as_mut().unwrap()[0] = ambient_temp; // I want it to panic if this don't work :)

                    adc.disable_system_monitoring();
                    adc.enable_pga();
                    adc.set_pga_gain(32);
                    adc.set_positive_input_channel(5);
                    adc.set_negative_input_channel(4);
                    continue;

                  } else {
                    let data = adc.calc_diff_measurement(raw_data);
                    let ambient_temp = ambient_temps.as_ref().unwrap()[0];
                    let temp = (typek_convert(ambient_temp as f32, data as f32) + 273.15) as f64;

                    if iteration == 1 {
                      adc.set_positive_input_channel(3);
                      adc.set_negative_input_channel(2);
                    } else if iteration == 2 {
                      adc.set_positive_input_channel(1);
                      adc.set_negative_input_channel(0);
                    } else if iteration == 3 {
                      adc.enable_internal_temp_sensor(1); // handles enabling and setting PGA gain
                    }

                    temp
                  }
                },

                SamRev3ADC::Tc2 => {
                  if iteration == 0 { // ambient temp
                    let data = adc.calc_diff_measurement(raw_data) * 1000.0;
                    let ambient_temp = data * 0.403 - 26.987;
                    ambient_temps.as_mut().unwrap()[1] = ambient_temp; // I want it to panic if this don't work :)

                    adc.disable_system_monitoring();
                    adc.enable_pga();
                    adc.set_pga_gain(32);
                    adc.set_positive_input_channel(5);
                    adc.set_negative_input_channel(4);
                    continue;

                  } else {
                    let data = adc.calc_diff_measurement(raw_data);
                    let ambient_temp = ambient_temps.as_ref().unwrap()[1];
                    let temp = (typek_convert(ambient_temp as f32, data as f32) + 273.15) as f64;

                    if iteration == 1 {
                      adc.set_positive_input_channel(3);
                      adc.set_negative_input_channel(2);
                    } else if iteration == 2 {
                      adc.set_positive_input_channel(1);
                      adc.set_negative_input_channel(0);
                    } else if iteration == 3 {
                      adc.enable_internal_temp_sensor(1); // handles enabling and setting PGA gain
                    }

                    temp
                  }

                }
    
              }
            },
    
            SamRev4Gnd(rev4_gnd_adc) => {
              match rev4_gnd_adc {
                SamRev4GndADC::CurrentLoopPt => {
                  let data = adc.calc_diff_measurement(raw_data) * 2.0;
                  adc.set_positive_input_channel((iteration + 1) % 6);

                  data
                },

                SamRev4GndADC::IValve => {
                  let data = adc.calc_diff_measurement(raw_data) * (1200.0 / 1000.0);
                  
                  if iteration == 1 {
                    adc.set_positive_input_channel(1);
                  } else if iteration == 3 {
                    adc.set_positive_input_channel(0);
                  } else if iteration == 5 {
                    adc.set_positive_input_channel(2);
                  }

                  data
                },

                SamRev4GndADC::VValve => {
                  let data = adc.calc_diff_measurement(raw_data) * 11.0;
                  adc.set_positive_input_channel(5 - ((iteration + 1) % 6)); // flipped

                  data
                },

                SamRev4GndADC::DiffSensors => {
                  let data = adc.calc_diff_measurement(raw_data) / 1000.0;

                  if iteration == 0 {
                    adc.set_positive_input_channel(2);
                    adc.set_negative_input_channel(3);
                  } else if iteration == 1 {
                    adc.set_positive_input_channel(0);
                    adc.set_negative_input_channel(1);
                  }

                  data
                },

                SamRev4GndADC::Rtd1 | SamRev4GndADC::Rtd2 | SamRev4GndADC::Rtd3 => {
                  let rtd_resistance = adc.calc_four_wire_rtd_resistance(raw_data, 2500.0);
                  let temp = if rtd_resistance <= 100.0 {
                    0.0014 * rtd_resistance.powi(2) + 2.2521 * rtd_resistance - 239.04
                  } else {
                    0.0014 * rtd_resistance.powi(2) + 2.1814 * rtd_resistance - 230.07
                  };

                  if iteration == 0 {
                    adc.set_positive_input_channel(3);
                    adc.set_negative_input_channel(4);
                    adc.set_ref_input_ref1();
                  } else if iteration == 1 {
                    adc.set_positive_input_channel(1);
                    adc.set_negative_input_channel(2);
                    adc.set_ref_input_ref0();
                  }

                  temp
                }
              }
            },
    
            SamRev4Flight(rev4_flight_adc) => {
              match rev4_flight_adc {
                SamRev4FlightADC::CurrentLoopPt => {
                  let data = adc.calc_diff_measurement(raw_data) * 2.0;
                  adc.set_positive_input_channel((iteration + 1) % 6);

                  data
                },

                SamRev4FlightADC::IValve => {
                  let data = adc.calc_diff_measurement(raw_data) * (1200.0 / 1560.0);
                  // toggle valve I selection pin (figure out how to access that data)

                  if iteration == 1 {
                    adc.set_positive_input_channel(1);
                  } else if iteration == 3 {
                    adc.set_positive_input_channel(2);
                  } else if iteration == 5 {
                    adc.set_positive_input_channel(0);
                  }
                  
                  data
                },

                SamRev4FlightADC::VValve => {
                  let data = adc.calc_diff_measurement(raw_data) * 11.0;
                  adc.set_positive_input_channel((iteration + 1) % 6);

                  data
                },

                SamRev4FlightADC::DiffSensors => {
                  let data = adc.calc_diff_measurement(raw_data) / 1000.0;

                  if iteration == 0 {
                    adc.set_positive_input_channel(2);
                    adc.set_negative_input_channel(3);
                  } else if iteration == 1 {
                    adc.set_positive_input_channel(0);
                    adc.set_negative_input_channel(1);
                  }

                  data
                },

                SamRev4FlightADC::Rtd1 | SamRev4FlightADC::Rtd2 | SamRev4FlightADC::Rtd3 => {
                  let rtd_resistance = adc.calc_four_wire_rtd_resistance(raw_data, 2500.0);
                  let temp = if rtd_resistance <= 100.0 {
                    0.0014 * rtd_resistance.powi(2) + 2.2521 * rtd_resistance - 239.04
                  } else {
                    0.0014 * rtd_resistance.powi(2) + 2.1814 * rtd_resistance - 230.07
                  };

                  if iteration == 0 {
                    adc.set_positive_input_channel(3);
                    adc.set_negative_input_channel(4);
                    adc.set_ref_input_ref1();
                  } else if iteration == 1 {
                    adc.set_positive_input_channel(1);
                    adc.set_negative_input_channel(2);
                    adc.set_ref_input_ref0();
                  }

                  temp
                }
              }
            },

            _ => panic!("Imposter ADC among us!")
          }
        },

        Err(e) => {
          eprintln!("{:?}: Error reading from {:?} iteration {}", SAM_VERSION, adc.kind, iteration);
          NAN
        }
      };

      let datapoint = generate_data_point(calc_data, 0.0, iteration, adc.kind);
      datapoints.push(datapoint);

      /*
      If SAM is either rev4 ground or rev4 flight the rail I/V data is from
      the onboard BeagleBone ADC. Here the file paths are set up, the value
      is read, modified if needed, and additional DataPoints are created
       */
      if *SAM_VERSION == SamVersion::Rev4Ground || *SAM_VERSION == SamVersion::Rev4Flight {
        let rail_paths: Vec<&str> = vec![r"/sys/bus/iio/devices/iio:device0/in_voltage0_raw", 
            r"/sys/bus/iio/devices/iio:device0/in_voltage1_raw", 
            r"/sys/bus/iio/devices/iio:device0/in_voltage2_raw",
            r"/sys/bus/iio/devices/iio:device0/in_voltage3_raw",
            r"/sys/bus/iio/devices/iio:device0/in_voltage4_raw"];

        for (i, path) in rail_paths.iter().enumerate() {
          let (val, ch_type) = read_onboard_adc(i, *path);
          datapoints.push(DataPoint { 
            value: val, 
            timestamp: 0.0, 
            channel: (i as u32) + 1, 
            channel_type: ch_type 
            }
          )
        }
      }

    }
  }
  
  datapoints
}

pub fn read_onboard_adc(channel: usize, rail_path: &str) -> (f64, ChannelType) {
  // read Linux system file associated with current onboard ADC channel
  let data = match fs::read_to_string(rail_path) {
    Ok(output) => output,
    Err(e) => {
      eprintln!("Fail to read {}, {}", rail_path, e);

      if *SAM_VERSION == SamVersion::Rev4Ground {
        if channel == 0 || channel == 2 || channel == 4 {
          return (NAN, ChannelType::RailVoltage)
        } else {
          return (NAN, ChannelType::RailCurrent)
        }
      } else {
        if channel == 0 || channel == 1 || channel == 3 {
          return (NAN, ChannelType::RailVoltage)
        } else {
          return (NAN, ChannelType::RailCurrent)
        }
      }
    }
  };

  // have to handle this possibility after obtaining the String
  if data.is_empty() {
    eprintln!("Empty data for on board ADC channel {}", channel);

    if *SAM_VERSION == SamVersion::Rev4Ground {
      if channel == 0 || channel == 2 || channel == 4 {
        return (NAN, ChannelType::RailVoltage)
      } else {
        return (NAN, ChannelType::RailCurrent)
      }
    } else { // rev4 flight
      if channel == 0 || channel == 1 || channel == 3 {
        return (NAN, ChannelType::RailVoltage)
      } else {
        return (NAN, ChannelType::RailCurrent)
      }
    }
  }

  // convert to f64 to inverse the voltage divider or current sense amplifications
  match data.trim().parse::<f64>() {
    Ok(data) => {
      let feedback = 1.8 * (data as f64) / ((1 << 12) as f64);

      if *SAM_VERSION == SamVersion::Rev4Ground {
        if channel == 0 || channel == 2 || channel == 4 {
          ((feedback * (4700.0 + 100000.0) / 4700.0), ChannelType::RailVoltage)
        } else {
          /*
          The inverse of the mathematical operations performed by the shunt
          resistor and current sense amplifier actually result in the ADC input
          voltage being equal to the rail current. Thus V = I :)
          */
          (feedback, ChannelType::RailCurrent)
        }
      } else { // rev4 flight
        if channel == 0 || channel == 1 || channel == 3 {
          ((feedback * (4700.0 + 100000.0) / 4700.0), ChannelType::RailVoltage)
        } else {
          /*
          The inverse of the mathematical operations performed by the shunt
          resistor and current sense amplifier actually result in the ADC input
          voltage being equal to the rail current. Thus V = I :)
          */
          (feedback, ChannelType::RailCurrent)
        }
      }

    },

    Err(e) => {
      eprintln!("Fail to convert from String to f64 for onboard ADC channel {}, {}", channel, e);

      if *SAM_VERSION == SamVersion::Rev4Ground {
        if channel == 0 || channel == 2 || channel == 4 {
          return (NAN, ChannelType::RailVoltage)
        } else {
          return (NAN, ChannelType::RailCurrent)
        }
      } else { // rev4 flight
        if channel == 0 || channel == 1 || channel == 3 {
          return (NAN, ChannelType::RailVoltage)
        } else {
          return (NAN, ChannelType::RailCurrent)
        }
      }
    }
  }
}