use std::time::{Instant, Duration};
use common::comm::{gpio::PinValue::Low, sam::DataPoint, ADCKind::{self, Sam, SamRev3, SamRev4}, SamADC, SamRev3ADC, SamRev4ADC};
use ads114s06::ADC;

use crate::{SAM_INFO, SamVersion};

pub fn init_adcs(adcs: &mut Vec<ADC>) {
  for (i, adc) in adcs.iter_mut().enumerate() {

    // pga register (same as SAM)
    adc.set_programmable_conversion_delay(14);

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
    // adc.set_ref_input_internal_2v5_ref(); not for RTDs so set for each ADC (non RTDs get 2v5, RTDs get REF0 or REF1)
    adc.enable_internal_voltage_reference_on_pwr_down();

    // idacmag register
    adc.disable_pga_output_monitoring();
    adc.open_low_side_pwr_switch();
    // adc.set_idac_magnitude(0); not for RTDs so handle for each case

    // vbias register
    adc.disable_vbias();

    // system monitor register
    adc.disable_system_monitoring();
    adc.disable_spi_timeout();
    adc.disable_crc_byte();
    adc.disable_status_byte();

    match adc.kind {
      Sam(sam_adc) => {
        adc.set_idac_magnitude(0);
        adc.disable_idac1();
        adc.disable_idac2();
        adc.set_ref_input_internal_2v5_ref();

        match sam_adc {
          SamADC::CurrentLoopPt => {
            adc.set_pga_gain(1);
            adc.disable_pga();
            adc.set_positive_input_channel(0);
            adc.set_negative_input_channel_to_aincom();
          },

          SamADC::IValve => {
            adc.set_pga_gain(1);
            adc.disable_pga();

            if SAM_INFO.version == SamVersion::Rev3 {
              adc.set_positive_input_channel(5);
            } else if SAM_INFO.version == SamVersion::Rev4Ground {
              adc.set_positive_input_channel(2);
            } else if SAM_INFO.version == SamVersion::Rev4Flight {
              adc.set_positive_input_channel(0);
            }

            adc.set_negative_input_channel_to_aincom();
          },

          SamADC::VValve => {
            adc.set_pga_gain(1);
            adc.disable_pga();

            if SAM_INFO.version == SamVersion::Rev4Flight {
              adc.set_positive_input_channel(0);
            } else {
              adc.set_positive_input_channel(5);
            }

            adc.set_negative_input_channel_to_aincom();
          }

          SamADC::DiffSensors => {
            adc.enable_pga();
            adc.set_pga_gain(32);
            
            if SAM_INFO.version == SamVersion::Rev3 {
              adc.set_positive_input_channel(1);
              adc.set_negative_input_channel(0);
            } else {
              adc.set_positive_input_channel(0);
              adc.set_positive_input_channel(1);
            }
          }
        }
      },

      SamRev3(sam_rev3_adc) => {
        adc.set_idac_magnitude(0);
        adc.disable_idac1();
        adc.disable_idac2();
        adc.set_ref_input_internal_2v5_ref();

        match sam_rev3_adc {
          SamRev3ADC::IPower | SamRev3ADC::VPower => {
            adc.set_pga_gain(1);
            adc.disable_pga();
            adc.set_positive_input_channel(0);
            adc.set_negative_input_channel_to_aincom();
          },

          SamRev3ADC::Tc1 | SamRev3ADC::Tc2 => {
            adc.enable_pga();
            adc.set_pga_gain(32);
            adc.set_positive_input_channel(1);
            adc.set_negative_input_channel(0);
          }
        }
      },

      SamRev4(sam_rev4_adc) => { // this is just RTD so no need to match
        adc.enable_idac1_output_channel(0);
        adc.set_idac_magnitude(1000); // 1000 uA or 1 mA
        adc.enable_idac1_output_channel(0);
        adc.enable_idac2_output_channel(5);
        adc.set_positive_input_channel(1);
        adc.set_negative_input_channel(2);
        adc.set_ref_input_ref0();
      },

      _ => fail!("Vespula BMS ADC snuck in here, bad boy! :(")
    }
  }
}

pub fn poll_adcs(adcs: &mut Vec<ADC>) -> Vec<DataPoint>{

}
