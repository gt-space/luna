use common::comm::sam::{ChannelType, DataPoint};
use common::comm::{
  ADCKind::{self, SamRev3, SamRev4Flight, SamRev4FlightV2, SamRev4Gnd},
  SamRev3ADC,
  SamRev4FlightADC,
  SamRev4FlightV2ADC,
  SamRev4GndADC,
};

pub fn generate_data_point(
  data: f64,
  timestamp: f64,
  iteration: u8,
  kind: ADCKind,
) -> DataPoint {
  DataPoint {
    value: data,
    timestamp,
    channel: iteration_to_channel(kind, iteration),
    channel_type: kind_to_channel_type(kind),
  }
}

// Gives the channel mapping to be used on the GUI
fn iteration_to_channel(kind: ADCKind, iteration: u8) -> u32 {
  let num = match kind {
    SamRev3(rev3_adc) => {
      match rev3_adc {
        SamRev3ADC::CurrentLoopPt
        | SamRev3ADC::IValve
        | SamRev3ADC::VValve
        | SamRev3ADC::IPower
        | SamRev3ADC::VPower
        | SamRev3ADC::DiffSensors => iteration + 1,

        SamRev3ADC::Tc1 => iteration, // the first iteration is PCB temp sense

        SamRev3ADC::Tc2 => iteration + 3, /* the first iteration is PCB temp
                                           * sense */
      }
    }

    SamRev4Gnd(rev4_gnd_adc) => match rev4_gnd_adc {
      SamRev4GndADC::CurrentLoopPt
      | SamRev4GndADC::IValve
      | SamRev4GndADC::VValve
      | SamRev4GndADC::DiffSensors
      | SamRev4GndADC::Rtd1 => iteration + 1,

      SamRev4GndADC::Rtd2 => (iteration + 1) + 2,

      SamRev4GndADC::Rtd3 => (iteration + 1) + 4,
    },

    SamRev4Flight(rev4_flight_adc) => match rev4_flight_adc {
      SamRev4FlightADC::CurrentLoopPt
      | SamRev4FlightADC::IValve
      | SamRev4FlightADC::VValve
      | SamRev4FlightADC::DiffSensors
      | SamRev4FlightADC::Rtd1 => iteration + 1,

      SamRev4FlightADC::Rtd2 => (iteration + 1) + 2,

      SamRev4FlightADC::Rtd3 => (iteration + 1) + 4,
    },
    SamRev4FlightV2(rev4_flight_adc) => match rev4_flight_adc {
      SamRev4FlightV2ADC::CurrentLoopPt
      | SamRev4FlightV2ADC::IValve
      | SamRev4FlightV2ADC::VValve
      | SamRev4FlightV2ADC::DiffSensors
      | SamRev4FlightV2ADC::Rtd1 => iteration + 1,

      SamRev4FlightV2ADC::Rtd2 => (iteration + 1) + 2,
    },

    _ => panic!("Imposter ADC among us!"),
  };

  //u32::try_from(num).unwrap()
  //u32::from(num)
  num as u32
}

/* Very useful for when you have Tc1, Tc2, and Tc3 becuase the measurement type
is just Tc.
 */
fn kind_to_channel_type(kind: ADCKind) -> ChannelType {
  match kind {
    SamRev3(rev3_adc) => match rev3_adc {
      SamRev3ADC::CurrentLoopPt => ChannelType::CurrentLoop,

      SamRev3ADC::IValve => ChannelType::ValveCurrent,

      SamRev3ADC::VValve => ChannelType::ValveVoltage,

      SamRev3ADC::IPower => ChannelType::RailCurrent,

      SamRev3ADC::VPower => ChannelType::RailVoltage,

      SamRev3ADC::DiffSensors => ChannelType::DifferentialSignal,

      SamRev3ADC::Tc1 | SamRev3ADC::Tc2 => ChannelType::Tc,
    },

    SamRev4Gnd(rev4_gnd_adc) => match rev4_gnd_adc {
      SamRev4GndADC::CurrentLoopPt => ChannelType::CurrentLoop,

      SamRev4GndADC::IValve => ChannelType::ValveCurrent,

      SamRev4GndADC::VValve => ChannelType::ValveVoltage,

      SamRev4GndADC::DiffSensors => ChannelType::DifferentialSignal,

      SamRev4GndADC::Rtd1 | SamRev4GndADC::Rtd2 | SamRev4GndADC::Rtd3 => {
        ChannelType::Rtd
      }
    },

    SamRev4Flight(rev4_flight_adc) => match rev4_flight_adc {
      SamRev4FlightADC::CurrentLoopPt => ChannelType::CurrentLoop,

      SamRev4FlightADC::IValve => ChannelType::ValveCurrent,

      SamRev4FlightADC::VValve => ChannelType::ValveVoltage,

      SamRev4FlightADC::DiffSensors => ChannelType::DifferentialSignal,

      SamRev4FlightADC::Rtd1
      | SamRev4FlightADC::Rtd2
      | SamRev4FlightADC::Rtd3 => ChannelType::Rtd,
    },
    
    SamRev4FlightV2(rev4_flight_adc) => match rev4_flight_adc {
      SamRev4FlightV2ADC::CurrentLoopPt => ChannelType::CurrentLoop,

      SamRev4FlightV2ADC::IValve => ChannelType::ValveCurrent,

      SamRev4FlightV2ADC::VValve => ChannelType::ValveVoltage,

      SamRev4FlightV2ADC::DiffSensors => ChannelType::DifferentialSignal,

      SamRev4FlightV2ADC::Rtd1
      | SamRev4FlightV2ADC::Rtd2 => ChannelType::Rtd,
    },


    _ => panic!("Imposter ADC among us!"),
  }
}
