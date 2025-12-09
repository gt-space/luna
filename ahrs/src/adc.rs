// use common::comm::sam::{ChannelType, DataPoint};
// use std::fs;

// const RAIL_PATHS: [&str; 5] = [
//   r"/sys/bus/iio/devices/iio:device0/in_voltage0_raw",
//   r"/sys/bus/iio/devices/iio:device0/in_voltage1_raw",
//   r"/sys/bus/iio/devices/iio:device0/in_voltage2_raw",
//   r"/sys/bus/iio/devices/iio:device0/in_voltage3_raw",
//   r"/sys/bus/iio/devices/iio:device0/in_voltage4_raw",
// ];

// pub fn read_rail_adcs() -> Vec<DataPoint> {
//   let mut datapoints = Vec::new();

//   for (i, path) in RAIL_PATHS.iter().enumerate() {
//     let (value, channel_type) = read_onboard_adc(i, path);
//     datapoints.push(DataPoint {
//       value,
//       timestamp: 0.0,
//       channel: (i as u32) + 1,
//       channel_type,
//     })
//   }

//   datapoints
// }

// pub fn read_onboard_adc(channel: usize, rail_path: &str) -> (f64, ChannelType) {
//   // read Linux system file associated with current onboard ADC channel
//   let data = match fs::read_to_string(rail_path) {
//     Ok(output) => output,
//     Err(e) => {
//       eprintln!("Fail to read {}, {}", rail_path, e);

//       if *SAM_VERSION == SamVersion::Rev4Ground {
//         if channel == 0 || channel == 2 || channel == 4 {
//           return (f64::NAN, ChannelType::RailVoltage);
//         } else {
//           return (f64::NAN, ChannelType::RailCurrent);
//         }
//       } else {
//         if channel == 0 || channel == 1 || channel == 3 {
//           return (f64::NAN, ChannelType::RailVoltage);
//         } else {
//           return (f64::NAN, ChannelType::RailCurrent);
//         }
//       }
//     }
//   };

//   // have to handle this possibility after obtaining the String
//   if data.is_empty() {
//     eprintln!("Empty data for on board ADC channel {}", channel);

//     if *SAM_VERSION == SamVersion::Rev4Ground {
//       if channel == 0 || channel == 2 || channel == 4 {
//         return (f64::NAN, ChannelType::RailVoltage);
//       } else {
//         return (f64::NAN, ChannelType::RailCurrent);
//       }
//     } else {
//       // rev4 flight
//       if channel == 0 || channel == 1 || channel == 3 {
//         return (f64::NAN, ChannelType::RailVoltage);
//       } else {
//         return (f64::NAN, ChannelType::RailCurrent);
//       }
//     }
//   }

//   // convert to f64 to inverse the voltage divider or current sense
//   // amplifications
//   match data.trim().parse::<f64>() {
//     Ok(data) => {
//       let feedback = 1.8 * data / ((1 << 12) as f64);

//       if *SAM_VERSION == SamVersion::Rev4Ground {
//         if channel == 0 || channel == 2 || channel == 4 {
//           (
//             (feedback * (4700.0 + 100000.0) / 4700.0),
//             ChannelType::RailVoltage,
//           )
//         } else {
//           /*
//           The inverse of the mathematical operations performed by the shunt
//           resistor and current sense amplifier actually result in the ADC input
//           voltage being equal to the rail current. Thus V = I :)
//           */
//           (feedback, ChannelType::RailCurrent)
//         }
//       } else {
//         // rev4 flight
//         if channel == 0 || channel == 1 || channel == 3 {
//           (
//             (feedback * (4700.0 + 100000.0) / 4700.0),
//             ChannelType::RailVoltage,
//           )
//         } else {
//           /*
//           The inverse of the mathematical operations performed by the shunt
//           resistor and current sense amplifier actually result in the ADC input
//           voltage being equal to the rail current. Thus V = I :)
//           */
//           (feedback, ChannelType::RailCurrent)
//         }
//       }
//     }

//     Err(e) => {
//       eprintln!(
//         "Fail to convert from String to f64 for onboard ADC channel {}, {}",
//         channel, e
//       );

//       if *SAM_VERSION == SamVersion::Rev4Ground {
//         if channel == 0 || channel == 2 || channel == 4 {
//           return (f64::NAN, ChannelType::RailVoltage);
//         } else {
//           return (f64::NAN, ChannelType::RailCurrent);
//         }
//       } else {
//         // rev4 flight
//         if channel == 0 || channel == 1 || channel == 3 {
//           return (f64::NAN, ChannelType::RailVoltage);
//         } else {
//           return (f64::NAN, ChannelType::RailCurrent);
//         }
//       }
//     }
//   }
// }
