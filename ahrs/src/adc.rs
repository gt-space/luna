use common::comm::bms::Rail;
use std::fs;

use crate::command::{RAIL_3V3, RAIL_5V};

pub fn read_5v_rail() -> Rail {
  let scale = |x| x * (4990.0 + 10000.0) / 4990.0;
  Rail {
    voltage: scale(read_onboard_adc_raw(RAIL_5V.0)),
    current: scale(read_onboard_adc_raw(RAIL_5V.1)),
  }
}

pub fn read_3v3_rail() -> Rail {
  let scale = |x| x * (10000.0 + 10000.0) / 10000.0;
  Rail {
    voltage: scale(read_onboard_adc_raw(RAIL_3V3.0)),
    current: scale(read_onboard_adc_raw(RAIL_3V3.1)),
  }
}

fn read_onboard_adc_raw(rail_path: &str) -> f64 {
  let data = match fs::read_to_string(rail_path) {
    Ok(data) => data,
    Err(e) => {
      eprintln!("Fail to read {rail_path}, {e}");
      return f64::NAN;
    }
  };

  if data.is_empty() {
    eprintln!("Empty data for onboard ADC {rail_path}");
    return f64::NAN;
  }

  match data.trim().parse::<f64>() {
    Ok(data) => 1.8 * (data / ((1 << 12) as f64)),
    Err(e) => {
      eprintln!(
        "Fail to convert from String to f64 for onboard ADC {rail_path}, {e}"
      );
      f64::NAN
    }
  }
}
