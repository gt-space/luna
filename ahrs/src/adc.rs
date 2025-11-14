use common::comm::bms::Rail;
use std::fs;


pub fn read_rail(current: &str, voltage: &str) -> Rail {
  Rail {
    current: read_onboard_adc(current),
    voltage: read_onboard_adc(voltage),
  }
}

fn read_onboard_adc(rail_path: &str) -> f64 {
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
    Ok(data) => {
      let feedback = 1.8 * (data / ((1 << 12) as f64));
      feedback * (4700.0 + 100000.0) / 4700.0
    }
    Err(e) => {
      eprintln!(
        "Fail to convert from String to f64 for onboard ADC {rail_path}, {e}");
      f64::NAN
    }
  }
}
