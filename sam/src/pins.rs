use hostname;
use crate::SAM_INFO;

pub fn config_pins() {
  if SAM_INFO.version == SamVersion::Rev3 {

  } else if SAM_INFO.version == SamVersion::Rev4Ground {

  } else if SAM_INFO.version == SamVersion::Rev4Flight {

  }
}

fn config_pin(pin: &str, mode: &str) {
  Command::new("dash")
    .args(["config-pin.sh", pin, mode])
    .output()
    .expect("failed to configure pin");
}
