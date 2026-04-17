use clap::{Parser, ValueEnum};
use std::path::PathBuf;

use crate::file_logger::LoggerConfig;

/// Runtime commands for the flight computer. 
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum RuntimeCommand {
  /// Disable all local sensor workers
  Desktop,
  /// Disable the GPS/RECO worker
  DisableGps,
  /// Disable the FC-local IMU
  DisableImu,
  /// Disable the FC-local magnetometer
  DisableMagnetometer,
  /// Disable the FC-local barometer
  DisableBarometer,
}

/// State that describes which local workers should be active or not
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WorkerPlan {
  desktop_mode: bool,
  gps_enabled: bool,
  imu_enabled: bool,
  magnetometer_enabled: bool,
  barometer_enabled: bool,
}

impl WorkerPlan {
  /// Parses `RuntimeCommand` instance and returns the computed WorkerPlan.
  fn from_commands(commands: &[RuntimeCommand]) -> Self {
    let mut plan = Self {
      desktop_mode: false,
      gps_enabled: true,
      imu_enabled: true,
      magnetometer_enabled: true,
      barometer_enabled: true,
    };

    for command in commands {
      match command {
        RuntimeCommand::Desktop => {
          plan.desktop_mode = true;
          plan.gps_enabled = false;
          plan.imu_enabled = false;
          plan.magnetometer_enabled = false;
          plan.barometer_enabled = false;
        }
        RuntimeCommand::DisableGps => plan.gps_enabled = false,
        RuntimeCommand::DisableImu => plan.imu_enabled = false,
        RuntimeCommand::DisableMagnetometer => {
          plan.magnetometer_enabled = false
        }
        RuntimeCommand::DisableBarometer => plan.barometer_enabled = false,
      }
    }

    plan
  }

  /// Returns `True` if we are in desktop mode, else 'False'
  pub fn desktop_mode(&self) -> bool {
    self.desktop_mode
  }

  /// Returns `True` if the GPS worker should be enabled, else 'False'
  pub fn gps_enabled(&self) -> bool {
    self.gps_enabled
  }

  /// Returns `True` if we should be collecting IMU data, else 'False'
  pub fn imu_enabled(&self) -> bool {
    self.imu_enabled
  }

  /// Returns `True` if we should be collecting magnetometer data, else 'False'
  pub fn magnetometer_enabled(&self) -> bool {
    self.magnetometer_enabled
  }

  /// Returns `True` if we should be collecting barometer data, else 'False'
  pub fn barometer_enabled(&self) -> bool {
    self.barometer_enabled
  }

  /// Returns `True` if the mag / bar worker should be enabled, else 'False'
  pub fn mag_bar_enabled(&self) -> bool {
    self.magnetometer_enabled || self.barometer_enabled
  }
}

#[derive(Debug)]
pub struct RuntimeConfig {
  pub worker_plan: WorkerPlan,
  pub logger_config: LoggerConfig,
  pub print_gps: bool,
}

impl RuntimeConfig {
  fn from_args(args: Args) -> Self {
    Self {
      worker_plan: WorkerPlan::from_commands(&args.commands),
      logger_config: LoggerConfig::from_flight_cli(
        args.disable_file_logging,
        args.log_dir,
        args.log_buffer_size,
        args.log_rotation_mb,
      ),
      print_gps: args.print_gps,
    }
  }
}

/// Command-line arguments for the flight computer.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
  /// Stackable runtime commands such as `disable-imu disable-magnetometer`
  #[arg(value_enum, value_name = "COMMAND")]
  commands: Vec<RuntimeCommand>,

  /// Disable file logging (enabled by default)
  #[arg(long, default_value_t = false, global = true)]
  disable_file_logging: bool,

  /// Directory for log files (default: $HOME/flight_logs)
  #[arg(long, global = true)]
  log_dir: Option<PathBuf>,

  /// Buffer size in samples (default: 100)
  #[arg(long, default_value_t = 100, global = true)]
  log_buffer_size: usize,

  /// File rotation size threshold in MB (default: 100)
  #[arg(long, default_value_t = 100, global = true)]
  log_rotation_mb: u64,

  /// Print GPS data to terminal at ~1Hz (disabled by default)
  #[arg(long, default_value_t = false, global = true)]
  print_gps: bool,
}

pub fn parse() -> RuntimeConfig {
  RuntimeConfig::from_args(Args::parse())
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn parses_stacked_sensor_disable_commands() {
    let config = RuntimeConfig::from_args(Args::parse_from([
      "flight-computer",
      "disable-imu",
      "disable-magnetometer",
    ]));

    assert!(!config.worker_plan.imu_enabled());
    assert!(!config.worker_plan.magnetometer_enabled());
    assert!(config.worker_plan.barometer_enabled());
    assert!(!config.worker_plan.desktop_mode());
    assert!(config.worker_plan.gps_enabled());
    assert!(config.worker_plan.mag_bar_enabled());
  }

  #[test]
  fn desktop_mode_disables_all_local_sensors() {
    let config =
      RuntimeConfig::from_args(Args::parse_from(["flight-computer", "desktop"]));

    assert!(config.worker_plan.desktop_mode());
    assert!(!config.worker_plan.gps_enabled());
    assert!(!config.worker_plan.imu_enabled());
    assert!(!config.worker_plan.magnetometer_enabled());
    assert!(!config.worker_plan.barometer_enabled());
    assert!(!config.worker_plan.mag_bar_enabled());
  }

  #[test]
  fn disable_gps_only_turns_off_gps_worker() {
    let config =
      RuntimeConfig::from_args(Args::parse_from(["flight-computer", "disable-gps"]));

    assert!(!config.worker_plan.desktop_mode());
    assert!(!config.worker_plan.gps_enabled());
    assert!(config.worker_plan.imu_enabled());
    assert!(config.worker_plan.magnetometer_enabled());
    assert!(config.worker_plan.barometer_enabled());
  }

  #[test]
  fn desktop_overrides_other_runtime_commands() {
    let config = RuntimeConfig::from_args(Args::parse_from([
      "flight-computer",
      "disable-gps",
      "desktop",
      "disable-barometer",
    ]));

    assert!(config.worker_plan.desktop_mode());
    assert!(!config.worker_plan.gps_enabled());
    assert!(!config.worker_plan.imu_enabled());
    assert!(!config.worker_plan.magnetometer_enabled());
    assert!(!config.worker_plan.barometer_enabled());
  }
}
