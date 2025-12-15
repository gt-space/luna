use std::{env, path::PathBuf, time::Duration};

use clap::Parser;
use once_cell::sync::OnceCell;

use crate::{
  file_logger::LoggerConfig,
  state::{InitData, State},
};

mod adc;
mod communication;
mod driver;
mod file_logger;
mod pins;
mod state;

pub static FC_ADDR: OnceCell<String> = OnceCell::new();

/// Command-line arguments for AHRS
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
  /// Hostname of flight computer
  #[arg(long, default_value = "flight")]
  target: String,

  /// Disable file logging (enabled by default)
  #[arg(long, default_value_t = false)]
  disable_file_logging: bool,

  /// Directory for log files (default: $HOME/flight_logs)
  #[arg(long)]
  log_dir: Option<PathBuf>,

  /// Buffer size in samples (default: 100)
  #[arg(long, default_value_t = 100)]
  log_buffer_size: usize,

  /// File rotation size threshold in MB (default: 100)
  #[arg(long, default_value_t = 100)]
  log_rotation_mb: u64,
}

fn main() {
  let args = Args::parse();

  FC_ADDR.set(args.target).unwrap();

  let imu_logger_config = LoggerConfig {
    enabled: !args.disable_file_logging,
    log_dir: args.log_dir.unwrap_or_else(|| {
      env::var("HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("."))
        .join("flight_logs")
    }),
    channel_capacity: args.log_buffer_size,
    batch_size: (args.log_buffer_size / 2).max(10).min(100), // Half of buffer, but at least 10 and at most 100
    batch_timeout: Duration::from_millis(500),
    file_size_limit: (args.log_rotation_mb as usize) * 1024 * 1024, // Convert MB to bytes
  };

  let mut state = State::Init(InitData { imu_logger_config });

  loop {
    state = state.next();
  }
}
