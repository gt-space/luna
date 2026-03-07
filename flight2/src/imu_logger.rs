//! IMU-specific file logger for the flight computer (Raspberry Pi 5).
//! This is adapted from `ahrs/src/file_logger.rs` but writes to a Pi-local
//! directory.

use std::{
  fs::{self, File},
  io::{BufWriter, Write},
  path::{Path, PathBuf},
  sync::mpsc::{self, TrySendError},
  thread,
  time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use common::comm::ahrs::Imu;
use serde::{Deserialize, Serialize};

/// IMU data with a timestamp attached for logging purposes
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TimestampedImu {
  /// Unix timestamp in seconds with nanosecond precision
  pub timestamp: f64,
  /// IMU data for this timestamp
  pub state: Imu,
}

/// Configuration for the IMU file logger
#[derive(Clone, Debug)]
pub struct LoggerConfig {
  /// Whether file logging is enabled
  pub enabled: bool,
  /// Directory where log files are stored
  pub log_dir: PathBuf,
  /// Maximum number of samples to buffer in the channel
  pub channel_capacity: usize,
  /// Number of samples to batch before writing
  pub batch_size: usize,
  /// Maximum time to wait before flushing a batch (even if not full)
  pub batch_timeout: Duration,
  /// Maximum file size in bytes before rotation
  pub file_size_limit: usize,
}

impl Default for LoggerConfig {
  fn default() -> Self {
    Self {
      enabled: true,
      log_dir: default_log_dir(),
      // Tight loop IMU assume ~1kHz
      // 500 samples = ~500ms buffer at 1kHz, providing headroom for disk I/O delays
      channel_capacity: 500,
      batch_size: 250, // Half of channel capacity
      batch_timeout: Duration::from_millis(500),
      // Log at ~1kHz, use 1GB to keep file creation rate reasonable
      file_size_limit: 1024 * 1024 * 1024, // 1GB
    }
  }
}

fn default_log_dir() -> PathBuf {
  // Pi-specific IMU log directory
  PathBuf::from("/home/ubuntu/imu_data_logs")
}

/// Error types for IMU file logger operations
#[derive(Debug)]
pub enum LoggerError {
  IoError(std::io::Error),
  SerializationError(postcard::Error),
  ChannelFull,          // Channel is full (expected under load, non-fatal)
  ChannelDisconnected,  // Channel is disconnected (writer thread died, fatal)
}

impl From<std::io::Error> for LoggerError {
  fn from(err: std::io::Error) -> Self {
    LoggerError::IoError(err)
  }
}

impl From<postcard::Error> for LoggerError {
  fn from(err: postcard::Error) -> Self {
    LoggerError::SerializationError(err)
  }
}

impl std::fmt::Display for LoggerError {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      LoggerError::IoError(e) => write!(f, "IO error: {}", e),
      LoggerError::SerializationError(e) => {
        write!(f, "Serialization error: {}", e)
      }
      LoggerError::ChannelFull => {
        write!(
          f,
          "Logging channel is full (disk I/O cannot keep up, message dropped)"
        )
      }
      LoggerError::ChannelDisconnected => {
        write!(
          f,
          "Logging channel disconnected (writer thread may have crashed)"
        )
      }
    }
  }
}

/// File logger that asynchronously writes IMU data to disk
pub struct FileLogger {
  sender: mpsc::SyncSender<TimestampedImu>,
  handle: Option<thread::JoinHandle<()>>,
}

impl FileLogger {
  /// Create a new file logger with the given configuration
  pub fn new(config: LoggerConfig) -> Result<Self, LoggerError> {
    if !config.enabled {
      // Return a dummy logger that does nothing but still accepts messages
      let (sender, receiver) = mpsc::sync_channel(config.channel_capacity);
      let handle = thread::spawn(move || {
        while receiver.recv().is_ok() {
          // Just discard messages - no I/O overhead
        }
      });
      return Ok(Self {
        sender,
        handle: Some(handle),
      });
    }

    // Ensure log directory exists
    fs::create_dir_all(&config.log_dir).map_err(LoggerError::IoError)?;

    // Create initial log file with timestamp
    let file_path = create_log_file_path(&config.log_dir)?;

    // Use bounded channel to prevent unbounded memory growth
    let (sender, receiver) = mpsc::sync_channel(config.channel_capacity);

    let thread_config = config.clone();

    let handle = thread::spawn(move || {
      Self::writer_thread(receiver, thread_config, file_path);
    });

    Ok(Self { sender, handle: Some(handle) })
  }

  /// Log an IMU value (non-blocking, may drop if channel is full)
  pub fn log(&self, state: Imu) -> Result<(), LoggerError> {
    let timestamp = current_timestamp();

    // Validate that the data is reasonable before logging
    let accel_valid = state.accelerometer.x.is_finite()
      && state.accelerometer.y.is_finite()
      && state.accelerometer.z.is_finite();
    let gyro_valid = state.gyroscope.x.is_finite()
      && state.gyroscope.y.is_finite()
      && state.gyroscope.z.is_finite();

    if !accel_valid || !gyro_valid {
      // Skip logging invalid data rather than corrupting the log file
      return Ok(());
    }

    let timestamped = TimestampedImu { timestamp, state };

    match self.sender.try_send(timestamped) {
      Ok(()) => Ok(()),
      Err(TrySendError::Full(_)) => Err(LoggerError::ChannelFull),
      Err(TrySendError::Disconnected(_)) => Err(LoggerError::ChannelDisconnected),
    }
  }

  fn writer_thread(
    receiver: mpsc::Receiver<TimestampedImu>,
    config: LoggerConfig,
    initial_file_path: PathBuf,
  ) {
    let mut current_file_path = initial_file_path;
    let mut current_file: Option<BufWriter<File>> = None;
    let mut batch: Vec<TimestampedImu> = Vec::with_capacity(config.batch_size);
    let mut last_flush = Instant::now();
    let mut file_size: usize = 0;

    loop {
      let should_flush_timeout = last_flush.elapsed() >= config.batch_timeout;
      let should_flush_batch = batch.len() >= config.batch_size;

      let timeout = if should_flush_timeout || should_flush_batch {
        Duration::ZERO
      } else {
        config.batch_timeout - last_flush.elapsed()
      };

      match receiver.recv_timeout(timeout) {
        Ok(state) => batch.push(state),
        Err(mpsc::RecvTimeoutError::Timeout) => {
          // just check flush conditions below
        }
        Err(mpsc::RecvTimeoutError::Disconnected) => {
          if !batch.is_empty() {
            Self::write_batch(
              &mut current_file,
              &mut current_file_path,
              &mut batch,
              &mut file_size,
              &config,
            );
          }
          break;
        }
      }

      if should_flush_timeout || should_flush_batch {
        if !batch.is_empty() {
          Self::write_batch(
            &mut current_file,
            &mut current_file_path,
            &mut batch,
            &mut file_size,
            &config,
          );
          last_flush = Instant::now();
        }
      }

      if file_size >= config.file_size_limit {
        Self::rotate_file(
          &mut current_file,
          &mut current_file_path,
          &mut file_size,
          &config,
        );
      }
    }

    if let Some(ref mut writer) = current_file {
      let _ = writer.flush();
    }
  }

  fn write_batch(
    current_file: &mut Option<BufWriter<File>>,
    current_file_path: &mut PathBuf,
    batch: &mut Vec<TimestampedImu>,
    file_size: &mut usize,
    config: &LoggerConfig,
  ) {
    if current_file.is_none() {
      match Self::open_file(current_file_path, config) {
        Ok(file) => *current_file = Some(file),
        Err(e) => {
          eprintln!("Failed to open log file {:?}: {}", current_file_path, e);
          batch.clear();
          return;
        }
      }
    }

    let writer = current_file.as_mut().unwrap();

    for state in batch.drain(..) {
      match postcard::to_allocvec(&state) {
        Ok(serialized) => {
          let len = serialized.len() as u64;
          let len_bytes = len.to_le_bytes();

          if let Err(e) = writer.write_all(&len_bytes) {
            eprintln!("Failed to write length prefix: {}", e);
            continue;
          }

          match writer.write_all(&serialized) {
            Ok(()) => {
              *file_size += 8 + serialized.len();
            }
            Err(e) => {
              eprintln!("Failed to write data: {} - file may be corrupted!", e);
              continue;
            }
          }
        }
        Err(e) => {
          eprintln!("Failed to serialize IMU state: {}", e);
        }
      }
    }

    if let Err(e) = writer.flush() {
      eprintln!("Failed to flush log file: {}", e);
    }
  }

  fn rotate_file(
    current_file: &mut Option<BufWriter<File>>,
    current_file_path: &mut PathBuf,
    file_size: &mut usize,
    config: &LoggerConfig,
  ) {
    if let Some(mut writer) = current_file.take() {
      if let Err(e) = writer.flush() {
        eprintln!("Failed to flush file during rotation: {}", e);
      }
      drop(writer);
    }

    match Self::open_file_path(config) {
      Ok(new_path) => {
        *current_file_path = new_path;
        *file_size = 0;
        match Self::open_file(&current_file_path, config) {
          Ok(file) => *current_file = Some(file),
          Err(e) => eprintln!("Failed to open new log file: {}", e),
        }
      }
      Err(e) => eprintln!("Failed to create new log file path: {}", e),
    }
  }

  fn open_file(
    path: &Path,
    _config: &LoggerConfig,
  ) -> Result<BufWriter<File>, std::io::Error> {
    let file = File::create(path)?;
    Ok(BufWriter::with_capacity(256 * 1024, file))
  }

  fn open_file_path(config: &LoggerConfig) -> Result<PathBuf, LoggerError> {
    Ok(create_log_file_path(&config.log_dir)?)
  }

  /// Shutdown the logger gracefully, flushing all pending data
  pub fn shutdown(self) -> Result<(), LoggerError> {
    drop(self.sender);

    if let Some(handle) = self.handle {
      handle.join().map_err(|_| {
        LoggerError::IoError(std::io::Error::new(
          std::io::ErrorKind::Other,
          "Logger thread panicked",
        ))
      })?;
    }

    Ok(())
  }
}

fn create_log_file_path(log_dir: &Path) -> Result<PathBuf, LoggerError> {
  use chrono::Local;

  let now = Local::now();
  let timestamp_str = now.format("%Y%m%d_%H%M%S").to_string();
  let filename = format!("imu_data_{}.postcard", timestamp_str);
  Ok(log_dir.join(filename))
}

fn current_timestamp() -> f64 {
  SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .map(|d| d.as_secs() as f64 + d.subsec_nanos() as f64 / 1_000_000_000.0)
    .unwrap_or(0.0)
}

