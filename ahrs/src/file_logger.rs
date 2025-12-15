//! This is a modified version of `flight2/src/file_logger.rs` adapter for AHRS.
//! While a generic version of this would be useful, the AHRS system may become a subsystem of FC soon, so this would be unnecessary for now.

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

/// Configuration for the file logger
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
      channel_capacity: 100,
      batch_size: 50,
      batch_timeout: Duration::from_millis(500),
      file_size_limit: 100 * 1024 * 1024, // 100MB
    }
  }
}

fn default_log_dir() -> PathBuf {
  std::env::var("HOME")
    .map(PathBuf::from)
    .unwrap_or_else(|_| PathBuf::from("."))
    .join("ahrs_imu_logs")
}

/// Error types for file logger operations
#[derive(Debug)]
pub enum LoggerError {
  IoError(std::io::Error),
  SerializationError(postcard::Error),
  ChannelSendError,
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
      LoggerError::ChannelSendError => {
        write!(f, "Failed to send to logging channel")
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
      // Use unbounded channel since we're just discarding quickly
      let (sender, receiver) = mpsc::sync_channel(config.channel_capacity);
      // Spawn a thread that just drains the receiver
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
    fs::create_dir_all(&config.log_dir).map_err(|e| LoggerError::IoError(e))?;

    // Create initial log file with timestamp
    let file_path = create_log_file_path(&config.log_dir)?;

    // Use bounded channel to prevent unbounded memory growth
    let (sender, receiver) = mpsc::sync_channel(config.channel_capacity);

    // Clone config for the background thread
    let thread_config = config.clone();

    let handle = thread::spawn(move || {
      Self::writer_thread(receiver, thread_config, file_path);
    });

    Ok(Self {
      sender,
      handle: Some(handle),
    })
  }

  /// Log a TimestampedImu value (non-blocking, may drop if channel is full)
  pub fn log(&self, state: Imu) -> Result<(), LoggerError> {
    let timestamp = current_timestamp();
    let timestamped = TimestampedImu { timestamp, state };

    // Use try_send to avoid blocking - drop message if channel is full
    match self.sender.try_send(timestamped) {
      Ok(()) => Ok(()),
      Err(TrySendError::Full(_)) => {
        // Channel is full - drop message (expected under heavy load)
        // Don't warn to avoid spamming stderr
        Err(LoggerError::ChannelSendError)
      }
      Err(TrySendError::Disconnected(_)) => Err(LoggerError::ChannelSendError),
    }
  }

  /// Clone the sender for sharing between threads
  /// This allows multiple threads to log without needing to clone the entire FileLogger
  pub fn clone_sender(&self) -> mpsc::SyncSender<TimestampedImu> {
    self.sender.clone()
  }

  /// Background writer thread that handles batching and file I/O
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
      // Check for batch timeout
      let should_flush_timeout = last_flush.elapsed() >= config.batch_timeout;
      let should_flush_batch = batch.len() >= config.batch_size;

      // Try to receive with timeout
      let timeout = if should_flush_timeout || should_flush_batch {
        Duration::ZERO
      } else {
        config.batch_timeout - last_flush.elapsed()
      };

      match receiver.recv_timeout(timeout) {
        Ok(state) => {
          batch.push(state);
        }
        Err(mpsc::RecvTimeoutError::Timeout) => {
          // Timeout - flush if we have data
        }
        Err(mpsc::RecvTimeoutError::Disconnected) => {
          // Channel closed - flush remaining data and exit
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

      // Flush if needed
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

      // Check if we need to rotate the file
      if file_size >= config.file_size_limit {
        Self::rotate_file(
          &mut current_file,
          &mut current_file_path,
          &mut file_size,
          &config,
        );
      }
    }

    // Flush any remaining data
    if let Some(ref mut writer) = current_file {
      let _ = writer.flush();
    }
  }

  /// Write a batch of timestamped states to the current file
  fn write_batch(
    current_file: &mut Option<BufWriter<File>>,
    current_file_path: &mut PathBuf,
    batch: &mut Vec<TimestampedImu>,
    file_size: &mut usize,
    config: &LoggerConfig,
  ) {
    // Ensure file is open
    if current_file.is_none() {
      match Self::open_file(current_file_path, config) {
        Ok(file) => *current_file = Some(file),
        Err(e) => {
          eprintln!("Failed to open log file {:?}: {}", current_file_path, e);
          batch.clear(); // Drop batch on error
          return;
        }
      }
    }

    let writer = current_file.as_mut().unwrap();

    // Serialize and write each state in the batch
    for state in batch.drain(..) {
      match postcard::to_allocvec(&state) {
        Ok(serialized) => {
          // Write length prefix (u64)
          let len = serialized.len() as u64;
          if let Err(e) = writer.write_all(&len.to_le_bytes()) {
            eprintln!("Failed to write length prefix: {}", e);
            continue;
          }

          // Write serialized data
          if let Err(e) = writer.write_all(&serialized) {
            eprintln!("Failed to write data: {}", e);
            continue;
          }

          *file_size += 8 + serialized.len(); // 8 bytes for length + data
        }
        Err(e) => {
          eprintln!("Failed to serialize state: {}", e);
        }
      }
    }

    // Flush the writer (but don't sync, for performance)
    if let Err(e) = writer.flush() {
      eprintln!("Failed to flush log file: {}", e);
    }
  }

  /// Rotate to a new file when size limit is reached
  fn rotate_file(
    current_file: &mut Option<BufWriter<File>>,
    current_file_path: &mut PathBuf,
    file_size: &mut usize,
    config: &LoggerConfig,
  ) {
    // Close current file
    if let Some(mut writer) = current_file.take() {
      let _ = writer.flush();
    }

    // Create new file
    match Self::open_file_path(config) {
      Ok(new_path) => {
        *current_file_path = new_path;
        *file_size = 0;
        match Self::open_file(&current_file_path, config) {
          Ok(file) => *current_file = Some(file),
          Err(e) => {
            eprintln!("Failed to open new log file: {}", e);
          }
        }
      }
      Err(e) => {
        eprintln!("Failed to create new log file path: {}", e);
      }
    }
  }

  /// Open a file for writing at the given path
  fn open_file(
    path: &Path,
    _config: &LoggerConfig,
  ) -> Result<BufWriter<File>, std::io::Error> {
    let file = File::create(path)?;
    Ok(BufWriter::with_capacity(256 * 1024, file)) // 256KB buffer
  }

  /// Generate a new log file path with current timestamp
  fn open_file_path(config: &LoggerConfig) -> Result<PathBuf, LoggerError> {
    Ok(create_log_file_path(&config.log_dir)?)
  }

  /// Shutdown the logger gracefully, flushing all pending data
  pub fn shutdown(self) -> Result<(), LoggerError> {
    // Drop sender to signal shutdown
    drop(self.sender);

    // Wait for thread to finish
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

/// Create a log file path with current timestamp
fn create_log_file_path(log_dir: &Path) -> Result<PathBuf, LoggerError> {
  use chrono::Local;

  let now = Local::now();
  let timestamp_str = now.format("%Y%m%d_%H%M%S").to_string();
  let filename = format!("ahrs_imu_data_{}.postcard", timestamp_str);
  Ok(log_dir.join(filename))
}

/// Get current timestamp as f64 (seconds since epoch with nanosecond precision)
pub(crate) fn current_timestamp() -> f64 {
  SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .map(|d| d.as_secs() as f64 + d.subsec_nanos() as f64 / 1_000_000_000.0)
    .unwrap_or(0.0)
}
