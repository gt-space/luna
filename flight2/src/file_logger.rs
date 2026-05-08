use std::{
  fs::{self, File},
  io::{self, Write},
  path::{Path, PathBuf},
  sync::mpsc::{self, TrySendError},
  thread,
  time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use common::comm::VehicleState;
use serde::{Deserialize, Serialize};

/// A VehicleState with a timestamp attached for logging purposes
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TimestampedVehicleState {
  /// Unix timestamp in seconds with nanosecond precision
  pub timestamp: f64,
  /// The vehicle state at this timestamp
  pub state: VehicleState,
}

/// Configuration for the file logger
#[derive(Clone, Debug)]
pub struct LoggerConfig {
  /// Whether file logging is enabled
  pub enabled: bool,
  /// Directories where log files are stored
  pub log_dirs: Vec<PathBuf>,
  /// Maximum number of samples to buffer in the channel
  pub channel_capacity: usize,
  /// Number of samples to batch before writing
  pub batch_size: usize,
  /// Maximum time to wait before flushing a batch (even if not full)
  pub batch_timeout: Duration,
  /// Maximum file size in bytes before rotation
  pub file_size_limit: usize,
  /// Ensure the log data is written to disk every time we flush our internal 
  /// buffer `fsync_rate` times. If zero, this functionality is disabled.
  pub fsync_rate: usize,
}

impl Default for LoggerConfig {
  fn default() -> Self {
    Self {
      enabled: true,
      log_dirs: default_log_dir(),
      channel_capacity: 100,
      batch_size: 50,
      batch_timeout: Duration::from_millis(500),
      file_size_limit: 100 * 1024 * 1024, // 100MB
      fsync_rate: 0
    }
  }
}

impl LoggerConfig {
  /// Builds a [`LoggerConfig`] from the flight-computer CLI file-logging options
  /// (`disable-file-logging`, `log-dir`, `log-buffer-size`, `log-rotation-mb`).
  pub fn from_cli_commands(
    disable_file_logging: bool,
    log_dirs: Option<Vec<PathBuf>>,
    log_buffer_size: usize,
    log_rotation_mb: u64,
    fsync_rate: usize
  ) -> Self {
    Self {
      enabled: !disable_file_logging,
      // TODO: use default_log_dir if unwrap fails?
      log_dirs: log_dirs.unwrap_or_else(|| {
        vec![std::env::var("HOME")
          .map(PathBuf::from)
          .unwrap_or_else(|_| PathBuf::from("."))
          .join("flight_logs")]
      }),
      channel_capacity: log_buffer_size,
      batch_size: (log_buffer_size / 2).clamp(10, 100),
      batch_timeout: Duration::from_millis(500),
      file_size_limit: (log_rotation_mb as usize) * 1024 * 1024,
      fsync_rate,
    }
  }
}

fn default_log_dir() -> Vec<PathBuf> {
  // TODO: Rewrite this to use $HOME?, similarly to 
  vec![PathBuf::from("/home/ubuntu/flight_logs")]
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

/// File logger that asynchronously writes VehicleState data to disk
pub struct FileLogger {
  sender: mpsc::SyncSender<TimestampedVehicleState>,
  handle: Option<thread::JoinHandle<()>>,
}

struct LogFileBuffer {
  directories: Vec<PathBuf>,
  files: Vec<LogFile>,
  buffer: Vec<TimestampedVehicleState>,
  size: usize,
  last_flush: Instant,
}

impl LogFileBuffer {
  pub fn new(directories: Vec<PathBuf>, batch_size: usize) -> LogFileBuffer {
    LogFileBuffer {
      files: directories.iter().map(|p| LogFile::new(&create_log_file_path(p))).collect(),
      directories,
      buffer: Vec::with_capacity(batch_size),
      size: 0,
      last_flush: Instant::now(),
    }
  }

  pub fn flush_buffer(&mut self) {
    if self.is_empty() {
      self.last_flush = Instant::now();
      return;
    }

    let mut buf = Vec::new();
    for state in self.buffer.drain(..) {
      let serialized = match postcard::to_allocvec(&state) {
        Ok(s) => s,
        Err(e) => {
          eprintln!("Failed to serialize VehicleState for file log: {e}");
          continue;
        }
      };

      buf.extend(serialized.len().to_le_bytes());
      buf.extend(serialized);
    }

    for log_file in &mut self.files {
      if !log_file.is_open() {
        if let Err(e) = log_file.open() {
          eprintln!("Failed to open log file {} for writing: {e}", log_file.path.display());
        }
      }

      if let Err(e) = log_file.write(&buf[..]) {
        eprintln!("Failed to write to log file {}: {e}", log_file.path.display());
      }
    }

    self.last_flush = Instant::now();
    self.size += buf.len();
  }

  pub fn rotate_files(&mut self) {
    for (file, log_dir) in self.files.iter_mut().zip(self.directories.iter()) {
      file.close();
      file.set_path(&create_log_file_path(log_dir));
      if let Err(e) = file.open() {
        eprintln!("Failed to open new log file: {e}");
      }
    }

    self.size = 0;
  }

  pub fn sync_to_disk(&mut self) {
    for log_file in &mut self.files {
      if let Err(e) = log_file.sync_to_disk() {
        eprintln!("Failed to write {} to disk: {e}", log_file.path.display());
      }
    }
  }

  /// Returns the number of VehicleState structs currently in the buffer.
  pub fn states_in_flight(&self) -> usize {
    self.buffer.len()
  }

  pub fn last_flush(&self) -> Instant {
    self.last_flush
  }

  pub fn is_empty(&self) -> bool {
    self.buffer.is_empty()
  }

  pub fn file_size(&self) -> usize {
    self.size
  }

  pub fn push(&mut self, state: TimestampedVehicleState) {
    self.buffer.push(state);
  }
}

struct LogFile {
  path: PathBuf,
  file: Option<File>,
}

/// The result of a write operation on a LogFile.
type LogFileWriteResult<T> = ::std::result::Result<T, LogFileWriteError>;

/// The specific error of a write operation on a LogFile.
enum LogFileWriteError {
  /// The log file is already open.
  Open,
  /// The log file is not open.
  NotOpen,
  /// An I/O error occured while trying to flush. 
  IoError(io::Error)
}

impl From<io::Error> for LogFileWriteError {
  fn from(value: io::Error) -> Self {
      Self::IoError(value)
  }
}

impl std::fmt::Display for LogFileWriteError {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      Self::IoError(e) => write!(f, "{e}"),
      Self::NotOpen => write!(f, "The log file is not open."),
      Self::Open => write!(f, "The log file is open"),
    }
  }
}

impl LogFile {
  pub fn new(path: &Path) -> Self {
    LogFile {
      path: path.to_path_buf(),
      file: None,
    }
  }

  /// Opens a log file using the set path.
  pub fn open(&mut self) -> LogFileWriteResult<()> {
    if self.is_open() {
      return Err(LogFileWriteError::Open);
    }

    self.file = Some(File::create(&self.path)?);
    Ok(())
  }

  pub fn is_open(&self) -> bool {
    self.file.is_some()
  }

  /// Writes the passed data to the log if a log is open.
  pub fn write(&mut self, data: &[u8]) -> LogFileWriteResult<()> {
    let Some(ref mut file) = self.file else {
      return Err(LogFileWriteError::NotOpen);
    };
    
    file.write_all(data)?;
    Ok(())
  }

  /// Sets the internal path to the passed path.
  pub fn set_path(&mut self, path: &Path) {
    self.close();
    self.path = path.to_path_buf();
  }

  /// Closes the file. Does not fsync file contents to disk upon closing.
  pub fn close(&mut self) {
    self.file.take();
  }

  /// Ensures that any cached data related to the log file is written to disk
  /// upon returning.
  pub fn sync_to_disk(&mut self) -> LogFileWriteResult<()> {    
    let Some(ref file) = self.file else {
      return Err(LogFileWriteError::NotOpen);
    };

    Ok(file.sync_all()?)
  }
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

    // Ensure log directories exist
    for directory in &config.log_dirs {
      fs::create_dir_all(directory)?;
    }

    // Use bounded channel to prevent unbounded memory growth
    let (sender, receiver) = mpsc::sync_channel(config.channel_capacity);

    let handle = thread::spawn(move || {
      Self::writer_thread(receiver, config);
    });

    Ok(Self {
      sender,
      handle: Some(handle),
    })
  }

  /// Log a VehicleState (non-blocking, may drop if channel is full)
  pub fn log(&self, state: VehicleState) -> Result<(), LoggerError> {
    let timestamp = current_timestamp();
    let timestamped = TimestampedVehicleState { timestamp, state };

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
  /// This allows multiple threads to log without needing to clone the entire
  /// FileLogger
  pub fn clone_sender(&self) -> mpsc::SyncSender<TimestampedVehicleState> {
    self.sender.clone()
  }

  /// Background writer thread that handles batching and file I/O
  fn writer_thread(
    receiver: mpsc::Receiver<TimestampedVehicleState>,
    config: LoggerConfig
  ) {
    let mut log_buffer = LogFileBuffer::new(config.log_dirs, config.batch_size);
    let is_disk_sync_enabled = config.fsync_rate != 0;
    let mut flush_counter: usize = 0;

    'a: loop {
      // Check for batch timeout
      let elapsed = log_buffer.last_flush().elapsed();
      let should_flush_timeout = elapsed >= config.batch_timeout;

      // Try to receive with timeout
      let timeout = if should_flush_timeout {
        Duration::ZERO
      } else {
        config.batch_timeout - elapsed
      };

      match receiver.recv_timeout(timeout) {
        Ok(state) => {
          log_buffer.push(state);
        }
        Err(mpsc::RecvTimeoutError::Timeout) => {
          // Timeout - flush if we have data
        }
        Err(mpsc::RecvTimeoutError::Disconnected) => {
          // Channel closed - flush remaining data and exit
          log_buffer.flush_buffer();
          if is_disk_sync_enabled {
            log_buffer.sync_to_disk();
          }
          break 'a;
        }
      }

      let should_flush_batch = log_buffer.states_in_flight() >= config.batch_size;
      // Flush if needed
      if should_flush_timeout || should_flush_batch {
        log_buffer.flush_buffer();
        flush_counter += 1;
      }

      // Check if we need to rotate the file
      if log_buffer.file_size() >= config.file_size_limit {
        log_buffer.rotate_files();
      }

      // Check if we need to sync our data to disk.
      if is_disk_sync_enabled && flush_counter == config.fsync_rate {
        log_buffer.sync_to_disk();
        flush_counter = 0;
      }
    }

    // Flush any remaining data
    log_buffer.flush_buffer();
    if is_disk_sync_enabled {
      log_buffer.sync_to_disk();
    }
  }

  /// Shutdown the logger gracefully, flushing all pending data
  pub fn shutdown(self) -> Result<(), LoggerError> {
    // Drop sender to signal shutdown
    drop(self.sender);

    // Wait for thread to finish
    if let Some(handle) = self.handle {
      handle.join().map_err(|_| {
        LoggerError::IoError(std::io::Error::other(
          "Logger thread panicked",
        ))
      })?;
    }

    Ok(())
  }
}

/// Create a log file path with current timestamp
fn create_log_file_path(log_dir: &Path) -> PathBuf {
  use chrono::Local;

  // Format: flight_data_YYYYMMDD_HHMMSS.postcard
  let now = Local::now();
  let timestamp_str = now.format("%Y%m%d_%H%M%S").to_string();
  let filename = format!("flight_data_{}.postcard", timestamp_str);
  log_dir.join(filename)
}

/// Get current timestamp as f64 (seconds since epoch with nanosecond precision)
pub fn current_timestamp() -> f64 {
  SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .map(|t| Duration::as_secs_f64(&t))
    .unwrap_or(0.0)
}

#[cfg(test)]
mod tests{
  use std::{collections::HashMap, io::{BufReader, Read}, sync::mpsc::SyncSender};
  use super::*;

  const HOME_DIRECTORY: &str = "/tmp/fc2_file_logger_test";

  fn generate_random_vehiclestates<F>(quantity: usize, mut closure: F) -> Vec<TimestampedVehicleState>
  where F: FnMut(TimestampedVehicleState) {
    let mut monostate = VehicleState::new();
    let mut states = Vec::new();
    for _ in 0..quantity {
      monostate.bms.battery_bus.current = rand::random();
      monostate.bms.battery_bus.voltage = rand::random();
      monostate.bms.charger = rand::random();
      monostate.bms.chassis = rand::random();
      monostate.bms.e_stop = rand::random();
      monostate.bms.ethernet_bus.current = rand::random();
      monostate.bms.ethernet_bus.voltage = rand::random();
      monostate.bms.fcb_bus.voltage = rand::random();
      monostate.bms.fcb_bus.current = rand::random();
      monostate.bms.five_volt_rail.current = rand::random();
      monostate.bms.five_volt_rail.voltage = rand::random();
      monostate.bms.rbf_tag = rand::random();
      monostate.bms.reco_load_switch_1 = rand::random();
      monostate.bms.reco_load_switch_2 = rand::random();
      monostate.bms.sam_power_bus.current = rand::random();
      monostate.bms.sam_power_bus.voltage = rand::random();
      monostate.bms.tel_bus.current = rand::random();
      monostate.bms.tel_bus.voltage = rand::random();
      monostate.bms.umbilical_bus.current = rand::random();
      monostate.bms.umbilical_bus.voltage = rand::random();

      let state = TimestampedVehicleState { 
        timestamp: current_timestamp(),
        state: monostate.clone(),
      };

      states.push(state.clone());
      closure(state);
    }

    states
  }

  fn random_string(length: usize) -> String {
    let mut vec = Vec::new();
    for _ in 0..length {
      vec.push(rand::random_range(97u8..=122u8));
    }
    String::from_utf8(vec).unwrap()
  }

  fn create_log_dirs(paths: &[&str]) -> (PathBuf, Vec<PathBuf>) {
    let parent = PathBuf::from(HOME_DIRECTORY).join(random_string(16));
    let directories = paths.iter().map(|p| parent.join(p)).collect::<Vec<_>>();
    for directory in &directories {
      fs::create_dir_all(directory).unwrap();
    }
    
    (parent.clone(), directories)
  }

  fn create_channel_and_thread(config: LoggerConfig) -> SyncSender<TimestampedVehicleState> {
    let (sender, receiver) = 
      mpsc::sync_channel::<TimestampedVehicleState>(config.channel_capacity);
    thread::spawn(move || FileLogger::writer_thread(receiver, config));
    sender
  }

  fn deserialize_log_file(file: &File) -> Vec<TimestampedVehicleState> {
    let mut states = Vec::new();
    let mut state_buf = [0u8; 5_000];
    let mut contents = BufReader::new(file);
    
    loop {
      let mut len_buf = [0u8; 8];
      match contents.read_exact(&mut len_buf) {
        Ok(()) => {},
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => break,
        Err(e) => panic!("{e}"),
      };
      let state_length = usize::from_le_bytes(len_buf);

      contents.read_exact(&mut state_buf[..state_length]).unwrap();
      states.push(postcard::from_bytes(&state_buf[..state_length]).unwrap());
    }

    states
  }

  fn open_files(parent: &Path, paths: &[&str]) -> HashMap<String, Vec<File>> {
    let mut res = HashMap::new();

    for path in paths {
      let walker = fs::read_dir(parent.join(path).as_path()).unwrap();
      let mut paths: Vec<PathBuf> = walker
        .into_iter()
        .map(|d| d.unwrap().path())
        .collect();
      paths.sort();
      let files = paths.into_iter().map(|p| File::open(p).unwrap()).collect();
      res.insert(path.to_string(), files);
    }

    res
  }

  fn log_write_integration(paths: &[&str], fsync_rate: usize) {
    let (parent, log_dirs) = create_log_dirs(paths);

    let config = LoggerConfig {
      fsync_rate,
      log_dirs,
      batch_size: 100,
      file_size_limit: 100 * 1024,
      batch_timeout: Duration::MAX,
      ..Default::default()
    };

    let sender = create_channel_and_thread(config.clone());

    let mut states = generate_random_vehiclestates(config.batch_size / 2, |s| {
      let _ = sender.send(s);
    });

    let state_size = postcard::to_allocvec(&states[0]).unwrap().len();
    thread::sleep(Duration::from_millis(100));

    let directories = open_files(parent.as_path(), paths);
    for files in directories.values() {
      assert_eq!(files.len(), 0, "When only writing part of the buffer, it should not write to the filesystem.");
    }
    
    states.extend(generate_random_vehiclestates(config.batch_size, |s| {
      sender.send(s).unwrap();
    }));
    thread::sleep(Duration::from_millis(200));

    let directories = open_files(parent.as_path(), paths);
    for files in directories.values() {
      assert_eq!(files.len(), 1, "Only one file should exist when the buffers first gets flushed");
      let file = files.last().unwrap();
      let observed_states = deserialize_log_file(file);

      assert_eq!(observed_states.len(), config.batch_size, "The length of the log doesn't match the expected length");
      for (observed, expected) in observed_states.iter().zip(states.iter().take(config.batch_size)) {
        assert_eq!(observed, expected, "The expected state and logged state are not equal");
      }
    }

    thread::sleep(Duration::from_secs(1));
    let written = (config.batch_size + config.batch_size / 2) * state_size;
    let amount_to_rotate = (config.file_size_limit - written) / state_size + config.batch_size;
    states.extend(generate_random_vehiclestates(amount_to_rotate, |s| {
      sender.send(s).unwrap();
      thread::sleep(Duration::from_micros(50));
    }));
    let total = (config.batch_size + config.batch_size / 2 + amount_to_rotate) / config.batch_size * config.batch_size;

    let directories = open_files(parent.as_path(), paths);
    for files in directories.values() {
      assert_eq!(files.len(), 2, "Two files should exist when the log gets rotated");
      let mut observed_states = Vec::new();
      for file in files {
        observed_states.extend(deserialize_log_file(file));
      }

      assert_eq!(observed_states.len(), total, "The length of the log doesn't match the expected length");
      for (observed, expected) in observed_states.iter().zip(states.iter().take(config.batch_size)) {
        assert_eq!(observed, expected, "The expected state and logged state are not equal");
      }
    }
  }

  fn log_write_flush_timeout(paths: &[&str], fsync_rate: usize) {
    let (parent, log_dirs) = create_log_dirs(paths);

    let config = LoggerConfig {
      fsync_rate,
      log_dirs,
      batch_size: 100,
      batch_timeout: Duration::from_millis(100),
      ..Default::default()
    };

    let sender = create_channel_and_thread(config.clone());
    let states = generate_random_vehiclestates(config.batch_size / 2, |s| {
      let _ = sender.send(s);
    });

    thread::sleep(Duration::from_millis(500));
    let directories = open_files(parent.as_path(), paths);
    for files in directories.values() {
      assert_eq!(files.len(), 1, "When writing part of the buffer with timeout, it should write to the filesystem.");
      let file = files.last().unwrap();
      let observed_states = deserialize_log_file(file);

      assert_eq!(observed_states.len(), config.batch_size / 2, "The length of the log doesn't match the expected length");
      for (observed, expected) in observed_states.iter().zip(states.iter().take(config.batch_size)) {
        assert_eq!(observed, expected, "The expected state and logged state are not equal");
      }
    }
  }

  #[test]
  fn single_write_no_fsync() {
    log_write_integration(&["os"], 0);
  }

  #[test]
  fn single_write_fsync() {
    log_write_integration(&["os"], 1);
  }

  #[test]
  fn multi_write_no_fsync() {
    log_write_integration(&["os", "blackbox1", "blackbox2", "blackbox3"], 0);
  }

  #[test]
  fn multi_write_fsync() {
    log_write_integration(&["os", "blackbox1", "blackbox2", "blackbox3"], 1);
  }

  #[test]
  fn timeout_single_write_no_fsync() {
    log_write_flush_timeout(&["os"], 0);
  }

  #[test]
  fn timeout_single_write_fsync() {
    log_write_flush_timeout(&["os"], 1);
  }

  #[test]
  fn timeout_multi_write_no_fsync() {
    log_write_flush_timeout(&["os", "blackbox1", "blackbox2", "blackbox3"], 0);
  }

  #[test]
  fn timeout_multi_write_fsync() {
    log_write_flush_timeout(&["os", "blackbox1", "blackbox2", "blackbox3"], 1);
  }
}