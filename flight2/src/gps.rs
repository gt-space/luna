use std::{
  sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
  },
  thread,
  time::Duration,
};

use chrono::TimeZone;
use common::comm::GpsState;
use zedf9p04b::{GPSError, GPS, PVT};

/// Single-slot mailbox for passing GPS samples from a background worker
/// thread into the flight computer main loop.
struct GpsMailbox {
  inner: Arc<Mutex<Option<GpsState>>>,
}

#[derive(Clone)]
pub struct GpsMailboxWriter {
  inner: Arc<Mutex<Option<GpsState>>>,
}

#[derive(Clone)]
pub struct GpsMailboxReader {
  inner: Arc<Mutex<Option<GpsState>>>,
}

impl GpsMailbox {
  fn new() -> (GpsMailboxWriter, GpsMailboxReader) {
    let inner = Arc::new(Mutex::new(None));
    (
      GpsMailboxWriter {
        inner: inner.clone(),
      },
      GpsMailboxReader { inner },
    )
  }
}

impl GpsMailboxWriter {
  /// Publish a new GPS sample, replacing any previous one.
  ///
  /// This uses a blocking mutex lock, but it only runs on the GPS worker
  /// thread, so it cannot stall the control loop.
  pub fn publish(&self, sample: GpsState) {
    if let Ok(mut slot) = self.inner.lock() {
      *slot = Some(sample);
    }
  }
}

impl GpsMailboxReader {
  /// Non-blocking attempt to take the latest GPS sample.
  ///
  /// If the mailbox is currently locked by the writer, this will simply
  /// return `None` instead of blocking.
  pub fn take_latest(&self) -> Option<GpsState> {
    match self.inner.try_lock() {
      Ok(mut slot) => slot.take(),
      Err(_) => None,
    }
  }
}

/// Handle used by the flight computer main loop to fetch GPS samples.
pub struct GpsHandle {
  reader: GpsMailboxReader,
  _running: Arc<AtomicBool>,
}

impl GpsHandle {
  /// Non-blocking attempt to get the most recent GPS sample.
  pub fn try_get_sample(&self) -> Option<GpsState> {
    self.reader.take_latest()
  }
}

pub struct GpsManager;

impl GpsManager {
  /// Spawn a background worker thread that talks to the GPS over I2C and
  /// publishes samples into a mailbox.
  pub fn spawn(i2c_bus: u8, address: Option<u16>) -> Result<GpsHandle, GPSError> {
    let (writer, reader) = GpsMailbox::new();
    let running = Arc::new(AtomicBool::new(true));
    let running_thread = running.clone();

    thread::spawn(move || {
      if let Err(e) = gps_worker_loop(i2c_bus, address, writer, running_thread) {
        eprintln!("GPS worker exited with error: {e}");
      }
    });

    Ok(GpsHandle {
      reader,
      _running: running,
    })
  }
}

fn gps_worker_loop(
  i2c_bus: u8,
  address: Option<u16>,
  writer: GpsMailboxWriter,
  running: Arc<AtomicBool>,
) -> Result<(), GPSError> {
  let mut gps = GPS::new(i2c_bus, address)?;

  // Configure periodic NAV-PVT messages on I2C (DDC).
  if let Err(e) = gps.set_nav_pvt_rate([1, 0, 0, 0, 0, 0]) {
    eprintln!("Failed to configure GPS NAV-PVT rate: {e}");
  }

  // Main GPS acquisition loop.
  while running.load(Ordering::Relaxed) {
    match gps.poll_pvt() {
      Ok(Some(pvt)) => {
        if let Some(state) = map_pvt_to_state(&pvt) {
          writer.publish(state);
        }
      }
      Ok(None) => {
        // No valid PVT this time; avoid busy-waiting.
        thread::sleep(Duration::from_millis(50));
      }
      Err(e) => {
        eprintln!("Error while polling GPS PVT: {e}");
        // Brief backoff on errors to avoid tight error loops.
        thread::sleep(Duration::from_millis(200));
      }
    }
  }

  Ok(())
}

fn map_pvt_to_state(pvt: &PVT) -> Option<GpsState> {
  let has_pos = pvt.position.is_some();
  let has_vel = pvt.velocity.is_some();

  if !has_pos && !has_vel && pvt.time.is_none() {
    return None;
  }

  let (latitude_deg, longitude_deg, altitude_m) = match &pvt.position {
    Some(pos) => (pos.lat, pos.lon, pos.alt),
    None => (0.0, 0.0, 0.0),
  };

  let (north_mps, east_mps, down_mps) = match &pvt.velocity {
    Some(vel) => (vel.north, vel.east, vel.down),
    None => (0.0, 0.0, 0.0),
  };

  let timestamp_unix_ms = pvt.time.as_ref().map(|t| t.timestamp_millis());

  Some(GpsState {
    latitude_deg,
    longitude_deg,
    altitude_m,
    north_mps,
    east_mps,
    down_mps,
    timestamp_unix_ms,
    has_fix: has_pos || has_vel,
  })
}


