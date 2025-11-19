use std::{
  sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
  },
  thread,
  time::{Duration, Instant},
};

use common::comm::{GpsState, RecoState, VehicleState};
use reco::{RecoDriver, FcGpsBody, RecoBody};
use zedf9p04b::{GPSError, GPS, PVT};
use std::sync::mpsc;

use crate::file_logger::TimestampedVehicleState;

/// Combined GPS and RECO state for mailbox
/// RECO array indices: 0 = MCU A (spidev1.2), 1 = MCU B (spidev1.1), 2 = MCU C (spidev1.0)
#[derive(Clone)]
pub struct GpsRecoState {
  pub gps: Option<GpsState>,
  pub reco: [Option<RecoState>; 3],
}

/// Single-slot mailbox for passing GPS and RECO samples from a background worker
/// thread into the flight computer main loop.
struct GpsMailbox {
  inner: Arc<Mutex<Option<GpsRecoState>>>,
}

#[derive(Clone)]
pub struct GpsMailboxWriter {
  inner: Arc<Mutex<Option<GpsRecoState>>>,
}

#[derive(Clone)]
pub struct GpsMailboxReader {
  inner: Arc<Mutex<Option<GpsRecoState>>>,
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
  /// Publish a new GPS and RECO sample, replacing any previous one.
  ///
  /// This uses a blocking mutex lock, but it only runs on the GPS worker
  /// thread, so it cannot stall the control loop.
  pub fn publish(&self, sample: GpsRecoState) {
    if let Ok(mut slot) = self.inner.lock() {
      *slot = Some(sample);
    }
  }
}

impl GpsMailboxReader {
  /// Non-blocking attempt to take the latest GPS and RECO sample.
  ///
  /// If the mailbox is currently locked by the writer, this will simply
  /// return `None` instead of blocking.
  pub fn take_latest(&self) -> Option<GpsRecoState> {
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
  /// Non-blocking attempt to get the most recent GPS and RECO sample.
  pub fn try_get_sample(&self) -> Option<GpsRecoState> {
    self.reader.take_latest()
  }

  /// Returns true if the GPS/RECO worker thread is still running.
  pub fn is_running(&self) -> bool {
    self._running.load(Ordering::Relaxed)
  }
}

pub struct GpsManager;

impl GpsManager {
  /// Spawn a background worker thread that talks to the GPS over I2C and
  /// publishes samples into a mailbox.
  /// 
  /// `vehicle_state_receiver` is used to receive vehicle state updates for logging.
  /// `file_logger_sender` is the sender for logging vehicle state at 200Hz.
  /// `print_gps` enables printing GPS data to terminal at ~1Hz.
  pub fn spawn(
    i2c_bus: u8,
    address: Option<u16>,
    vehicle_state_receiver: mpsc::Receiver<VehicleState>,
    file_logger_sender: Option<mpsc::SyncSender<TimestampedVehicleState>>,
    print_gps: bool,
  ) -> Result<GpsHandle, GPSError> {
    let (writer, reader) = GpsMailbox::new();
    let running = Arc::new(AtomicBool::new(true));
    // Clone for the thread so we can keep the original Arc in the handle.
    let running_for_thread = running.clone();

    thread::spawn(move || {
      let result = gps_worker_loop(
        i2c_bus,
        address,
        writer,
        running_for_thread.clone(),
        vehicle_state_receiver,
        file_logger_sender,
        print_gps,
      );

      // Mark the worker as no longer running, regardless of success or error.
      running_for_thread.store(false, Ordering::Relaxed);

      if let Err(e) = result {
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
  vehicle_state_receiver: mpsc::Receiver<VehicleState>,
  file_logger_sender: Option<mpsc::SyncSender<TimestampedVehicleState>>,
  print_gps: bool,
) -> Result<(), GPSError> {
  // Optional performance debug logging for GPS/RECO worker.
  let perf_debug = std::env::var("GPS_RECO_PERF_DEBUG").is_ok();
  if perf_debug {
    eprintln!("GPS_RECO_PERF_DEBUG enabled");
  }

  // Rate limiting for GPS printing to terminal (~1Hz)
  let print_interval = Duration::from_secs(1);
  let mut last_print_time = Instant::now();

  let mut gps = GPS::new(i2c_bus, address)?;

  // Configure GPS to run at 20Hz using periodic mode
  // 50ms period = 20Hz, nav_rate=1 (every measurement), time_ref=0 (UTC)
  if let Err(e) = gps.set_measurement_rate(50, 1, 0) {
    eprintln!("Failed to configure GPS measurement rate: {e}");
  }

  if let Err(e) = gps.set_nav_pvt_rate([1, 0, 0, 0, 0, 0]) {
    eprintln!("Failed to configure GPS NAV-PVT rate: {e}");
  }

  // Initialize RECO drivers for all three MCUs
  // MCU A: spidev1.2, MCU B: spidev1.1, MCU C: spidev1.0
  let mut reco_drivers: [Option<RecoDriver>; 3] = [
    match RecoDriver::new("/dev/spidev1.2") {
      Ok(driver) => {
        eprintln!("RECO driver MCU A (spidev1.2) initialized successfully");
        Some(driver)
      }
      Err(e) => {
        eprintln!("Failed to initialize RECO driver MCU A (spidev1.2): {e}. Continuing without this MCU.");
        None
      }
    },
    match RecoDriver::new("/dev/spidev1.1") {
      Ok(driver) => {
        eprintln!("RECO driver MCU B (spidev1.1) initialized successfully");
        Some(driver)
      }
      Err(e) => {
        eprintln!("Failed to initialize RECO driver MCU B (spidev1.1): {e}. Continuing without this MCU.");
        None
      }
    },
    match RecoDriver::new("/dev/spidev1.0") {
      Ok(driver) => {
        eprintln!("RECO driver MCU C (spidev1.0) initialized successfully");
        Some(driver)
      }
      Err(e) => {
        eprintln!("Failed to initialize RECO driver MCU C (spidev1.0): {e}. Continuing without this MCU.");
        None
      }
    },
  ];

  // Track last GPS data and valid flag
  let mut last_gps_state: Option<GpsState> = None;
  let mut gps_valid = false;

  // Track latest vehicle state for logging
  let mut latest_vehicle_state: Option<VehicleState> = None;

  // Rate limiting for GPS reads (20Hz -> 50ms interval)
  let gps_interval = Duration::from_millis(50);
  let mut last_gps_poll = Instant::now();

  // Rate limiting for 200Hz RECO transactions (5ms interval)
  let reco_interval = Duration::from_millis(5);
  let mut last_reco_time = Instant::now();
  
  // Track last time we published to mailbox (only publish when GPS data changes, not every 5ms)
  let mut last_publish_time = Instant::now();
  let publish_interval = Duration::from_millis(50); // Publish at most 20Hz to reduce contention

  // Rate limiting for logging to file (decoupled from RECO rate).
  // Target 200 Hz logging (5ms interval) to match the main loop LOG_INTERVAL.
  let log_interval = Duration::from_millis(5); // 200 Hz logging
  let mut last_log_time = Instant::now();

  // Main GPS acquisition and RECO transaction loop
  while running.load(Ordering::Relaxed) {
    // Receive vehicle state updates from main loop (non-blocking)
    while let Ok(state) = vehicle_state_receiver.try_recv() {
      latest_vehicle_state = Some(state);
    }

    // Read GPS data at 20Hz (non-blocking, uses periodic mode)
    let mut gps_data_changed = false;
    let loop_now = Instant::now();
    if loop_now.duration_since(last_gps_poll) >= gps_interval {
      last_gps_poll = loop_now;

      let gps_start = if perf_debug {
        Some(Instant::now())
      } else {
        None
      };

      match gps.read_pvt() {
        Ok(Some(pvt)) => {
          if let Some(state) = map_pvt_to_state(&pvt) {
            last_gps_state = Some(state.clone());
            gps_valid = true; // Fresh GPS data arrived, set valid to true
            gps_data_changed = true;
            
            // Print GPS data to terminal if enabled and enough time has passed
            if print_gps && loop_now.duration_since(last_print_time) >= print_interval {
              print_gps_state(&state);
              last_print_time = loop_now;
            }
          }
        }
        Ok(None) => {
          // No PVT data available yet, this is normal
          // Still print "no fix" message if printing is enabled
          if print_gps && loop_now.duration_since(last_print_time) >= print_interval {
            println!("GPS: No fix");
            last_print_time = loop_now;
          }
        }
        Err(e) => {
          eprintln!("Error while reading GPS PVT: {e}");
        }
      }

      if let Some(start) = gps_start {
        let dur = start.elapsed();
        if dur > Duration::from_millis(20) {
          eprintln!(
            "GPS worker: gps.read_pvt() took {:.2} ms",
            dur.as_secs_f64() * 1000.0
          );
        }
      }
    }

    // Make RECO transaction at 200Hz (every 5ms)
    if loop_now.duration_since(last_reco_time) >= reco_interval {
      last_reco_time = loop_now;

      let reco_start = if perf_debug {
        Some(Instant::now())
      } else {
        None
      };

      // Prepare GPS data for RECO
      let gps_body = if let Some(ref gps_state) = last_gps_state {
        FcGpsBody {
          velocity_north: gps_state.north_mps as f32,
          velocity_east: gps_state.east_mps as f32,
          velocity_down: gps_state.down_mps as f32,
          latitude: gps_state.latitude_deg as f32,
          longitude: gps_state.longitude_deg as f32,
          altitude: gps_state.altitude_m as f32,
          valid: gps_valid,
        }
      } else {
        // No GPS data yet, send zeros with valid=false
        FcGpsBody {
          velocity_north: 0.0,
          velocity_east: 0.0,
          velocity_down: 0.0,
          latitude: 0.0,
          longitude: 0.0,
          altitude: 0.0,
          valid: false,
        }
      };

      // Send GPS data to all three RECO MCUs and receive telemetry from each
      let mut reco_states: [Option<RecoState>; 3] = [None, None, None];
      
      for (index, reco_driver_opt) in reco_drivers.iter_mut().enumerate() {
        let mcu_name = match index {
          0 => "MCU A (spidev1.2)",
          1 => "MCU B (spidev1.1)",
          2 => "MCU C (spidev1.0)",
          _ => unreachable!(),
        };
        
        if let Some(ref mut reco_driver) = reco_driver_opt {
          match reco_driver.send_gps_data_and_receive_reco(&gps_body) {
            Ok(reco_body) => {
              // Convert RecoBody to RecoState
              reco_states[index] = Some(map_reco_body_to_state(&reco_body));
            }
            Err(e) => {
              if std::env::var("RECO_DEBUG").is_ok() {
                eprintln!("Error in RECO transaction with {}: {e}. Using zeroed RECO values.", mcu_name);
              }
              // Return zeroed RECO state on error
              reco_states[index] = Some(RecoState::default());
            }
          }
        } else {
          // No RECO driver for this MCU, use zeroed values
          reco_states[index] = Some(RecoState::default());
        }
      }

      // After first send, set valid to false for subsequent sends
      if gps_valid {
        gps_valid = false;
      }

      // Log vehicle state at 200Hz if logger is available
      if let Some(ref logger_sender) = file_logger_sender {
        if let Some(ref state) = latest_vehicle_state {
          let now_for_log = Instant::now();
          if now_for_log.duration_since(last_log_time) >= log_interval {
            // Create updated state with latest GPS and RECO data
            let mut updated_state = state.clone();
            updated_state.gps = last_gps_state.clone();
            updated_state.reco = reco_states.clone();
            updated_state.gps_valid = gps_valid;
            updated_state.reco_valid = true;
            
            // Create timestamped state using the same timestamp function as FileLogger
            use crate::file_logger;
            let timestamp = file_logger::current_timestamp();
            let timestamped = file_logger::TimestampedVehicleState {
              timestamp,
              state: updated_state,
            };
            
            // Log (non-blocking, may drop if channel is full)
            let _ = logger_sender.try_send(timestamped);
            last_log_time = now_for_log;
          }
        }
      }

      // Only publish to mailbox when GPS data changed or at reduced rate (max 20Hz)
      // This reduces mailbox contention and prevents main loop slowdown
      let now = Instant::now();
      if gps_data_changed || now.duration_since(last_publish_time) >= publish_interval {
        writer.publish(GpsRecoState {
          gps: last_gps_state.clone(),
          reco: reco_states,
        });
        last_publish_time = now;
      }
    }

    // Small delay to avoid busy-waiting
    // At 200Hz, we're checking every 5ms, so a small sleep is fine
    thread::sleep(Duration::from_millis(1));
  }

  Ok(())
}

/// Print GPS state to terminal in a readable format
fn print_gps_state(state: &GpsState) {
  println!("GPS: lat={:.6}° lon={:.6}° alt={:.2}m | vel=[{:.2}, {:.2}, {:.2}] m/s (N/E/D) | fix={} | ts={:?}",
    state.latitude_deg,
    state.longitude_deg,
    state.altitude_m,
    state.north_mps,
    state.east_mps,
    state.down_mps,
    state.has_fix,
    state.timestamp_unix_ms
  );
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

fn map_reco_body_to_state(reco_body: &RecoBody) -> RecoState {
  RecoState {
    quaternion: reco_body.quaternion,
    lla_pos: reco_body.lla_pos,
    velocity: reco_body.velocity,
    g_bias: reco_body.g_bias,
    a_bias: reco_body.a_bias,
    g_sf: reco_body.g_sf,
    a_sf: reco_body.a_sf,
    lin_accel: reco_body.lin_accel,
    angular_rate: reco_body.angular_rate,
    mag_data: reco_body.mag_data,
    temperature: reco_body.temperature,
    pressure: reco_body.pressure,
    stage1_enabled: reco_body.stage1_enabled,
    stage2_enabled: reco_body.stage2_enabled,
    vref_a_stage1: reco_body.vref_a_stage1,
    vref_a_stage2: reco_body.vref_a_stage2,
    vref_b_stage1: reco_body.vref_b_stage1,
    vref_b_stage2: reco_body.vref_b_stage2,
    vref_c_stage1: reco_body.vref_c_stage1,
    vref_c_stage2: reco_body.vref_c_stage2,
    vref_d_stage1: reco_body.vref_d_stage1,
    vref_d_stage2: reco_body.vref_d_stage2,
    vref_e_stage1_1: reco_body.vref_e_stage1_1,
    vref_e_stage1_2: reco_body.vref_e_stage1_2,
  }
}


