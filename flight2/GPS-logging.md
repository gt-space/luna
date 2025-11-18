## GPS Data Retrieval and Logging Pipeline

This document explains how GPS data flows from the u-blox ZED-F9P driver into the flight computer, and how it is logged to disk for later analysis.

- **Scope**: Flight computer integration and logging.
- **Driver details**: See `firmware/zedf9p04b/README.md` for low-level GPS driver documentation.

---

## High-Level Data Flow

- **Background worker**: A dedicated GPS worker thread talks to the ZED-F9P over I2C and produces `GpsState` samples.
- **Mailbox**: The worker publishes each new sample into a single-slot mailbox (`GpsMailbox`), replacing the previous one.
- **Main loop ingestion**: The flight computer main loop non-blockingly pulls the latest sample (if any) once per control iteration and updates `VehicleState`.
- **Validity flag**: `VehicleState.gps_valid` is set to `true` when a fresh sample is ingested and reset to `false` after telemetry for that iteration is sent.
- **Logging**: The current `VehicleState` (including GPS fields) is passed to the asynchronous `FileLogger`, which writes timestamped entries to `.postcard` log files.
- **Post-processing**: The `utility` binary reads the `.postcard` logs and exports them to CSV, including GPS-related columns.

---

## 1. GPS Background Worker Thread

**Relevant file**: `flight2/src/gps.rs`

- **Mailbox types**
  - `GpsMailbox`, `GpsMailboxWriter`, `GpsMailboxReader` form a simple single-slot mailbox:
    - Internally: `Arc<Mutex<Option<GpsState>>>`.
    - **Writer** (`GpsMailboxWriter::publish`) overwrites the slot with the latest `GpsState`.
    - **Reader** (`GpsMailboxReader::take_latest`) uses `try_lock` to avoid blocking; it returns `Some(sample)` and clears the slot, or `None` if locked/empty.

- **Handle for the main loop**
  - `GpsHandle` wraps a `GpsMailboxReader`:
    - `GpsHandle::try_get_sample()` is a non-blocking call used from the main control loop to grab at most one fresh sample per iteration.

- **Spawning the worker**
  - `GpsManager::spawn(i2c_bus: u8, address: Option<u16>) -> Result<GpsHandle, GPSError>`:
    - Creates the mailbox (writer + reader).
    - Creates an `Arc<AtomicBool>` `running` flag.
    - Spawns a thread that runs `gps_worker_loop(i2c_bus, address, writer, running_thread)`.
    - Returns a `GpsHandle` to the caller (the flight computer main).

- **Worker loop**
  - `gps_worker_loop` does the following:
    - Constructs the GPS driver: `let mut gps = GPS::new(i2c_bus, address)?;`.
    - Configures periodic NAV-PVT messages over I2C:
      - `gps.set_nav_pvt_rate([1, 0, 0, 0, 0, 0])` (enable NAV‑PVT on DDC/I2C).
    - Enters a loop while `running.load(Ordering::Relaxed)` is `true`:
      - Calls `gps.poll_pvt()`:
        - `Ok(Some(pvt))`: maps to `GpsState` via `map_pvt_to_state(&pvt)` and publishes via `writer.publish(state)`.
        - `Ok(None)`: no valid PVT; sleeps `50 ms` to avoid busy-waiting.
        - `Err(e)`: logs the error and sleeps `200 ms` before retrying.
    - `map_pvt_to_state` constructs a `common::comm::GpsState`:
      - Position → `latitude_deg`, `longitude_deg`, `altitude_m`.
      - Velocity → `north_mps`, `east_mps`, `down_mps`.
      - Time → `timestamp_unix_ms` (via `timestamp_millis()`).
      - `has_fix` is `true` if position or velocity is present.

**Key properties**:

- **Backpressure**: The mailbox holds at most one sample; if the main loop is slow, intermediate GPS samples are dropped, but the latest state is always available.
- **Non-blocking**: The worker uses a blocking mutex (safe because it’s off the main loop), while the main loop uses a non-blocking `try_lock` on the mailbox.

---

## 2. Main Loop Integration and `VehicleState` Updates

**Relevant files**: `flight2/src/main.rs`, `flight2/src/device.rs`, `common/src/comm.rs`

- **Spawning the worker**
  - In `main.rs`:
    - `let gps_handle = match gps::GpsManager::spawn(1, None) { ... };`
    - On success, the handle is stored as `Some(GpsHandle)`; on failure, the flight computer continues without GPS.

- **Per-iteration ingestion**
  - Inside the main control loop (`main.rs`):
    - The code checks for new samples:
      - 
        ```rust
        if let Some(handle) = gps_handle.as_ref() {
          if let Some(sample) = handle.try_get_sample() {
            devices.update_gps(sample);
          }
        }
        ```
    - This is **non-blocking** and will at most consume one GPS sample per iteration.

- **Updating `VehicleState`**
  - In `device.rs`:
    - `Devices` owns a `VehicleState` (`self.state`).
    - `Devices::update_gps(&mut self, sample: GpsState)`:
      - `self.state.gps = Some(sample);`
      - `self.state.gps_valid = true;`
    - `Devices::invalidate_gps(&mut self)`:
      - `self.state.gps_valid = false;`
  - In `common/src/comm.rs`, `VehicleState` contains:
    - `pub gps: Option<GpsState>,`
    - `pub gps_valid: bool,`
    - The documentation notes:
      - `gps` holds the latest sample (for logging / debugging).
      - `gps_valid` is `true` only for the control-loop iteration in which a new sample was consumed.

- **Resetting `gps_valid` after sending telemetry**
  - Still in the main loop (`main.rs`):
    - Telemetry is periodically pushed to Servo via:
      - `servo::push(&socket, servo_address, devices.get_state(), file_logger.as_ref())`
    - Immediately after sending:
      - 
        ```rust
        devices.invalidate_gps();
        ```
    - This yields the following semantics:
      - **Fresh GPS for current iteration**: `gps_valid == true`.
      - **After telemetry sent**: `gps_valid == false` until the next `update_gps` call.

**Why this design**:

- Keeps `gps` as the latest known sample for logging, even if it’s logically stale.
- Uses `gps_valid` as an explicit marker for “fresh this tick” so downstream code and logs can distinguish fresh vs. stale GPS data.

---

## 3. File Logging of GPS-Enhanced `VehicleState`

**Relevant file**: `flight2/src/file_logger.rs`

- **Timestamped wrapper**
  - `TimestampedVehicleState`:
    - `timestamp: f64` (Unix seconds with sub-second precision).
    - `state: VehicleState` (includes `gps` and `gps_valid`).

- **Logger configuration**
  - `LoggerConfig` (constructed in `main.rs` using CLI args):
    - `enabled`: allow disabling logging at runtime.
    - `log_dir`: directory for log files (default `$HOME/flight_logs`).
    - `channel_capacity`: bounded channel size.
    - `batch_size`: batch size before write.
    - `batch_timeout`: max time before flushing a batch.
    - `file_size_limit`: file-rotation limit.

- **Asynchronous writer thread**
  - `FileLogger::new(config)`:
    - Ensures the log directory exists.
    - Creates a bounded `sync_channel<TimestampedVehicleState>`.
    - Spawns a background writer thread with `writer_thread(receiver, config, initial_file_path)`.
  - `FileLogger::log(&self, state: VehicleState)`:
    - Wraps the state with a timestamp (`current_timestamp()`).
    - Uses `try_send` to enqueue; if the channel is full, the sample is dropped (for backpressure).

- **Writer thread behavior**
  - `writer_thread`:
    - Buffers incoming `TimestampedVehicleState` in a `batch` vector.
    - Flushes the batch to disk when:
      - Batch size reaches `batch_size`, or
      - `batch_timeout` elapses.
    - Each entry is serialized with postcard and written as:
      - 8-byte little-endian length prefix (`u64`).
      - Serialized bytes of `TimestampedVehicleState`.
    - Tracks `file_size` and rotates to a new log file when it exceeds `file_size_limit`.

**Effect on GPS data**:

- Each log entry captures:
  - The current `VehicleState.gps` (if any).
  - The current `VehicleState.gps_valid` flag.
  - Everything else (BMS, AHRS, valves, sensors, abort stage, etc.).
- Because `gps_valid` is reset each time after sending telemetry, the logs allow you to:
  - See **when** a GPS sample was freshly ingested (`gps_valid == true`).
  - Still inspect the last known GPS position/velocity even when `gps_valid == false`.

---

## 4. Post-Processing: From `.postcard` to CSV (including GPS)

**Relevant file**: `utility/src/main.rs`

- The `utility` binary:
  - Reads `.postcard` log files produced by `FileLogger`.
  - Deserializes `TimestampedVehicleState` (matching the type in `file_logger.rs`).
  - Scans all entries to build a set of CSV columns.

- **GPS-related columns**
  - When any entry has `state.gps.is_some()`:
    - Columns like:
      - `gps.latitude_deg`
      - `gps.longitude_deg`
      - `gps.altitude_m`
      - `gps.north_mps`
      - `gps.east_mps`
      - `gps.down_mps`
      - `gps.timestamp_unix_ms`
      - `gps.has_fix`
  - Additionally:
    - `gps_valid`

- **Using the data**
  - The CSV output lets you correlate:
    - Flight computer timestamp (`timestamp`).
    - GPS position/velocity/time and `has_fix`.
    - `gps_valid` (whether the GPS data was fresh on that control-loop iteration).
    - Other vehicle telemetry (BMS, AHRS, valves, etc.).

This completes the overview of how GPS data is retrieved via the ZED-F9P driver, moved through a background worker thread and mailbox, integrated into `VehicleState`, and finally logged and exported for analysis.


