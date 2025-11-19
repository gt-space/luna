## GPS and RECO Data Retrieval and Logging Pipeline

This document explains how GPS and RECO data flows from the u-blox ZED-F9P driver and RECO board into the flight computer, and how it is logged to disk for later analysis.

- **Scope**: Flight computer integration and logging.
- **Driver details**: 
  - See `firmware/zedf9p04b/README.md` for low-level GPS driver documentation.
  - See `firmware/reco/README.md` for RECO driver documentation.

---

## High-Level Data Flow

- **Background worker**: A dedicated GPS/RECO worker thread:
  - Talks to the ZED-F9P over I2C at 20Hz using periodic mode and produces `GpsState` samples.
  - Communicates with three RECO MCUs (MCU A: spidev1.2, MCU B: spidev1.1, MCU C: spidev1.0) over SPI at 200Hz to get RECO telemetry.
  - Sends GPS data to all three RECO MCUs at 200Hz (same GPS data to each, with valid bit set appropriately).
- **Mailbox**: The worker publishes GPS and RECO samples into a single-slot mailbox (`GpsMailbox`), replacing the previous one. Publishing is limited to max 20Hz to reduce contention with the main loop.
- **Main loop ingestion**: The flight computer main loop non-blockingly pulls the latest sample (if any) once per control iteration and updates `VehicleState` with GPS and all three RECO states.
- **Validity flags**: 
  - `VehicleState.gps_valid` is set to `true` when a fresh sample is ingested and reset to `false` after telemetry for that iteration is sent.
  - `VehicleState.reco_valid` follows the same pattern for RECO data.
- **Logging**: The GPS worker thread logs the complete `VehicleState` (including GPS and RECO fields) at 200Hz to the asynchronous `FileLogger`, which writes timestamped entries to `.postcard` log files. Logging continues independently of servo connection status.
- **Post-processing**: The `utility` binary reads the `.postcard` logs and exports them to CSV, including GPS and RECO-related columns.

---

## 1. GPS Background Worker Thread

**Relevant file**: `flight2/src/gps.rs`

- **Mailbox types**
  - `GpsMailbox`, `GpsMailboxWriter`, `GpsMailboxReader` form a simple single-slot mailbox:
    - Internally: `Arc<Mutex<Option<GpsRecoState>>>`.
    - `GpsRecoState` contains:
      - `gps: Option<GpsState>` - Latest GPS sample
      - `reco: [Option<RecoState>; 3]` - Latest RECO samples from all three MCUs (index 0 = MCU A/spidev1.2, 1 = MCU B/spidev1.1, 2 = MCU C/spidev1.0)
    - **Writer** (`GpsMailboxWriter::publish`) overwrites the slot with the latest `GpsRecoState`.
    - **Reader** (`GpsMailboxReader::take_latest`) uses `try_lock` to avoid blocking; it returns `Some(sample)` and clears the slot, or `None` if locked/empty.

- **Handle for the main loop**
  - `GpsHandle` wraps a `GpsMailboxReader`:
    - `GpsHandle::try_get_sample()` is a non-blocking call used from the main control loop to grab at most one fresh sample per iteration.

- **Spawning the worker**
  - `GpsManager::spawn(i2c_bus: u8, address: Option<u16>, vehicle_state_receiver: mpsc::Receiver<VehicleState>, file_logger: Option<FileLogger>) -> Result<GpsHandle, GPSError>`:
    - Creates the mailbox (writer + reader).
    - Creates an `Arc<AtomicBool>` `running` flag.
    - Receives a channel receiver for vehicle state updates (for logging) and an optional file logger.
    - Spawns a thread that runs `gps_worker_loop(i2c_bus, address, writer, running_thread, vehicle_state_receiver, file_logger)`.
    - Returns a `GpsHandle` to the caller (the flight computer main).

- **Worker loop**
  - `gps_worker_loop` does the following:
    - Constructs the GPS driver: `let mut gps = GPS::new(i2c_bus, address)?;`.
    - Configures GPS to run at 20Hz using periodic mode:
      - `gps.set_measurement_rate(50, 1, 0)` (50ms period = 20Hz, nav_rate=1, UTC time).
    - Initializes three RECO drivers:
      - MCU A: `/dev/spidev1.2`
      - MCU B: `/dev/spidev1.1`
      - MCU C: `/dev/spidev1.0`
      - If any driver fails to initialize, that MCU is skipped (continues with remaining MCUs).
    - Enters a loop while `running.load(Ordering::Relaxed)` is `true`:
      - Receives vehicle state updates from main loop via channel (non-blocking).
      - Reads GPS data at 20Hz using `gps.read_pvt()` (periodic mode):
        - `Ok(Some(pvt))`: maps to `GpsState` via `map_pvt_to_state(&pvt)`, sets `gps_valid = true`.
        - `Ok(None)`: no PVT data available yet (normal).
        - `Err(e)`: logs the error.
      - Makes RECO transactions at 200Hz (every 5ms):
        - Prepares `FcGpsBody` from latest GPS data (or zeros if no GPS yet).
        - Sets `valid` bit: `true` when fresh GPS data arrived, `false` after first send.
        - Cycles through all three RECO drivers and sends the same GPS data to each.
        - Receives `RecoBody` from each MCU and converts to `RecoState`.
        - On transaction failure for a MCU: logs error and uses zeroed `RecoState` for that MCU.
      - Logs vehicle state at 200Hz (if file logger available):
        - Updates latest vehicle state with current GPS and RECO data.
        - Logs complete `VehicleState` via `FileLogger::log()` (non-blocking, may drop if channel full).
      - Publishes to mailbox at reduced rate (max 20Hz) to prevent main loop contention:
        - Publishes immediately when GPS data changes.
        - Otherwise publishes at most every 50ms (20Hz).
    - `map_pvt_to_state` constructs a `common::comm::GpsState`:
      - Position → `latitude_deg`, `longitude_deg`, `altitude_m`.
      - Velocity → `north_mps`, `east_mps`, `down_mps`.
      - Time → `timestamp_unix_ms` (via `timestamp_millis()`).
      - `has_fix` is `true` if position or velocity is present.
    - `map_reco_body_to_state` converts `reco::RecoBody` to `common::comm::RecoState`:
      - Includes quaternion, position, velocity, biases, scale factors, linear acceleration, angular rates, magnetometer data, temperature, pressure, and stage/VREF flags.

**Key properties**:

- **Backpressure**: The mailbox holds at most one sample; if the main loop is slow, intermediate GPS/RECO samples are dropped, but the latest state is always available.
- **Non-blocking**: The worker uses a blocking mutex (safe because it's off the main loop), while the main loop uses a non-blocking `try_lock` on the mailbox.
- **Rate limiting**: Mailbox publishing is limited to max 20Hz to prevent main loop slowdown from excessive mutex contention.
- **High-frequency logging**: File logging happens at 200Hz in the GPS worker thread, independent of servo connection status.
- **RECO redundancy**: All three RECO MCUs receive the same GPS data and provide independent telemetry for redundancy.

---

## 2. Main Loop Integration and `VehicleState` Updates

**Relevant files**: `flight2/src/main.rs`, `flight2/src/device.rs`, `common/src/comm.rs`

- **Spawning the worker**
  - In `main.rs`:
    - Creates a channel for vehicle state updates: `let (vehicle_state_sender, vehicle_state_receiver) = mpsc::channel();`
    - `let gps_handle = match gps::GpsManager::spawn(1, None, vehicle_state_receiver, file_logger.clone()) { ... };`
    - On success, the handle is stored as `Some(GpsHandle)`; on failure, the flight computer continues without GPS.

- **Per-iteration ingestion**
  - Inside the main control loop (`main.rs`):
    - The code checks for new samples:
      - 
        ```rust
        if let Some(handle) = gps_handle.as_ref() {
          if let Some(gps_reco_sample) = handle.try_get_sample() {
            if let Some(gps) = gps_reco_sample.gps {
              devices.update_gps(gps);
            }
            devices.update_reco(gps_reco_sample.reco);
          }
        }
        ```
    - This is **non-blocking** and will at most consume one GPS/RECO sample per iteration.
    - Sends vehicle state to GPS worker for logging (non-blocking, may drop if channel full):
      ```rust
      if let Some(ref sender) = gps_handle.as_ref().map(|_| &vehicle_state_sender) {
        let _ = sender.try_send(devices.get_state().clone());
      }
      ```

- **Updating `VehicleState`**
  - In `device.rs`:
    - `Devices` owns a `VehicleState` (`self.state`).
    - `Devices::update_gps(&mut self, sample: GpsState)`:
      - `self.state.gps = Some(sample);`
      - `self.state.gps_valid = true;`
    - `Devices::update_reco(&mut self, samples: [Option<RecoState>; 3])`:
      - `self.state.reco = samples;` (stores all three MCU states)
      - `self.state.reco_valid = true;`
    - `Devices::invalidate_gps(&mut self)`:
      - `self.state.gps_valid = false;`
    - `Devices::invalidate_reco(&mut self)`:
      - `self.state.reco_valid = false;`
  - In `common/src/comm.rs`, `VehicleState` contains:
    - `pub gps: Option<GpsState>,`
    - `pub gps_valid: bool,`
    - `pub reco: [Option<RecoState>; 3],` (index 0 = MCU A/spidev1.2, 1 = MCU B/spidev1.1, 2 = MCU C/spidev1.0)
    - `pub reco_valid: bool,`
    - The documentation notes:
      - `gps` holds the latest sample (for logging / debugging).
      - `gps_valid` is `true` only for the control-loop iteration in which a new sample was consumed.
      - `reco` holds the latest samples from all three MCUs (for logging / debugging).
      - `reco_valid` is `true` only for the control-loop iteration in which new samples were consumed.

- **Resetting validity flags after sending telemetry**
  - Still in the main loop (`main.rs`):
    - Telemetry is periodically pushed to Servo via:
      - `servo::push(&socket, servo_address, devices.get_state())` (file logging removed from servo path)
    - Immediately after sending:
      - 
        ```rust
        devices.invalidate_gps();
        devices.invalidate_reco();
        ```
    - This yields the following semantics:
      - **Fresh GPS/RECO for current iteration**: `gps_valid == true` / `reco_valid == true`.
      - **After telemetry sent**: `gps_valid == false` / `reco_valid == false` until the next `update_gps` / `update_reco` call.

**Why this design**:

- Keeps `gps` as the latest known sample for logging, even if it’s logically stale.
- Uses `gps_valid` as an explicit marker for “fresh this tick” so downstream code and logs can distinguish fresh vs. stale GPS data.

---

## 3. File Logging of GPS and RECO-Enhanced `VehicleState`

**Relevant files**: `flight2/src/file_logger.rs`, `flight2/src/gps.rs`

- **Timestamped wrapper**
  - `TimestampedVehicleState`:
    - `timestamp: f64` (Unix seconds with sub-second precision).
    - `state: VehicleState` (includes `gps`, `gps_valid`, `reco`, and `reco_valid`).

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

- **Logging location and frequency**
  - **Location**: Logging happens in the GPS worker thread (`gps.rs`), not in the servo send path.
  - **Frequency**: Logging occurs at 200Hz (every 5ms) in the GPS worker thread.
  - **Independence**: Logging continues even if the flight computer is disconnected from servo.
  - **Data flow**:
    - Main loop sends vehicle state updates to GPS worker via channel (non-blocking).
    - GPS worker receives vehicle state updates and merges with latest GPS and RECO data.
    - GPS worker logs complete `VehicleState` at 200Hz via `FileLogger::log()`.

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

**Effect on GPS and RECO data**:

- Each log entry captures:
  - The current `VehicleState.gps` (if any).
  - The current `VehicleState.gps_valid` flag.
  - The current `VehicleState.reco` array with all three MCU states (if any).
  - The current `VehicleState.reco_valid` flag.
  - Everything else (BMS, AHRS, valves, sensors, abort stage, etc.).
- Because validity flags are reset each time after sending telemetry, the logs allow you to:
  - See **when** GPS/RECO samples were freshly ingested (`gps_valid == true` / `reco_valid == true`).
  - Still inspect the last known GPS position/velocity and RECO telemetry even when validity flags are `false`.
  - Analyze data at 200Hz resolution for high-frequency events.

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

- **RECO-related columns**
  - When any entry has `state.reco[0].is_some()`, `state.reco[1].is_some()`, or `state.reco[2].is_some()`:
    - Columns for each MCU (indexed as `reco_0`, `reco_1`, `reco_2`):
      - `reco_0.quaternion[0-3]` (MCU A - spidev1.2)
      - `reco_0.lla_pos[0-2]` (longitude, latitude, altitude)
      - `reco_0.velocity[0-2]` (north, east, down)
      - `reco_0.lin_accel[0-2]` (linear acceleration)
      - `reco_0.angular_rate[0-2]` (angular rates)
      - `reco_0.temperature`
      - `reco_0.pressure`
      - `reco_0.stage1_enabled`, `reco_0.stage2_enabled`
      - `reco_0.vref_*_stage*` flags
      - (Similar columns for `reco_1` (MCU B) and `reco_2` (MCU C))
  - Additionally:
    - `reco_valid`

- **Using the data**
  - The CSV output lets you correlate:
    - Flight computer timestamp (`timestamp`).
    - GPS position/velocity/time and `has_fix`.
    - `gps_valid` (whether the GPS data was fresh on that control-loop iteration).
    - RECO telemetry from all three MCUs (quaternion, position, velocity, acceleration, angular rates, etc.).
    - `reco_valid` (whether the RECO data was fresh on that control-loop iteration).
    - Other vehicle telemetry (BMS, AHRS, valves, etc.).
    - High-frequency data at 200Hz resolution for detailed analysis.

This completes the overview of how GPS and RECO data is retrieved via the ZED-F9P driver and RECO board, moved through a background worker thread and mailbox, integrated into `VehicleState`, and finally logged at 200Hz and exported for analysis.


