use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};

use common::comm::{reco::TargetMCU, GpsState, RecoState, VehicleState};
use reco::{
    AltimeterOffsets, BAMRecoDriver, EkfStateVector, FcGpsBody, InitialCovarianceMatrix,
    MeasurementNoiseMatrix, ProcessNoiseMatrix, RecoBody, RecoDriver, RecoError, TimerValues,
};
use zedf9p04b::{GPSError, GPS, PVT};

use crate::{device::get_reco_rbf_values, file_logger::TimestampedVehicleState};

type SharedGpsState = Arc<Mutex<Option<GpsState>>>;

/// Combined GPS and RECO state for mailbox
/// RECO array indices: 0 = MCU A (spidev1.2), 1 = MCU B (spidev1.1), 2 = MCU C
/// (spidev1.0)
/// BAM only uses index 0, indices 1 and 2 are empty.
#[derive(Clone)]
pub struct GpsRecoState {
    pub gps: Option<GpsState>,
    pub reco: [Option<RecoState>; 3],
}

/// Control messages for one-shot/special RECO commands.
#[derive(Clone, Debug)]
pub enum RecoControlMessage {
    /// Informs all RECO MCUs (or BAM) that the rocket has launched.
    Launch,

    /// Requests that all RECO MCUs (or BAM) initialize (or reinitialize) their EKFs.
    InitEKF,

    /// Sends process noise matrix to RECO.
    ProcessNoiseMatrix {
        target: TargetMCU,
        matrix: ProcessNoiseMatrix,
    },

    /// Sends measurement noise matrix to RECO.
    MeasurementNoiseMatrix {
        target: TargetMCU,
        matrix: MeasurementNoiseMatrix,
    },

    /// Sends EKF state vector to RECO.
    EkfStateVector {
        target: TargetMCU,
        vector: EkfStateVector,
    },

    /// Sends initial covariance matrix to RECO.
    InitialCovarianceMatrix {
        target: TargetMCU,
        matrix: InitialCovarianceMatrix,
    },

    /// Sends timer values to RECO.
    TimerValues {
        target: TargetMCU,
        values: TimerValues,
    },

    /// Sends altimeter offsets to RECO.
    AltimeterOffsets {
        target: TargetMCU,
        offsets: AltimeterOffsets,
    },
}

#[derive(Clone)]
pub struct GpsMailboxWriter {
    inner: Arc<Mutex<Option<GpsRecoState>>>,
}

#[derive(Clone)]
pub struct GpsMailboxReader {
    inner: Arc<Mutex<Option<GpsRecoState>>>,
}

/// Creates the writer/reader pair for the single-slot GPS/RECO mailbox. Both
/// the writer and reader share the same inner state, guarded by a mutex.
fn create_gps_reco_mailbox() -> (GpsMailboxWriter, GpsMailboxReader) {
    let inner = Arc::new(Mutex::new(None));
    (
        GpsMailboxWriter {
            inner: inner.clone(),
        },
        GpsMailboxReader { inner },
    )
}

impl GpsMailboxWriter {
    /// Publish a new GPS and RECO sample, replacing any previous one.
    ///
    /// This uses a blocking mutex lock, but it only runs on the GPS worker
    /// thread, so it cannot stall the control loop.
    pub fn publish(&self, mut sample: GpsRecoState) {
        if let Ok(mut slot) = self.inner.lock() {
            // keep the old unread sample if new data is None
            if let Some(unread) = slot.as_ref() {
                for (incoming, previous) in sample.reco.iter_mut().zip(unread.reco.iter()) {
                    if incoming.is_none() {
                        *incoming = previous.clone();
                    }
                }
            }
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
    /// Sender for special/one-shot RECO control messages.
    reco_control_sender: mpsc::Sender<RecoControlMessage>,
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

    /// Enqueue a special RECO control message for the GPS/RECO worker thread to
    /// process. Errors if the worker has exited and the channel is closed.
    pub fn send_reco_control(&self, msg: RecoControlMessage) -> Result<(), RecoControlSendError> {
        self.reco_control_sender
            .send(msg)
            .map_err(|_| RecoControlSendError)
    }
}

#[derive(Debug)]
pub struct RecoControlSendError;

pub struct GpsManager;

const RECO_MCU_NAMES: [&str; 3] = [
    "MCU A (spidev1.2)",
    "MCU B (spidev1.1)",
    "MCU C (spidev1.0)",
];

fn reco_mcu_name(bam: bool, index: usize) -> &'static str {
    if bam {
        "BAM"
    } else {
        RECO_MCU_NAMES[index]
    }
}

/// Enum to combine the logic of the two posisble drivers
enum RecoConnection {
    Spi(RecoDriver),
    Bam(BAMRecoDriver),
}

impl RecoConnection {
    fn send_control(&mut self, message: &RecoControlMessage) -> Result<(), RecoError> {
        // Both drivers expose the same control methods.
        macro_rules! send {
            ($method:ident $(, $arg:expr)*) => {
                match self {
                    Self::Spi(driver) => driver.$method($($arg),*),
                    Self::Bam(driver) => driver.$method($($arg),*),
                }
            };
        }

        match message {
            RecoControlMessage::Launch => send!(send_launched),
            RecoControlMessage::InitEKF => send!(send_init_ekf),
            RecoControlMessage::ProcessNoiseMatrix { matrix, .. } => {
                send!(send_process_noise_matrix, matrix)
            }
            RecoControlMessage::MeasurementNoiseMatrix { matrix, .. } => {
                send!(send_measurement_noise_matrix, matrix)
            }
            RecoControlMessage::EkfStateVector { vector, .. } => {
                send!(send_ekf_state_vector, vector)
            }
            RecoControlMessage::InitialCovarianceMatrix { matrix, .. } => {
                send!(send_initial_covariance_matrix, matrix)
            }
            RecoControlMessage::TimerValues { values, .. } => send!(send_timer_values, values),
            RecoControlMessage::AltimeterOffsets { offsets, .. } => {
                send!(send_altimeter_offsets, offsets)
            }
        }
    }

    fn sample(&mut self, gps_body: &FcGpsBody) -> Result<Option<RecoBody>, RecoError> {
        match self {
            Self::Spi(driver) => driver.send_gps_data_and_receive_reco(gps_body).map(Some),
            Self::Bam(driver) => {
                // UDP send and receive are independent; a delayed reply must
                // not block the worker or be discarded if sending fails.
                if let Err(e) = driver.send_gps_data(gps_body) {
                    if std::env::var("RECO_DEBUG").is_ok() {
                        eprintln!("Error sending GPS data to BAM: {e}");
                    }
                }
                let mut latest = None;
                for _ in 0..32 {
                    match driver.try_receive_data() {
                        Ok(Some(body)) => latest = Some(body),
                        Ok(None) => break,
                        Err(e) => {
                            if std::env::var("RECO_DEBUG").is_ok() {
                                eprintln!("Error receiving BAM telemetry: {e}");
                            }
                        }
                    }
                }
                Ok(latest)
            }
        }
    }
}

fn send_reco_command(
    reco_drivers: &mut [Option<RecoConnection>; 3],
    bam: bool,
    message: &RecoControlMessage,
) {
    let (target, command_name) = match message {
        RecoControlMessage::Launch => (TargetMCU::All, "launch message"),
        RecoControlMessage::InitEKF => (TargetMCU::All, "EKF-init message"),
        RecoControlMessage::ProcessNoiseMatrix { target, .. } => (*target, "process-noise matrix"),
        RecoControlMessage::MeasurementNoiseMatrix { target, .. } => {
            (*target, "measurement-noise matrix")
        }
        RecoControlMessage::EkfStateVector { target, .. } => (*target, "EKF state vector"),
        RecoControlMessage::InitialCovarianceMatrix { target, .. } => {
            (*target, "initial covariance matrix")
        }
        RecoControlMessage::TimerValues { target, .. } => (*target, "timer values"),
        RecoControlMessage::AltimeterOffsets { target, .. } => (*target, "altimeter offsets"),
    };
    // BAM has one MCU, so every GUI target addresses that single driver.
    let target_index = if bam {
        Some(0)
    } else {
        match target {
            TargetMCU::All => None,
            TargetMCU::A => Some(0),
            TargetMCU::B => Some(1),
            TargetMCU::C => Some(2),
        }
    };

    for (index, driver) in reco_drivers.iter_mut().enumerate() {
        if target_index.is_some_and(|target_index| target_index != index) {
            continue;
        }
        let name = reco_mcu_name(bam, index);
        if let Some(driver) = driver {
            if let Err(e) = driver.send_control(message) {
                eprintln!("Error sending RECO {command_name} to {name}: {e}");
            }
        } else if target_index.is_some() {
            eprintln!("Error sending RECO {command_name} to {name}: driver not initialized");
        }
    }
}

impl GpsManager {
    /// Spawn a background worker thread that talks to the GPS over I2C and
    /// publishes samples into a mailbox.
    /// `bam` selects one UDP MCU instead of the normal three SPI MCUs.
    ///
    /// `vehicle_state_receiver` is used to receive vehicle state updates for
    /// logging. `file_logger_sender` is the sender for logging vehicle state at
    /// 200Hz. `print_gps` enables printing GPS data to terminal at ~1Hz.
    pub fn spawn(
        i2c_bus: u8,
        address: Option<u16>,
        vehicle_state_receiver: mpsc::Receiver<VehicleState>,
        file_logger_sender: Option<mpsc::SyncSender<TimestampedVehicleState>>,
        print_gps: bool,
        bam: bool,
    ) -> Result<GpsHandle, GPSError> {
        let (writer, reader) = create_gps_reco_mailbox();
        let running = Arc::new(AtomicBool::new(true));
        // Shared GPS state between the dedicated GPS reader thread and the
        // RECO/logging worker.
        let shared_gps_state: SharedGpsState = Arc::new(Mutex::new(None));

        // Channel for special RECO control messages.
        let (reco_control_sender, reco_control_receiver) = mpsc::channel::<RecoControlMessage>();

        // Spawn dedicated GPS reader thread (20Hz, I2C only).
        {
            let running_for_gps = running.clone();
            let gps_state_for_gps = shared_gps_state.clone();
            thread::spawn(move || {
                let result = gps_reader_loop(
                    i2c_bus,
                    address,
                    running_for_gps.clone(),
                    gps_state_for_gps,
                    print_gps,
                );

                // Keep normal RECO shutdown, but let BAM UDP continue without GPS
                if !bam {
                    running_for_gps.store(false, Ordering::Relaxed);
                }

                if let Err(e) = result {
                    eprintln!("GPS reader exited with error: {e}");
                }
            });
        }

        // Spawn RECO + logging worker thread (200Hz, SPI normally or UDP with BAM, no I2C).
        {
            let running_for_reco = running.clone();
            let gps_state_for_reco = shared_gps_state.clone();
            let reco_control_receiver = reco_control_receiver;
            thread::spawn(move || {
                let result = gps_worker_loop(
                    gps_state_for_reco,
                    writer,
                    running_for_reco.clone(),
                    vehicle_state_receiver,
                    file_logger_sender,
                    reco_control_receiver,
                    bam,
                );

                // Mark the worker as no longer running, regardless of success
                // or error.
                running_for_reco.store(false, Ordering::Relaxed);

                if let Err(e) = result {
                    eprintln!("GPS/RECO worker exited with error: {e}");
                }
            });
        }

        let handle = GpsHandle {
            reader,
            _running: running,
            reco_control_sender: reco_control_sender.clone(),
        };

        Ok(handle)
    }
}

fn gps_reader_loop(
    i2c_bus: u8,
    address: Option<u16>,
    running: Arc<AtomicBool>,
    shared_gps_state: SharedGpsState,
    print_gps: bool,
) -> Result<(), GPSError> {
    // Optional performance debug logging for GPS reader.
    let perf_debug = std::env::var("GPS_READER_PERF_DEBUG").is_ok();
    if perf_debug {
        eprintln!("GPS_READER_PERF_DEBUG enabled");
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

    // Rate limiting for GPS reads (20Hz -> 50ms interval)
    let gps_interval = Duration::from_millis(50);
    let mut last_gps_poll = Instant::now();

    let mut curr_time = if perf_debug {
        Some(Instant::now())
    } else {
        None
    };

    let mut prev_last_gps_poll;

    while running.load(Ordering::Relaxed) {
        let loop_now = Instant::now();

        if loop_now.duration_since(last_gps_poll) >= gps_interval {
            prev_last_gps_poll = last_gps_poll;
            last_gps_poll = loop_now;

            let gps_start = if perf_debug {
                Some(Instant::now())
            } else {
                None
            };

            match gps.read_pvt() {
                Ok(Some(pvt)) => {
                    if let Some(start) = gps_start {
                        let dur = start.elapsed();
                        if dur > Duration::from_millis(1) {
                            eprintln!(
                                "GPS reader: gps.read_pvt_raw() took {:.2} ms",
                                dur.as_secs_f64() * 1000.0
                            );
                        }
                    }

                    if let Some(state) = map_pvt_to_state(&pvt) {
                        if let Some(start) = gps_start {
                            let dur = start.elapsed();
                            if dur > Duration::from_millis(1) {
                                eprintln!(
                                    "GPS reader: gps.read_pvt_mapping() took {:.2} ms",
                                    dur.as_secs_f64() * 1000.0
                                );
                            }
                        }

                        // Update shared GPS state for the RECO/logging worker
                        // and main loop.
                        if let Ok(mut guard) = shared_gps_state.lock() {
                            *guard = Some(state.clone());
                        }

                        // Print GPS data to terminal if enabled and enough time
                        // has passed.
                        if print_gps && loop_now.duration_since(last_print_time) >= print_interval {
                            print_gps_state(&state);
                            last_print_time = loop_now;
                        }
                    }
                }
                Ok(None) => {
                    // No PVT data available yet, this is normal.
                    // Still print "no fix" message if printing is enabled.
                    if print_gps && loop_now.duration_since(last_print_time) >= print_interval {
                        println!("GPS: No fix");
                        last_print_time = loop_now;
                    }
                    last_gps_poll = prev_last_gps_poll;
                    thread::sleep(Duration::from_millis(1));
                }
                Err(e) => {
                    eprintln!("Error while reading GPS PVT: {e}");
                }
            }

            if let Some(start) = gps_start {
                let dur = start.elapsed();
                if dur > Duration::from_millis(1) {
                    eprintln!(
                        "GPS reader: gps.read_pvt() took {:.2} ms",
                        dur.as_secs_f64() * 1000.0
                    );
                    eprintln!(
                        "Time between last gps read and now{:?}",
                        Instant::now().duration_since(curr_time.unwrap())
                    );
                    curr_time = Some(Instant::now());
                }
            }
        }

        // Small delay to avoid busy-waiting.
        thread::sleep(Duration::from_millis(1));
    }

    Ok(())
}

fn gps_worker_loop(
    shared_gps_state: SharedGpsState,
    writer: GpsMailboxWriter,
    running: Arc<AtomicBool>,
    vehicle_state_receiver: mpsc::Receiver<VehicleState>,
    file_logger_sender: Option<mpsc::SyncSender<TimestampedVehicleState>>,
    reco_control_receiver: mpsc::Receiver<RecoControlMessage>,
    bam: bool,
) -> Result<(), GPSError> {
    // Optional performance debug logging for GPS/RECO worker.
    let perf_debug = std::env::var("GPS_RECO_PERF_DEBUG").is_ok();
    if perf_debug {
        eprintln!("GPS_RECO_PERF_DEBUG enabled");
    }

    // Optional verbose printing of RECO data received from the MCUs.
    // Enable by setting the environment variable `PRINT_RECV_FROM_RECO`.
    let print_recv_from_reco = std::env::var("PRINT_RECV_FROM_RECO").is_ok();

    let mut reco_drivers: [Option<RecoConnection>; 3] = [None, None, None];
    if bam {
        match BAMRecoDriver::from_config() {
            Ok(driver) => {
                eprintln!("BAM UDP driver initialized successfully");
                reco_drivers[0] = Some(RecoConnection::Bam(driver));
            }
            Err(e) => {
                eprintln!(
                    "Failed to initialize BAM UDP driver: {e}. Continuing without BAM telemetry."
                );
            }
        }
    } else {
        for (index, path) in ["/dev/spidev1.2", "/dev/spidev1.1", "/dev/spidev1.0"]
            .into_iter()
            .enumerate()
        {
            let name = reco_mcu_name(false, index);
            match RecoDriver::new(path) {
                Ok(driver) => {
                    eprintln!("RECO driver {name} initialized successfully");
                    reco_drivers[index] = Some(RecoConnection::Spi(driver));
                }
                Err(e) => {
                    eprintln!("Failed to initialize RECO driver {name}: {e}. Continuing without this MCU.");
                }
            }
        }
    }

    // Track last GPS data and valid flag
    let mut last_gps_state: Option<GpsState> = None;
    let mut gps_valid = false;
    // Track if GPS data changed (used for mailbox publishing)
    let mut gps_data_changed = false;
    // Keep samples that arrive between mailbox publications (for BAM UDP comms)
    let mut pending_reco_states: [Option<RecoState>; 3] = [None, None, None];

    // Track latest vehicle state for logging
    // Initialize with default state so we can always log even if no messages
    // received yet
    let mut latest_vehicle_state: Option<VehicleState> = Some(VehicleState::default());

    // Rate limiting for 200Hz RECO transactions (5ms interval)
    let reco_interval = Duration::from_millis(5);
    let mut last_reco_time = Instant::now();

    // Track last time we published to mailbox (only publish when GPS data
    // changed, not every 5ms)
    let mut last_publish_time = Instant::now();
    let publish_interval = Duration::from_millis(50); // Publish at most 20Hz to
                                                      // reduce contention

    // Main GPS acquisition, RECO transaction, and logging loop
    while running.load(Ordering::Relaxed) {
        // All RECO access happens on this worker thread to avoid contention.
        while let Ok(message) = reco_control_receiver.try_recv() {
            send_reco_command(&mut reco_drivers, bam, &message);
        }

        // Receive vehicle state updates from main loop (non-blocking)
        while let Ok(state) = vehicle_state_receiver.try_recv() {
            latest_vehicle_state = Some(state);
        }

        let loop_now = Instant::now();

        // Make RECO transaction at 200Hz (every 5ms)
        // Do this FIRST to ensure it runs at exactly 200Hz, regardless of GPS
        // reading delays
        if loop_now.duration_since(last_reco_time) >= reco_interval {
            last_reco_time = loop_now;

            let reco_start = if perf_debug {
                Some(Instant::now())
            } else {
                None
            };

            // Pull the latest GPS state from the GPS reader thread and detect
            // if it changed.
            if let Ok(guard) = shared_gps_state.lock() {
                if let Some(ref shared_state) = *guard {
                    let shared_ts = shared_state.timestamp_unix_ms;
                    let last_ts = last_gps_state.as_ref().and_then(|s| s.timestamp_unix_ms);
                    if last_ts != shared_ts {
                        if reco_start.is_some() {
                            println!("GPS data changed");
                            println!("shared_ts: {:?}", shared_ts.unwrap());
                            println!("last_ts: {:?}", last_ts);
                            println!("shared_state: {:?}", shared_state);
                            println!("last_gps_state: {:?}", last_gps_state);
                        }
                        last_gps_state = Some(shared_state.clone());
                        gps_valid = true; // New GPS sample arrived for this
                                          // RECO/logging cycle.
                        gps_data_changed = true;
                    }
                }
            }

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

            let mut reco_states: [Option<RecoState>; 3] = [None, None, None];
            for (index, driver) in reco_drivers.iter_mut().enumerate() {
                let name = reco_mcu_name(bam, index);
                // Normal RECO will still use default RecoState for unavailable MCUs
                // BAM will keep them as None
                if !bam {
                    reco_states[index] = Some(RecoState::default());
                }
                if let Some(driver) = driver {
                    match driver.sample(&gps_body) {
                        Ok(Some(body)) => {
                            let state = map_reco_body_to_state(&body);
                            // Optional nicely formatted GPS/RECO debug output
                            // controlled by
                            // PRINT_RECV_FROM_RECO.
                            if print_recv_from_reco {
                                println!(
                    "[RECO {}] SENT gps: lat={:.6}, lon={:.6}, alt={:.2}, vn={:.2}, ve={:.2}, vd={:.2}, valid={}",
                    name,
                    gps_body.latitude,
                    gps_body.longitude,
                    gps_body.altitude,
                    gps_body.velocity_north,
                    gps_body.velocity_east,
                    gps_body.velocity_down,
                    gps_body.valid,
                  );
                                println!("[RECO {}] RECV state: {:#?}", name, state);
                            }
                            reco_states[index] = Some(state);
                        }
                        Ok(None) => {}
                        Err(e) => {
                            if std::env::var("RECO_DEBUG").is_ok() {
                                eprintln!("Error in RECO transaction with {name}: {e}. Using zeroed RECO values.");
                            }
                        }
                    }
                }
                if reco_states[index].is_some() {
                    pending_reco_states[index] = reco_states[index].clone();
                }
            }

            // Log vehicle state at 200Hz if logger is available
            // This runs every 5ms (200Hz) as part of the RECO transaction loop
            // Since RECO transactions run at exactly 200Hz (every 5ms), we log
            // every iteration No need for separate time check -
            // RECO timing already ensures 200Hz rate
            if let Some(ref logger_sender) = file_logger_sender {
                // Use the latest vehicle state (should always be Some after
                // initialization)
                if let Some(ref state) = latest_vehicle_state {
                    // Create updated state with latest GPS and RECO data
                    let mut updated_state = state.clone();
                    updated_state.gps = last_gps_state.clone();
                    updated_state.reco = reco_states.clone().into();
                    updated_state.gps_valid = gps_valid;
                    updated_state.reco_valid = reco_states.iter().any(Option::is_some);
                    updated_state.rbf.reco = get_reco_rbf_values(&reco_states);

                    // Create timestamped state using the same timestamp
                    // function as FileLogger
                    use crate::file_logger;
                    let timestamp = file_logger::current_timestamp();
                    let timestamped = file_logger::TimestampedVehicleState {
                        timestamp,
                        state: updated_state,
                    };

                    // Log (non-blocking, may drop if channel is full)
                    // If channel is full, this will silently fail - consider
                    // increasing channel capacity
                    if logger_sender.try_send(timestamped).is_err() {
                        // Channel is full - this means the file logger can't
                        // keep up This shouldn't happen
                        // at 200Hz, but if it does, we're dropping
                        // samples
                    }
                }
            }

            // Only publish to mailbox when GPS data changed or at reduced rate
            // (max 20Hz) This reduces mailbox contention and
            // prevents main loop slowdown
            let now = Instant::now();
            if gps_data_changed || now.duration_since(last_publish_time) >= publish_interval {
                writer.publish(GpsRecoState {
                    gps: last_gps_state.clone(),
                    reco: std::mem::take(&mut pending_reco_states),
                });
                last_publish_time = now;
                gps_data_changed = false;
            }

            // After first send, set valid to false for subsequent sends
            if gps_valid {
                gps_valid = false;
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
        num_satellites: pvt.num_sats.unwrap_or(0),
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
        vref_ch1_dr1: reco_body.vref_ch1_dr1,
        vref_ch1_dr2: reco_body.vref_ch1_dr2,
        vref_ch2_dr1: reco_body.vref_ch2_dr1,
        vref_ch2_dr2: reco_body.vref_ch2_dr2,
        sns1_current: reco_body.sns1_current,
        sns2_current: reco_body.sns2_current,
        v_rail_24v: reco_body.v_rail_24v,
        v_rail_3v3: reco_body.v_rail_3v3,
        fading_memory_baro: reco_body.fading_memory_baro,
        fading_memory_gps: reco_body.fading_memory_gps,
        stage1_enabled: reco_body.stage1_enabled,
        stage2_enabled: reco_body.stage2_enabled,
        reco_recvd_launch: reco_body.reco_recvd_launch,
        reco_driver_faults: reco_body.reco_driver_faults,
        ekf_blown_up: reco_body.ekf_blown_up,
        drouge_timer_enable: reco_body.drouge_timer_enable,
        main_timer_enable: reco_body.main_timer_enable,
        rbf_enabled: reco_body.rbf_enabled,
    }
}
