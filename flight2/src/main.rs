mod device;
mod servo;
mod state;
mod sequence;
mod file_logger;
mod gps;

// TODO: Make it so you enter servo's socket address.
// TODO: Clean up domain socket on exit.
use std::{collections::HashMap, default, env, net::{SocketAddr, TcpStream, UdpSocket}, os::unix::net::UnixDatagram, path::PathBuf, process::Command, sync::mpsc, thread, time::{Duration, Instant}};
use common::{comm::{AbortStage, FlightControlMessage, Sequence}, sequence::{MMAP_PATH, SOCKET_PATH}};
use common::comm::bms;
use crate::{device::Devices, servo::ServoError, sequence::Sequences, state::Ingestible, device::Mappings, device::AbortStages, file_logger::{FileLogger, LoggerConfig}};
use mmap_sync::synchronizer::Synchronizer;
use wyhash::WyHash;
use mmap_sync::locks::LockDisabled;
use servo::servo_keep_alive_delay;
use clap::Parser;

const SERVO_SOCKET_ADDRESSES: [(&str, u16); 4] = [
  ("192.168.1.10", 5025),
  ("server-01.local", 5025),
  ("server-02.local", 5025),
  ("localhost", 5025),
];
const FC_SOCKET_ADDRESS: (&str, u16) = ("0.0.0.0", 4573);
const DEVICE_COMMAND_PORT: u16 = 8378;
const SERVO_DATA_PORT: u16 = 7201;

/// How quickly a sequence must read from the shared VehicleState before the
/// data becomes corrupted.
const MMAP_GRACE_PERIOD: Duration = Duration::from_millis(20);

/// How long from the last received message before a board is considered
/// disconnected.
const TIME_TO_LIVE: Duration = Duration::from_millis(350);

/// How many times a reconnect will be tried with a disconnected servo.
const SERVO_RECONNECT_RETRY_COUNT: u8 = 1;

/// The TCP timeout for re-establishing connection with a disconnected servo.
const SERVO_RECONNECT_TIMEOUT: Duration = Duration::from_millis(50);

/// How often the refresh rate data decays over time.
const DECAY: f64 = 0.9;

/// How often we want to update servo
const FC_TO_SERVO_RATE: Duration = Duration::from_millis(10);

// How often we want to log
const LOG_INTERVAL: Duration = Duration::from_millis(5);

/// How often we want to send hearbeats
const SEND_HEARTBEAT_RATE: Duration = Duration::from_millis(50);

/// If we do not hear from servo for this amount of time, we abort
const SERVO_TO_FC_TIME_TO_LIVE: Duration = Duration::from_secs(1); // 1 second buffer

const GOLDFISH_SYSTEM_SAFE_TIMER: Duration = Duration::from_secs(60 * 25); // 25 minutes

/// If the umbilical bus voltage drops below this threshold and we have observed 
/// valid umbilical bus voltage samples, we start the goldfish system safe timer.
/// Ground computer configuration should not be affected. 
const UMBILICAL_BUS_VOLTAGE_THRESHOLD: f64 = 10.0; // 10 V

/// Command-line arguments for the flight computer
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
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
    
    /// Print GPS data to terminal at ~1Hz (disabled by default)
    #[arg(long, default_value_t = false)]
    print_gps: bool,
}

fn main() -> ! {
  // Parse command-line arguments
  let args = Args::parse();
  
  Command::new("rm").arg(SOCKET_PATH).output().unwrap();
  // TODO: kill duplicate process on boot

  // Checks if all the python dependencies are in order.
  if let Err(missing) = check_python_dependencies(&["common"]) {
    let mut error_message = "The following packages are missing:".to_string();

    for dependency in missing {
      error_message.push_str("\n\t");
      error_message.push_str(dependency);
    }

    panic!("{}", error_message);
  }

  // Initialize file logger
  let file_logger_config = LoggerConfig {
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

  let file_logger = match FileLogger::new(file_logger_config.clone()) {
    Ok(logger) => {
      if !args.disable_file_logging {
        println!("File logging enabled. Log directory: {:?}", 
                 file_logger_config.log_dir);
      }
      Some(logger)
    }
    Err(e) => {
      eprintln!("Warning: Failed to initialize file logger: {}. Continuing without file logging.", e);
      None
    }
  };

  let socket: UdpSocket = UdpSocket::bind(FC_SOCKET_ADDRESS).expect(&format!("Couldn't open port {} on IP address {}", FC_SOCKET_ADDRESS.1, FC_SOCKET_ADDRESS.0));
  socket.set_nonblocking(true).expect("Cannot set incoming to non-blocking.");
  let command_socket: UnixDatagram = UnixDatagram::bind(SOCKET_PATH).expect(&format!("Could not open sequence command socket on path '{SOCKET_PATH}'."));
  command_socket.set_nonblocking(true).expect("Cannot set sequence command socket to non-blocking.");

  // TODO: HAVE THIS IN A STRUCT CALLED MAIN LOOP DATA
  let mut mappings: Mappings = Vec::new();
  let mut devices: Devices = Devices::new();
  let mut sequences: Sequences = HashMap::new();
  let mut synchronizer: Synchronizer<WyHash, LockDisabled, 1024, 500_000> = Synchronizer::with_params(MMAP_PATH.as_ref());
  let mut abort_sequence: Option<Sequence> = None;
  let mut abort_stages: AbortStages = Vec::new();

  // Create channel for sending vehicle state to GPS worker for logging (bounded for try_send)
  let (vehicle_state_sender, vehicle_state_receiver) = mpsc::sync_channel(100);

  // Clone file logger sender for GPS worker thread
  let file_logger_sender = file_logger.as_ref().map(|logger| logger.clone_sender());

  // Spawn GPS worker thread. If initialization fails, continue without GPS/RECO.
  let (gps_handle, reco_cmd_sender) = match gps::GpsManager::spawn(1, None, vehicle_state_receiver, file_logger_sender, args.print_gps) {
    Ok((handle, reco_sender)) => {
      println!("GPS worker started successfully on I2C bus 1.");
      if args.print_gps {
        println!("GPS data printing enabled (rate: ~1Hz)");
      }
      (Some(handle), Some(reco_sender))
    }
    Err(e) => {
      eprintln!("Failed to start GPS/RECO worker: {e}. Continuing without GPS/RECO.");
      (None, None)
    }
  };
  
  println!("Flight Computer running on version {}\n", env!("CARGO_PKG_VERSION"));
  println!("!!!! ATTENTION !!! ATTENTION !!!!");
  println!(" THIS VERSION IS HIGHLY UNSTABLE ");
  println!("!!!! ATTENTION !!! ATTENTION !!!!");
  println!("DO NOT USE FOR ANYTHING DANGEROUS");
  println!("!!!! ATTENTION !!! ATTENTION !!!!");
  thread::sleep(Duration::from_secs(5));
  println!("\nStarting...\n");

  // Enable optional performance debug logging for the main loop.
  let fc_perf_debug = env::var("FC_PERF_DEBUG").is_ok();
  if fc_perf_debug {
    eprintln!("FC_PERF_DEBUG enabled");
  }

  let mut last_received_from_servo = Instant::now(); // last time that we had an established connection with servo
  let (mut servo_stream, mut servo_address)= loop {
    match servo::establish(&SERVO_SOCKET_ADDRESSES, None, 3, Duration::from_secs(2)) {
      Ok(s) => {
        println!("Connected to servo successfully. Beginning control cycle...\n");
        last_received_from_servo = Instant::now();
        break s;
      },
      Err(e) => {
        println!("Couldn't connect due to error: {e}\n");
        thread::sleep(Duration::from_secs(2));
      },
    }
  };

  // TODO: put this information into a struct, maybe call it main_loop_info or something?  
  let mut last_sent_to_servo = Instant::now(); // for sending messages to servo
  let mut last_heartbeat_sent = Instant::now(); // for sending messages to boards
  let mut aborted = false;
  let mut last_sent_to_gps_worker = Instant::now();
  // Tracks when umbilical bus voltage first drops to 0 V.
  let mut umbilical_drop_start: Option<Instant> = None;
  // Tracks whether we've already disabled SAM power for the current umbilical drop event.
  let mut sam_power_disabled_for_goldfish = false;
  // Tracks whether we've ever observed a valid umbilical bus voltage sample on this run.
  // This prevents the Goldfish timer from running in configurations where the umbilical
  // bus is not physically connected (ie. ground computer)
  let mut seen_valid_umbilical_voltage = false;
  loop {
    let loop_start = Instant::now();

    // Pull any new message from servo if we are still communicating with it.
    let servo_message = if devices.servo_communication_enabled() {
      get_servo_data(
        &mut servo_stream,
        &mut servo_address,
        &mut last_received_from_servo,
        &mut aborted,
        &mut devices,
      )
    } else {
      None
    };

    let servo_disconnect_abort_active = devices.monitor_servo_disconnects();

    if !aborted
      && servo_disconnect_abort_active
      && (Instant::now().duration_since(last_received_from_servo) > SERVO_TO_FC_TIME_TO_LIVE) 
    {
      println!(
        "FC to Servo timer of {} has expired while servo disconnect monitoring is enabled. Sending abort messages to boards.",
        SERVO_TO_FC_TIME_TO_LIVE.as_secs_f64()
      );
      aborted = true;
      // On servo loss-of-communication while on the ground, we immediately abort after
      // SERVO_TO_FC_TIME_TO_LIVE seconds.
      devices.send_sams_abort(&socket, &mappings, &mut abort_stages, &mut sequences, true);
    }

    // decoding servo message, if it was received
    if let Some(command) = servo_message {
      println!("Recieved a FlightControlMessage: {command:#?}");

      match command {
        FlightControlMessage::Abort => {
          // check which type of abort should happen, abort stage or abort seq
          if devices.get_state().abort_stage.name != "DEFAULT" {
            devices.send_sams_abort(&socket, &mappings, &mut abort_stages, &mut sequences, true); // abort message means we use stage timers
          } else {
            abort(&mappings, &mut sequences, &abort_sequence);
          }
        },
        FlightControlMessage::AbortStageConfig(config) => devices.create_abort_stage(&mappings, &mut abort_stages, config),
        FlightControlMessage::SetAbortStage(stage_name) => devices.handle_setting_abort_stage(&socket, stage_name, &mut abort_stages),
        FlightControlMessage::AhrsCommand(c) => devices.send_ahrs_command(&socket, c),
        FlightControlMessage::BmsCommand(c) => devices.send_bms_command(&socket, c),
        FlightControlMessage::Trigger(_) => todo!(),
        FlightControlMessage::Mappings(m) => {
          mappings = m;
      
          // send clear message to sams. this is needed as with new mappings we restart the
          // abort stage sequence and are in the default stage again. 
          devices.send_sam_clear_abort_stage(&socket);

          // restart the abort stage sequence
          start_abort_stage_process(&mut abort_stages, &mappings, &mut sequences, &mut devices);
        },
        FlightControlMessage::Sequence(s) if s.name == "abort" => abort_sequence = Some(s),
        FlightControlMessage::Sequence(ref s) => sequence::execute(&mappings, s, &mut sequences),
        FlightControlMessage::StopSequence(n) => {
          if let Err(e) = sequence::kill(&mut sequences, &n) {
            eprintln!("There was an issue in stopping sequence '{n}': {e}");
          }
        },
        FlightControlMessage::CameraEnable(should_enable) => devices.send_sams_toggle_camera(&socket, should_enable),
        _ => eprintln!("Received a FlightControlMessage that is not supported: {command:#?}"),
      };
    }

    // updates records
    devices.update_last_updates();

    // Ingest any newly available GPS and RECO samples without blocking the control loop.
    if let Some(handle) = gps_handle.as_ref() {
      if let Some(gps_reco_sample) = handle.try_get_sample() {
        if let Some(gps) = gps_reco_sample.gps {
          devices.update_gps(gps);
        }
        // Update all three RECO MCU states
        devices.update_reco(gps_reco_sample.reco);
      }
    }

    // Send vehicle state to GPS worker for logging (non-blocking, may drop if channel is full).
    // If the GPS worker is not running (e.g., missing hardware), fall back to logging directly
    // from the main loop using the FileLogger.
    let now = Instant::now();
    if now.duration_since(last_sent_to_gps_worker) >= LOG_INTERVAL {
      if let Some(handle) = gps_handle.as_ref() {
        if handle.is_running() {
          let _ = vehicle_state_sender.try_send(devices.get_state().clone());
        } else if let Some(ref logger) = file_logger.as_ref() {
          let _ = logger.log(devices.get_state().clone());
        }
      } else if let Some(ref logger) = file_logger.as_ref() {
        let _ = logger.log(devices.get_state().clone());
      }

      last_sent_to_gps_worker = now;
    }

    if devices.servo_communication_enabled() && Instant::now().duration_since(last_sent_to_servo) > FC_TO_SERVO_RATE {
      // send servo the current vehicle telemetry (file logging removed - now done in GPS worker)
      if let Err(e) = servo::push(&socket, servo_address, devices.get_state()) {
        eprintln!("Issue in sending servo the vehicle telemetry: {e}");
      }

      // After sending, mark GPS and RECO as consumed/invalid until a new sample arrives.
      devices.invalidate_gps();
      devices.invalidate_reco();

      last_sent_to_servo = Instant::now();
    }

    // receive telemetry
    let telemetry = device::receive(&socket);

    // process telemetry from boards
    devices.update_state(telemetry, &mappings, &socket);

    update_goldfish_system_safe_timer(
      &mut devices,
      &socket,
      &mut umbilical_drop_start,
      &mut sam_power_disabled_for_goldfish,
      &mut seen_valid_umbilical_voltage,
    );

    // updates all running sequences with the newest received data
    if let Err(e) = state::sync_sequences(&mut synchronizer, devices.get_state()) {
      println!("There was an error in synchronizing vehicle state: {e}");
    }

    let need_to_send_heartbeat = Instant::now().duration_since(last_heartbeat_sent) > SEND_HEARTBEAT_RATE;
    // Update board lifetimes and send heartbeats to connected boards.
    for device in devices.iter() {
      if device.is_disconnected() {
        continue;
      }

      if need_to_send_heartbeat {
        if let Err(e) = device.send_heartbeat(&socket, &devices, &mappings) {
          println!(
            "There was an error in notifying board {} at IP {} that the FC is still connected: {e}", 
            device.get_board_id(),
            device.get_ip()
          );
          continue;
        }
        last_heartbeat_sent = Instant::now();
      }
    }

    
    // Increment heartbeats until we reach the threshold [20], where we send a board the current abort stage's 
    // abort valve states. If we are in a default stage, then those are none. 
    if need_to_send_heartbeat {
      for device in devices.iter_mut() {
        if device.get_num_heartbeats() <= 20 {
          device.increment_num_heartbeats();
        } 
      }
    }

    // TODO: this is not really optimal, figure out a better way to do this
    for device in devices.iter() {
      //println!("{}", device.get_num_heartbeats());
      if device.get_num_heartbeats() == 20 {
        devices.send_sams_abort_stage(&socket, &Some(device.get_board_id()));
      }
    }

    for device in devices.iter_mut() {
      if device.get_num_heartbeats() == 20 {
      device.increment_num_heartbeats();
      }
    }

    // sequences and triggers
    let sam_commands = sequence::pull_commands(&command_socket);
    let should_abort = devices.send_sam_commands(
      &socket,
      &mappings,
      sam_commands,
      &mut abort_stages,
      &mut sequences,
      &reco_cmd_sender,
    );

    if should_abort {
      // check which type of abort should happen, abort stage or abort seq
      if devices.get_state().abort_stage.name != "DEFAULT" {
        devices.send_sams_abort(&socket, &mappings, &mut abort_stages, &mut sequences, true); // not servo LOC, abort with stage timers
      } else {
        abort(&mappings, &mut sequences, &abort_sequence);
      }
    }

    // triggers

    // Optional performance diagnostics for the main loop.
    if fc_perf_debug {
      let loop_duration = loop_start.elapsed();
      if loop_duration > Duration::from_millis(50) {
        eprintln!(
          "FC main loop iteration took {:.2} ms",
          loop_duration.as_secs_f64() * 1000.0
        );
      }
    }
  }
}

fn abort(mappings: &Mappings, sequences: &mut Sequences, abort_sequence: &Option<Sequence>) {
  if let Some(ref sequence) = abort_sequence {
    for (name, sequence) in &mut *sequences {
      if name != "AbortStage" {
        if let Err(e) = sequence.kill() {
          println!("Couldn't kill a sequence in preperation for abort, continuing normally: {e}");
        }
      }
    }

    sequence::execute(&mappings, sequence, sequences);
  } else {
    println!("Received an abort command, but no abort sequence has been set. Continuing normally...");
  }
}

/// Pulls data from Servo, if available.
/// # Error Handling
/// 
/// ## FC-Servo Connection Dropped
/// If the connection between the FC and Servo was severed, the connection
/// will tried to be re-established. If a new connection is successfully
/// established, servo_stream and servo_address will be set to mirror the
/// change. Otherwise, a notification will be printed to the terminal and None
/// will be returned.
/// 
/// ## Servo Message Deserialization Fails
/// If postcard returns an error during message deserialization, None will be
/// returned.
/// 
/// ## Transport Layer failed
/// If reading from servo_stream is not possible, None will be returned.
fn get_servo_data(
  servo_stream: &mut TcpStream,
  servo_address: &mut SocketAddr,
  last_received_from_servo: &mut Instant,
  aborted: &mut bool,
  devices: &mut Devices,
) -> Option<FlightControlMessage> {
  // If we've been instructed to permanently stop communicating with servo after a
  // disconnect, short-circuit immediately.
  if !devices.servo_communication_enabled() {
    return None;
  }

  let monitor_servo_disconnects = devices.monitor_servo_disconnects();

  match servo::pull(servo_stream) {
    Ok(message) => {
      *last_received_from_servo = Instant::now();
      message
    },
    Err(e) => {
      eprintln!("Issue in pulling data from Servo: {e}");

      match e {
        ServoError::ServoDisconnected => {
          if monitor_servo_disconnects {
            eprintln!("Attempting to reconnect to servo... ");

            match servo::establish(
              &SERVO_SOCKET_ADDRESSES,
              Some(servo_address),
              SERVO_RECONNECT_RETRY_COUNT,
              SERVO_RECONNECT_TIMEOUT,
            ) {
              Ok(s) => {
                (*servo_stream, *servo_address) = s;
                *last_received_from_servo = Instant::now();
                *aborted = false;
                eprintln!("Connection successfully re-established.");
              }
              Err(e) => {
                eprintln!("Connection could not be re-established: {e}. Continuing...");
              }
            };
          } else {
            eprintln!(
              "Servo disconnected, but monitoring is disabled; ceasing further communication with servo."
            );
            // Once we've seen a disconnect with monitoring disabled, stop all future
            // attempts to reconnect to, pull from, or push telemetry to servo.
            devices.set_servo_communication_enabled(false);
          }
        }
        ServoError::DeserializationFailed(_) => {},
        ServoError::TransportFailed(_) => {},
      };
    
      None
    }
  }
}

/// Goldfish system safe timer.
///
/// We monitor the BMS umbilical bus voltage. When it drops to 0 V, we start a
/// timer. If the timer exceeds GOLDFISH_SYSTEM_SAFE_TIMER while the umbilical
/// bus remains at 0 V, we disable SAM power via the BMS. If the umbilical bus
/// voltage becomes > 0 V before the timer elapses, we reset the timer and do
/// nothing.
fn update_goldfish_system_safe_timer(
  devices: &mut Devices,
  socket: &UdpSocket,
  umbilical_drop_start: &mut Option<Instant>,
  sam_power_disabled_for_goldfish: &mut bool,
  seen_valid_umbilical_voltage: &mut bool,
) {
  let umbilical_voltage = devices.get_state().bms.umbilical_bus.voltage;
  // If we have ever seen a voltage at or above the threshold, consider the
  // umbilical bus "real" and allow the Goldfish timer to operate.
  if umbilical_voltage >= UMBILICAL_BUS_VOLTAGE_THRESHOLD {
    *seen_valid_umbilical_voltage = true;
  }

  // If we've never seen a valid umbilical voltage, we're likely in a ground-only
  // configuration with no umbilical connected. In that case, skip the Goldfish
  // timer entirely to avoid unintentionally depowering SAMs.
  if *seen_valid_umbilical_voltage {
    if umbilical_voltage < UMBILICAL_BUS_VOLTAGE_THRESHOLD {
      match umbilical_drop_start {
        None => {
          *umbilical_drop_start = Some(Instant::now());
          *sam_power_disabled_for_goldfish = false;
          println!(
            "Umbilical bus voltage dropped to {} V; starting Goldfish system safe timer ({} s).",
            umbilical_voltage,
            GOLDFISH_SYSTEM_SAFE_TIMER.as_secs()
          );
        }
        Some(start) => {
          if !*sam_power_disabled_for_goldfish
            && Instant::now().duration_since(*start) > GOLDFISH_SYSTEM_SAFE_TIMER
          {
            println!(
              "Umbilical bus has been at {} V for at least {} s; disabling SAM power via BMS.",
              umbilical_voltage,
              GOLDFISH_SYSTEM_SAFE_TIMER.as_secs()
            );
            devices.send_bms_command(socket, bms::Command::SamLoadSwitch(false));
            *sam_power_disabled_for_goldfish = true;
          }
        }
      }
    } else {
      if umbilical_drop_start.is_some() {
        println!(
          "Umbilical bus voltage restored to {} V; resetting Goldfish system safe timer.",
          umbilical_voltage
        );
      }
      *umbilical_drop_start = None;
      *sam_power_disabled_for_goldfish = false;
    }
  }
}

fn start_abort_stage_process(abort_stages: &mut AbortStages, mappings: &Mappings, sequences: &mut Sequences, devices: &mut Devices) {
  // if any abort stage sequences exist, kill them
  for (name, sequence) in &mut *sequences {
    if name == "AbortStage" {
        if let Err(e) = sequence.kill() {
            println!("Couldn't kill AbortStage sequence in preperation for starting new AbortStage sequence: {e}");
            return;
        }
    }
  }
  sequences.remove_entry("AbortStage");

  let abort_stage_body = r#"
import time
while True:
    try:
        if curr_abort_stage() != "FLIGHT" and aborted_in_this_stage() == False and eval(curr_abort_condition()) == True:
            #print("ABORTING")
            abort()
    except Exception as e:
        print("ERROR:", e)
    wait_for(10*ms)
"#;
  
  // create abort stage and store in abort_stages 
  let default_stage = AbortStage { 
    name: "DEFAULT".to_string(),
    abort_condition: "False".to_string(), // never abort in this situation? 
    aborted: false,
    valve_safe_states: HashMap::new(),
  };
  abort_stages.push(default_stage.clone());

  devices.set_abort_stage(&default_stage);

  let abort_stage_seq = Sequence{
    name: "AbortStage".to_string(),
    script: abort_stage_body.to_string(),
  };
  sequence::execute(mappings, &abort_stage_seq, sequences);
}

/// Checks if python3 and the passed python modules exist.
fn check_python_dependencies<'a>(dependencies: &[&'a str]) -> Result<(), Vec<&'a str>> {
  let mut imports = vec!["".to_string()];

  for dependency in dependencies {
    imports.push(format!("import {}", dependency));
  }

  let mut missing_imports = Vec::new();
  for (i, statement) in imports.iter().enumerate() {
    let dependency_check = Command::new("python3")
      .args(["-c", statement.as_str()])
      .output().unwrap()
      .status.code().unwrap();

    match dependency_check {
      0 => {},
      127 => return Err(vec!["python3"]),
      _ => missing_imports.push(dependencies[i - 1]),
    };
  }

  Ok(())
}