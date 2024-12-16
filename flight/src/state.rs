use crate::{
  forwarder,
  handler::{self, create_device_handler},
  switchboard::{self, commander::Command},
  CommandSender,
  SERVO_PORT,
  SWITCHBOARD_ADDRESS,
};
use bimap::BiHashMap;
use common::{
  comm::{Computer, FlightControlMessage, NodeMapping, Sequence, VehicleState},
  sequence,
};
use jeflog::{fail, pass, task, warn};
use postcard::experimental::max_size::MaxSize;
use pyo3::Python;
use std::{
  fmt,
  io::{self, Read, Write},
  net::{IpAddr, TcpStream, UdpSocket},
  sync::{Arc, Mutex, OnceLock},
  thread::{self, ThreadId},
  time::Duration,
};

/// Holds all shared state that should be accessible concurrently in multiple
/// contexts.
///
/// Everything in this struct should be wrapped with `Arc<Mutex<T>>`. **Do not
/// abuse this struct.** It is intended for what would typically be global
/// state.
#[derive(Clone, Debug)]
pub struct SharedState {
  pub vehicle_state: Arc<Mutex<VehicleState>>,
  pub mappings: Arc<Mutex<Vec<NodeMapping>>>,
  pub server_address: Arc<Mutex<Option<IpAddr>>>,
  pub triggers: Arc<Mutex<Vec<common::comm::Trigger>>>,
  pub sequences: Arc<Mutex<BiHashMap<String, ThreadId>>>,
  pub abort_sequence: Arc<Mutex<Option<Sequence>>>,
}

pub(crate) static COMMANDER_TX: OnceLock<CommandSender> =
  OnceLock::<CommandSender>::new();

#[derive(Debug)]
pub enum ProgramState {
  /// The initialization state, which primarily spawns background threads
  /// and transitions to the `ServerDiscovery` state.
  Init,

  /// State which loops through potential server hostnames until locating the
  /// server and connecting to it via TCP.
  ServerDiscovery {
    /// The shared flight state.
    shared: SharedState,
  },

  /// State which waits for an operator command, such as setting mappings or
  /// running a sequence.
  WaitForOperator {
    server_socket: TcpStream,

    /// The shared flight state.
    shared: SharedState,
  },

  /// State which spawns a thread to run a sequence before returning to the
  /// `WaitForOperator` state.
  RunSequence {
    server_socket: TcpStream,

    /// A full description of the sequence to run.
    sequence: Sequence,

    /// The shared flight state.
    shared: SharedState,
  },
}

impl ProgramState {
  /// Perform transition to the next state, returning the next state.
  pub fn next(self) -> Self {
    match self {
      ProgramState::Init => init(),
      ProgramState::ServerDiscovery { shared } => server_discovery(shared),
      ProgramState::WaitForOperator {
        server_socket,
        shared,
      } => wait_for_operator(server_socket, shared),
      ProgramState::RunSequence {
        server_socket,
        sequence,
        shared,
      } => run_sequence(server_socket, sequence, shared),
    }
  }
}

impl fmt::Display for ProgramState {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::Init => write!(f, "Init"),
      Self::ServerDiscovery { .. } => write!(f, "ServerDiscovery"),
      Self::WaitForOperator { server_socket, .. } => {
        let peer_address = server_socket
          .peer_addr()
          .map(|addr| addr.to_string())
          .unwrap_or("unknown".to_owned());

        write!(f, "WaitForOperator(server = {peer_address})")
      }
      Self::RunSequence { sequence, .. } => {
        write!(f, "RunSequence(name = {})", sequence.name)
      }
    }
  }
}

fn init() -> ProgramState {
  let home_socket = UdpSocket::bind(SWITCHBOARD_ADDRESS).unwrap_or_else(|_| {
    panic!("Cannot create bind on address {:#?}", SWITCHBOARD_ADDRESS);
  });

  let shared = SharedState {
    vehicle_state: Arc::new(Mutex::new(VehicleState::new())),
    mappings: Arc::new(Mutex::new(Vec::new())),
    server_address: Arc::new(Mutex::new(None)),
    triggers: Arc::new(Mutex::new(Vec::new())),
    sequences: Arc::new(Mutex::new(BiHashMap::new())),
    abort_sequence: Arc::new(Mutex::new(None)),
  };

  let command_tx = match switchboard::start(shared.clone(), home_socket) {
    Ok(command_tx) => command_tx,
    Err(error) => {
      fail!("Failed to create switchboard: {error}");
      return ProgramState::Init;
    }
  };

  sequence::initialize(shared.mappings.clone());
  sequence::set_device_handler(create_device_handler(
    shared.clone(),
    command_tx.clone(),
  ));

  COMMANDER_TX
    .set(command_tx)
    .expect("Could not set the channel for BMS and AHRS commands");

  thread::spawn(check_triggers(&shared));

  ProgramState::ServerDiscovery { shared }
}

fn server_discovery(shared: SharedState) -> ProgramState {
  task!("Locating control server.");

  let potential_hostnames = ["server-01.local", "server-02.local", "localhost"];

  for host in potential_hostnames {
    task!(
      "Attempting to connect to \x1b[1m{}:{SERVO_PORT}\x1b[0m.",
      host
    );

    let Ok(mut stream) = TcpStream::connect((host, SERVO_PORT)) else {
      fail!("Failed to connect to \x1b[1m{}:{SERVO_PORT}\x1b[0m.", host);
      continue;
    };

    pass!(
      "Successfully connected to \x1b[1m{}:{SERVO_PORT}\x1b[0m.",
      host
    );
    pass!(
      "Found control server at \x1b[1m{}:{SERVO_PORT}\x1b[0m.",
      host
    );

    let hostname = hostname::get()
      .ok()
      .and_then(|name| name.into_string().ok());

    let computer;

    if let Some(hostname) = hostname {
      if hostname.starts_with("flight") {
        computer = Computer::Flight;
      } else if hostname.starts_with("ground") {
        computer = Computer::Ground;
      } else {
        warn!("Hostname does not start with 'flight' or 'ground'. Defaulting to flight.");
        computer = Computer::Flight;
      }
    } else {
      warn!("Failed to get local hostname. Defaulting to flight.");
      computer = Computer::Flight;
    }

    // buffer containing the serialized identity message to be sent to the
    // control server
    let mut identity = [0; Computer::POSTCARD_MAX_SIZE];

    if let Err(error) = postcard::to_slice(&computer, &mut identity) {
      fail!("Failed to serialize Computer: {error}");
      continue;
    }

    if let Err(error) = stream.write_all(&identity) {
      warn!("Failed to send identity message to control server: {error}");
      continue;
    }

    *shared.server_address.lock().unwrap() =
      Some(stream.peer_addr().unwrap().ip());
    thread::spawn(forwarder::forward_vehicle_state(&shared));

    return ProgramState::WaitForOperator {
      server_socket: stream,
      shared,
    };
  }

  fail!("Failed to locate control server. Retrying.");
  ProgramState::ServerDiscovery { shared }
}

fn wait_for_operator(
  mut server_socket: TcpStream,
  shared: SharedState,
) -> ProgramState {
  let mut buffer = vec![0; 1_000_000];

  match server_socket.read(&mut buffer) {
    Ok(size) => {
      // if the size is zero, a TCP shutdown packet was sent. the connection is
      // closed.
      if size == 0 {
        return ProgramState::ServerDiscovery { shared };
      }

      match postcard::from_bytes::<FlightControlMessage>(&buffer) {
        Ok(message) => {
          match message {
            FlightControlMessage::Mappings(mappings) => {
              pass!("Received mappings from server: {mappings:#?}");
              *shared.mappings.lock().unwrap() = mappings;
              ProgramState::WaitForOperator {
                server_socket,
                shared,
              }
            }
            FlightControlMessage::Sequence(sequence) => {
              pass!("Received sequence from server: {sequence:#?}");

              // if the abort sequence was set, don't run it
              // set the shared abort sequence and return early
              if sequence.name == "abort" {
                *shared.abort_sequence.lock().unwrap() = Some(sequence);
                return ProgramState::WaitForOperator {
                  server_socket,
                  shared,
                };
              }

              ProgramState::RunSequence {
                server_socket,
                sequence,
                shared,
              }
            }
            FlightControlMessage::Trigger(trigger) => {
              pass!("Received trigger from server: {trigger:#?}");

              // update existing trigger if one has the same name
              // otherwise, add a new trigger to the vec
              let mut triggers = shared.triggers.lock().unwrap();

              let existing =
                triggers.iter().position(|t| t.name == trigger.name);

              if let Some(index) = existing {
                triggers[index] = trigger;
              } else {
                triggers.push(trigger);
              }

              // necessary to allow passing 'shared' back to WaitForOperator
              drop(triggers);

              ProgramState::WaitForOperator {
                server_socket,
                shared,
              }
            }
            FlightControlMessage::StopSequence(name) => {
              pass!("Received instruction to stop sequence from server.");
              let stopped =
                shared.sequences.lock().unwrap().remove_by_left(&name);

              if stopped.is_some() {
                pass!("Stopped sequence '{name}'.");
              } else {
                warn!("Sequence '{name}' was not running.");
              }

              ProgramState::WaitForOperator {
                server_socket,
                shared,
              }
            }
            FlightControlMessage::Abort => {
              pass!("Received abort instruction from server.");
              handler::abort(&shared);
              ProgramState::WaitForOperator {
                server_socket,
                shared,
              }
            }
            FlightControlMessage::BmsCommand(command) => {
              pass!("Received BMS Command from Servo: {command}");
              match COMMANDER_TX.get() {
                Some(commander) => {
                  if let Err(e) = commander.send(
                    ("bms-01".to_string(), Command::Bms(command))
                  ) {
                    fail!("Could not send BMS command to commander in switchboard: {e}.")
                  };
                }
                None => fail!("Could not obtain the BMS/AHRS command channel. Command couldn't be sent.")
              };

              ProgramState::WaitForOperator {
                server_socket,
                shared,
              }
            }
            FlightControlMessage::AhrsCommand(command) => {
              pass!("Received AHRS Command from Servo: {command}");
              match COMMANDER_TX.get() {
                Some(commander) => {
                  if let Err(e) = commander.send(
                    ("ahrs-01".to_string(), Command::Ahrs(command))
                  ) {
                    fail!("Could not send AHRS command to commander in switchboard: {e}.")
                  };
                }
                None => fail!("Could not obtain the BMS/AHRS command channel. Command couldn't be sent.")
              };

              ProgramState::WaitForOperator {
                server_socket,
                shared,
              }
            }
          }
        }
        Err(error) => {
          warn!(
            "Failed to deserialize control message: {}.",
            error.to_string()
          );
          ProgramState::WaitForOperator {
            server_socket,
            shared,
          }
        }
      }
    }
    Err(ref error) if error.kind() == io::ErrorKind::WouldBlock => {
      ProgramState::WaitForOperator {
        server_socket,
        shared,
      }
    }
    Err(error) => {
      fail!(
        "Failed to read from server socket: {}. Dropping connection.",
        error.to_string()
      );
      ProgramState::ServerDiscovery { shared }
    }
  }
}

/// Spawns a thread which runs the specified sequence before returning to
/// `WaitForOperator`.
fn run_sequence(
  server_socket: TcpStream,
  sequence: Sequence,
  shared: SharedState,
) -> ProgramState {
  let sequence_name = sequence.name.clone();

  let thread_id = thread::spawn(|| sequence::run(sequence)).thread().id();

  shared
    .sequences
    .lock()
    .unwrap()
    .insert(sequence_name, thread_id);

  ProgramState::WaitForOperator {
    server_socket,
    shared,
  }
}

/// Constructs a closure which continuously checks if any triggers have tripped,
/// running the corresponding script inline if so.
fn check_triggers(shared: &SharedState) -> impl FnOnce() {
  let triggers = shared.triggers.clone();

  // return closure instead of using the function itself because of
  // borrow-checking rules regarding moving the 'triggers' reference across
  // closure bounds
  move || {
    loop {
      let mut triggers = triggers.lock().unwrap();

      for trigger in triggers.iter_mut() {
        // perform check by running condition as Python script and getting
        // truth value
        let check = Python::with_gil(|py| {
          py.eval(&trigger.condition, None, None)
            .and_then(|condition| condition.extract::<bool>())
        });

        // checks if the condition evaluated true
        if check.as_ref().is_ok_and(|c| *c) {
          let sequence = Sequence {
            name: format!("trigger_{}", trigger.name),
            script: trigger.script.clone(),
          };

          // run sequence in the same thread so there is no rapid-fire
          // sequence dispatches if a trigger is tripped
          // note: this is intentionally blocking
          common::sequence::run(sequence);
        }

        if let Err(error) = check {
          fail!(
            "Trigger '{}' raised exception during execution: {error}",
            trigger.name
          );
          trigger.active = false;
        }
      }

      // drop triggers before waiting so the lock isn't held over the wait
      drop(triggers);
      thread::sleep(Duration::from_millis(10));
    }
  }
}
