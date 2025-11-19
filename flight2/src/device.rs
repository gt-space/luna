use core::fmt;
use std::{collections::HashMap, io, net::{IpAddr, SocketAddr, UdpSocket}, ops::Deref, time::{Duration, Instant}};
use common::comm::{ahrs, bms, flight::{DataMessage, ValveSafeState, SequenceDomainCommand}, sam::SamControlMessage, AbortStage, CompositeValveState, NodeMapping, SensorType, Statistics, ValveAction, ValveState, VehicleState};
use reco::{RecoDriver, VotingLogic};

use crate::{sequence::Sequences, Ingestible, DECAY, DEVICE_COMMAND_PORT, TIME_TO_LIVE};

pub(crate) type Mappings = Vec<NodeMapping>;
pub(crate) type AbortStages = Vec<AbortStage>;

#[derive(Clone)]
pub(crate) struct Device {
    id: String,
    address: SocketAddr,
    last_recieved: Instant, 
    num_heartbeats: u32, 
}

impl Device {
    fn new(id: String, address: SocketAddr) -> Self {
        Device { id, address, last_recieved: Instant::now(), num_heartbeats: 0 }
    }

    /// Should be ran whenever data is received from a board to update.
    pub(crate) fn reset_timer(&mut self) {
        if self.is_disconnected() {
            println!("{} at {} reconnected!", self.address.ip(), self.id);
        }

        self.last_recieved = Instant::now();
    }

    pub(crate) fn send_heartbeat(&self, socket: &UdpSocket, devices: &Devices, mappings: &Mappings) -> Result<()> {
        let mut buf: [u8; 1024] = [0; 1024];
        let serialized = postcard::to_slice(&DataMessage::FlightHeartbeat, &mut buf)
            .map_err(|e| Error::SerializationFailed(e))?;
        socket.send_to(serialized, self.address).map_err(|e| Error::TransportFailed(e))?;

        Ok(())
    }

    pub(crate) fn is_disconnected(&self) -> bool {
        Instant::now().duration_since(self.last_recieved) > TIME_TO_LIVE
    }

    /// Sends a message on a socket to a board with id `destination`
    fn serialize_and_send<T: serde::ser::Serialize>(&self, socket: &UdpSocket, destination: &str, message: &T, devices: &Devices) -> std::result::Result<(), String> {
        let mut buf: [u8; 1024] = [0; 1024];

        let Some(device) = devices.iter().find(|d| d.id == *destination) else {
            return Err("Tried to sent a message to a board that hasn't been connected yet.".to_string());
        };

        if let Err(e) = postcard::to_slice::<T>(message, &mut buf) {
            return Err(format!("Couldn't serialize message: {e}"));
        };

        if let Err(e) = device.send(socket, &buf) {
            return Err(format!("Couldn't send message to {destination}: {e}"));
        };

        return Ok(())
    }

    /// Sends data to the device via a given socket.
    pub(crate) fn send(&self, socket: &UdpSocket, buf: &[u8]) -> Result<()> {
        socket.send_to(buf, (self.address.ip(), DEVICE_COMMAND_PORT)).map_err(|e| Error::TransportFailed(e))?;
        Ok(())
    }

    pub(crate) fn get_board_id(&self) -> &String {
        &self.id
    }

    pub(crate) fn get_ip(&self) -> IpAddr {
        self.address.ip()
    }

    pub(crate) fn get_num_heartbeats(&self) -> u32 {
        self.num_heartbeats
    }

    pub(crate) fn increment_num_heartbeats(&mut self) {
        self.num_heartbeats += 1;
    }
}

pub(crate) struct Devices {
    devices: Vec<Device>,
    state: VehicleState,
    last_updates: HashMap<String, Instant>,
    reco_driver: Option<RecoDriver>,
}

impl Devices {
    /// Creates an empty set to hold Devices
    pub(crate) fn new() -> Self {
        let reco_driver = match RecoDriver::new("/dev/spidev1.1") {
            Ok(driver) => {
                println!("Initialized RECO driver on /dev/spidev1.1");
                Some(driver)
            }
            Err(e) => {
                eprintln!("Failed to initialize RECO driver: {e}");
                None
            }
        };

        Devices {
            devices: Vec::new(),
            state: VehicleState::new(),
            last_updates: HashMap::new(),
            reco_driver,
        }
    }

    /// Inserts a device into the set, overwriting an existing device.
    /// Overwriting a device replaces all of its associated data, as if it were
    /// connecting for the first time. Returns a reference to the newly inserted
    /// device and the overwritten device, if it existed.
    pub(crate) fn register_device(&mut self, id: &String, address: SocketAddr) -> Option<Device> {
        let device = Device::new(id.clone(), address);

        if let Some(copy) = self.devices.iter_mut().find(|d| d.id == device.id) {
            let old = copy.clone();
            *copy = device;
            return Some(old);
        } else {
            self.devices.push(device);
            return None;
        }
    }

    /// should be ran whenever data is sent
    /// TODO: INTEGRATE THIS WITH THE MAIN DATA
    pub(crate) fn update_last_updates(&mut self) {
        let now = Instant::now();

        for (name, stats) in &mut self.state.rolling {
            if !self.last_updates.contains_key(name.as_str()) {
                continue;
            }

            let last_update_time = *self.last_updates
                .get(name.as_str())
                .expect("Already checked if it existed. This should not happen.");

            stats.time_since_last_update = now.duration_since(last_update_time).as_secs_f64();
        }
    }

    /// Updates the VehicleState struct with the newly recieved board telemetry
    pub(crate) fn update_state(&mut self, telemetry: Vec<(SocketAddr, DataMessage)>, mappings: &Mappings, socket: &UdpSocket) {
        for (address, message) in telemetry {
            match message {
                DataMessage::FlightHeartbeat => continue,
                DataMessage::Ahrs(ref id, _) |
                DataMessage::Bms(ref id, _) |
                DataMessage::Sam(ref id, _) => {
                    let Some(device) = self.devices.iter_mut().find(|d| d.id == *id) else {
                        println!("Received data from a device that hasn't been registered. Ignoring...");
                        continue;
                    };

                    // TODO: Comment out moving averages
                    let now = Instant::now();
                    let mut delta_time = Duration::new(0, 0);

                    match self.last_updates.get_mut(id) {
                        Some(last_update) => {
                            delta_time = now - *last_update;
                            *last_update = now;
                        }
                        None => { self.last_updates.insert(id.clone(), now); }
                    };
                    

                    match self.state.rolling.get_mut(id) {
                        Some(stat) => {
                            stat.rolling_average = stat.rolling_average.mul_f64(DECAY)
                              + delta_time.mul_f64(1.0 - DECAY);
                            stat.delta_time = delta_time;
                        }
                        None => {
                            self.state.rolling.insert(
                                id.clone(),
                                Statistics {
                                    ..Default::default()
                                },
                            );
                        }
                    }

                    device.reset_timer();
                },
                DataMessage::Identity(ref id) => {
                    if let Err(e) = handshake(&address, socket) {
                        println!("Connection with {id} couldn't be established: {e}");
                    } else {
                        println!("Connection established with {id}.");
                        if let Some(old_device) = self.register_device(id, address) {
                            println!("Overwrote data of previously registered {id} at {}", old_device.address.ip());
                        }
                    }

                    continue;
                }
            }
            
            message.ingest(&mut self.state, mappings);
        }
    }

    /// Sends a message on a socket to a board with id `destination`
    fn serialize_and_send<T: serde::ser::Serialize>(&self, socket: &UdpSocket, destination: &str, message: &T) -> std::result::Result<(), String> {
        let mut buf: [u8; 1024] = [0; 1024];

        let Some(device) = self.devices.iter().find(|d| d.id == *destination) else {
            return Err("Tried to sent a message to a board that hasn't been connected yet.".to_string());
        };

        if let Err(e) = postcard::to_slice::<T>(message, &mut buf) {
            return Err(format!("Couldn't serialize message: {e}"));
        };

        if let Err(e) = device.send(socket, &buf) {
            return Err(format!("Couldn't send message to {destination}: {e}"));
        };

        return Ok(())
    }

    ///
    pub(crate) fn send_sam_commands(&mut self, socket: &UdpSocket, mappings: &Mappings, commands: Vec<SequenceDomainCommand>, abort_stages: &mut AbortStages, sequences: &mut Sequences) -> bool {
        let mut should_abort = false;
        
        for command in commands {
            match command {
                SequenceDomainCommand::ActuateValve { valve, state } => {
                    let Some(mapping) = mappings.iter().find(|m| m.text_id == valve) else {
                        eprintln!("Failed to actuate valve: mapping '{valve}' is not defined.");
                        continue;
                    };
    
                    let closed = state == ValveState::Closed;
                    let normally_closed = mapping.normally_closed.unwrap_or(true);
                    let powered = closed != normally_closed;

                    if let Some(existing) = self.state.valve_states.get_mut(&valve) {
                        existing.commanded = state;
                    } else {
                        self.state.valve_states.insert(
                            valve,
                            CompositeValveState {
                                commanded: state,
                                actual: ValveState::Undetermined
                            }
                        );
                    }

                    let command = SamControlMessage::ActuateValve { channel: mapping.channel, powered };

                    if let Err(msg) = self.serialize_and_send(socket, &mapping.board_id, &command) {
                        println!("{}", msg);
                    }
                },
                SequenceDomainCommand::RecoLaunch => {
                    match self.reco_driver.as_mut() {
                        Some(reco) => {
                            if let Err(e) = reco.send_launched() {
                                eprintln!("Failed to send launch message to RECO: {e}");
                            } else {
                                println!("Sent launch message to RECO.");
                            }
                        }
                        None => {
                            eprintln!("RECO driver not initialized; cannot send launch message.");
                        }
                    }
                },
                SequenceDomainCommand::SetRecoVotingLogic { mcu_1_enabled, mcu_2_enabled, mcu_3_enabled } => {
                    let voting_logic = VotingLogic {
                        processor_1_enabled: mcu_1_enabled,
                        processor_2_enabled: mcu_2_enabled,
                        processor_3_enabled: mcu_3_enabled,
                    };

                    match self.reco_driver.as_mut() {
                        Some(reco) => {
                            if let Err(e) = reco.send_voting_logic(&voting_logic) {
                                eprintln!(
                                    "Failed to send voting logic to RECO (mcu_1: {}, mcu_2: {}, mcu_3: {}): {e}",
                                    mcu_1_enabled, mcu_2_enabled, mcu_3_enabled
                                );
                            } else {
                                println!(
                                    "Sent voting logic to RECO (mcu_1: {}, mcu_2: {}, mcu_3: {}).",
                                    mcu_1_enabled, mcu_2_enabled, mcu_3_enabled
                                );
                            }
                        }
                        None => {
                            eprintln!("RECO driver not initialized; cannot send voting logic.");
                        }
                    }
                },
                SequenceDomainCommand::CreateAbortStage { stage_name, abort_condition, valve_safe_states} => {
                    // check to see if stage_name matches an already created stage name. if so, return error
                    /*if let Some(name) = abort_stages.iter().find(|m| m.name == stage_name) {
                        eprintln!("A stage already exists with the name {stage_name}, so skipping creation of stage.");
                        continue;
                    }*/ // DO WE NEED THIS? IF THIS IS THERE CANT CHANGE STAGE INFO IF WE MADE MISTAKE. BUT WHAT IF IN THIS STAGE CURRENTLY?
                    // check to see if safe_valve_states is valid for every entry, if not return error
                    let mut valve_lookup: HashMap<String, (&str, u32, bool)> = HashMap::new();
                    for mapping in mappings {
                        if mapping.sensor_type == SensorType::Valve {
                            let normally_closed = mapping.normally_closed.unwrap_or(true);
                            valve_lookup.insert(mapping.text_id.clone(), (&mapping.board_id, mapping.channel, normally_closed));
                        }
                    }

                    // stores [sam_board_id, (channel_num, powered, timer)]. every valve that an operator set an abort config for
                    let mut board_valves: HashMap<String, Vec<ValveAction>> = HashMap::new();
                    for (valve_name, valve_state_info) in valve_safe_states {
                        // get the mapping for the current valve
                        let Some(&(board_id, channel, normally_closed)) = valve_lookup.get(&valve_name)
                        else {
                            eprintln!("Abort valve '{}' not found in mappings. Skipping command.", valve_name);
                            continue;
                        };

                        // determine if we want to give power to this valve
                        let closed = valve_state_info.desired_state == ValveState::Closed;
                        let powered = closed != normally_closed;

                        // append our determination of whether to power this valve to its SAM board vector
                         board_valves.entry(board_id.clone().to_string())
                            .or_insert_with(Vec::new)
                            .push( ValveAction { 
                                channel_num: channel, 
                                powered: powered, 
                                timer: Duration::from_secs(valve_state_info.safing_timer as u64) 
                            });
                    }
                    
                    // remove this stage if it existed previously
                    if let Some(stage) = abort_stages.iter().position(|s| s.name == stage_name) {
                        abort_stages.swap_remove(stage);
                    }

                    // add to global abort_stages
                    abort_stages.push( AbortStage { 
                        name: stage_name, 
                        abort_condition: abort_condition, 
                        aborted: false, 
                        valve_safe_states: board_valves 
                    });
                },
                // TODO: should we not allow setting an abort stage if we already in that abort stage?
                SequenceDomainCommand::SetAbortStage { stage_name } => {
                    // change the abort stage in vehicle state by looking through saved abort stage configs. 
                    // if name doesn't match up throw an error
                    if let Some(stage) = abort_stages.iter().find(|m| m.name == stage_name) {
                        self.set_abort_stage(&stage);
                    } else {
                        eprintln!("Tried to set abort stage to {stage_name} but could not find the stage.");
                        continue;
                    }
                    
                    self.send_sams_abort_stage(socket, &None);
                },
                SequenceDomainCommand::AbortViaStage => {
                    //println!("Sending abort message to sams");
                    self.send_sams_abort(socket, mappings, abort_stages, sequences, true); // command from a sequence, so yes we want to use stage timers
                },
                // TODO: shouldn't we break out of the loop here? if we receive an abort command why are we not flushing commands that come in after 
                SequenceDomainCommand::Abort => should_abort = true,
            }
        }

        should_abort
    }

    // sends all sams the current abort stage's safe valve states. if "None" board_id is passed, message is sent
    // to all sams. else, a message is sent to the board id passed in (if it is valid)
    pub(crate) fn send_sams_abort_stage(&self, socket: &UdpSocket, board_id: &Option<&String>) {
        // send sams the safe states that their valves should be in.
        // if a channel is not specified, it means we want that valve to just stay in
        // whatever state they are in already

        // individual board
        if board_id.is_some() {
            if let Some(device) = self.devices.iter().find(|d| d.get_board_id().deref() == board_id.unwrap() && board_id.unwrap().starts_with("sam")) {
                if let Some(valve_states_to_send) = self.state.abort_stage.valve_safe_states.get(device.get_board_id()) {
                    let command = SamControlMessage::AbortStageValveStates { 
                        valve_states: valve_states_to_send.clone(),
                    };

                    // send message to this sam board
                    if let Err(msg) = self.serialize_and_send(socket, board_id.unwrap(), &command) {
                        println!("{}", msg); 
                    } else {
                        println!("Sent {} abort stage's valve safe states to SAM: {}", self.state.abort_stage.name, board_id.unwrap());
                    }
                } else {
                    println!("No abort stage configuration to send to {}", device.get_board_id());
                }
            } else {
                eprintln!("Invalid board id passed in when trying to send sams abort stage: Either your board does not exist or is not a sam.");
            }
        } else {
            for (board_id, valves) in self.state.abort_stage.valve_safe_states.iter() {
                // create message for this sam board
                let command = SamControlMessage::AbortStageValveStates { valve_states: valves.clone() };

                // send message to this sam board
                if let Err(msg) = self.serialize_and_send(socket, &board_id, &command) {
                    println!("{}", msg); 
                } else {
                    println!("Sent {} abort stage's valve safe states to SAM: {}", self.state.abort_stage.name, board_id);
                }
            }
        }
    }
    pub(crate) fn send_sams_abort(&mut self, socket: &UdpSocket, mappings: &Mappings, abort_stages: &mut AbortStages, sequences: &mut Sequences, use_stage_timers: bool) {
        // kill all sequences besides the abort stage sequence
        for (name, sequence) in &mut *sequences {
            if name != "AbortStage" {
                if let Err(e) = sequence.kill() {
                    println!("Couldn't kill a sequence in preperation for abort, continuing normally: {e}");
                }
            }
        }

        // send message to sams 
        for device in self.devices.iter() {
            if device.get_board_id().starts_with("sam") {
                let command = SamControlMessage::Abort { use_stage_timers: use_stage_timers };
                // send message to this sam board
                if let Err(msg) = self.serialize_and_send(socket, device.get_board_id(), &command) {
                    println!("{}", msg); 
                } else {
                    println!("Sent abort message to SAM: {}, which will use {} stage's safe valves.", 
                        device.get_board_id(), self.state.abort_stage.name);
                }
            }
        }

        // update state to say that we have aborted in this stage
        self.state.abort_stage.aborted = true;
    }

    // Clears any stored abort stages on sams
    pub(crate) fn send_sam_clear_abort_stage(&self, socket: &UdpSocket) {
        for device in self.devices.iter() {
            if device.get_board_id().starts_with("sam") {
                let command = SamControlMessage::ClearStoredAbortStage {  };
                if let Err(msg) = self.serialize_and_send(socket, device.get_board_id(), &command) {
                        println!("{}", msg);
                } else {
                    println!("Cleared abort stage from {} memory", device.get_board_id());
                }
            }
        }
    }

    pub(crate) fn send_bms_command(&self, socket: &UdpSocket, command: bms::Command) {
        let Some(bms) = self.devices.iter().find(|d| d.id.starts_with("bms")) else {
            println!("Couldn't send a BMS command as BMS isn't connected.");
            return;
        };

        if let Err(msg) = self.serialize_and_send(socket, &bms.id, &command) {
            println!("{}", msg);
        }
    }

    pub(crate) fn send_ahrs_command(&self, socket: &UdpSocket, command: ahrs::Command) {
        let Some(ahrs) = self.devices.iter().find(|d| d.id.starts_with("ahrs")) else {
            println!("Couldn't send an AHRS command as AHRS isn't connected.");
            return;
        };

        if let Err(msg) = self.serialize_and_send(socket, &ahrs.id, &command) {
            println!("{}", msg);
        }
    }

    pub(crate) fn get_state(&self) -> &VehicleState {
        return &self.state;
    }

    pub(crate) fn set_abort_stage(&mut self, stage: &AbortStage) {
        self.state.abort_stage = stage.clone();
    }
    
    pub(crate) fn iter_mut(&mut self) -> ::core::slice::IterMut<'_, Device> {
        self.devices.iter_mut()
    }

    pub(crate) fn iter(&self) -> ::core::slice::Iter<'_, Device> {
        self.devices.iter()
    }
}

/// performs a flight handshake with the board.
pub(crate) fn handshake(address: &SocketAddr, socket: &UdpSocket) -> Result<()> {
    let mut buf: [u8; 1024] = [0; 1024];
    let serialized = postcard::to_slice(&DataMessage::Identity("flight-01".to_string()), &mut buf)
        .map_err(|e| Error::SerializationFailed(e))?;
    socket.send_to(serialized, address).map_err(|e| Error::TransportFailed(e))?;
    Ok(())
}

/// Gets the most recent UDP Commands
pub(crate) fn receive(socket: &UdpSocket) -> Vec<(SocketAddr, DataMessage)> {
    let mut messages = Vec::new();
    let mut buf: [u8; 1024] = [0; 1024];
    
    loop {
        let (size, address) = match socket.recv_from(&mut buf) {
            Ok(metadata) => metadata,
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
            Err(e) => {
                eprintln!("Can't get receive incoming ethernet packets: {e:#?}");
                break;
            }
        };

        let serialized_message = match postcard::from_bytes::<DataMessage>(&buf[..size]) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Received a message from a board, but couldn't decode it, packet was of size {}: {e}", size);
                continue;
            }
        };

        messages.push((address, serialized_message));
    };

    messages
}

type Result<T> = ::std::result::Result<T, Error>;
pub(crate) enum Error {
    SerializationFailed(postcard::Error),
    TransportFailed(io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SerializationFailed(e) => write!(f, "Couldn't serialize an outgoing message: {e}"),
            Self::TransportFailed(e) => write!(f, "Couldn't send data to a device: {e}"),
        }
    }
}
