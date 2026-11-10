use std::{
    collections::{HashMap, HashSet},
    fmt, io,
    net::{IpAddr, SocketAddr, UdpSocket},
    time::{Duration, Instant},
};

use common::comm::{
    bms, fc_sensors,
    flight::{DataMessage, SequenceDomainCommand},
    reco::{
        GuiCommand as SharedRecoCommand, SequenceCommand as RecoSequenceCommand, TargetedGuiCommand,
    },
    sam::{SamControlMessage, SamDataPoint, AbortStageAck, ValveActionMillis},
    AbortStage, AbortStageConfig, AbortTransitionStatus, CompositeValveState, GpsState, Measurement, NodeMapping,
    RecoState, SensorType, Statistics, ValveAction, ValveState, VehicleState,
};
use lis2mdl::MagnetometerData;

use crate::{
    abort_transition::{required_boards, unsynchronized, BoardPhase, Phase, Transition},
    gps::{GpsHandle, RecoControlMessage},
    sensors::{BarometerData, ImuAdcSample},
    sequence::Sequences,
    state::process_flight_pt_data,
    Ingestible, DECAY, DEVICE_COMMAND_PORT, TIME_TO_LIVE,
};

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
        Device {
            id,
            address,
            last_recieved: Instant::now(),
            num_heartbeats: 0,
        }
    }

    /// Should be ran whenever data is received from a board to update.
    pub(crate) fn reset_timer(&mut self) {
        if self.is_disconnected() {
            println!("{} at {} reconnected!", self.address.ip(), self.id);
        }

        self.last_recieved = Instant::now();
    }

    pub(crate) fn send_heartbeat(&self, socket: &UdpSocket) -> Result<()> {
        let mut buf: [u8; 1024] = [0; 1024];
        let serialized = postcard::to_slice(&DataMessage::FlightHeartbeat, &mut buf)
            .map_err(Error::SerializationFailed)?;
        socket
            .send_to(serialized, self.address)
            .map_err(Error::TransportFailed)?;

        Ok(())
    }

    pub(crate) fn is_disconnected(&self) -> bool {
        Instant::now().duration_since(self.last_recieved) > TIME_TO_LIVE
    }

    /// Sends data to the device via a given socket.
    pub(crate) fn send(&self, socket: &UdpSocket, buf: &[u8]) -> Result<()> {
        socket
            .send_to(buf, (self.address.ip(), DEVICE_COMMAND_PORT))
            .map_err(Error::TransportFailed)?;
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
    /// Whether the FC should abort on servo disconnect.
    servo_disconnect_abort_enabled: bool,
    transition: Option<Transition>,
    pending_stage: Option<AbortStage>,
    cleared_boards: HashSet<String>,
    v2_used: bool,
    unsynchronized: bool,
    revision_epoch: Option<u64>,
    revision_counter: u64,
    recovery_dir: std::path::PathBuf,
    required_boards: Option<HashSet<String>>,
    invalid_saved_stages: HashSet<String>,
}

impl Devices {
    /// Creates an empty set to hold Devices
    pub(crate) fn new() -> Self {
        let recovery_dir = std::path::PathBuf::from(
            std::env::var("LUNA_ABORT_EPOCH_DIR")
                .unwrap_or_else(|_| "/var/lib/luna/abort-revisions".into()),
        );
        Self::with_recovery_dir(recovery_dir)
    }

    fn with_recovery_dir(recovery_dir: std::path::PathBuf) -> Self {
        let revision_epoch = crate::revision_epoch::reserve(&recovery_dir)
            .map_err(|e| eprintln!("Abort-stage revisions unavailable: {e}"))
            .ok();
        let required_boards = crate::revision_epoch::load_required(&recovery_dir)
            .map_err(|e| eprintln!("Abort-stage inventory unavailable: {e}"))
            .ok();
        let mut state = VehicleState::new();
        state.abort_transition = Some(AbortTransitionStatus {
            request_id: None,
            revision: 0,
            stage: state.abort_stage.name.clone(),
            phase: "unsent".into(),
            unsynchronized: true,
            sams: required_boards
                .as_ref()
                .into_iter()
                .flat_map(|boards| boards.iter())
                .cloned()
                .map(|id| (id, "unsent".into()))
                .collect(),
        });
        Devices {
            devices: Vec::new(),
            state,
            last_updates: HashMap::new(),
            servo_disconnect_abort_enabled: true,
            transition: None,
            pending_stage: None,
            cleared_boards: HashSet::new(),
            v2_used: false,
            unsynchronized: true,
            revision_epoch,
            revision_counter: 0,
            recovery_dir,
            required_boards,
            invalid_saved_stages: HashSet::new(),
        }
    }

    /// Update flight-computer-local IMU and ADC data into the
    /// VehicleState
    pub(crate) fn update_fc_imu_adc(&mut self, sample: &ImuAdcSample, mappings: &Mappings) {
        self.state.fc_sensors.imu = sample.imu;
        self.state.fc_sensors.adc = sample.adc;
        process_flight_pt_data(&mut self.state, sample.adc.current_loop_pt, mappings);
    }

    pub(crate) fn update_fc_magnetometer(&mut self, mag: &MagnetometerData) {
        self.state.fc_sensors.magnetometer = fc_sensors::Vector {
            x: mag.x as f64,
            y: mag.y as f64,
            z: mag.z as f64,
        };
    }

    pub(crate) fn update_fc_barometer(&mut self, bar: &BarometerData) {
        self.state.fc_sensors.barometer = fc_sensors::Barometer {
            temperature: bar.temperature,
            pressure: bar.pressure,
        };
    }

    /// Inserts a device into the set, overwriting an existing device.
    /// Overwriting a device replaces all of its associated data, as if it were
    /// connecting for the first time. Returns a reference to the newly inserted
    /// device and the overwritten device, if it existed.
    pub(crate) fn register_device(&mut self, id: &str, address: SocketAddr) -> Option<Device> {
        let device = Device::new(id.to_string(), address);

        if let Some(copy) = self.devices.iter_mut().find(|d| d.id == device.id) {
            let old = copy.clone();
            *copy = device;
            Some(old)
        } else {
            self.devices.push(device);
            None
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

            let last_update_time = *self
                .last_updates
                .get(name.as_str())
                .expect("Already checked if it existed. This should not happen.");

            stats.time_since_last_update = now.duration_since(last_update_time).as_secs_f64();
        }
    }

    /// Updates the VehicleState struct with the newly recieved board telemetry
    pub(crate) fn update_state(
        &mut self,
        telemetry: Vec<(SocketAddr, DataMessage)>,
        mappings: &Mappings,
        socket: &UdpSocket,
    ) {
        for (address, message) in telemetry {
            match message {
                DataMessage::FlightHeartbeat => continue,
                DataMessage::Bms(ref id, _) | DataMessage::Sam(ref id, _) => {
                    let Some(device) = self.devices.iter_mut().find(|d| d.id == *id) else {
                        println!(
                            "Received data from a device that hasn't been registered. Ignoring..."
                        );
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
                        None => {
                            self.last_updates.insert(id.clone(), now);
                        }
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
                }
                DataMessage::Identity(ref id) => {
                    if let Err(e) = handshake(&address, socket) {
                        println!("Connection with {id} couldn't be established: {e}");
                    } else {
                        println!("Connection established with {id}.");
                        if let Some(old_device) = self.register_device(id, address) {
                            println!(
                                "Overwrote data of previously registered {id} at {}",
                                old_device.address.ip()
                            );
                        }
                        if let Some(transition) = &mut self.transition {
                            if transition.phase == Phase::Confirmed && transition.boards.contains_key(id) {
                                transition.phase = Phase::PartiallyApplied;
                                self.sync_abort_transition_status();
                            }
                        }
                    }

                    continue;
                }
            }

            message.ingest(&mut self.state, mappings);
            if let DataMessage::Sam(id, points) = &message {
                if self.devices.iter().any(|device| device.id == *id && device.address.ip() == address.ip()) {
                    for point in points.iter() {
                        if let SamDataPoint::AbortStageAck { revision, phase } = point {
                            if let Some(transition) = &mut self.transition {
                                transition.ack(id, *revision, *phase == AbortStageAck::Prepared, *phase == AbortStageAck::Activated, *phase == AbortStageAck::Rejected);
                            }
                        }
                    }
                }
            }
        }
    }

    pub(crate) fn sync_configured_valves(&mut self, mappings: &Mappings) {
        let configured_valves = mappings
            .iter()
            .filter(|mapping| mapping.sensor_type == SensorType::Valve)
            .map(|mapping| mapping.text_id.clone())
            .collect::<std::collections::HashSet<_>>();

        self.state
            .valve_states
            .retain(|valve_name, _| configured_valves.contains(valve_name));

        for valve_name in configured_valves {
            self.state
                .valve_states
                .entry(valve_name)
                .or_insert(CompositeValveState {
                    commanded: ValveState::Undetermined,
                    actual: ValveState::Undetermined,
                });
        }

        let configured_sensors = mappings
            .iter()
            .filter_map(|mapping| Some((mapping.text_id.clone(), mapping.sensor_type.unit()?)))
            .collect::<HashMap<_, _>>();

        self.state
            .sensor_readings
            .retain(|sensor_name, _| configured_sensors.contains_key(sensor_name));

        for (sensor_name, unit) in configured_sensors {
            self.state
                .sensor_readings
                .entry(sensor_name)
                .or_insert(Measurement { value: 0.0, unit });
        }
    }

    /// Sends a message on a socket to a board with id `destination`
    fn serialize_and_send<T: serde::ser::Serialize>(
        &self,
        socket: &UdpSocket,
        destination: &str,
        message: &T,
    ) -> std::result::Result<(), String> {
        let mut buf: [u8; 1024] = [0; 1024];

        let Some(device) = self.devices.iter().find(|d| d.id == *destination) else {
            return Err(
                "Tried to sent a message to a board that hasn't been connected yet.".to_string(),
            );
        };

        if let Err(e) = postcard::to_slice::<T>(message, &mut buf) {
            return Err(format!("Couldn't serialize message: {e}"));
        };

        if let Err(e) = device.send(socket, &buf) {
            return Err(format!("Couldn't send message to {destination}: {e}"));
        };

        Ok(())
    }

    /// Send `commands` to the SAMs.
    pub(crate) fn send_sam_commands(
        &mut self,
        socket: &UdpSocket,
        mappings: &Mappings,
        commands: Vec<SequenceDomainCommand>,
        abort_stages: &mut AbortStages,
        gps_handle: Option<&GpsHandle>,
    ) -> bool {
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
                                actual: ValveState::Undetermined,
                            },
                        );
                    }

                    let command = SamControlMessage::ActuateValve {
                        channel: mapping.channel,
                        powered,
                    };

                    if let Err(msg) = self.serialize_and_send(socket, &mapping.board_id, &command) {
                        println!("{}", msg);
                    }
                }

                SequenceDomainCommand::CreateAbortStage {
                    stage_name,
                    abort_condition,
                    valve_safe_states,
                } => {
                    self.create_abort_stage(
                        mappings,
                        abort_stages,
                        AbortStageConfig {
                            stage_name,
                            abort_condition,
                            valve_safe_states,
                        },
                    );
                }

                // TODO: should we not allow setting an abort stage if we already in
                // that abort stage?
                SequenceDomainCommand::SetAbortStage { stage_name } => {
                    self.start_abort_transition(mappings, abort_stages, stage_name, None);
                }
                SequenceDomainCommand::SetAbortStageConfirmed { stage_name, request_id } => {
                    self.start_abort_transition(mappings, abort_stages, stage_name, Some(request_id));
                }

                SequenceDomainCommand::RecoCommand(reco_command) => {
                    self.handle_sequence_reco_message(gps_handle, reco_command);
                }

                SequenceDomainCommand::LaunchLugArm {
                    sam_hostname,
                    should_enable,
                } => {
                    self.send_sams_toggle_launch_lug_arm(socket, sam_hostname, should_enable);
                }
                SequenceDomainCommand::LaunchLugDetonate {
                    sam_hostname,
                    should_enable,
                } => {
                    self.send_sams_toggle_launch_lug_detonate(socket, sam_hostname, should_enable);
                }
                SequenceDomainCommand::CameraEnable { should_enable } => {
                    self.send_sams_toggle_camera(socket, should_enable);
                }
                SequenceDomainCommand::SetServoDisconnectMonitoring { enabled } => {
                    self.servo_disconnect_abort_enabled = enabled;
                    println!(
                        "Abort on servo disconnect set to {}.",
                        if enabled { "enabled" } else { "disabled" }
                    );
                }
                // TODO: shouldn't we break out of the loop here? if we receive an abort
                // command why are we not flushing commands that come in after
                SequenceDomainCommand::Abort => should_abort = true,
                SequenceDomainCommand::ClearAbortStatus => {
                    self.send_sams_clear_abort_status(socket);
                }
            }
        }

        should_abort
    }

    /// Enqueues a RECO command for the RECO worker to process.
    /// These are commands that are not part of the normal flight-reco
    /// communication, rather more specific commands.
    pub(crate) fn enqueue_reco_command(
        &self,
        gps_handle: Option<&GpsHandle>,
        reco_command: RecoControlMessage,
        label: &str,
    ) {
        if let Some(gps_handle) = gps_handle {
            if gps_handle.send_reco_control(reco_command).is_err() {
                eprintln!("Failed to enqueue {label} command for RECO worker.");
            }
        } else {
            eprintln!("Received {label} command, but RECO worker is not initialized.");
        }
    }

    /// Handles a RECO command sent from the GUI path.
    pub(crate) fn handle_gui_reco_command(
        &self,
        gps_handle: Option<&GpsHandle>,
        command: TargetedGuiCommand,
    ) {
        let TargetedGuiCommand { target, command } = command;
        let (reco_command, label) = match command {
            SharedRecoCommand::ProcessNoiseMatrix(process_noise_matrix) => (
                RecoControlMessage::ProcessNoiseMatrix {
                    target,
                    matrix: process_noise_matrix,
                },
                "RecoProcessNoiseMatrix",
            ),
            SharedRecoCommand::MeasurementNoiseMatrix(measurement_noise_matrix) => (
                RecoControlMessage::MeasurementNoiseMatrix {
                    target,
                    matrix: measurement_noise_matrix,
                },
                "RecoMeasurementNoiseMatrix",
            ),
            SharedRecoCommand::EkfStateVector(state_vector) => (
                RecoControlMessage::EkfStateVector {
                    target,
                    vector: state_vector,
                },
                "RecoEkfStateVector",
            ),
            SharedRecoCommand::InitialCovarianceMatrix(initial_covariance_matrix) => (
                RecoControlMessage::InitialCovarianceMatrix {
                    target,
                    matrix: initial_covariance_matrix,
                },
                "RecoInitialCovarianceMatrix",
            ),
            SharedRecoCommand::TimerValues(timer_values) => (
                RecoControlMessage::TimerValues {
                    target,
                    values: timer_values,
                },
                "RecoTimerValues",
            ),
            SharedRecoCommand::AltimeterOffsets(altimeter_offsets) => (
                RecoControlMessage::AltimeterOffsets {
                    target,
                    offsets: altimeter_offsets,
                },
                "RecoAltimeterOffsets",
            ),
        };

        self.enqueue_reco_command(gps_handle, reco_command, label);
    }

    /// Processes a RECO command sent from a sequence.
    fn handle_sequence_reco_message(
        &self,
        gps_handle: Option<&GpsHandle>,
        command: RecoSequenceCommand,
    ) {
        let (reco_command, label) = match command {
            RecoSequenceCommand::Launch => (RecoControlMessage::Launch, "RecoLaunch"),
            RecoSequenceCommand::InitEKF => (RecoControlMessage::InitEKF, "RecoInitEKF"),
        };

        self.enqueue_reco_command(gps_handle, reco_command, label);
    }

    pub(crate) fn create_abort_stage(
        &mut self,
        mappings: &Mappings,
        abort_stages: &mut AbortStages,
        stage_config: AbortStageConfig,
    ) {
        let stage_name = stage_config.stage_name.clone();
        let mut valve_lookup: HashMap<String, (&str, u32, bool)> = HashMap::new();
        for mapping in mappings {
            if mapping.sensor_type == SensorType::Valve {
                let normally_closed = mapping.normally_closed.unwrap_or(true);
                valve_lookup.insert(
                    mapping.text_id.clone(),
                    (&mapping.board_id, mapping.channel, normally_closed),
                );
            }
        }

        // stores [sam_board_id, (channel_num, powered, timer)]. every valve
        // that an operator set an abort config for
        let mut board_valves: HashMap<String, Vec<ValveAction>> = HashMap::new();
        let mut used_channels = HashSet::new();
        for (valve_name, valve_state_info) in stage_config.valve_safe_states {
            // get the mapping for the current valve
            let Some(&(board_id, channel, normally_closed)) = valve_lookup.get(&valve_name) else {
                self.reject_abort_stage_config(&stage_name, mappings);
                eprintln!(
                    "Rejecting abort stage: valve '{}' not found in mappings.",
                    valve_name
                );
                return;
            };
            if !board_id.starts_with("sam") || !(1..=6).contains(&channel) {
                self.reject_abort_stage_config(&stage_name, mappings);
                eprintln!("Rejecting abort stage: invalid SAM channel {board_id}:{channel}");
                return;
            }
            if !used_channels.insert((board_id.to_string(), channel)) {
                self.reject_abort_stage_config(&stage_name, mappings);
                eprintln!("Rejecting abort stage: duplicate SAM channel {board_id}:{channel}");
                return;
            }

            // determine if we want to give power to this valve
            let closed = valve_state_info.desired_state == ValveState::Closed;
            let powered = closed != normally_closed;

            // append our determination of whether to power this valve to its
            // SAM board vector
            board_valves
                .entry(board_id.to_string())
                .or_default()
                .push(ValveAction {
                    channel_num: channel,
                    powered,
                    timer: Duration::from_millis(valve_state_info.safing_timer as u64),
                });
        }

        let updated = AbortStage {
            name: stage_config.stage_name,
            abort_condition: stage_config.abort_condition,
            aborted: false,
            valve_safe_states: board_valves,
        };
        self.invalid_saved_stages.remove(&updated.name);
        if self.state.abort_stage.name == updated.name
            && (self.state.abort_stage.abort_condition != updated.abort_condition || self.state.abort_stage.valve_safe_states != updated.valve_safe_states)
            && self.transition.as_ref().is_none_or(|t| t.phase == Phase::Confirmed) {
            self.transition = None;
            self.state.abort_transition = Some(AbortTransitionStatus {
                request_id: None, revision: 0, stage: updated.name.clone(), phase: "unsent".into(), unsynchronized: self.unsynchronized,
                sams: required_boards(mappings.iter().map(|m| m.board_id.as_str()),
                    self.state.abort_stage.valve_safe_states.keys().map(String::as_str),
                    updated.valve_safe_states.keys().map(String::as_str))
                    .into_iter().map(|id| (id, "unsent".into())).collect(),
            });
        }
        if let Some(index) = abort_stages.iter().position(|stage| stage.name == updated.name) {
            abort_stages.swap_remove(index);
        }
        abort_stages.push(updated);
    }

    fn reject_abort_stage_config(&mut self, name: &str, mappings: &Mappings) {
        self.invalid_saved_stages.insert(name.to_string());
        if let Some(transition) = &mut self.transition {
            if transition.stage == name && matches!(transition.phase, Phase::Preparing | Phase::Activating) {
                transition.fail();
                self.sync_abort_transition_status();
                return;
            }
        }
        if self.state.abort_stage.name != name { return; }
        if self.transition.as_ref().is_none_or(|transition| transition.phase == Phase::Confirmed) {
            self.transition = None;
            self.state.abort_transition = Some(AbortTransitionStatus {
                request_id: None, revision: 0, stage: name.to_string(),
                phase: "unsent".into(), unsynchronized: self.unsynchronized,
                sams: required_boards(mappings.iter().map(|m| m.board_id.as_str()),
                    self.state.abort_stage.valve_safe_states.keys().map(String::as_str),
                    std::iter::empty())
                    .into_iter().map(|id| (id, "unsent".into())).collect(),
            });
        }
    }

    pub(crate) fn start_abort_transition(
        &mut self,
        mappings: &Mappings,
        stages: &AbortStages,
        name: String,
        request_id: Option<u128>,
    ) {
        self.v2_used = true;
        if let Some(previous) = &mut self.transition {
            previous.supersede();
            self.sync_abort_transition_status();
        }
        let stage = if self.invalid_saved_stages.contains(&name) {
            None
        } else {
            stages.iter().find(|stage| stage.name == name).cloned()
        };
        let mut required = required_boards(
            mappings.iter().map(|mapping| mapping.board_id.as_str()),
            self.state
                .abort_stage
                .valve_safe_states
                .keys()
                .map(String::as_str),
            stage
                .iter()
                .flat_map(|stage| stage.valve_safe_states.keys().map(String::as_str)),
        );
        if let Some(previous) = &self.transition {
            required.extend(previous.boards.keys().cloned());
        }
        required.extend(self.cleared_boards.iter().cloned());
        if let Some(known) = &mut self.required_boards {
            required.extend(known.iter().cloned());
            if !mappings.is_empty() && stage.is_some() {
                if let Err(error) = crate::revision_epoch::record_required(
                    &self.recovery_dir,
                    known,
                    required.iter().cloned(),
                ) {
                    eprintln!("Abort-stage inventory unavailable: {error}");
                    self.required_boards = None;
                }
            }
        }
        let revision = if !mappings.is_empty() && self.required_boards.is_some() {
            self.next_revision()
        } else {
            None
        };
        let Some(revision) = revision else {
            self.transition = Some(Transition::new(
                0,
                request_id,
                name,
                required,
                Instant::now(),
            ));
            self.transition.as_mut().unwrap().fail();
            self.sync_abort_transition_status();
            return;
        };
        let mut transition = Transition::new(revision, request_id, name, required, Instant::now());
        if stage.is_none() {
            transition.fail();
        }
        self.pending_stage = stage;
        self.transition = Some(transition);
        self.sync_abort_transition_status();
    }

    fn next_revision(&mut self) -> Option<u128> {
        let counter = self.revision_counter.checked_add(1)?;
        let epoch = self.revision_epoch?;
        self.revision_counter = counter;
        Some(((epoch as u128) << 64) | counter as u128)
    }

    pub(crate) fn transition_in_progress(&self) -> bool {
        self.transition.as_ref().is_some_and(|t| matches!(t.phase, Phase::Preparing | Phase::Activating | Phase::PartiallyApplied))
    }

    pub(crate) fn legacy_reconnect_sync_allowed(&self) -> bool {
        !self.v2_used && !self.transition_in_progress()
    }

    pub(crate) fn invalidate_disconnected_sams(&mut self) {
        let devices = &self.devices;
        if self.transition.as_mut().is_some_and(|transition| transition.invalidate_if_disconnected(
            |id| devices.iter().any(|device| device.id == id && !device.is_disconnected())
        )) {
            self.sync_abort_transition_status();
        }
    }

    pub(crate) fn advance_abort_transition(&mut self, socket: &UdpSocket) {
        let Some(transition) = &mut self.transition else { return; };
        let boards = transition.tick(Instant::now());
        let (revision, phase) = (transition.revision, transition.phase);
        let mut invalid_timer = false;
        for board in boards {
            let command = match phase {
                Phase::Preparing => {
                    let actions = self.pending_stage.as_ref()
                        .and_then(|stage| stage.valve_safe_states.get(&board));
                    let converted = actions.into_iter().flatten()
                        .map(ValveActionMillis::try_from)
                        .collect::<std::result::Result<Vec<_>, _>>();
                    let Ok(valve_states) = converted else {
                        eprintln!("Rejecting abort stage: valve delay exceeds wire format");
                        invalid_timer = true;
                        break;
                    };
                    SamControlMessage::PrepareAbortStageMillis { revision, valve_states }
                }
                Phase::Activating => SamControlMessage::ActivateAbortStage { revision },
                _ => continue,
            };
            if let Err(error) = self.serialize_and_send(socket, &board, &command) { eprintln!("Abort-stage dispatch to {board}: {error}"); }
        }
        if invalid_timer {
            if let Some(transition) = &mut self.transition { transition.fail(); }
        }
        if phase == Phase::Confirmed {
            if let Some(stage) = self.pending_stage.take() { self.set_abort_stage(&stage); }
            self.cleared_boards.clear();
        }
        self.sync_abort_transition_status();
    }

    fn sync_abort_transition_status(&mut self) {
        if let Some(transition) = &self.transition {
            self.unsynchronized = unsynchronized(self.unsynchronized, transition.phase);
        }
        if self.transition.as_ref().is_some_and(|t| t.phase == Phase::PartiallyApplied)
            && self.state.abort_transition.as_ref().is_none_or(|status| status.phase != "partially_applied") {
            eprintln!("CRITICAL: abort-stage transition partially applied; SAMs may have different active configurations");
        }
        self.state.abort_transition = self.transition.as_ref().map(|transition| AbortTransitionStatus {
            request_id: transition.request_id.map(|id| format!("{id:032x}")),
            revision: transition.revision,
            stage: transition.stage.clone(),
            phase: match transition.phase {
                Phase::Preparing => "preparing", Phase::Activating => "activating",
                Phase::Confirmed => "confirmed", Phase::Failed => "failed",
                Phase::PartiallyApplied => "partially_applied",
            }.into(),
            unsynchronized: self.unsynchronized,
            sams: transition.boards.iter().map(|(id, phase)| (id.clone(), match phase {
                BoardPhase::Pending => "pending", BoardPhase::Prepared => "prepared",
                BoardPhase::Activated => "activated", BoardPhase::Rejected => "rejected",
                BoardPhase::Timeout => "timeout",
            }.into())).collect(),
        });
    }

    pub(crate) fn send_sams_abort_stage(&self, socket: &UdpSocket, board_id: &Option<&String>) {
        let mut found = false;
        for device in self.devices.iter().filter(|device| {
            device.get_board_id().starts_with("sam")
                && board_id.is_none_or(|requested| requested.as_str() == device.get_board_id().as_str())
        }) {
            found = true;
            let id = device.get_board_id();
            let valve_states = self.state.abort_stage.valve_safe_states.get(id).cloned().unwrap_or_default();
            if valve_states.iter().any(|valve| valve.timer.subsec_nanos() != 0) {
                eprintln!("Cannot synchronize subsecond abort delay to legacy SAM {id}");
                continue;
            }
            let command = SamControlMessage::AbortStageValveStates { valve_states };
            if let Err(error) = self.serialize_and_send(socket, id, &command) {
                eprintln!("Abort-stage synchronization to {id}: {error}");
            }
        }
        if board_id.is_some() && !found {
            eprintln!("Cannot synchronize abort stage: requested SAM is not registered");
        }
    }

    /// Clears the abort status from all sams. This is used to tell a SAM that
    /// we are no longer in an abort state, which allows for future aborts to be
    /// performed.
    pub(crate) fn send_sams_clear_abort_status(&mut self, socket: &UdpSocket) {
        for device in self.devices.iter() {
            if device.get_board_id().starts_with("sam") {
                let command = SamControlMessage::ClearAbortStatus;
                if let Err(msg) = self.serialize_and_send(socket, device.get_board_id(), &command) {
                    println!("{}", msg);
                } else {
                    println!("Cleared abort status from {}", device.get_board_id());
                }
            }
        }

        // clear aborted state for this abort stage
        self.state.abort_stage.aborted = false;
    }

    pub(crate) fn send_sams_abort(&mut self, socket: &UdpSocket, sequences: &mut Sequences) {
        if let Some(transition) = &mut self.transition {
            if matches!(transition.phase, Phase::Preparing | Phase::Activating) { transition.fail(); self.sync_abort_transition_status(); }
        }
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
                let command = SamControlMessage::Abort;
                // send message to this sam board
                if let Err(msg) = self.serialize_and_send(socket, device.get_board_id(), &command) {
                    println!("{}", msg);
                } else {
                    println!(
                        "Sent abort message to SAM: {}, which will use {} stage's safe valves.",
                        device.get_board_id(),
                        self.state.abort_stage.name
                    );
                }
            }
        }

        // update state to say that we have aborted in this stage
        self.state.abort_stage.aborted = true;
    }

    // Clears any stored abort stages on sams
    pub(crate) fn send_sam_clear_abort_stage(
        &mut self,
        socket: &UdpSocket,
        old_mappings: &Mappings,
        mappings: &Mappings,
    ) {
        self.unsynchronized |= self
            .transition
            .as_ref()
            .is_some_and(Transition::may_have_activated);
        let boards = required_boards(
            old_mappings
                .iter()
                .chain(mappings)
                .map(|m| m.board_id.as_str()),
            self.state
                .abort_stage
                .valve_safe_states
                .keys()
                .map(String::as_str),
            self.transition
                .iter()
                .flat_map(|t| t.boards.keys().map(String::as_str)),
        );
        if let Some(known) = &mut self.required_boards {
            if let Err(error) = crate::revision_epoch::record_required(
                &self.recovery_dir,
                known,
                boards.iter().cloned(),
            ) {
                eprintln!("Abort-stage inventory unavailable: {error}");
                self.required_boards = None;
            }
        }
        self.cleared_boards.extend(boards);
        self.transition = None;
        self.pending_stage = None;
        self.state.abort_transition = Some(AbortTransitionStatus {
            request_id: None,
            revision: 0,
            stage: self.state.abort_stage.name.clone(),
            phase: "unsent".into(),
            unsynchronized: self.unsynchronized,
            sams: self
                .cleared_boards
                .iter()
                .cloned()
                .map(|id| (id, "unsent".into()))
                .collect(),
        });
        if self.required_boards.is_none() {
            return;
        }
        let versioned_clear = self
            .next_revision()
            .map(|revision| SamControlMessage::ClearStoredAbortStageV2 { revision });
        if versioned_clear.is_none() {
            eprintln!("Cannot send revisioned SAM clear: durable revision unavailable");
        }
        for device in self.devices.iter() {
            if device.get_board_id().starts_with("sam") {
                if !self.v2_used {
                    if let Err(msg) = self.serialize_and_send(
                        socket,
                        device.get_board_id(),
                        &SamControlMessage::ClearStoredAbortStage {},
                    ) {
                        eprintln!("{msg}");
                    }
                }
                if let Some(command) = &versioned_clear {
                    if let Err(msg) =
                        self.serialize_and_send(socket, device.get_board_id(), command)
                    {
                        eprintln!("{msg}");
                    }
                }
            }
        }
    }

    pub(crate) fn send_sams_toggle_camera(&self, socket: &UdpSocket, should_enable: bool) {
        for device in self.devices.iter() {
            // only send camera enable message to flight sams
            if device.get_board_id().starts_with("sam-2")
                || device.get_board_id().starts_with("sam-3")
            {
                // create message for this sam board
                let command = SamControlMessage::CameraEnable(should_enable);

                // send message to this sam board
                if let Err(msg) = self.serialize_and_send(socket, device.get_board_id(), &command) {
                    println!("{}", msg);
                }
            }
        }
    }

    pub(crate) fn send_sams_toggle_launch_lug_arm(
        &self,
        socket: &UdpSocket,
        sam_hostname: String,
        should_enable: bool,
    ) {
        if let Some(device) = self
            .devices
            .iter()
            .find(|d| d.get_board_id() == &sam_hostname)
        {
            let command = SamControlMessage::LaunchLugArm(should_enable);

            // send message to this sam board
            if let Err(msg) = self.serialize_and_send(socket, device.get_board_id(), &command) {
                println!("{}", msg);
            }
        } else {
            eprintln!("Invalid board id passed in when trying to send sams launch lug arm: Either your board does not exist or is not a sam.");
        }
    }

    pub(crate) fn send_sams_toggle_launch_lug_detonate(
        &self,
        socket: &UdpSocket,
        sam_hostname: String,
        should_enable: bool,
    ) {
        if let Some(device) = self
            .devices
            .iter()
            .find(|d| d.get_board_id() == &sam_hostname)
        {
            let command = SamControlMessage::LaunchLugDetonate(should_enable);

            // send message to this sam board
            if let Err(msg) = self.serialize_and_send(socket, device.get_board_id(), &command) {
                println!("{}", msg);
            }
        } else {
            eprintln!("Invalid board id passed in when trying to send sams launch lug detonate: Either your board does not exist or is not a sam.");
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

    /// Update GPS-related fields on the vehicle state with a new sample.
    pub(crate) fn update_gps(&mut self, sample: GpsState) {
        self.state.gps = Some(sample);
        self.state.gps_valid = true;
    }

    /// Mark GPS data as invalid for the current control-loop iteration.
    ///
    /// The underlying GPS values are preserved for debugging/logging, but
    /// downstream code should treat them as stale once this flag is false.
    pub(crate) fn invalidate_gps(&mut self) {
        self.state.gps_valid = false;
    }

    /// Update RECO-related fields on the vehicle state with new samples from
    /// all three MCUs. The array should contain: [MCU A (spidev1.2), MCU B
    /// (spidev1.1), MCU C (spidev1.0)]
    pub(crate) fn update_reco(&mut self, samples: [Option<RecoState>; 3]) {
        self.state.rbf.reco = get_reco_rbf_values(&samples);
        self.state.reco = samples.into();
        self.state.reco_valid = true;
    }

    /// Mark RECO data as invalid for the current control-loop iteration.
    ///
    /// The underlying RECO values are preserved for debugging/logging, but
    /// downstream code should treat them as stale once this flag is false.
    pub(crate) fn invalidate_reco(&mut self) {
        self.state.reco_valid = false;
    }

    pub(crate) fn get_state(&self) -> &VehicleState {
        &self.state
    }

    /// Returns whether the FC should monitor servo disconnects.
    pub(crate) fn servo_disconnect_abort_enabled(&self) -> bool {
        self.servo_disconnect_abort_enabled
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

/// Gets the RBF values for each RECO MCU from the RECO state samples.
pub(crate) fn get_reco_rbf_values(samples: &[Option<RecoState>; 3]) -> [u8; 3] {
    [
        samples[0]
            .as_ref()
            .map(|s| u8::from(s.rbf_enabled))
            .unwrap_or(0),
        samples[1]
            .as_ref()
            .map(|s| u8::from(s.rbf_enabled))
            .unwrap_or(0),
        samples[2]
            .as_ref()
            .map(|s| u8::from(s.rbf_enabled))
            .unwrap_or(0),
    ]
}

/// performs a flight handshake with the board.
pub(crate) fn handshake(address: &SocketAddr, socket: &UdpSocket) -> Result<()> {
    let mut buf: [u8; 1024] = [0; 1024];
    let serialized = postcard::to_slice(&DataMessage::Identity("flight-01".to_string()), &mut buf)
        .map_err(Error::SerializationFailed)?;
    socket
        .send_to(serialized, address)
        .map_err(Error::TransportFailed)?;
    Ok(())
}

/// Gets the most recent UDP Commands
pub(crate) fn receive(socket: &UdpSocket) -> Vec<(SocketAddr, DataMessage<'static>)> {
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
    }

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
            Self::SerializationFailed(e) => {
                write!(f, "Couldn't serialize an outgoing message: {e}")
            }
            Self::TransportFailed(e) => {
                write!(f, "Couldn't send data to a device: {e}")
            }
        }
    }
}

#[cfg(test)]
mod abort_transition_loopback_tests {
    use std::borrow::Cow;

    use super::*;
    use common::comm::ValveSafeState;

    fn mapping(board: &str) -> NodeMapping {
        NodeMapping {
            text_id: format!("valve-{board}"),
            board_id: board.into(),
            sensor_type: SensorType::Valve,
            channel: 0,
            computer: common::comm::Computer::Flight,
            max: None,
            min: None,
            calibrated_offset: 0.0,
            powered_threshold: None,
            normally_closed: None,
        }
    }

    #[test]
    fn invalid_abort_config_does_not_save_a_partial_stage() {
        let dir = std::env::temp_dir().join(format!("luna-invalid-abort-{}", std::process::id()));
        let mut devices = Devices::with_recovery_dir(dir.clone());
        let mut stages = Vec::new();
        let safe = ValveSafeState { desired_state: ValveState::Closed, safing_timer: 200 };
        let config = |names: &[&str]| AbortStageConfig {
            stage_name: "test".into(),
            abort_condition: "False".into(),
            valve_safe_states: names.iter().map(|id| (id.to_string(), safe.clone())).collect(),
        };

        let mut valid = mapping("sam-test");
        valid.channel = 1;
        devices.create_abort_stage(&vec![valid.clone()], &mut stages, config(&["missing"]));
        assert!(stages.is_empty());

        let mut invalid_channel = valid.clone();
        invalid_channel.channel = 7;
        devices.create_abort_stage(&vec![invalid_channel], &mut stages, config(&["valve-sam-test"]));
        assert!(stages.is_empty());

        let mut duplicate = valid.clone();
        duplicate.text_id = "second-valve".into();
        devices.create_abort_stage(&vec![valid.clone(), duplicate], &mut stages,
            config(&["valve-sam-test", "second-valve"]));
        assert!(stages.is_empty());

        devices.create_abort_stage(&vec![valid], &mut stages, config(&["valve-sam-test"]));
        assert_eq!(stages.len(), 1);
        assert_eq!(stages[0].valve_safe_states["sam-test"][0].timer, Duration::from_millis(200));
        assert!(!devices.invalid_saved_stages.contains("test"));
        let active = devices.state.abort_stage.clone();
        let mut replacement = config(&["valve-sam-test"]);
        replacement
            .valve_safe_states
            .get_mut("valve-sam-test")
            .unwrap()
            .safing_timer = 500;
        devices.create_abort_stage(
            &vec![NodeMapping { channel: 1, ..mapping("sam-test") }],
            &mut stages,
            replacement,
        );
        assert_eq!(stages.len(), 1);
        assert_eq!(stages[0].valve_safe_states["sam-test"][0].timer, Duration::from_millis(500));
        assert_eq!(devices.state.abort_stage, active);
        devices.state.abort_stage = stages[0].clone();
        devices.state.abort_transition = Some(AbortTransitionStatus {
            request_id: None, revision: 1, stage: "test".into(),
            phase: "confirmed".into(), unsynchronized: false,
            sams: HashMap::from([("sam-test".into(), "activated".into())]),
        });
        devices.create_abort_stage(&vec![mapping("sam-test")], &mut stages, config(&["missing"]));
        assert!(devices.invalid_saved_stages.contains("test"));
        assert_eq!(stages.len(), 1);
        assert_eq!(devices.state.abort_transition.as_ref().unwrap().phase, "unsent");
        devices.transition = Some(Transition::new(2, Some(3), "test".into(), ["sam-test".into()], Instant::now()));
        devices.create_abort_stage(&vec![mapping("sam-test")], &mut stages, config(&["missing"]));
        assert_eq!(devices.state.abort_transition.as_ref().unwrap().phase, "failed");
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn restart_requires_all_historical_sams_and_fresh_acks() {
        for case in ["confirmed", "preparing", "activating", "partial", "removed"] {
            let dir =
                std::env::temp_dir().join(format!("luna-reboot-{}-{case}", std::process::id()));
            std::fs::create_dir_all(&dir).unwrap();
            std::fs::write(dir.join("required-sams"), []).unwrap();
            std::fs::write(dir.join("required-sams-state"), b"0000000000000000cbf29ce484222325\n").unwrap();
            let old_mappings = vec![mapping("sam-old")];
            let new_mappings = vec![mapping("sam-new")];
            let stage = AbortStage {
                name: "test".into(),
                abort_condition: "False".into(),
                aborted: false,
                valve_safe_states: HashMap::from([
                    ("sam-old".into(), Vec::new()),
                    ("sam-peer".into(), Vec::new()),
                ]),
            };
            let mut before = Devices::with_recovery_dir(dir.clone());
            before.start_abort_transition(
                &old_mappings,
                &vec![stage.clone()],
                "test".into(),
                Some(1),
            );
            let old_revision = before.transition.as_ref().unwrap().revision;
            if case != "preparing" {
                let transition = before.transition.as_mut().unwrap();
                for board in ["sam-old", "sam-peer"] {
                    transition.ack(board, old_revision, true, false, false);
                }
                transition.tick(Instant::now());
                transition.ack("sam-old", old_revision, false, true, false);
                if case == "confirmed" {
                    transition.ack("sam-peer", old_revision, false, true, false);
                    transition.tick(Instant::now());
                } else if case == "partial" {
                    transition.fail();
                }
                before.sync_abort_transition_status();
            }
            if case == "removed" {
                let socket = UdpSocket::bind("127.0.0.1:0").unwrap();
                before.send_sam_clear_abort_stage(&socket, &old_mappings, &new_mappings);
            }
            drop(before);

            let mut after = Devices::with_recovery_dir(dir.clone());
            assert!(
                after
                    .state
                    .abort_transition
                    .as_ref()
                    .unwrap()
                    .unsynchronized,
                "{case}"
            );
            assert_ne!(
                after.state.abort_transition.as_ref().unwrap().phase,
                "confirmed",
                "{case}"
            );
            assert!(
                after.required_boards.as_ref().unwrap().contains("sam-old"),
                "{case}"
            );
            after.register_device("sam-old", "127.0.0.1:4573".parse().unwrap());
            assert!(
                after
                    .state
                    .abort_transition
                    .as_ref()
                    .unwrap()
                    .unsynchronized,
                "{case}"
            );
            let next = AbortStage {
                name: "next".into(),
                abort_condition: "False".into(),
                aborted: false,
                valve_safe_states: HashMap::from([("sam-new".into(), Vec::new())]),
            };
            after.start_abort_transition(&new_mappings, &vec![next], "next".into(), Some(2));
            let transition = after.transition.as_mut().unwrap();
            assert!(transition.revision > old_revision, "{case}");
            for board in ["sam-old", "sam-peer", "sam-new"] {
                assert!(transition.boards.contains_key(board), "{case}: {board}");
                transition.ack(board, old_revision, true, false, false);
                assert_eq!(transition.boards[board], BoardPhase::Pending);
                transition.ack(board, transition.revision, true, false, false);
            }
            transition.tick(Instant::now());
            for board in ["sam-old", "sam-peer", "sam-new"] {
                transition.ack(board, transition.revision, false, true, false);
            }
            transition.tick(Instant::now());
            after.sync_abort_transition_status();
            assert_eq!(
                after.state.abort_transition.as_ref().unwrap().phase,
                "confirmed",
                "{case}"
            );
            assert!(
                !after
                    .state
                    .abort_transition
                    .as_ref()
                    .unwrap()
                    .unsynchronized,
                "{case}"
            );
            std::fs::remove_dir_all(dir).unwrap();
        }
    }

    #[test]
    fn missing_inventory_or_unloaded_mappings_cannot_confirm() {
        let dir = std::env::temp_dir().join(format!("luna-unverified-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let mut devices = Devices::with_recovery_dir(dir.clone());
        let stage = AbortStage {
            name: "test".into(),
            abort_condition: "False".into(),
            aborted: false,
            valve_safe_states: HashMap::from([("sam-test".into(), Vec::new())]),
        };
        devices.start_abort_transition(
            &vec![mapping("sam-test")],
            &vec![stage.clone()],
            "test".into(),
            Some(1),
        );
        assert_eq!(
            devices.state.abort_transition.as_ref().unwrap().phase,
            "failed"
        );
        assert!(
            devices
                .state
                .abort_transition
                .as_ref()
                .unwrap()
                .unsynchronized
        );
        std::fs::write(dir.join("required-sams"), []).unwrap();
        std::fs::write(dir.join("required-sams-state"), b"0000000000000000cbf29ce484222325\n").unwrap();
        let mut devices = Devices::with_recovery_dir(dir.clone());
        devices.start_abort_transition(&Vec::new(), &vec![stage], "test".into(), Some(2));
        assert_eq!(
            devices.state.abort_transition.as_ref().unwrap().phase,
            "failed"
        );
        assert!(
            devices
                .state
                .abort_transition
                .as_ref()
                .unwrap()
                .unsynchronized
        );
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn prepare_retry_activate_and_ack_over_loopback() {
        let recovery_dir = std::env::temp_dir().join(format!("luna-loopback-{}", std::process::id()));
        std::fs::create_dir_all(&recovery_dir).unwrap();
        std::fs::write(recovery_dir.join("required-sams"), []).unwrap();
        std::fs::write(recovery_dir.join("required-sams-state"), b"0000000000000000cbf29ce484222325\n").unwrap();
        let flight = UdpSocket::bind("127.0.0.1:0").unwrap();
        flight.set_nonblocking(true).unwrap();
        let sam = UdpSocket::bind(("127.0.0.1", DEVICE_COMMAND_PORT)).unwrap();
        sam.set_read_timeout(Some(Duration::from_secs(1))).unwrap();
        let mut devices = Devices {
            devices: Vec::new(), state: VehicleState::new(), last_updates: HashMap::new(),
            servo_disconnect_abort_enabled: true, transition: None, pending_stage: None,
            cleared_boards: HashSet::new(), v2_used: false, unsynchronized: false,
            revision_epoch: Some(1), revision_counter: 0,
            recovery_dir: recovery_dir.clone(), required_boards: Some(HashSet::new()),
            invalid_saved_stages: HashSet::new(),
        };
        devices.register_device("sam-test", sam.local_addr().unwrap());
        let stage = AbortStage {
            name: "test".into(), abort_condition: "False".into(), aborted: false,
            valve_safe_states: HashMap::from([("sam-test".into(), Vec::new())]),
        };
        let mappings = vec![mapping("sam-test")];
        devices.start_abort_transition(&mappings, &vec![stage], "test".into(), Some(42));
        let mut buffer = [0u8; 1024];
        devices.advance_abort_transition(&flight);
        let (size, _) = sam.recv_from(&mut buffer).unwrap();
        let revision = match postcard::from_bytes::<SamControlMessage>(&buffer[..size]).unwrap() {
            SamControlMessage::PrepareAbortStageMillis { revision, .. } => revision,
            other => panic!("expected PREPARE, got {other:?}"),
        };
        std::thread::sleep(Duration::from_millis(260));
        devices.advance_abort_transition(&flight);
        let (size, _) = sam.recv_from(&mut buffer).unwrap();
        assert!(matches!(postcard::from_bytes::<SamControlMessage>(&buffer[..size]).unwrap(),
            SamControlMessage::PrepareAbortStageMillis { revision: retry, .. } if retry == revision));

        let ack = |phase| DataMessage::Sam("sam-test".into(), Cow::Owned(vec![
            SamDataPoint::AbortStageAck { revision, phase }
        ]));
        let send_ack = |phase| {
            let bytes = postcard::to_allocvec(&ack(phase)).unwrap();
            sam.send_to(&bytes, flight.local_addr().unwrap()).unwrap();
        };
        send_ack(AbortStageAck::Prepared);
        devices.update_state(receive(&flight), &mappings, &flight);
        devices.advance_abort_transition(&flight);
        let (size, _) = sam.recv_from(&mut buffer).unwrap();
        assert!(matches!(postcard::from_bytes::<SamControlMessage>(&buffer[..size]).unwrap(),
            SamControlMessage::ActivateAbortStage { revision: sent } if sent == revision));
        send_ack(AbortStageAck::Activated);
        devices.update_state(receive(&flight), &mappings, &flight);
        devices.advance_abort_transition(&flight);
        assert_eq!(devices.state.abort_transition.as_ref().unwrap().phase, "confirmed");
        std::thread::sleep(TIME_TO_LIVE + Duration::from_millis(10));
        devices.invalidate_disconnected_sams();
        assert_eq!(devices.state.abort_transition.as_ref().unwrap().phase, "partially_applied");
        assert!(devices.state.abort_transition.as_ref().unwrap().unsynchronized);

        let stage = AbortStage {
            name: "test".into(), abort_condition: "False".into(), aborted: false,
            valve_safe_states: HashMap::from([("sam-test".into(), Vec::new())]),
        };
        devices.start_abort_transition(&mappings, &vec![stage], "test".into(), Some(44));
        devices.advance_abort_transition(&flight);
        let _ = sam.recv_from(&mut buffer).unwrap();
        std::thread::sleep(Duration::from_secs(5));
        devices.advance_abort_transition(&flight);
        assert_eq!(devices.state.abort_transition.as_ref().unwrap().phase, "failed");
        assert!(devices.state.abort_transition.as_ref().unwrap().unsynchronized);
        assert_eq!(devices.state.abort_stage.name, "test");

        devices.send_sam_clear_abort_stage(&flight, &mappings, &mappings);
        let (size, _) = sam.recv_from(&mut buffer).unwrap();
        let clear_revision = match postcard::from_bytes::<SamControlMessage>(&buffer[..size]).unwrap() {
            SamControlMessage::ClearStoredAbortStageV2 { revision } => revision,
            other => panic!("expected revisioned clear, got {other:?}"),
        };
        assert!(clear_revision > revision);
        assert_eq!(devices.state.abort_transition.as_ref().unwrap().phase, "unsent");
        let stage = AbortStage {
            name: "test".into(), abort_condition: "False".into(), aborted: false,
            valve_safe_states: HashMap::from([("sam-test".into(), Vec::new())]),
        };
        devices.start_abort_transition(&mappings, &vec![stage], "test".into(), Some(43));
        devices.advance_abort_transition(&flight);
        let (size, _) = sam.recv_from(&mut buffer).unwrap();
        let next_revision = match postcard::from_bytes::<SamControlMessage>(&buffer[..size]).unwrap() {
            SamControlMessage::PrepareAbortStageMillis { revision, .. } => revision,
            other => panic!("expected PREPARE after clear, got {other:?}"),
        };
        assert!(next_revision > clear_revision);
        let send_next_ack = |phase| {
            let message = DataMessage::Sam("sam-test".into(), Cow::Owned(vec![
                SamDataPoint::AbortStageAck { revision: next_revision, phase }
            ]));
            sam.send_to(&postcard::to_allocvec(&message).unwrap(), flight.local_addr().unwrap()).unwrap();
        };
        send_next_ack(AbortStageAck::Prepared);
        devices.update_state(receive(&flight), &mappings, &flight);
        devices.advance_abort_transition(&flight);
        let (size, _) = sam.recv_from(&mut buffer).unwrap();
        assert!(matches!(postcard::from_bytes::<SamControlMessage>(&buffer[..size]).unwrap(),
            SamControlMessage::ActivateAbortStage { revision } if revision == next_revision));
        send_next_ack(AbortStageAck::Activated);
        devices.update_state(receive(&flight), &mappings, &flight);
        devices.advance_abort_transition(&flight);
        assert_eq!(devices.state.abort_transition.as_ref().unwrap().phase, "confirmed");
        devices.state.abort_stage.valve_safe_states.insert("sam-test".into(), vec![ValveAction {
            channel_num: 1, powered: true, timer: Duration::from_millis(200),
        }]);
        devices.send_sams_abort_stage(&flight, &Some(&"sam-test".into()));
        sam.set_nonblocking(true).unwrap();
        assert_eq!(sam.recv_from(&mut buffer).unwrap_err().kind(), io::ErrorKind::WouldBlock);
        sam.set_nonblocking(false).unwrap();
        devices.state.abort_stage.valve_safe_states.get_mut("sam-test").unwrap()[0].timer = Duration::from_secs(2);
        devices.send_sams_abort_stage(&flight, &Some(&"sam-test".into()));
        let (size, _) = sam.recv_from(&mut buffer).unwrap();
        assert_eq!(buffer[0], 1);
        assert!(matches!(postcard::from_bytes::<SamControlMessage>(&buffer[..size]).unwrap(),
            SamControlMessage::AbortStageValveStates { valve_states } if valve_states[0].timer == Duration::from_secs(2)));
        std::fs::remove_dir_all(recovery_dir).unwrap();
    }
}
