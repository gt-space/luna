// To parse this data:
//
//   import { Convert, Types } from "./types";
//
//   const types = Convert.toTypes(json);
//
// These functions will throw an error if the JSON doesn't
// match the expected interface, even if the JSON is valid.

export interface Types {
    abort_stage_config:              AbortStageConfig;
    active_configuration:            ActiveConfiguration;
    camera_enable:                   CameraEnable;
    delete_abort_config_request:     DeleteAbortConfigRequest;
    delete_mappings_request:         DeleteMappingsRequest;
    delete_sequence_request:         DeleteSequenceRequest;
    delete_trigger_request:          DeleteTriggerRequest;
    execute_sql_request:             ExecuteSQLRequest;
    execute_sql_response:            ExecuteSQLResponse;
    export_request:                  ExportRequest;
    lug_arm:                         LugArm;
    lug_detonate:                    LugDetonate;
    node_mapping:                    NodeMappingElement;
    operator_command_request:        OperatorCommandRequest;
    reco_gui_command_payload:        RecoGUICommandPayload;
    reco_gui_command_request:        RecoGUICommandRequest;
    retrieve_abort_configs_response: RetrieveAbortConfigsResponse;
    retrieve_sequence_response:      RetrieveSequenceResponse;
    run_sequence_request:            RunSequenceRequest;
    save_sequence_request:           SaveSequenceRequest;
    sequence_with_configuration:     SequenceWithConfigurationElement;
    set_abort_config:                SetAbortConfig;
    set_mappings_request:            SetMappingsRequest;
    stop_sequence_request:           StopSequenceRequest;
    telemetry_source_query:          TelemetrySourceQuery;
    telemetry_source_stats:          TelemetrySourceStats;
    telemetry_stats_response:        TelemetryStatsResponse;
    trigger:                         Trigger;
    vector:                          Vector;
    vehicle_state:                   VehicleState;
    [property: string]: unknown;
}

/**
 * An input config from a user
 */
export interface AbortStageConfig {
    /**
     * The condition upon which the trigger script is run, written in Python.
     */
    abort_condition: string;
    /**
     * The unique, human-readable name which identifies the AbortStage.
     */
    stage_name: string;
    /**
     * Desired safe states of valves that we want
     */
    valve_safe_states: { [key: string]: ValveSafeStateValue };
    [property: string]: unknown;
}

/**
 * Information about a specific valve's safe state
 */
export interface ValveSafeStateValue {
    /**
     * Desired state of a valve
     */
    desired_state: DesiredState;
    /**
     * Timer (in milliseconds!!!) that allows us to delay putting a valve in
     * its safe state by some amount of time Can't use Instant here since
     * Instant does not implement serde::Serialize or deserialize
     */
    safing_timer: number;
    [property: string]: unknown;
}

/**
 * Desired state of a valve
 *
 * The state or commanded state of a valve.
 *
 * Actual state of the valve, determined using voltage and current
 * measurements.
 *
 * Commanded state of the valve, according to software.
 *
 * Undetermined state, whether because the valve is unmapped or has not
 * been commanded yet.
 *
 * Valve disconnected.
 *
 * Valve open.
 *
 * Valve closed.
 *
 * Fault in valve.
 */
export type DesiredState = "undetermined" | "disconnected" | "open" | "closed" | "fault";

/**
 * Request/response struct for getting and setting the active configuration.
 */
export interface ActiveConfiguration {
    configuration_id: string;
    [property: string]: unknown;
}

/**
 * Obvious
 */
export interface CameraEnable {
    enabled: boolean;
    [property: string]: unknown;
}

/**
 * Request struct to delete a sequence from the database.
 */
export interface DeleteAbortConfigRequest {
    /**
     * The name stored in the database identifying the sequence to be deleted.
     */
    name: string;
    [property: string]: unknown;
}

/**
 * The request struct used with the route function to delete mappings.
 */
export interface DeleteMappingsRequest {
    /**
     * The configuration ID of the mappings being deleted.
     */
    configuration_id: string;
    /**
     * The mappings to be deleted. If this is `None`, then all mappings
     * with the corresponding configuration ID will be deleted.
     */
    mappings?: NodeMappingElement[] | null;
    [property: string]: unknown;
}

/**
 * The mapping of an individual node.
 */
export interface NodeMappingElement {
    /**
     * A string identifying an individual board, corresponding to the hostname
     * sans ".local".
     */
    board_id: string;
    /**
     * The calibrated offset of the sensor.
     * This is only used for sensors with channel type PT.
     */
    calibrated_offset?: number;
    /**
     * A number identifying which channel on the SAM board controls the node.
     */
    channel: number;
    /**
     * Which computer controls the SAM board, "flight" or "ground".
     */
    computer: Computer;
    /**
     * The maximum value reading of the sensor.
     * This is only used for sensors with channel type CurrentLoop or
     * DifferentialSignal.
     */
    max?: number | null;
    /**
     * The minimum value reading of the sensor.
     * This is only used for sensors with channel type CurrentLoop or
     * DifferentialSignal.
     */
    min?: number | null;
    /**
     * Indicator of whether the valve is normally open or normally closed.
     */
    normally_closed?: boolean | null;
    /**
     * The threshold, in Amps, at which the valve is considered powered.
     */
    powered_threshold?: number | null;
    /**
     * The channel type of the node, such as "valve".
     */
    sensor_type: SensorType;
    /**
     * The text identifier, or name, of the node.
     */
    text_id: string;
    [property: string]: unknown;
}

/**
 * Which computer controls the SAM board, "flight" or "ground".
 *
 * Used in a `NodeMapping` to determine which computer the action should be
 * sent to.
 *
 * The flight computer
 *
 * The ground computer
 */
export type Computer = "flight" | "ground";

/**
 * The channel type of the node, such as "valve".
 *
 * Represents all possible sensor types that may be used in a `NodeMapping`.
 *
 * Load cell, measuring force.
 *
 * Pressure transducer, which measures the pressure of a fluid.
 *
 * Current of the power tail of the board.
 *
 * Voltage of the power rail of the board.
 *
 * Resistance thermometer, measuring temperature.
 *
 * Thermocouple, measuring temperature.
 *
 * Valve, which can be actuated and read with voltage and current.
 */
export type SensorType = "load_cell" | "pt" | "rail_current" | "rail_voltage" | "rtd" | "tc" | "valve";

/**
 * Request struct to delete a sequence from the database.
 */
export interface DeleteSequenceRequest {
    /**
     * The name stored in the database identifying the sequence to be deleted.
     */
    name: string;
    [property: string]: unknown;
}

/**
 * Request struct used to delete a trigger from the database and make it
 * inactive on the flight computer.
 */
export interface DeleteTriggerRequest {
    /**
     * The name of the trigger to be deleted.
     */
    name: string;
    [property: string]: unknown;
}

export interface ExecuteSQLRequest {
    raw_sql: string;
    [property: string]: unknown;
}

export interface ExecuteSQLResponse {
    column_names: string[];
    rows:         Array<unknown[]>;
    [property: string]: unknown;
}

/**
 * Request struct for export requests.
 */
export interface ExportRequest {
    format: string;
    from:   number;
    to:     number;
    [property: string]: unknown;
}

/**
 * Request struct for changing the arming state of the launch lugs.
 */
export interface LugArm {
    armed: boolean;
    [property: string]: unknown;
}

/**
 * Request struct for detonating the launch lugs.
 */
export interface LugDetonate {
    enabled: boolean;
    [property: string]: unknown;
}

/**
 * Request struct containing all necessary information to execute a command.
 */
export interface OperatorCommandRequest {
    command: string;
    state?:  null | string;
    target?: null | string;
    [property: string]: unknown;
}

/**
 * Payload body for RECO GUI parameter commands.
 *
 * Send process noise matrices to RECO.
 *
 * Send measurement noise values to RECO.
 *
 * Send an initial EKF state vector to RECO.
 *
 * Send an initial covariance matrix to RECO.
 *
 * Send timer values to RECO.
 *
 * Send FMF parameters to RECO.
 */
export interface RecoGUICommandPayload {
    message_type: MessageType;
    payload:      Payload;
    [property: string]: unknown;
}

export type MessageType = "process_noise_matrix" | "measurement_noise_matrix" | "ekf_state_vector" | "initial_covariance_matrix" | "timer_values" | "altimeter_offsets";

/**
 * EKF process noise matrix (12x12 represented as four 3x3 submatrices).
 *
 * Measurement noise matrix sent to RECO for EKF updates.
 *
 * Initial EKF state vector sent to RECO.
 *
 * Initial covariance diagonal entries sent to RECO.
 *
 * Timer configuration sent to RECO.
 *
 * Altimeter offset configuration sent to RECO.
 */
export interface Payload {
    /**
     * Accelerometer bias covariance matrix (row-major 3x3)
     */
    nu_au_mat?: [number, number, number, number, number, number, number, number, number, ...number[]];
    /**
     * Accelerometer covariance matrix (row-major 3x3)
     */
    nu_av_mat?: [number, number, number, number, number, number, number, number, number, ...number[]];
    /**
     * Gyro bias covariance matrix (row-major 3x3)
     */
    nu_gu_mat?: [number, number, number, number, number, number, number, number, number, ...number[]];
    /**
     * Gyro covariance matrix (row-major 3x3)
     */
    nu_gv_mat?: [number, number, number, number, number, number, number, number, number, ...number[]];
    /**
     * Barometer noise term
     */
    barometer_noise?: number;
    /**
     * GPS noise matrix (row-major 3x3)
     */
    gps_noise_matrix?: [number, number, number, number, number, number, number, number, number, ...number[]];
    /**
     * Accelerometer bias offset
     */
    a_bias?: [number, number, number, ...number[]];
    /**
     * Acceleration scale factor
     */
    a_sf?: [number, number, number, ...number[]];
    /**
     * Gyroscope bias offset
     */
    g_bias?: [number, number, number, ...number[]];
    /**
     * Gyro scale factor
     */
    g_sf?: [number, number, number, ...number[]];
    /**
     * Position of vehicle in longitude, latitude, altitude frame
     */
    lla_pos?: [number, number, number, ...number[]];
    /**
     * Attitude of vehicle (quaternion)
     */
    quaternion?: [number, number, number, number, ...number[]];
    /**
     * Velocity of vehicle
     */
    velocity?: [number, number, number, ...number[]];
    /**
     * Uncertainty (variance) in accelerometer bias
     */
    abias_unc0?: [number, number, number, ...number[]];
    /**
     * Uncertainty (variance) in acceleration scale factor
     */
    asf_unc0?: [number, number, number, ...number[]];
    /**
     * Uncertainty (variance) in attitude
     */
    att_unc0?: [number, number, number, ...number[]];
    /**
     * Uncertainty (variance) in gyro bias
     */
    gbias_unc0?: [number, number, number, ...number[]];
    /**
     * Uncertainty (variance) in gyro scale factor
     */
    gsf_unc0?: [number, number, number, ...number[]];
    /**
     * Uncertainty (variance) in position
     */
    pos_unc0?: [number, number, number, ...number[]];
    /**
     * Uncertainty (variance) in velocity
     */
    vel_unc0?: [number, number, number, ...number[]];
    /**
     * Number of seconds after launch to deploy drogue
     */
    drouge_timer?: number;
    /**
     * If true, use timer instead of EKF for drogue
     */
    drouge_timer_enable?: number;
    /**
     * Number of seconds after launch to deploy main
     */
    main_timer?: number;
    /**
     * If true, use timer instead of altimeter for main
     */
    main_timer_enable?: number;
    /**
     * EKF lockout time in milliseconds.
     */
    ekf_lockout_time?: number;
    /**
     * Flight barometer FMF parameter.
     */
    flight_baro_fmf_parameter?: number;
    /**
     * Flight GPS FMF parameter.
     */
    flight_gps_fmf_parameter?: number;
    /**
     * Ground barometer FMF parameter.
     */
    ground_baro_fmf_parameter?: number;
    /**
     * Ground GPS FMF parameter.
     */
    ground_gps_fmf_parameter?: number;
    /**
     * Altitude offset.
     */
    h_offset_alt?: number;
    /**
     * Altitude offset filter parameter.
     */
    h_offset_filter?: number;
    [property: string]: unknown;
}

/**
 * Request body for RECO GUI parameter commands.
 *
 * Send process noise matrices to RECO.
 *
 * Send measurement noise values to RECO.
 *
 * Send an initial EKF state vector to RECO.
 *
 * Send an initial covariance matrix to RECO.
 *
 * Send timer values to RECO.
 *
 * Send FMF parameters to RECO.
 */
export interface RecoGUICommandRequest {
    /**
     * The RECO MCU that should receive the command. We use
     * #[serde(default)] to make the target field optional, and if no target is
     * specified `target` defaults to the specified default value for this
     * type, which is `TargetMCU::All`.
     */
    target?:      Target;
    message_type: MessageType;
    payload:      Payload;
    [property: string]: unknown;
}

/**
 * The RECO MCU that should receive the command. We use
 * #[serde(default)] to make the target field optional, and if no target is
 * specified `target` defaults to the specified default value for this
 * type, which is `TargetMCU::All`.
 *
 * Selects which RECO MCU should receive a GUI parameter command.
 *
 * Sends the command to all RECO MCUs. This is the default if no target
 * is specified.
 *
 * Sends the command to RECO MCU A.
 *
 * Sends the command to RECO MCU B.
 *
 * Sends the command to RECO MCU C.
 */
export type Target = "all" | "a" | "b" | "c";

/**
 * Response struct for getting the sequences stored in the database.
 */
export interface RetrieveAbortConfigsResponse {
    /**
     * The collection of all sequences present on the control server.
     */
    stages: AbortStageConfig[];
    [property: string]: unknown;
}

/**
 * Response struct for getting the sequences stored in the database.
 */
export interface RetrieveSequenceResponse {
    /**
     * The collection of all sequences present on the control server.
     */
    sequences: SequenceWithConfigurationElement[];
    [property: string]: unknown;
}

/**
 * Used in sequences response struct to attach the configuration ID.
 */
export interface SequenceWithConfigurationElement {
    /**
     * The ID of the configuration associated with the sequence.
     */
    configuration_id?: null | string;
    /**
     * The name of the sequence.
     */
    name: string;
    /**
     * The Python sequence script.
     */
    script: string;
    [property: string]: unknown;
}

/**
 * Request struct for running a sequence on the flight computer.
 */
export interface RunSequenceRequest {
    /**
     * Force the sequence to be executed, even if the configuration IDs do not
     * match.
     */
    force?: boolean | null;
    /**
     * The name of the sequence to run, as recorded in the database.
     */
    name: string;
    [property: string]: unknown;
}

/**
 * Request struct for saving a sequence without running it.
 */
export interface SaveSequenceRequest {
    /**
     * The ID of the associated configuration (provides extra check).
     */
    configuration_id?: null | string;
    /**
     * The unqiue name of the sequence that identifies it.
     */
    name: string;
    /**
     * The Base64-encoded script to save.
     */
    script: string;
    [property: string]: unknown;
}

/**
 * Guess
 */
export interface SetAbortConfig {
    stage_name: string;
    [property: string]: unknown;
}

/**
 * Request struct for setting a mapping.
 */
export interface SetMappingsRequest {
    /**
     * An ID uniquely identifying the configuration being set or modified
     */
    configuration_id: string;
    /**
     * Array of all mappings in no specific order
     */
    mappings: NodeMappingElement[];
    [property: string]: unknown;
}

/**
 * Request struct for stopping a sequence.
 */
export interface StopSequenceRequest {
    /**
     * Name of the sequence to be stopped.
     */
    name: string;
    [property: string]: unknown;
}

/**
 * Represents a query for the source of telemetry data.
 */
export interface TelemetrySourceQuery {
    source?: null | string;
    [property: string]: unknown;
}

/**
 * The GUI-visible transport stats for one telemetry source.
 */
export interface TelemetrySourceStats {
    packet_size_bytes?:    number | null;
    time_since_update_ms?: number | null;
    update_rate_hz?:       number | null;
    [property: string]: unknown;
}

/**
 * Transport stats for all live telemetry sources.
 */
export interface TelemetryStatsResponse {
    tel:       TelemetrySourceStats;
    umbilical: TelemetrySourceStats;
    [property: string]: unknown;
}

/**
 * A trigger with a
 */
export interface Trigger {
    /**
     * Whether or not the trigger is active
     */
    active: boolean;
    /**
     * The condition upon which the trigger script is run, written in Python.
     */
    condition: string;
    /**
     * The unique, human-readable name which identifies the trigger.
     */
    name: string;
    /**
     * The script run when the condition is met, written in Python.
     */
    script: string;
    [property: string]: unknown;
}

/**
 * Represents a vector
 *
 * Accelerometer (x, y, z) data
 *
 * Gyroscope (rx, ry, rz) data
 *
 * Magnetometer data
 */
export interface Vector {
    /**
     * X value
     */
    x: number;
    /**
     * Y value
     */
    y: number;
    /**
     * Z value
     */
    z: number;
    [property: string]: unknown;
}

/**
 * Holds the state of the SAMs and valves using `HashMap`s which convert a
 * node's name to its state.
 */
export interface VehicleState {
    /**
     * Defines the current abort stage that we are in
     */
    abort_stage: AbortStage;
    /**
     * Holds the state of every device on BMS
     */
    bms: Bms;
    /**
     * Holds the state of the flight computer board's sensors
     */
    fc_sensors: FcSensors;
    /**
     * Latest GPS state sample, if any.
     */
    gps?: null | GpsObject;
    /**
     * Whether the current `gps` sample is fresh for this control-loop
     * iteration. The flight computer should set this to `true` when it
     * ingests a new GPS sample, and set it to `false` immediately after
     * sending telemetry to the server.
     */
    gps_valid: boolean;
    /**
     * Aggregated RBF information for BMS, RECO, and SAM boards.
     */
    rbf: Rbf;
    /**
     * Latest RECO state samples from all three MCUs, if any.
     * Index 0: MCU A (spidev1.2)
     * Index 1: MCU B (spidev1.1)
     * Index 2: MCU C (spidev1.0)
     */
    reco: [null | RecoObject, null | RecoObject, null | RecoObject, ...(null | RecoObject)[]];
    /**
     * Whether the current `reco` samples are fresh for this control-loop
     * iteration. The flight computer should set this to `true` when it
     * ingests new RECO samples, and set it to `false` immediately after
     * sending telemetry to the server.
     */
    reco_valid: boolean;
    /**
     * Holds a HashMap from Board ID to a 2-tuple of the Rolling Average of
     * obtaining a data packet from the Board ID and the duration between the
     * last recieved and second-to-last recieved packet of the Board ID.
     */
    rolling: { [key: string]: RollingValue };
    /**
     * Holds the latest readings of all sensors on the vehicle.
     */
    sensor_readings: { [key: string]: SensorReadingValue };
    /**
     * Holds the actual and commanded states of all valves on the vehicle.
     */
    valve_states: { [key: string]: ValveStateValue };
    [property: string]: unknown;
}

/**
 * Defines the current abort stage that we are in
 *
 * Represents a single abort stage via its name, a condition that causes an
 * abort in this stage, and valve "safe" states that valves will go to in an
 * abort.
 */
export interface AbortStage {
    /**
     * Condition that, if met, we abort.
     * Can use the eval() in python to run strings as code
     */
    abort_condition: string;
    /**
     * Whether we have aborted in this stage yet
     */
    aborted: boolean;
    /**
     * Name of the abort stage
     */
    name: string;
    /**
     * "Safe" valve states we want boards to go if an abort occurs
     */
    valve_safe_states: { [key: string]: ValveSafeStateElement[] };
    [property: string]: unknown;
}

/**
 * Specifies what a valve should do
 */
export interface ValveSafeStateElement {
    /**
     * channel number that this type is talking about
     */
    channel_num: number;
    /**
     * whether we want to be powered or unpowered
     */
    powered: boolean;
    /**
     * amount of time we want to wait until we actuate a valve into its abort
     * safe state. ie. if timer = 10 secs for OMV, on an abort OMV will go to
     * its abort safe state 10 secs after the board enters an abort state
     */
    timer: Timer;
    [property: string]: unknown;
}

/**
 * amount of time we want to wait until we actuate a valve into its abort
 * safe state. ie. if timer = 10 secs for OMV, on an abort OMV will go to
 * its abort safe state 10 secs after the board enters an abort state
 *
 * The difference in time between the last and second-to-last recieved
 * packet.
 *
 * A rolling average of some board's data rate.
 */
export interface Timer {
    nanos: number;
    secs:  number;
    [property: string]: unknown;
}

/**
 * Holds the state of every device on BMS
 *
 * Represents the state of BMS as a whole
 */
export interface Bms {
    /**
     * battery bus data
     */
    battery_bus: BatteryBus;
    /**
     * charger data
     */
    charger: number;
    /**
     * chassis data
     */
    chassis: number;
    /**
     * estop data
     */
    e_stop: number;
    /**
     * ethernet load switch bus data
     */
    ethernet_bus: BatteryBus;
    /**
     * flight computer board bus data
     */
    fcb_bus: BatteryBus;
    /**
     * 5v rail data
     */
    five_volt_rail: BatteryBus;
    /**
     * rbf tag data
     */
    rbf_tag: number;
    /**
     * reco load switch 1 data
     */
    reco_load_switch_1: number;
    /**
     * reco load switch 2 data
     */
    reco_load_switch_2: number;
    /**
     * sam power bus data
     */
    sam_power_bus: BatteryBus;
    /**
     * tel load switch bus data
     */
    tel_bus: BatteryBus;
    /**
     * umbilical bus data
     */
    umbilical_bus: BatteryBus;
    [property: string]: unknown;
}

/**
 * battery bus data
 *
 * Describes the state of some power bus
 *
 * ethernet load switch bus data
 *
 * flight computer board bus data
 *
 * 5v rail data
 *
 * sam power bus data
 *
 * tel load switch bus data
 *
 * umbilical bus data
 *
 * 3V3 rail
 *
 * 5V rail
 */
export interface BatteryBus {
    /**
     * current data
     */
    current: number;
    /**
     * voltage data
     */
    voltage: number;
    [property: string]: unknown;
}

/**
 * Holds the state of the flight computer board's sensors
 *
 * Represents the state of the flight computer's onboard sensors.
 */
export interface FcSensors {
    /**
     * ADC data
     */
    adc: ADC;
    /**
     * Barometer data
     */
    barometer: Barometer;
    /**
     * IMU data
     */
    imu: Imu;
    /**
     * Magnetometer data
     */
    magnetometer: Vector;
    [property: string]: unknown;
}

/**
 * ADC data
 *
 * Represents ADC data sampled on the flight computer.
 */
export interface ADC {
    /**
     * Current-loop PT reading before mapping conversion.
     */
    current_loop_pt: number;
    /**
     * 3V3 rail
     */
    rail_3v3: BatteryBus;
    /**
     * 5V rail
     */
    rail_5v: BatteryBus;
    [property: string]: unknown;
}

/**
 * Barometer data
 *
 * Represents the state of the Barometer
 */
export interface Barometer {
    /**
     * Pressure data
     */
    pressure: number;
    /**
     * Temperature data
     */
    temperature: number;
    [property: string]: unknown;
}

/**
 * IMU data
 *
 * Represents the state of the IMU
 */
export interface Imu {
    /**
     * Accelerometer (x, y, z) data
     */
    accelerometer: Vector;
    /**
     * Gyroscope (rx, ry, rz) data
     */
    gyroscope: Vector;
    [property: string]: unknown;
}

/**
 * GPS state as seen by the flight computer.
 *
 * This is intentionally independent of any particular GPS driver so that it's
 * stable for serialization and logging.
 */
export interface GpsObject {
    /**
     * Ellipsoidal altitude above mean sea level, in meters.
     */
    altitude_m: number;
    /**
     * Down component of velocity (m/s) in NED frame.
     */
    down_mps: number;
    /**
     * East component of velocity (m/s) in NED frame.
     */
    east_mps: number;
    /**
     * Whether this sample corresponds to a valid GNSS fix.
     */
    has_fix: boolean;
    /**
     * Latitude in degrees (WGS84), positive north.
     */
    latitude_deg: number;
    /**
     * Longitude in degrees (WGS84), positive east.
     */
    longitude_deg: number;
    /**
     * North component of velocity (m/s) in NED frame.
     */
    north_mps: number;
    /**
     * Number of satellites used in the fix
     */
    num_satellites: number;
    /**
     * Unix timestamp in milliseconds for this fix, if available.
     */
    timestamp_unix_ms?: number | null;
    [property: string]: unknown;
}

/**
 * Aggregated RBF information for BMS, RECO, and SAM boards.
 *
 * Aggregated RBF status exposed to downstream telemetry consumers.
 */
export interface Rbf {
    /**
     * Latest BMS RBF reading, if any.
     */
    bms: number;
    /**
     * RECO RBF reading for each MCU
     */
    reco: [number, number, number, ...number[]];
    /**
     * RBF state for each SAM board (board_id, rbf_value)
     */
    sam: { [key: string]: number };
    [property: string]: unknown;
}

/**
 * RECO state as seen by the flight computer.
 *
 * This is intentionally independent of any particular RECO driver so that it's
 * stable for serialization and logging.
 */
export interface RecoObject {
    /**
     * Accelerometer bias offset [x, y, z]
     */
    a_bias: [number, number, number, ...number[]];
    /**
     * Acceleration scale factor [x, y, z]
     */
    a_sf: [number, number, number, ...number[]];
    /**
     * Angular rates (pitch, yaw, roll) in rad/s
     */
    angular_rate: [number, number, number, ...number[]];
    /**
     * Use timer instead of EKF for drogue
     */
    drouge_timer_enable: boolean;
    /**
     * EKF has blown up flag
     */
    ekf_blown_up: boolean;
    /**
     * Barometer value from fading memory filter
     */
    fading_memory_baro: number;
    /**
     * GPS value from fading memory filter
     */
    fading_memory_gps: number;
    /**
     * Gyroscope bias offset [x, y, z]
     */
    g_bias: [number, number, number, ...number[]];
    /**
     * Gyro scale factor [x, y, z]
     */
    g_sf: [number, number, number, ...number[]];
    /**
     * Linear acceleration [x, y, z] in m/s²
     */
    lin_accel: [number, number, number, ...number[]];
    /**
     * Position [longitude, latitude, altitude] in degrees and meters
     */
    lla_pos: [number, number, number, ...number[]];
    /**
     * Magnetometer data [x, y, z]
     */
    mag_data: [number, number, number, ...number[]];
    /**
     * Use timer instead of altimeter for main
     */
    main_timer_enable: boolean;
    /**
     * Pressure in Pa
     */
    pressure: number;
    /**
     * Quaternion representing vehicle attitude [w, x, y, z]
     */
    quaternion: [number, number, number, number, ...number[]];
    /**
     * Whether RBF is installed
     */
    rbf_enabled: boolean;
    /**
     * Fault status bytes for RECO drivers/channels
     */
    reco_driver_faults: [number, number, number, number, number, number, number, number, number, number, ...number[]];
    /**
     * Whether RECO has received the launch command
     */
    reco_recvd_launch: boolean;
    /**
     * Recovery Driver 1 current
     */
    sns1_current: number;
    /**
     * Recovery Driver 2 current
     */
    sns2_current: number;
    /**
     * Stage 1 enabled flag
     */
    stage1_enabled: boolean;
    /**
     * Stage 2 enabled flag
     */
    stage2_enabled: boolean;
    /**
     * Temperature in Kelvin
     */
    temperature: number;
    /**
     * 24V rail voltage
     */
    v_rail_24v: number;
    /**
     * 3.3V rail voltage
     */
    v_rail_3v3: number;
    /**
     * Velocity of vehicle [north, east, down] in m/s
     */
    velocity: [number, number, number, ...number[]];
    /**
     * Channel 1 Driver 1 voltage
     */
    vref_ch1_dr1: number;
    /**
     * Channel 1 Driver 2 voltage
     */
    vref_ch1_dr2: number;
    /**
     * Channel 2 Driver 1 voltage
     */
    vref_ch2_dr1: number;
    /**
     * Channel 2 Driver 2 voltage
     */
    vref_ch2_dr2: number;
    [property: string]: unknown;
}

/**
 * Used by the Flight Computer for debugging data rates.
 */
export interface RollingValue {
    /**
     * The difference in time between the last and second-to-last recieved
     * packet.
     */
    delta_time: Timer;
    /**
     * A rolling average of some board's data rate.
     */
    rolling_average: Timer;
    /**
     * time since last update in seconds
     */
    time_since_last_update: number;
    [property: string]: unknown;
}

/**
 * Holds a single measurement for either a sensor or valve.
 *
 * This enum simply wraps two other types, `SensorMeasurement` and
 * `ValveMeasurement`. The reason to keep this in separate structs instead of
 * properties of the variants is that these values often need to passed around
 * independently in flight code, and enum variant properties are not mutable
 * without reconstructing the variant. This is annoying. Essentially, this
 * looks like bad / less readable code but is necessary, and convenience
 * constructs are provided to make code cleaner.
 */
export interface SensorReadingValue {
    /**
     * The unit associated with the measurement.
     */
    unit: Unit;
    /**
     * The raw value associated with the measurement.
     */
    value: number;
    [property: string]: unknown;
}

/**
 * The unit associated with the measurement.
 *
 * Every unit needed to be passed around in communications, mainly for sensor
 * readings.
 *
 * Unknown or not-yet-restored unit metadata.
 *
 * Current, in amperes.
 *
 * Pressure, in pounds per square inch.
 *
 * Temperature, in Kelvin.
 *
 * Force, in pounds.
 *
 * Electric potential, in volts.
 */
export type Unit = "unknown" | "amps" | "psi" | "kelvin" | "pounds" | "volts";

/**
 * Stores the estimated actual valve state as well as the software-commanded
 * state.
 */
export interface ValveStateValue {
    /**
     * Actual state of the valve, determined using voltage and current
     * measurements.
     */
    actual: DesiredState;
    /**
     * Commanded state of the valve, according to software.
     */
    commanded: DesiredState;
    [property: string]: unknown;
}

// Converts JSON strings to/from your types
// and asserts the results of JSON.parse at runtime
export class Convert {
    public static toTypes(json: string): Types {
        return cast(JSON.parse(json), r("Types"));
    }

    public static typesToJson(value: Types): string {
        return JSON.stringify(uncast(value, r("Types")), null, 2);
    }
}

function invalidValue(typ: any, val: any, key: any, parent: any = ''): never {
    const prettyTyp = prettyTypeName(typ);
    const parentText = parent ? ` on ${parent}` : '';
    const keyText = key ? ` for key "${key}"` : '';
    throw Error(`Invalid value${keyText}${parentText}. Expected ${prettyTyp} but got ${JSON.stringify(val)}`);
}

function prettyTypeName(typ: any): string {
    if (Array.isArray(typ)) {
        if (typ.length === 2 && typ[0] === undefined) {
            return `an optional ${prettyTypeName(typ[1])}`;
        } else {
            return `one of [${typ.map(a => { return prettyTypeName(a); }).join(", ")}]`;
        }
    } else if (typeof typ === "object" && typ.literal !== undefined) {
        return typ.literal;
    } else {
        return typeof typ;
    }
}

function jsonToJSProps(typ: any): any {
    if (typ.jsonToJS === undefined) {
        const map: any = {};
        typ.props.forEach((p: any) => map[p.json] = { key: p.js, typ: p.typ });
        typ.jsonToJS = map;
    }
    return typ.jsonToJS;
}

function jsToJSONProps(typ: any): any {
    if (typ.jsToJSON === undefined) {
        const map: any = {};
        typ.props.forEach((p: any) => map[p.js] = { key: p.json, typ: p.typ });
        typ.jsToJSON = map;
    }
    return typ.jsToJSON;
}

function transform(val: any, typ: any, getProps: any, key: any = '', parent: any = ''): any {
    function transformPrimitive(typ: string, val: any): any {
        if (typeof typ === typeof val) return val;
        return invalidValue(typ, val, key, parent);
    }

    function transformUnion(typs: any[], val: any): any {
        // val must validate against one typ in typs
        const l = typs.length;
        for (let i = 0; i < l; i++) {
            const typ = typs[i];
            try {
                return transform(val, typ, getProps);
            } catch (_) {}
        }
        return invalidValue(typs, val, key, parent);
    }

    function transformEnum(cases: string[], val: any): any {
        if (cases.indexOf(val) !== -1) return val;
        return invalidValue(cases.map(a => { return l(a); }), val, key, parent);
    }

    function transformArray(typ: any, val: any): any {
        // val must be an array with no invalid elements
        if (!Array.isArray(val)) return invalidValue(l("array"), val, key, parent);
        return val.map(el => transform(el, typ, getProps));
    }

    function transformDate(val: any): any {
        if (val === null) {
            return null;
        }
        const d = new Date(val);
        if (isNaN(d.valueOf())) {
            return invalidValue(l("Date"), val, key, parent);
        }
        return d;
    }

    function transformObject(props: { [k: string]: any }, additional: any, val: any): any {
        if (val === null || typeof val !== "object" || Array.isArray(val)) {
            return invalidValue(l(ref || "object"), val, key, parent);
        }
        const result: any = {};
        Object.getOwnPropertyNames(props).forEach(key => {
            const prop = props[key];
            const v = Object.prototype.hasOwnProperty.call(val, key) ? val[key] : undefined;
            result[prop.key] = transform(v, prop.typ, getProps, key, ref);
        });
        Object.getOwnPropertyNames(val).forEach(key => {
            if (!Object.prototype.hasOwnProperty.call(props, key)) {
                result[key] = transform(val[key], additional, getProps, key, ref);
            }
        });
        return result;
    }

    if (typ === "any") return val;
    if (typ === null) {
        if (val === null) return val;
        return invalidValue(typ, val, key, parent);
    }
    if (typ === false) return invalidValue(typ, val, key, parent);
    let ref: any = undefined;
    while (typeof typ === "object" && typ.ref !== undefined) {
        ref = typ.ref;
        typ = typeMap[typ.ref];
    }
    if (Array.isArray(typ)) return transformEnum(typ, val);
    if (typeof typ === "object") {
        return typ.hasOwnProperty("unionMembers") ? transformUnion(typ.unionMembers, val)
            : typ.hasOwnProperty("arrayItems")    ? transformArray(typ.arrayItems, val)
            : typ.hasOwnProperty("props")         ? transformObject(getProps(typ), typ.additional, val)
            : invalidValue(typ, val, key, parent);
    }
    // Numbers can be parsed by Date but shouldn't be.
    if (typ === Date && typeof val !== "number") return transformDate(val);
    return transformPrimitive(typ, val);
}

function cast<T>(val: any, typ: any): T {
    return transform(val, typ, jsonToJSProps);
}

function uncast<T>(val: T, typ: any): any {
    return transform(val, typ, jsToJSONProps);
}

function l(typ: any) {
    return { literal: typ };
}

function a(typ: any) {
    return { arrayItems: typ };
}

function u(...typs: any[]) {
    return { unionMembers: typs };
}

function o(props: any[], additional: any) {
    return { props, additional };
}

function m(additional: any) {
    return { props: [], additional };
}

function r(name: string) {
    return { ref: name };
}

const typeMap: any = {
    "Types": o([
        { json: "abort_stage_config", js: "abort_stage_config", typ: r("AbortStageConfig") },
        { json: "active_configuration", js: "active_configuration", typ: r("ActiveConfiguration") },
        { json: "camera_enable", js: "camera_enable", typ: r("CameraEnable") },
        { json: "delete_abort_config_request", js: "delete_abort_config_request", typ: r("DeleteAbortConfigRequest") },
        { json: "delete_mappings_request", js: "delete_mappings_request", typ: r("DeleteMappingsRequest") },
        { json: "delete_sequence_request", js: "delete_sequence_request", typ: r("DeleteSequenceRequest") },
        { json: "delete_trigger_request", js: "delete_trigger_request", typ: r("DeleteTriggerRequest") },
        { json: "execute_sql_request", js: "execute_sql_request", typ: r("ExecuteSQLRequest") },
        { json: "execute_sql_response", js: "execute_sql_response", typ: r("ExecuteSQLResponse") },
        { json: "export_request", js: "export_request", typ: r("ExportRequest") },
        { json: "lug_arm", js: "lug_arm", typ: r("LugArm") },
        { json: "lug_detonate", js: "lug_detonate", typ: r("LugDetonate") },
        { json: "node_mapping", js: "node_mapping", typ: r("NodeMappingElement") },
        { json: "operator_command_request", js: "operator_command_request", typ: r("OperatorCommandRequest") },
        { json: "reco_gui_command_payload", js: "reco_gui_command_payload", typ: r("RecoGUICommandPayload") },
        { json: "reco_gui_command_request", js: "reco_gui_command_request", typ: r("RecoGUICommandRequest") },
        { json: "retrieve_abort_configs_response", js: "retrieve_abort_configs_response", typ: r("RetrieveAbortConfigsResponse") },
        { json: "retrieve_sequence_response", js: "retrieve_sequence_response", typ: r("RetrieveSequenceResponse") },
        { json: "run_sequence_request", js: "run_sequence_request", typ: r("RunSequenceRequest") },
        { json: "save_sequence_request", js: "save_sequence_request", typ: r("SaveSequenceRequest") },
        { json: "sequence_with_configuration", js: "sequence_with_configuration", typ: r("SequenceWithConfigurationElement") },
        { json: "set_abort_config", js: "set_abort_config", typ: r("SetAbortConfig") },
        { json: "set_mappings_request", js: "set_mappings_request", typ: r("SetMappingsRequest") },
        { json: "stop_sequence_request", js: "stop_sequence_request", typ: r("StopSequenceRequest") },
        { json: "telemetry_source_query", js: "telemetry_source_query", typ: r("TelemetrySourceQuery") },
        { json: "telemetry_source_stats", js: "telemetry_source_stats", typ: r("TelemetrySourceStats") },
        { json: "telemetry_stats_response", js: "telemetry_stats_response", typ: r("TelemetryStatsResponse") },
        { json: "trigger", js: "trigger", typ: r("Trigger") },
        { json: "vector", js: "vector", typ: r("Vector") },
        { json: "vehicle_state", js: "vehicle_state", typ: r("VehicleState") },
    ], "any"),
    "AbortStageConfig": o([
        { json: "abort_condition", js: "abort_condition", typ: "" },
        { json: "stage_name", js: "stage_name", typ: "" },
        { json: "valve_safe_states", js: "valve_safe_states", typ: m(r("ValveSafeStateValue")) },
    ], "any"),
    "ValveSafeStateValue": o([
        { json: "desired_state", js: "desired_state", typ: r("DesiredState") },
        { json: "safing_timer", js: "safing_timer", typ: 0 },
    ], "any"),
    "ActiveConfiguration": o([
        { json: "configuration_id", js: "configuration_id", typ: "" },
    ], "any"),
    "CameraEnable": o([
        { json: "enabled", js: "enabled", typ: true },
    ], "any"),
    "DeleteAbortConfigRequest": o([
        { json: "name", js: "name", typ: "" },
    ], "any"),
    "DeleteMappingsRequest": o([
        { json: "configuration_id", js: "configuration_id", typ: "" },
        { json: "mappings", js: "mappings", typ: u(undefined, u(a(r("NodeMappingElement")), null)) },
    ], "any"),
    "NodeMappingElement": o([
        { json: "board_id", js: "board_id", typ: "" },
        { json: "calibrated_offset", js: "calibrated_offset", typ: u(undefined, 3.14) },
        { json: "channel", js: "channel", typ: 0 },
        { json: "computer", js: "computer", typ: r("Computer") },
        { json: "max", js: "max", typ: u(undefined, u(3.14, null)) },
        { json: "min", js: "min", typ: u(undefined, u(3.14, null)) },
        { json: "normally_closed", js: "normally_closed", typ: u(undefined, u(true, null)) },
        { json: "powered_threshold", js: "powered_threshold", typ: u(undefined, u(3.14, null)) },
        { json: "sensor_type", js: "sensor_type", typ: r("SensorType") },
        { json: "text_id", js: "text_id", typ: "" },
    ], "any"),
    "DeleteSequenceRequest": o([
        { json: "name", js: "name", typ: "" },
    ], "any"),
    "DeleteTriggerRequest": o([
        { json: "name", js: "name", typ: "" },
    ], "any"),
    "ExecuteSQLRequest": o([
        { json: "raw_sql", js: "raw_sql", typ: "" },
    ], "any"),
    "ExecuteSQLResponse": o([
        { json: "column_names", js: "column_names", typ: a("") },
        { json: "rows", js: "rows", typ: a(a("any")) },
    ], "any"),
    "ExportRequest": o([
        { json: "format", js: "format", typ: "" },
        { json: "from", js: "from", typ: 3.14 },
        { json: "to", js: "to", typ: 3.14 },
    ], "any"),
    "LugArm": o([
        { json: "armed", js: "armed", typ: true },
    ], "any"),
    "LugDetonate": o([
        { json: "enabled", js: "enabled", typ: true },
    ], "any"),
    "OperatorCommandRequest": o([
        { json: "command", js: "command", typ: "" },
        { json: "state", js: "state", typ: u(undefined, u(null, "")) },
        { json: "target", js: "target", typ: u(undefined, u(null, "")) },
    ], "any"),
    "RecoGUICommandPayload": o([
        { json: "message_type", js: "message_type", typ: r("MessageType") },
        { json: "payload", js: "payload", typ: r("Payload") },
    ], "any"),
    "Payload": o([
        { json: "nu_au_mat", js: "nu_au_mat", typ: u(undefined, a(3.14)) },
        { json: "nu_av_mat", js: "nu_av_mat", typ: u(undefined, a(3.14)) },
        { json: "nu_gu_mat", js: "nu_gu_mat", typ: u(undefined, a(3.14)) },
        { json: "nu_gv_mat", js: "nu_gv_mat", typ: u(undefined, a(3.14)) },
        { json: "barometer_noise", js: "barometer_noise", typ: u(undefined, 3.14) },
        { json: "gps_noise_matrix", js: "gps_noise_matrix", typ: u(undefined, a(3.14)) },
        { json: "a_bias", js: "a_bias", typ: u(undefined, a(3.14)) },
        { json: "a_sf", js: "a_sf", typ: u(undefined, a(3.14)) },
        { json: "g_bias", js: "g_bias", typ: u(undefined, a(3.14)) },
        { json: "g_sf", js: "g_sf", typ: u(undefined, a(3.14)) },
        { json: "lla_pos", js: "lla_pos", typ: u(undefined, a(3.14)) },
        { json: "quaternion", js: "quaternion", typ: u(undefined, a(3.14)) },
        { json: "velocity", js: "velocity", typ: u(undefined, a(3.14)) },
        { json: "abias_unc0", js: "abias_unc0", typ: u(undefined, a(3.14)) },
        { json: "asf_unc0", js: "asf_unc0", typ: u(undefined, a(3.14)) },
        { json: "att_unc0", js: "att_unc0", typ: u(undefined, a(3.14)) },
        { json: "gbias_unc0", js: "gbias_unc0", typ: u(undefined, a(3.14)) },
        { json: "gsf_unc0", js: "gsf_unc0", typ: u(undefined, a(3.14)) },
        { json: "pos_unc0", js: "pos_unc0", typ: u(undefined, a(3.14)) },
        { json: "vel_unc0", js: "vel_unc0", typ: u(undefined, a(3.14)) },
        { json: "drouge_timer", js: "drouge_timer", typ: u(undefined, 3.14) },
        { json: "drouge_timer_enable", js: "drouge_timer_enable", typ: u(undefined, 0) },
        { json: "main_timer", js: "main_timer", typ: u(undefined, 3.14) },
        { json: "main_timer_enable", js: "main_timer_enable", typ: u(undefined, 0) },
        { json: "ekf_lockout_time", js: "ekf_lockout_time", typ: u(undefined, 0) },
        { json: "flight_baro_fmf_parameter", js: "flight_baro_fmf_parameter", typ: u(undefined, 3.14) },
        { json: "flight_gps_fmf_parameter", js: "flight_gps_fmf_parameter", typ: u(undefined, 3.14) },
        { json: "ground_baro_fmf_parameter", js: "ground_baro_fmf_parameter", typ: u(undefined, 3.14) },
        { json: "ground_gps_fmf_parameter", js: "ground_gps_fmf_parameter", typ: u(undefined, 3.14) },
        { json: "h_offset_alt", js: "h_offset_alt", typ: u(undefined, 3.14) },
        { json: "h_offset_filter", js: "h_offset_filter", typ: u(undefined, 3.14) },
    ], "any"),
    "RecoGUICommandRequest": o([
        { json: "target", js: "target", typ: u(undefined, r("Target")) },
        { json: "message_type", js: "message_type", typ: r("MessageType") },
        { json: "payload", js: "payload", typ: r("Payload") },
    ], "any"),
    "RetrieveAbortConfigsResponse": o([
        { json: "stages", js: "stages", typ: a(r("AbortStageConfig")) },
    ], "any"),
    "RetrieveSequenceResponse": o([
        { json: "sequences", js: "sequences", typ: a(r("SequenceWithConfigurationElement")) },
    ], "any"),
    "SequenceWithConfigurationElement": o([
        { json: "configuration_id", js: "configuration_id", typ: u(undefined, u(null, "")) },
        { json: "name", js: "name", typ: "" },
        { json: "script", js: "script", typ: "" },
    ], "any"),
    "RunSequenceRequest": o([
        { json: "force", js: "force", typ: u(undefined, u(true, null)) },
        { json: "name", js: "name", typ: "" },
    ], "any"),
    "SaveSequenceRequest": o([
        { json: "configuration_id", js: "configuration_id", typ: u(undefined, u(null, "")) },
        { json: "name", js: "name", typ: "" },
        { json: "script", js: "script", typ: "" },
    ], "any"),
    "SetAbortConfig": o([
        { json: "stage_name", js: "stage_name", typ: "" },
    ], "any"),
    "SetMappingsRequest": o([
        { json: "configuration_id", js: "configuration_id", typ: "" },
        { json: "mappings", js: "mappings", typ: a(r("NodeMappingElement")) },
    ], "any"),
    "StopSequenceRequest": o([
        { json: "name", js: "name", typ: "" },
    ], "any"),
    "TelemetrySourceQuery": o([
        { json: "source", js: "source", typ: u(undefined, u(null, "")) },
    ], "any"),
    "TelemetrySourceStats": o([
        { json: "packet_size_bytes", js: "packet_size_bytes", typ: u(undefined, u(0, null)) },
        { json: "time_since_update_ms", js: "time_since_update_ms", typ: u(undefined, u(3.14, null)) },
        { json: "update_rate_hz", js: "update_rate_hz", typ: u(undefined, u(3.14, null)) },
    ], "any"),
    "TelemetryStatsResponse": o([
        { json: "tel", js: "tel", typ: r("TelemetrySourceStats") },
        { json: "umbilical", js: "umbilical", typ: r("TelemetrySourceStats") },
    ], "any"),
    "Trigger": o([
        { json: "active", js: "active", typ: true },
        { json: "condition", js: "condition", typ: "" },
        { json: "name", js: "name", typ: "" },
        { json: "script", js: "script", typ: "" },
    ], "any"),
    "Vector": o([
        { json: "x", js: "x", typ: 3.14 },
        { json: "y", js: "y", typ: 3.14 },
        { json: "z", js: "z", typ: 3.14 },
    ], "any"),
    "VehicleState": o([
        { json: "abort_stage", js: "abort_stage", typ: r("AbortStage") },
        { json: "bms", js: "bms", typ: r("Bms") },
        { json: "fc_sensors", js: "fc_sensors", typ: r("FcSensors") },
        { json: "gps", js: "gps", typ: u(undefined, u(null, r("GpsObject"))) },
        { json: "gps_valid", js: "gps_valid", typ: true },
        { json: "rbf", js: "rbf", typ: r("Rbf") },
        { json: "reco", js: "reco", typ: a(u(null, r("RecoObject"))) },
        { json: "reco_valid", js: "reco_valid", typ: true },
        { json: "rolling", js: "rolling", typ: m(r("RollingValue")) },
        { json: "sensor_readings", js: "sensor_readings", typ: m(r("SensorReadingValue")) },
        { json: "valve_states", js: "valve_states", typ: m(r("ValveStateValue")) },
    ], "any"),
    "AbortStage": o([
        { json: "abort_condition", js: "abort_condition", typ: "" },
        { json: "aborted", js: "aborted", typ: true },
        { json: "name", js: "name", typ: "" },
        { json: "valve_safe_states", js: "valve_safe_states", typ: m(a(r("ValveSafeStateElement"))) },
    ], "any"),
    "ValveSafeStateElement": o([
        { json: "channel_num", js: "channel_num", typ: 0 },
        { json: "powered", js: "powered", typ: true },
        { json: "timer", js: "timer", typ: r("Timer") },
    ], "any"),
    "Timer": o([
        { json: "nanos", js: "nanos", typ: 0 },
        { json: "secs", js: "secs", typ: 0 },
    ], "any"),
    "Bms": o([
        { json: "battery_bus", js: "battery_bus", typ: r("BatteryBus") },
        { json: "charger", js: "charger", typ: 3.14 },
        { json: "chassis", js: "chassis", typ: 3.14 },
        { json: "e_stop", js: "e_stop", typ: 3.14 },
        { json: "ethernet_bus", js: "ethernet_bus", typ: r("BatteryBus") },
        { json: "fcb_bus", js: "fcb_bus", typ: r("BatteryBus") },
        { json: "five_volt_rail", js: "five_volt_rail", typ: r("BatteryBus") },
        { json: "rbf_tag", js: "rbf_tag", typ: 3.14 },
        { json: "reco_load_switch_1", js: "reco_load_switch_1", typ: 3.14 },
        { json: "reco_load_switch_2", js: "reco_load_switch_2", typ: 3.14 },
        { json: "sam_power_bus", js: "sam_power_bus", typ: r("BatteryBus") },
        { json: "tel_bus", js: "tel_bus", typ: r("BatteryBus") },
        { json: "umbilical_bus", js: "umbilical_bus", typ: r("BatteryBus") },
    ], "any"),
    "BatteryBus": o([
        { json: "current", js: "current", typ: 3.14 },
        { json: "voltage", js: "voltage", typ: 3.14 },
    ], "any"),
    "FcSensors": o([
        { json: "adc", js: "adc", typ: r("ADC") },
        { json: "barometer", js: "barometer", typ: r("Barometer") },
        { json: "imu", js: "imu", typ: r("Imu") },
        { json: "magnetometer", js: "magnetometer", typ: r("Vector") },
    ], "any"),
    "ADC": o([
        { json: "current_loop_pt", js: "current_loop_pt", typ: 3.14 },
        { json: "rail_3v3", js: "rail_3v3", typ: r("BatteryBus") },
        { json: "rail_5v", js: "rail_5v", typ: r("BatteryBus") },
    ], "any"),
    "Barometer": o([
        { json: "pressure", js: "pressure", typ: 3.14 },
        { json: "temperature", js: "temperature", typ: 3.14 },
    ], "any"),
    "Imu": o([
        { json: "accelerometer", js: "accelerometer", typ: r("Vector") },
        { json: "gyroscope", js: "gyroscope", typ: r("Vector") },
    ], "any"),
    "GpsObject": o([
        { json: "altitude_m", js: "altitude_m", typ: 3.14 },
        { json: "down_mps", js: "down_mps", typ: 3.14 },
        { json: "east_mps", js: "east_mps", typ: 3.14 },
        { json: "has_fix", js: "has_fix", typ: true },
        { json: "latitude_deg", js: "latitude_deg", typ: 3.14 },
        { json: "longitude_deg", js: "longitude_deg", typ: 3.14 },
        { json: "north_mps", js: "north_mps", typ: 3.14 },
        { json: "num_satellites", js: "num_satellites", typ: 0 },
        { json: "timestamp_unix_ms", js: "timestamp_unix_ms", typ: u(undefined, u(0, null)) },
    ], "any"),
    "Rbf": o([
        { json: "bms", js: "bms", typ: 0 },
        { json: "reco", js: "reco", typ: a(0) },
        { json: "sam", js: "sam", typ: m(0) },
    ], "any"),
    "RecoObject": o([
        { json: "a_bias", js: "a_bias", typ: a(3.14) },
        { json: "a_sf", js: "a_sf", typ: a(3.14) },
        { json: "angular_rate", js: "angular_rate", typ: a(3.14) },
        { json: "drouge_timer_enable", js: "drouge_timer_enable", typ: true },
        { json: "ekf_blown_up", js: "ekf_blown_up", typ: true },
        { json: "fading_memory_baro", js: "fading_memory_baro", typ: 3.14 },
        { json: "fading_memory_gps", js: "fading_memory_gps", typ: 3.14 },
        { json: "g_bias", js: "g_bias", typ: a(3.14) },
        { json: "g_sf", js: "g_sf", typ: a(3.14) },
        { json: "lin_accel", js: "lin_accel", typ: a(3.14) },
        { json: "lla_pos", js: "lla_pos", typ: a(3.14) },
        { json: "mag_data", js: "mag_data", typ: a(3.14) },
        { json: "main_timer_enable", js: "main_timer_enable", typ: true },
        { json: "pressure", js: "pressure", typ: 3.14 },
        { json: "quaternion", js: "quaternion", typ: a(3.14) },
        { json: "rbf_enabled", js: "rbf_enabled", typ: true },
        { json: "reco_driver_faults", js: "reco_driver_faults", typ: a(0) },
        { json: "reco_recvd_launch", js: "reco_recvd_launch", typ: true },
        { json: "sns1_current", js: "sns1_current", typ: 3.14 },
        { json: "sns2_current", js: "sns2_current", typ: 3.14 },
        { json: "stage1_enabled", js: "stage1_enabled", typ: true },
        { json: "stage2_enabled", js: "stage2_enabled", typ: true },
        { json: "temperature", js: "temperature", typ: 3.14 },
        { json: "v_rail_24v", js: "v_rail_24v", typ: 3.14 },
        { json: "v_rail_3v3", js: "v_rail_3v3", typ: 3.14 },
        { json: "velocity", js: "velocity", typ: a(3.14) },
        { json: "vref_ch1_dr1", js: "vref_ch1_dr1", typ: 3.14 },
        { json: "vref_ch1_dr2", js: "vref_ch1_dr2", typ: 3.14 },
        { json: "vref_ch2_dr1", js: "vref_ch2_dr1", typ: 3.14 },
        { json: "vref_ch2_dr2", js: "vref_ch2_dr2", typ: 3.14 },
    ], "any"),
    "RollingValue": o([
        { json: "delta_time", js: "delta_time", typ: r("Timer") },
        { json: "rolling_average", js: "rolling_average", typ: r("Timer") },
        { json: "time_since_last_update", js: "time_since_last_update", typ: 3.14 },
    ], "any"),
    "SensorReadingValue": o([
        { json: "unit", js: "unit", typ: r("Unit") },
        { json: "value", js: "value", typ: 3.14 },
    ], "any"),
    "ValveStateValue": o([
        { json: "actual", js: "actual", typ: r("DesiredState") },
        { json: "commanded", js: "commanded", typ: r("DesiredState") },
    ], "any"),
    "DesiredState": [
        "closed",
        "disconnected",
        "fault",
        "open",
        "undetermined",
    ],
    "Computer": [
        "flight",
        "ground",
    ],
    "SensorType": [
        "load_cell",
        "pt",
        "rail_current",
        "rail_voltage",
        "rtd",
        "tc",
        "valve",
    ],
    "MessageType": [
        "altimeter_offsets",
        "ekf_state_vector",
        "initial_covariance_matrix",
        "measurement_noise_matrix",
        "process_noise_matrix",
        "timer_values",
    ],
    "Target": [
        "a",
        "all",
        "b",
        "c",
    ],
    "Unit": [
        "amps",
        "kelvin",
        "pounds",
        "psi",
        "unknown",
        "volts",
    ],
};
