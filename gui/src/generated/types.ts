// To parse this data:
//
//   import { Convert, Types } from "./types";
//
//   const types = Convert.toTypes(json);
//
// These functions will throw an error if the JSON doesn't
// match the expected interface, even if the JSON is valid.

export interface Types {
    $schema:    string;
    title:      string;
    type:       ValveSafeStatesType;
    properties: { [key: string]: Property };
    required:   string[];
    $defs:      Defs;
}

export interface Defs {
    AbortStage:                   AbortStage;
    AbortStageConfig:             AbortStageConfig;
    ActiveConfiguration:          ActiveConfiguration;
    AdcData:                      ADCData;
    AltimeterOffsets:             AltimeterOffsets;
    Barometer:                    Barometer;
    Bms:                          Bms;
    Bus:                          Bus;
    CameraEnable:                 CameraEnable;
    CompositeValveState:          CompositeValveState;
    Computer:                     Computer;
    DeleteAbortConfigRequest:     Request;
    DeleteMappingsRequest:        DeleteMappingsRequest;
    DeleteSequenceRequest:        Request;
    DeleteTriggerRequest:         Request;
    Duration:                     Duration;
    EkfStateVector:               EkfStateVector;
    ExecuteSqlRequest:            ExecuteSQLRequest;
    ExecuteSqlResponse:           ExecuteSQLResponse;
    ExportRequest:                ExportRequest;
    FcSensors:                    FcSensors;
    GpsState:                     GpsState;
    Imu:                          Imu;
    InitialCovarianceMatrix:      InitialCovarianceMatrix;
    LugArm:                       LugArm;
    LugDetonate:                  CameraEnable;
    Measurement:                  Measurement;
    MeasurementNoiseMatrix:       MeasurementNoiseMatrix;
    NodeMapping:                  NodeMapping;
    OperatorCommandRequest:       OperatorCommandRequest;
    ProcessNoiseMatrix:           ProcessNoiseMatrix;
    RbfState:                     RbfState;
    RecoGuiCommandPayload:        RecoGUICommandPayload;
    RecoGuiCommandRequest:        RecoGUICommandRequest;
    RecoState:                    RecoState;
    RecoTriState:                 RecoTriState;
    RetrieveAbortConfigsResponse: RetrieveAbortConfigsResponse;
    RetrieveSequenceResponse:     RetrieveSequenceResponse;
    RunSequenceRequest:           RunSequenceRequest;
    SaveSequenceRequest:          SaveSequenceRequest;
    SensorType:                   Computer;
    SequenceWithConfiguration:    SaveSequenceRequest;
    SetAbortConfig:               SetAbortConfig;
    SetMappingsRequest:           SetMappingsRequest;
    Statistics:                   Statistics;
    StopSequenceRequest:          Request;
    TargetMCU:                    Computer;
    TelemetrySourceQuery:         TelemetrySourceQuery;
    TelemetrySourceStats:         TelemetrySourceStats;
    TelemetryStatsResponse:       TelemetryStatsResponse;
    TimerValues:                  TimerValues;
    Trigger:                      Trigger;
    Unit:                         Computer;
    ValveAction:                  ValveAction;
    ValveSafeState:               ValveSafeState;
    ValveState:                   Computer;
    Vector:                       Vector;
    VehicleState:                 VehicleState;
}

export interface AbortStage {
    description: string;
    type:        ValveSafeStatesType;
    properties:  AbortStageProperties;
    required:    string[];
}

export interface AbortStageProperties {
    abort_condition:   AbortCondition;
    aborted:           AbortCondition;
    name:              AbortCondition;
    valve_safe_states: PurpleValveSafeStates;
}

export interface AbortCondition {
    description: string;
    type:        AbortConditionType;
}

export type AbortConditionType = "string" | "boolean";

export interface PurpleValveSafeStates {
    description:          string;
    type:                 ValveSafeStatesType;
    additionalProperties: AdditionalProperties;
}

export interface AdditionalProperties {
    type:  ABiasType;
    items: Property;
}

export interface Property {
    $ref: string;
}

export type ABiasType = "array";

export type ValveSafeStatesType = "object";

export interface AbortStageConfig {
    description: string;
    type:        ValveSafeStatesType;
    properties:  AbortStageConfigProperties;
    required:    string[];
}

export interface AbortStageConfigProperties {
    abort_condition:   AbortCondition;
    stage_name:        AbortCondition;
    valve_safe_states: RollingClass;
}

export interface RollingClass {
    description:          string;
    type:                 ValveSafeStatesType;
    additionalProperties: Property;
}

export interface ActiveConfiguration {
    description: string;
    type:        ValveSafeStatesType;
    properties:  ActiveConfigurationProperties;
    required:    string[];
}

export interface ActiveConfigurationProperties {
    configuration_id: ConfigurationID;
}

export interface ConfigurationID {
    type: AbortConditionType;
}

export interface ADCData {
    description: string;
    type:        ValveSafeStatesType;
    properties:  ADCDataProperties;
    required:    string[];
}

export interface ADCDataProperties {
    current_loop_pt: CurrentLoopPt;
    rail_3v3:        Rail3_V3;
    rail_5v:         Rail3_V3;
}

export interface CurrentLoopPt {
    description?: string;
    type:         CurrentLoopPtType;
    format:       Format;
    minimum?:     number;
    maximum?:     number;
    default?:     number;
}

export type Format = "double" | "uint32" | "float" | "uint64" | "uint8";

export type CurrentLoopPtType = "number" | "integer";

export interface Rail3_V3 {
    description: string;
    $ref:        string;
}

export interface AltimeterOffsets {
    description: string;
    type:        ValveSafeStatesType;
    properties:  AltimeterOffsetsProperties;
    required:    string[];
}

export interface AltimeterOffsetsProperties {
    ekf_lockout_time:          CurrentLoopPt;
    flight_baro_fmf_parameter: CurrentLoopPt;
    flight_gps_fmf_parameter:  CurrentLoopPt;
    ground_baro_fmf_parameter: CurrentLoopPt;
    ground_gps_fmf_parameter:  CurrentLoopPt;
    h_offset_alt:              CurrentLoopPt;
    h_offset_filter:           CurrentLoopPt;
}

export interface Barometer {
    description: string;
    type:        ValveSafeStatesType;
    properties:  BarometerProperties;
    required:    string[];
}

export interface BarometerProperties {
    pressure:    CurrentLoopPt;
    temperature: CurrentLoopPt;
}

export interface Bms {
    description: string;
    type:        ValveSafeStatesType;
    properties:  BmsProperties;
    required:    string[];
}

export interface BmsProperties {
    battery_bus:        Rail3_V3;
    charger:            CurrentLoopPt;
    chassis:            CurrentLoopPt;
    e_stop:             CurrentLoopPt;
    ethernet_bus:       Rail3_V3;
    fcb_bus:            Rail3_V3;
    five_volt_rail:     Rail3_V3;
    rbf_tag:            CurrentLoopPt;
    reco_load_switch_1: CurrentLoopPt;
    reco_load_switch_2: CurrentLoopPt;
    sam_power_bus:      Rail3_V3;
    tel_bus:            Rail3_V3;
    umbilical_bus:      Rail3_V3;
}

export interface Bus {
    description: string;
    type:        ValveSafeStatesType;
    properties:  BusProperties;
    required:    string[];
}

export interface BusProperties {
    current: CurrentLoopPt;
    voltage: CurrentLoopPt;
}

export interface CameraEnable {
    description: string;
    type:        ValveSafeStatesType;
    properties:  CameraEnableProperties;
    required:    string[];
}

export interface CameraEnableProperties {
    enabled: ConfigurationID;
}

export interface CompositeValveState {
    description: string;
    type:        ValveSafeStatesType;
    properties:  CompositeValveStateProperties;
    required:    string[];
}

export interface CompositeValveStateProperties {
    actual:    Rail3_V3;
    commanded: Rail3_V3;
}

export interface Computer {
    description: string;
    oneOf:       ComputerOneOf[];
}

export interface ComputerOneOf {
    description: string;
    type:        AbortConditionType;
    const:       string;
}

export interface Request {
    description: string;
    type:        ValveSafeStatesType;
    properties:  DeleteAbortConfigRequestProperties;
    required:    string[];
}

export interface DeleteAbortConfigRequestProperties {
    name: AbortCondition;
}

export interface DeleteMappingsRequest {
    description: string;
    type:        ValveSafeStatesType;
    properties:  DeleteMappingsRequestProperties;
    required:    string[];
}

export interface DeleteMappingsRequestProperties {
    configuration_id: AbortCondition;
    mappings:         Mappings;
}

export interface Mappings {
    description: string;
    type:        string[];
    items:       Property;
}

export interface Duration {
    type:       ValveSafeStatesType;
    properties: DurationProperties;
    required:   string[];
}

export interface DurationProperties {
    nanos: CurrentLoopPt;
    secs:  CurrentLoopPt;
}

export interface EkfStateVector {
    description: string;
    type:        ValveSafeStatesType;
    properties:  EkfStateVectorProperties;
    required:    string[];
}

export interface EkfStateVectorProperties {
    a_bias:     ABias;
    a_sf:       ABias;
    g_bias:     ABias;
    g_sf:       ABias;
    lla_pos:    ABias;
    quaternion: ABias;
    velocity:   ABias;
}

export interface ABias {
    description: string;
    type:        ABiasType;
    items:       From;
    maxItems:    number;
    minItems:    number;
}

export interface From {
    type:   CurrentLoopPtType;
    format: Format;
}

export interface ExecuteSQLRequest {
    type:       ValveSafeStatesType;
    properties: ExecuteSQLRequestProperties;
    required:   string[];
}

export interface ExecuteSQLRequestProperties {
    raw_sql: ConfigurationID;
}

export interface ExecuteSQLResponse {
    type:       ValveSafeStatesType;
    properties: ExecuteSQLResponseProperties;
    required:   string[];
}

export interface ExecuteSQLResponseProperties {
    column_names: ColumnNames;
    rows:         Rows;
}

export interface ColumnNames {
    type:  ABiasType;
    items: ConfigurationID;
}

export interface Rows {
    type:  ABiasType;
    items: RowsItems;
}

export interface RowsItems {
    type:  ABiasType;
    items: boolean;
}

export interface ExportRequest {
    description: string;
    type:        ValveSafeStatesType;
    properties:  ExportRequestProperties;
    required:    string[];
}

export interface ExportRequestProperties {
    format: ConfigurationID;
    from:   From;
    to:     From;
}

export interface FcSensors {
    description: string;
    type:        ValveSafeStatesType;
    properties:  FcSensorsProperties;
    required:    string[];
}

export interface FcSensorsProperties {
    adc:          Rail3_V3;
    barometer:    Rail3_V3;
    imu:          Rail3_V3;
    magnetometer: Rail3_V3;
}

export interface GpsState {
    description: string;
    type:        ValveSafeStatesType;
    properties:  GpsStateProperties;
    required:    string[];
}

export interface GpsStateProperties {
    altitude_m:        CurrentLoopPt;
    down_mps:          CurrentLoopPt;
    east_mps:          CurrentLoopPt;
    has_fix:           AbortCondition;
    latitude_deg:      CurrentLoopPt;
    longitude_deg:     CurrentLoopPt;
    north_mps:         CurrentLoopPt;
    num_satellites:    CurrentLoopPt;
    timestamp_unix_ms: TimestampUnixMS;
}

export interface TimestampUnixMS {
    description: string;
    type:        string[];
    format:      string;
}

export interface Imu {
    description: string;
    type:        ValveSafeStatesType;
    properties:  ImuProperties;
    required:    string[];
}

export interface ImuProperties {
    accelerometer: Rail3_V3;
    gyroscope:     Rail3_V3;
}

export interface InitialCovarianceMatrix {
    description: string;
    type:        ValveSafeStatesType;
    properties:  InitialCovarianceMatrixProperties;
    required:    string[];
}

export interface InitialCovarianceMatrixProperties {
    abias_unc0: ABias;
    asf_unc0:   ABias;
    att_unc0:   ABias;
    gbias_unc0: ABias;
    gsf_unc0:   ABias;
    pos_unc0:   ABias;
    vel_unc0:   ABias;
}

export interface LugArm {
    description: string;
    type:        ValveSafeStatesType;
    properties:  LugArmProperties;
    required:    string[];
}

export interface LugArmProperties {
    armed: ConfigurationID;
}

export interface Measurement {
    description: string;
    type:        ValveSafeStatesType;
    properties:  MeasurementProperties;
    required:    string[];
}

export interface MeasurementProperties {
    unit:  Rail3_V3;
    value: CurrentLoopPt;
}

export interface MeasurementNoiseMatrix {
    description: string;
    type:        ValveSafeStatesType;
    properties:  MeasurementNoiseMatrixProperties;
    required:    string[];
}

export interface MeasurementNoiseMatrixProperties {
    barometer_noise:  CurrentLoopPt;
    gps_noise_matrix: ABias;
}

export interface NodeMapping {
    description: string;
    type:        ValveSafeStatesType;
    properties:  NodeMappingProperties;
    required:    string[];
}

export interface NodeMappingProperties {
    board_id:          AbortCondition;
    calibrated_offset: CurrentLoopPt;
    channel:           CurrentLoopPt;
    computer:          Rail3_V3;
    max:               TimestampUnixMS;
    min:               TimestampUnixMS;
    normally_closed:   NormallyClosed;
    powered_threshold: TimestampUnixMS;
    sensor_type:       Rail3_V3;
    text_id:           AbortCondition;
}

export interface NormallyClosed {
    description: string;
    type:        string[];
}

export interface OperatorCommandRequest {
    description: string;
    type:        ValveSafeStatesType;
    properties:  OperatorCommandRequestProperties;
    required:    string[];
}

export interface OperatorCommandRequestProperties {
    command: ConfigurationID;
    state:   State;
    target:  State;
}

export interface State {
    type: string[];
}

export interface ProcessNoiseMatrix {
    description: string;
    type:        ValveSafeStatesType;
    properties:  ProcessNoiseMatrixProperties;
    required:    string[];
}

export interface ProcessNoiseMatrixProperties {
    nu_au_mat: ABias;
    nu_av_mat: ABias;
    nu_gu_mat: ABias;
    nu_gv_mat: ABias;
}

export interface RbfState {
    description: string;
    type:        ValveSafeStatesType;
    properties:  RbfStateProperties;
    required:    string[];
}

export interface RbfStateProperties {
    bms:  CurrentLoopPt;
    reco: Reco;
    sam:  Sam;
}

export interface Reco {
    description: string;
    type:        ABiasType;
    items:       CurrentLoopPt;
    maxItems:    number;
    minItems:    number;
}

export interface Sam {
    description:          string;
    type:                 ValveSafeStatesType;
    additionalProperties: CurrentLoopPt;
}

export interface RecoGUICommandPayload {
    description: string;
    oneOf:       RecoGUICommandPayloadOneOf[];
}

export interface RecoGUICommandPayloadOneOf {
    description: string;
    type:        ValveSafeStatesType;
    properties:  OneOfProperties;
    required:    Required[];
}

export interface OneOfProperties {
    message_type: MessageType;
    payload:      Property;
}

export interface MessageType {
    type:  AbortConditionType;
    const: string;
}

export type Required = "message_type" | "payload";

export interface RecoGUICommandRequest {
    description: string;
    type:        ValveSafeStatesType;
    properties:  RecoGUICommandRequestProperties;
    oneOf:       RecoGUICommandPayloadOneOf[];
}

export interface RecoGUICommandRequestProperties {
    target: Target;
}

export interface Target {
    description: string;
    $ref:        string;
    default:     string;
}

export interface RecoState {
    description: string;
    type:        ValveSafeStatesType;
    properties:  RecoStateProperties;
    required:    string[];
}

export interface RecoStateProperties {
    a_bias:              ABias;
    a_sf:                ABias;
    angular_rate:        ABias;
    drouge_timer_enable: AbortCondition;
    ekf_blown_up:        AbortCondition;
    fading_memory_baro:  CurrentLoopPt;
    fading_memory_gps:   CurrentLoopPt;
    g_bias:              ABias;
    g_sf:                ABias;
    lin_accel:           ABias;
    lla_pos:             ABias;
    mag_data:            ABias;
    main_timer_enable:   AbortCondition;
    pressure:            CurrentLoopPt;
    quaternion:          ABias;
    rbf_enabled:         AbortCondition;
    reco_driver_faults:  Reco;
    reco_recvd_launch:   AbortCondition;
    sns1_current:        CurrentLoopPt;
    sns2_current:        CurrentLoopPt;
    stage1_enabled:      AbortCondition;
    stage2_enabled:      AbortCondition;
    temperature:         CurrentLoopPt;
    v_rail_24v:          CurrentLoopPt;
    v_rail_3v3:          CurrentLoopPt;
    velocity:            ABias;
    vref_ch1_dr1:        CurrentLoopPt;
    vref_ch1_dr2:        CurrentLoopPt;
    vref_ch2_dr1:        CurrentLoopPt;
    vref_ch2_dr2:        CurrentLoopPt;
}

export interface RecoTriState {
    description: string;
    type:        ABiasType;
    items:       RecoTriStateItems;
    maxItems:    number;
    minItems:    number;
}

export interface RecoTriStateItems {
    anyOf: AnyOf[];
}

export interface AnyOf {
    $ref?: string;
    type?: string;
}

export interface RetrieveAbortConfigsResponse {
    description: string;
    type:        ValveSafeStatesType;
    properties:  RetrieveAbortConfigsResponseProperties;
    required:    string[];
}

export interface RetrieveAbortConfigsResponseProperties {
    stages: Stages;
}

export interface Stages {
    description: string;
    type:        ABiasType;
    items:       Property;
}

export interface RetrieveSequenceResponse {
    description: string;
    type:        ValveSafeStatesType;
    properties:  RetrieveSequenceResponseProperties;
    required:    string[];
}

export interface RetrieveSequenceResponseProperties {
    sequences: Stages;
}

export interface RunSequenceRequest {
    description: string;
    type:        ValveSafeStatesType;
    properties:  RunSequenceRequestProperties;
    required:    string[];
}

export interface RunSequenceRequestProperties {
    force: NormallyClosed;
    name:  AbortCondition;
}

export interface SaveSequenceRequest {
    description: string;
    type:        ValveSafeStatesType;
    properties:  SaveSequenceRequestProperties;
    required:    string[];
}

export interface SaveSequenceRequestProperties {
    configuration_id: NormallyClosed;
    name:             AbortCondition;
    script:           AbortCondition;
}

export interface SetAbortConfig {
    description: string;
    type:        ValveSafeStatesType;
    properties:  SetAbortConfigProperties;
    required:    string[];
}

export interface SetAbortConfigProperties {
    stage_name: ConfigurationID;
}

export interface SetMappingsRequest {
    description: string;
    type:        ValveSafeStatesType;
    properties:  SetMappingsRequestProperties;
    required:    string[];
}

export interface SetMappingsRequestProperties {
    configuration_id: AbortCondition;
    mappings:         Stages;
}

export interface Statistics {
    description: string;
    type:        ValveSafeStatesType;
    properties:  StatisticsProperties;
    required:    string[];
}

export interface StatisticsProperties {
    delta_time:             Rail3_V3;
    rolling_average:        Rail3_V3;
    time_since_last_update: CurrentLoopPt;
}

export interface TelemetrySourceQuery {
    description: string;
    type:        ValveSafeStatesType;
    properties:  TelemetrySourceQueryProperties;
}

export interface TelemetrySourceQueryProperties {
    source: State;
}

export interface TelemetrySourceStats {
    description: string;
    type:        ValveSafeStatesType;
    properties:  TelemetrySourceStatsProperties;
}

export interface TelemetrySourceStatsProperties {
    packet_size_bytes:    PacketSizeBytes;
    time_since_update_ms: TimeSinceUpdateMS;
    update_rate_hz:       TimeSinceUpdateMS;
}

export interface PacketSizeBytes {
    type:    string[];
    format:  string;
    minimum: number;
}

export interface TimeSinceUpdateMS {
    type:   string[];
    format: Format;
}

export interface TelemetryStatsResponse {
    description: string;
    type:        ValveSafeStatesType;
    properties:  TelemetryStatsResponseProperties;
    required:    string[];
}

export interface TelemetryStatsResponseProperties {
    tel:       Property;
    umbilical: Property;
}

export interface TimerValues {
    description: string;
    type:        ValveSafeStatesType;
    properties:  TimerValuesProperties;
    required:    string[];
}

export interface TimerValuesProperties {
    drouge_timer:        CurrentLoopPt;
    drouge_timer_enable: CurrentLoopPt;
    main_timer:          CurrentLoopPt;
    main_timer_enable:   CurrentLoopPt;
}

export interface Trigger {
    description: string;
    type:        ValveSafeStatesType;
    properties:  TriggerProperties;
    required:    string[];
}

export interface TriggerProperties {
    active:    AbortCondition;
    condition: AbortCondition;
    name:      AbortCondition;
    script:    AbortCondition;
}

export interface ValveAction {
    description: string;
    type:        ValveSafeStatesType;
    properties:  ValveActionProperties;
    required:    string[];
}

export interface ValveActionProperties {
    channel_num: CurrentLoopPt;
    powered:     AbortCondition;
    timer:       Rail3_V3;
}

export interface ValveSafeState {
    description: string;
    type:        ValveSafeStatesType;
    properties:  ValveSafeStateProperties;
    required:    string[];
}

export interface ValveSafeStateProperties {
    desired_state: Rail3_V3;
    safing_timer:  CurrentLoopPt;
}

export interface Vector {
    description: string;
    type:        ValveSafeStatesType;
    properties:  VectorProperties;
    required:    string[];
}

export interface VectorProperties {
    x: CurrentLoopPt;
    y: CurrentLoopPt;
    z: CurrentLoopPt;
}

export interface VehicleState {
    description: string;
    type:        ValveSafeStatesType;
    properties:  VehicleStateProperties;
    required:    string[];
}

export interface VehicleStateProperties {
    abort_stage:     Rail3_V3;
    bms:             Rail3_V3;
    fc_sensors:      Rail3_V3;
    gps:             Gps;
    gps_valid:       AbortCondition;
    rbf:             Rail3_V3;
    reco:            Rail3_V3;
    reco_valid:      AbortCondition;
    rolling:         RollingClass;
    sensor_readings: RollingClass;
    valve_states:    RollingClass;
}

export interface Gps {
    description: string;
    anyOf:       AnyOf[];
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
        { json: "$schema", js: "$schema", typ: "" },
        { json: "title", js: "title", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: m(r("Property")) },
        { json: "required", js: "required", typ: a("") },
        { json: "$defs", js: "$defs", typ: r("Defs") },
    ], false),
    "Defs": o([
        { json: "AbortStage", js: "AbortStage", typ: r("AbortStage") },
        { json: "AbortStageConfig", js: "AbortStageConfig", typ: r("AbortStageConfig") },
        { json: "ActiveConfiguration", js: "ActiveConfiguration", typ: r("ActiveConfiguration") },
        { json: "AdcData", js: "AdcData", typ: r("ADCData") },
        { json: "AltimeterOffsets", js: "AltimeterOffsets", typ: r("AltimeterOffsets") },
        { json: "Barometer", js: "Barometer", typ: r("Barometer") },
        { json: "Bms", js: "Bms", typ: r("Bms") },
        { json: "Bus", js: "Bus", typ: r("Bus") },
        { json: "CameraEnable", js: "CameraEnable", typ: r("CameraEnable") },
        { json: "CompositeValveState", js: "CompositeValveState", typ: r("CompositeValveState") },
        { json: "Computer", js: "Computer", typ: r("Computer") },
        { json: "DeleteAbortConfigRequest", js: "DeleteAbortConfigRequest", typ: r("Request") },
        { json: "DeleteMappingsRequest", js: "DeleteMappingsRequest", typ: r("DeleteMappingsRequest") },
        { json: "DeleteSequenceRequest", js: "DeleteSequenceRequest", typ: r("Request") },
        { json: "DeleteTriggerRequest", js: "DeleteTriggerRequest", typ: r("Request") },
        { json: "Duration", js: "Duration", typ: r("Duration") },
        { json: "EkfStateVector", js: "EkfStateVector", typ: r("EkfStateVector") },
        { json: "ExecuteSqlRequest", js: "ExecuteSqlRequest", typ: r("ExecuteSQLRequest") },
        { json: "ExecuteSqlResponse", js: "ExecuteSqlResponse", typ: r("ExecuteSQLResponse") },
        { json: "ExportRequest", js: "ExportRequest", typ: r("ExportRequest") },
        { json: "FcSensors", js: "FcSensors", typ: r("FcSensors") },
        { json: "GpsState", js: "GpsState", typ: r("GpsState") },
        { json: "Imu", js: "Imu", typ: r("Imu") },
        { json: "InitialCovarianceMatrix", js: "InitialCovarianceMatrix", typ: r("InitialCovarianceMatrix") },
        { json: "LugArm", js: "LugArm", typ: r("LugArm") },
        { json: "LugDetonate", js: "LugDetonate", typ: r("CameraEnable") },
        { json: "Measurement", js: "Measurement", typ: r("Measurement") },
        { json: "MeasurementNoiseMatrix", js: "MeasurementNoiseMatrix", typ: r("MeasurementNoiseMatrix") },
        { json: "NodeMapping", js: "NodeMapping", typ: r("NodeMapping") },
        { json: "OperatorCommandRequest", js: "OperatorCommandRequest", typ: r("OperatorCommandRequest") },
        { json: "ProcessNoiseMatrix", js: "ProcessNoiseMatrix", typ: r("ProcessNoiseMatrix") },
        { json: "RbfState", js: "RbfState", typ: r("RbfState") },
        { json: "RecoGuiCommandPayload", js: "RecoGuiCommandPayload", typ: r("RecoGUICommandPayload") },
        { json: "RecoGuiCommandRequest", js: "RecoGuiCommandRequest", typ: r("RecoGUICommandRequest") },
        { json: "RecoState", js: "RecoState", typ: r("RecoState") },
        { json: "RecoTriState", js: "RecoTriState", typ: r("RecoTriState") },
        { json: "RetrieveAbortConfigsResponse", js: "RetrieveAbortConfigsResponse", typ: r("RetrieveAbortConfigsResponse") },
        { json: "RetrieveSequenceResponse", js: "RetrieveSequenceResponse", typ: r("RetrieveSequenceResponse") },
        { json: "RunSequenceRequest", js: "RunSequenceRequest", typ: r("RunSequenceRequest") },
        { json: "SaveSequenceRequest", js: "SaveSequenceRequest", typ: r("SaveSequenceRequest") },
        { json: "SensorType", js: "SensorType", typ: r("Computer") },
        { json: "SequenceWithConfiguration", js: "SequenceWithConfiguration", typ: r("SaveSequenceRequest") },
        { json: "SetAbortConfig", js: "SetAbortConfig", typ: r("SetAbortConfig") },
        { json: "SetMappingsRequest", js: "SetMappingsRequest", typ: r("SetMappingsRequest") },
        { json: "Statistics", js: "Statistics", typ: r("Statistics") },
        { json: "StopSequenceRequest", js: "StopSequenceRequest", typ: r("Request") },
        { json: "TargetMCU", js: "TargetMCU", typ: r("Computer") },
        { json: "TelemetrySourceQuery", js: "TelemetrySourceQuery", typ: r("TelemetrySourceQuery") },
        { json: "TelemetrySourceStats", js: "TelemetrySourceStats", typ: r("TelemetrySourceStats") },
        { json: "TelemetryStatsResponse", js: "TelemetryStatsResponse", typ: r("TelemetryStatsResponse") },
        { json: "TimerValues", js: "TimerValues", typ: r("TimerValues") },
        { json: "Trigger", js: "Trigger", typ: r("Trigger") },
        { json: "Unit", js: "Unit", typ: r("Computer") },
        { json: "ValveAction", js: "ValveAction", typ: r("ValveAction") },
        { json: "ValveSafeState", js: "ValveSafeState", typ: r("ValveSafeState") },
        { json: "ValveState", js: "ValveState", typ: r("Computer") },
        { json: "Vector", js: "Vector", typ: r("Vector") },
        { json: "VehicleState", js: "VehicleState", typ: r("VehicleState") },
    ], false),
    "AbortStage": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("AbortStageProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "AbortStageProperties": o([
        { json: "abort_condition", js: "abort_condition", typ: r("AbortCondition") },
        { json: "aborted", js: "aborted", typ: r("AbortCondition") },
        { json: "name", js: "name", typ: r("AbortCondition") },
        { json: "valve_safe_states", js: "valve_safe_states", typ: r("PurpleValveSafeStates") },
    ], false),
    "AbortCondition": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("AbortConditionType") },
    ], false),
    "PurpleValveSafeStates": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "additionalProperties", js: "additionalProperties", typ: r("AdditionalProperties") },
    ], false),
    "AdditionalProperties": o([
        { json: "type", js: "type", typ: r("ABiasType") },
        { json: "items", js: "items", typ: r("Property") },
    ], false),
    "Property": o([
        { json: "$ref", js: "$ref", typ: "" },
    ], false),
    "AbortStageConfig": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("AbortStageConfigProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "AbortStageConfigProperties": o([
        { json: "abort_condition", js: "abort_condition", typ: r("AbortCondition") },
        { json: "stage_name", js: "stage_name", typ: r("AbortCondition") },
        { json: "valve_safe_states", js: "valve_safe_states", typ: r("RollingClass") },
    ], false),
    "RollingClass": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "additionalProperties", js: "additionalProperties", typ: r("Property") },
    ], false),
    "ActiveConfiguration": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("ActiveConfigurationProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "ActiveConfigurationProperties": o([
        { json: "configuration_id", js: "configuration_id", typ: r("ConfigurationID") },
    ], false),
    "ConfigurationID": o([
        { json: "type", js: "type", typ: r("AbortConditionType") },
    ], false),
    "ADCData": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("ADCDataProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "ADCDataProperties": o([
        { json: "current_loop_pt", js: "current_loop_pt", typ: r("CurrentLoopPt") },
        { json: "rail_3v3", js: "rail_3v3", typ: r("Rail3_V3") },
        { json: "rail_5v", js: "rail_5v", typ: r("Rail3_V3") },
    ], false),
    "CurrentLoopPt": o([
        { json: "description", js: "description", typ: u(undefined, "") },
        { json: "type", js: "type", typ: r("CurrentLoopPtType") },
        { json: "format", js: "format", typ: r("Format") },
        { json: "minimum", js: "minimum", typ: u(undefined, 0) },
        { json: "maximum", js: "maximum", typ: u(undefined, 0) },
        { json: "default", js: "default", typ: u(undefined, 3.14) },
    ], false),
    "Rail3_V3": o([
        { json: "description", js: "description", typ: "" },
        { json: "$ref", js: "$ref", typ: "" },
    ], false),
    "AltimeterOffsets": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("AltimeterOffsetsProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "AltimeterOffsetsProperties": o([
        { json: "ekf_lockout_time", js: "ekf_lockout_time", typ: r("CurrentLoopPt") },
        { json: "flight_baro_fmf_parameter", js: "flight_baro_fmf_parameter", typ: r("CurrentLoopPt") },
        { json: "flight_gps_fmf_parameter", js: "flight_gps_fmf_parameter", typ: r("CurrentLoopPt") },
        { json: "ground_baro_fmf_parameter", js: "ground_baro_fmf_parameter", typ: r("CurrentLoopPt") },
        { json: "ground_gps_fmf_parameter", js: "ground_gps_fmf_parameter", typ: r("CurrentLoopPt") },
        { json: "h_offset_alt", js: "h_offset_alt", typ: r("CurrentLoopPt") },
        { json: "h_offset_filter", js: "h_offset_filter", typ: r("CurrentLoopPt") },
    ], false),
    "Barometer": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("BarometerProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "BarometerProperties": o([
        { json: "pressure", js: "pressure", typ: r("CurrentLoopPt") },
        { json: "temperature", js: "temperature", typ: r("CurrentLoopPt") },
    ], false),
    "Bms": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("BmsProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "BmsProperties": o([
        { json: "battery_bus", js: "battery_bus", typ: r("Rail3_V3") },
        { json: "charger", js: "charger", typ: r("CurrentLoopPt") },
        { json: "chassis", js: "chassis", typ: r("CurrentLoopPt") },
        { json: "e_stop", js: "e_stop", typ: r("CurrentLoopPt") },
        { json: "ethernet_bus", js: "ethernet_bus", typ: r("Rail3_V3") },
        { json: "fcb_bus", js: "fcb_bus", typ: r("Rail3_V3") },
        { json: "five_volt_rail", js: "five_volt_rail", typ: r("Rail3_V3") },
        { json: "rbf_tag", js: "rbf_tag", typ: r("CurrentLoopPt") },
        { json: "reco_load_switch_1", js: "reco_load_switch_1", typ: r("CurrentLoopPt") },
        { json: "reco_load_switch_2", js: "reco_load_switch_2", typ: r("CurrentLoopPt") },
        { json: "sam_power_bus", js: "sam_power_bus", typ: r("Rail3_V3") },
        { json: "tel_bus", js: "tel_bus", typ: r("Rail3_V3") },
        { json: "umbilical_bus", js: "umbilical_bus", typ: r("Rail3_V3") },
    ], false),
    "Bus": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("BusProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "BusProperties": o([
        { json: "current", js: "current", typ: r("CurrentLoopPt") },
        { json: "voltage", js: "voltage", typ: r("CurrentLoopPt") },
    ], false),
    "CameraEnable": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("CameraEnableProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "CameraEnableProperties": o([
        { json: "enabled", js: "enabled", typ: r("ConfigurationID") },
    ], false),
    "CompositeValveState": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("CompositeValveStateProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "CompositeValveStateProperties": o([
        { json: "actual", js: "actual", typ: r("Rail3_V3") },
        { json: "commanded", js: "commanded", typ: r("Rail3_V3") },
    ], false),
    "Computer": o([
        { json: "description", js: "description", typ: "" },
        { json: "oneOf", js: "oneOf", typ: a(r("ComputerOneOf")) },
    ], false),
    "ComputerOneOf": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("AbortConditionType") },
        { json: "const", js: "const", typ: "" },
    ], false),
    "Request": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("DeleteAbortConfigRequestProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "DeleteAbortConfigRequestProperties": o([
        { json: "name", js: "name", typ: r("AbortCondition") },
    ], false),
    "DeleteMappingsRequest": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("DeleteMappingsRequestProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "DeleteMappingsRequestProperties": o([
        { json: "configuration_id", js: "configuration_id", typ: r("AbortCondition") },
        { json: "mappings", js: "mappings", typ: r("Mappings") },
    ], false),
    "Mappings": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: a("") },
        { json: "items", js: "items", typ: r("Property") },
    ], false),
    "Duration": o([
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("DurationProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "DurationProperties": o([
        { json: "nanos", js: "nanos", typ: r("CurrentLoopPt") },
        { json: "secs", js: "secs", typ: r("CurrentLoopPt") },
    ], false),
    "EkfStateVector": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("EkfStateVectorProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "EkfStateVectorProperties": o([
        { json: "a_bias", js: "a_bias", typ: r("ABias") },
        { json: "a_sf", js: "a_sf", typ: r("ABias") },
        { json: "g_bias", js: "g_bias", typ: r("ABias") },
        { json: "g_sf", js: "g_sf", typ: r("ABias") },
        { json: "lla_pos", js: "lla_pos", typ: r("ABias") },
        { json: "quaternion", js: "quaternion", typ: r("ABias") },
        { json: "velocity", js: "velocity", typ: r("ABias") },
    ], false),
    "ABias": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ABiasType") },
        { json: "items", js: "items", typ: r("From") },
        { json: "maxItems", js: "maxItems", typ: 0 },
        { json: "minItems", js: "minItems", typ: 0 },
    ], false),
    "From": o([
        { json: "type", js: "type", typ: r("CurrentLoopPtType") },
        { json: "format", js: "format", typ: r("Format") },
    ], false),
    "ExecuteSQLRequest": o([
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("ExecuteSQLRequestProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "ExecuteSQLRequestProperties": o([
        { json: "raw_sql", js: "raw_sql", typ: r("ConfigurationID") },
    ], false),
    "ExecuteSQLResponse": o([
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("ExecuteSQLResponseProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "ExecuteSQLResponseProperties": o([
        { json: "column_names", js: "column_names", typ: r("ColumnNames") },
        { json: "rows", js: "rows", typ: r("Rows") },
    ], false),
    "ColumnNames": o([
        { json: "type", js: "type", typ: r("ABiasType") },
        { json: "items", js: "items", typ: r("ConfigurationID") },
    ], false),
    "Rows": o([
        { json: "type", js: "type", typ: r("ABiasType") },
        { json: "items", js: "items", typ: r("RowsItems") },
    ], false),
    "RowsItems": o([
        { json: "type", js: "type", typ: r("ABiasType") },
        { json: "items", js: "items", typ: true },
    ], false),
    "ExportRequest": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("ExportRequestProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "ExportRequestProperties": o([
        { json: "format", js: "format", typ: r("ConfigurationID") },
        { json: "from", js: "from", typ: r("From") },
        { json: "to", js: "to", typ: r("From") },
    ], false),
    "FcSensors": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("FcSensorsProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "FcSensorsProperties": o([
        { json: "adc", js: "adc", typ: r("Rail3_V3") },
        { json: "barometer", js: "barometer", typ: r("Rail3_V3") },
        { json: "imu", js: "imu", typ: r("Rail3_V3") },
        { json: "magnetometer", js: "magnetometer", typ: r("Rail3_V3") },
    ], false),
    "GpsState": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("GpsStateProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "GpsStateProperties": o([
        { json: "altitude_m", js: "altitude_m", typ: r("CurrentLoopPt") },
        { json: "down_mps", js: "down_mps", typ: r("CurrentLoopPt") },
        { json: "east_mps", js: "east_mps", typ: r("CurrentLoopPt") },
        { json: "has_fix", js: "has_fix", typ: r("AbortCondition") },
        { json: "latitude_deg", js: "latitude_deg", typ: r("CurrentLoopPt") },
        { json: "longitude_deg", js: "longitude_deg", typ: r("CurrentLoopPt") },
        { json: "north_mps", js: "north_mps", typ: r("CurrentLoopPt") },
        { json: "num_satellites", js: "num_satellites", typ: r("CurrentLoopPt") },
        { json: "timestamp_unix_ms", js: "timestamp_unix_ms", typ: r("TimestampUnixMS") },
    ], false),
    "TimestampUnixMS": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: a("") },
        { json: "format", js: "format", typ: "" },
    ], false),
    "Imu": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("ImuProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "ImuProperties": o([
        { json: "accelerometer", js: "accelerometer", typ: r("Rail3_V3") },
        { json: "gyroscope", js: "gyroscope", typ: r("Rail3_V3") },
    ], false),
    "InitialCovarianceMatrix": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("InitialCovarianceMatrixProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "InitialCovarianceMatrixProperties": o([
        { json: "abias_unc0", js: "abias_unc0", typ: r("ABias") },
        { json: "asf_unc0", js: "asf_unc0", typ: r("ABias") },
        { json: "att_unc0", js: "att_unc0", typ: r("ABias") },
        { json: "gbias_unc0", js: "gbias_unc0", typ: r("ABias") },
        { json: "gsf_unc0", js: "gsf_unc0", typ: r("ABias") },
        { json: "pos_unc0", js: "pos_unc0", typ: r("ABias") },
        { json: "vel_unc0", js: "vel_unc0", typ: r("ABias") },
    ], false),
    "LugArm": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("LugArmProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "LugArmProperties": o([
        { json: "armed", js: "armed", typ: r("ConfigurationID") },
    ], false),
    "Measurement": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("MeasurementProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "MeasurementProperties": o([
        { json: "unit", js: "unit", typ: r("Rail3_V3") },
        { json: "value", js: "value", typ: r("CurrentLoopPt") },
    ], false),
    "MeasurementNoiseMatrix": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("MeasurementNoiseMatrixProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "MeasurementNoiseMatrixProperties": o([
        { json: "barometer_noise", js: "barometer_noise", typ: r("CurrentLoopPt") },
        { json: "gps_noise_matrix", js: "gps_noise_matrix", typ: r("ABias") },
    ], false),
    "NodeMapping": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("NodeMappingProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "NodeMappingProperties": o([
        { json: "board_id", js: "board_id", typ: r("AbortCondition") },
        { json: "calibrated_offset", js: "calibrated_offset", typ: r("CurrentLoopPt") },
        { json: "channel", js: "channel", typ: r("CurrentLoopPt") },
        { json: "computer", js: "computer", typ: r("Rail3_V3") },
        { json: "max", js: "max", typ: r("TimestampUnixMS") },
        { json: "min", js: "min", typ: r("TimestampUnixMS") },
        { json: "normally_closed", js: "normally_closed", typ: r("NormallyClosed") },
        { json: "powered_threshold", js: "powered_threshold", typ: r("TimestampUnixMS") },
        { json: "sensor_type", js: "sensor_type", typ: r("Rail3_V3") },
        { json: "text_id", js: "text_id", typ: r("AbortCondition") },
    ], false),
    "NormallyClosed": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: a("") },
    ], false),
    "OperatorCommandRequest": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("OperatorCommandRequestProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "OperatorCommandRequestProperties": o([
        { json: "command", js: "command", typ: r("ConfigurationID") },
        { json: "state", js: "state", typ: r("State") },
        { json: "target", js: "target", typ: r("State") },
    ], false),
    "State": o([
        { json: "type", js: "type", typ: a("") },
    ], false),
    "ProcessNoiseMatrix": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("ProcessNoiseMatrixProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "ProcessNoiseMatrixProperties": o([
        { json: "nu_au_mat", js: "nu_au_mat", typ: r("ABias") },
        { json: "nu_av_mat", js: "nu_av_mat", typ: r("ABias") },
        { json: "nu_gu_mat", js: "nu_gu_mat", typ: r("ABias") },
        { json: "nu_gv_mat", js: "nu_gv_mat", typ: r("ABias") },
    ], false),
    "RbfState": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("RbfStateProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "RbfStateProperties": o([
        { json: "bms", js: "bms", typ: r("CurrentLoopPt") },
        { json: "reco", js: "reco", typ: r("Reco") },
        { json: "sam", js: "sam", typ: r("Sam") },
    ], false),
    "Reco": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ABiasType") },
        { json: "items", js: "items", typ: r("CurrentLoopPt") },
        { json: "maxItems", js: "maxItems", typ: 0 },
        { json: "minItems", js: "minItems", typ: 0 },
    ], false),
    "Sam": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "additionalProperties", js: "additionalProperties", typ: r("CurrentLoopPt") },
    ], false),
    "RecoGUICommandPayload": o([
        { json: "description", js: "description", typ: "" },
        { json: "oneOf", js: "oneOf", typ: a(r("RecoGUICommandPayloadOneOf")) },
    ], false),
    "RecoGUICommandPayloadOneOf": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("OneOfProperties") },
        { json: "required", js: "required", typ: a(r("Required")) },
    ], false),
    "OneOfProperties": o([
        { json: "message_type", js: "message_type", typ: r("MessageType") },
        { json: "payload", js: "payload", typ: r("Property") },
    ], false),
    "MessageType": o([
        { json: "type", js: "type", typ: r("AbortConditionType") },
        { json: "const", js: "const", typ: "" },
    ], false),
    "RecoGUICommandRequest": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("RecoGUICommandRequestProperties") },
        { json: "oneOf", js: "oneOf", typ: a(r("RecoGUICommandPayloadOneOf")) },
    ], false),
    "RecoGUICommandRequestProperties": o([
        { json: "target", js: "target", typ: r("Target") },
    ], false),
    "Target": o([
        { json: "description", js: "description", typ: "" },
        { json: "$ref", js: "$ref", typ: "" },
        { json: "default", js: "default", typ: "" },
    ], false),
    "RecoState": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("RecoStateProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "RecoStateProperties": o([
        { json: "a_bias", js: "a_bias", typ: r("ABias") },
        { json: "a_sf", js: "a_sf", typ: r("ABias") },
        { json: "angular_rate", js: "angular_rate", typ: r("ABias") },
        { json: "drouge_timer_enable", js: "drouge_timer_enable", typ: r("AbortCondition") },
        { json: "ekf_blown_up", js: "ekf_blown_up", typ: r("AbortCondition") },
        { json: "fading_memory_baro", js: "fading_memory_baro", typ: r("CurrentLoopPt") },
        { json: "fading_memory_gps", js: "fading_memory_gps", typ: r("CurrentLoopPt") },
        { json: "g_bias", js: "g_bias", typ: r("ABias") },
        { json: "g_sf", js: "g_sf", typ: r("ABias") },
        { json: "lin_accel", js: "lin_accel", typ: r("ABias") },
        { json: "lla_pos", js: "lla_pos", typ: r("ABias") },
        { json: "mag_data", js: "mag_data", typ: r("ABias") },
        { json: "main_timer_enable", js: "main_timer_enable", typ: r("AbortCondition") },
        { json: "pressure", js: "pressure", typ: r("CurrentLoopPt") },
        { json: "quaternion", js: "quaternion", typ: r("ABias") },
        { json: "rbf_enabled", js: "rbf_enabled", typ: r("AbortCondition") },
        { json: "reco_driver_faults", js: "reco_driver_faults", typ: r("Reco") },
        { json: "reco_recvd_launch", js: "reco_recvd_launch", typ: r("AbortCondition") },
        { json: "sns1_current", js: "sns1_current", typ: r("CurrentLoopPt") },
        { json: "sns2_current", js: "sns2_current", typ: r("CurrentLoopPt") },
        { json: "stage1_enabled", js: "stage1_enabled", typ: r("AbortCondition") },
        { json: "stage2_enabled", js: "stage2_enabled", typ: r("AbortCondition") },
        { json: "temperature", js: "temperature", typ: r("CurrentLoopPt") },
        { json: "v_rail_24v", js: "v_rail_24v", typ: r("CurrentLoopPt") },
        { json: "v_rail_3v3", js: "v_rail_3v3", typ: r("CurrentLoopPt") },
        { json: "velocity", js: "velocity", typ: r("ABias") },
        { json: "vref_ch1_dr1", js: "vref_ch1_dr1", typ: r("CurrentLoopPt") },
        { json: "vref_ch1_dr2", js: "vref_ch1_dr2", typ: r("CurrentLoopPt") },
        { json: "vref_ch2_dr1", js: "vref_ch2_dr1", typ: r("CurrentLoopPt") },
        { json: "vref_ch2_dr2", js: "vref_ch2_dr2", typ: r("CurrentLoopPt") },
    ], false),
    "RecoTriState": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ABiasType") },
        { json: "items", js: "items", typ: r("RecoTriStateItems") },
        { json: "maxItems", js: "maxItems", typ: 0 },
        { json: "minItems", js: "minItems", typ: 0 },
    ], false),
    "RecoTriStateItems": o([
        { json: "anyOf", js: "anyOf", typ: a(r("AnyOf")) },
    ], false),
    "AnyOf": o([
        { json: "$ref", js: "$ref", typ: u(undefined, "") },
        { json: "type", js: "type", typ: u(undefined, "") },
    ], false),
    "RetrieveAbortConfigsResponse": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("RetrieveAbortConfigsResponseProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "RetrieveAbortConfigsResponseProperties": o([
        { json: "stages", js: "stages", typ: r("Stages") },
    ], false),
    "Stages": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ABiasType") },
        { json: "items", js: "items", typ: r("Property") },
    ], false),
    "RetrieveSequenceResponse": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("RetrieveSequenceResponseProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "RetrieveSequenceResponseProperties": o([
        { json: "sequences", js: "sequences", typ: r("Stages") },
    ], false),
    "RunSequenceRequest": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("RunSequenceRequestProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "RunSequenceRequestProperties": o([
        { json: "force", js: "force", typ: r("NormallyClosed") },
        { json: "name", js: "name", typ: r("AbortCondition") },
    ], false),
    "SaveSequenceRequest": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("SaveSequenceRequestProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "SaveSequenceRequestProperties": o([
        { json: "configuration_id", js: "configuration_id", typ: r("NormallyClosed") },
        { json: "name", js: "name", typ: r("AbortCondition") },
        { json: "script", js: "script", typ: r("AbortCondition") },
    ], false),
    "SetAbortConfig": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("SetAbortConfigProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "SetAbortConfigProperties": o([
        { json: "stage_name", js: "stage_name", typ: r("ConfigurationID") },
    ], false),
    "SetMappingsRequest": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("SetMappingsRequestProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "SetMappingsRequestProperties": o([
        { json: "configuration_id", js: "configuration_id", typ: r("AbortCondition") },
        { json: "mappings", js: "mappings", typ: r("Stages") },
    ], false),
    "Statistics": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("StatisticsProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "StatisticsProperties": o([
        { json: "delta_time", js: "delta_time", typ: r("Rail3_V3") },
        { json: "rolling_average", js: "rolling_average", typ: r("Rail3_V3") },
        { json: "time_since_last_update", js: "time_since_last_update", typ: r("CurrentLoopPt") },
    ], false),
    "TelemetrySourceQuery": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("TelemetrySourceQueryProperties") },
    ], false),
    "TelemetrySourceQueryProperties": o([
        { json: "source", js: "source", typ: r("State") },
    ], false),
    "TelemetrySourceStats": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("TelemetrySourceStatsProperties") },
    ], false),
    "TelemetrySourceStatsProperties": o([
        { json: "packet_size_bytes", js: "packet_size_bytes", typ: r("PacketSizeBytes") },
        { json: "time_since_update_ms", js: "time_since_update_ms", typ: r("TimeSinceUpdateMS") },
        { json: "update_rate_hz", js: "update_rate_hz", typ: r("TimeSinceUpdateMS") },
    ], false),
    "PacketSizeBytes": o([
        { json: "type", js: "type", typ: a("") },
        { json: "format", js: "format", typ: "" },
        { json: "minimum", js: "minimum", typ: 0 },
    ], false),
    "TimeSinceUpdateMS": o([
        { json: "type", js: "type", typ: a("") },
        { json: "format", js: "format", typ: r("Format") },
    ], false),
    "TelemetryStatsResponse": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("TelemetryStatsResponseProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "TelemetryStatsResponseProperties": o([
        { json: "tel", js: "tel", typ: r("Property") },
        { json: "umbilical", js: "umbilical", typ: r("Property") },
    ], false),
    "TimerValues": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("TimerValuesProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "TimerValuesProperties": o([
        { json: "drouge_timer", js: "drouge_timer", typ: r("CurrentLoopPt") },
        { json: "drouge_timer_enable", js: "drouge_timer_enable", typ: r("CurrentLoopPt") },
        { json: "main_timer", js: "main_timer", typ: r("CurrentLoopPt") },
        { json: "main_timer_enable", js: "main_timer_enable", typ: r("CurrentLoopPt") },
    ], false),
    "Trigger": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("TriggerProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "TriggerProperties": o([
        { json: "active", js: "active", typ: r("AbortCondition") },
        { json: "condition", js: "condition", typ: r("AbortCondition") },
        { json: "name", js: "name", typ: r("AbortCondition") },
        { json: "script", js: "script", typ: r("AbortCondition") },
    ], false),
    "ValveAction": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("ValveActionProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "ValveActionProperties": o([
        { json: "channel_num", js: "channel_num", typ: r("CurrentLoopPt") },
        { json: "powered", js: "powered", typ: r("AbortCondition") },
        { json: "timer", js: "timer", typ: r("Rail3_V3") },
    ], false),
    "ValveSafeState": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("ValveSafeStateProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "ValveSafeStateProperties": o([
        { json: "desired_state", js: "desired_state", typ: r("Rail3_V3") },
        { json: "safing_timer", js: "safing_timer", typ: r("CurrentLoopPt") },
    ], false),
    "Vector": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("VectorProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "VectorProperties": o([
        { json: "x", js: "x", typ: r("CurrentLoopPt") },
        { json: "y", js: "y", typ: r("CurrentLoopPt") },
        { json: "z", js: "z", typ: r("CurrentLoopPt") },
    ], false),
    "VehicleState": o([
        { json: "description", js: "description", typ: "" },
        { json: "type", js: "type", typ: r("ValveSafeStatesType") },
        { json: "properties", js: "properties", typ: r("VehicleStateProperties") },
        { json: "required", js: "required", typ: a("") },
    ], false),
    "VehicleStateProperties": o([
        { json: "abort_stage", js: "abort_stage", typ: r("Rail3_V3") },
        { json: "bms", js: "bms", typ: r("Rail3_V3") },
        { json: "fc_sensors", js: "fc_sensors", typ: r("Rail3_V3") },
        { json: "gps", js: "gps", typ: r("Gps") },
        { json: "gps_valid", js: "gps_valid", typ: r("AbortCondition") },
        { json: "rbf", js: "rbf", typ: r("Rail3_V3") },
        { json: "reco", js: "reco", typ: r("Rail3_V3") },
        { json: "reco_valid", js: "reco_valid", typ: r("AbortCondition") },
        { json: "rolling", js: "rolling", typ: r("RollingClass") },
        { json: "sensor_readings", js: "sensor_readings", typ: r("RollingClass") },
        { json: "valve_states", js: "valve_states", typ: r("RollingClass") },
    ], false),
    "Gps": o([
        { json: "description", js: "description", typ: "" },
        { json: "anyOf", js: "anyOf", typ: a(r("AnyOf")) },
    ], false),
    "AbortConditionType": [
        "boolean",
        "string",
    ],
    "ABiasType": [
        "array",
    ],
    "ValveSafeStatesType": [
        "object",
    ],
    "Format": [
        "double",
        "float",
        "uint32",
        "uint64",
        "uint8",
    ],
    "CurrentLoopPtType": [
        "integer",
        "number",
    ],
    "Required": [
        "message_type",
        "payload",
    ],
};
