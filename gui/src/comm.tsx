import { emit, listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { createSignal } from "solid-js";
import { ACTIVITY_WARN_THRESH, DISCONNECT_ACTIVITY_THRESH, SERVER_PORT } from "./appdata";
import { appWindow } from '@tauri-apps/api/window';
import { Buffer } from 'buffer';
import { abort } from "process";

// signals work well for updating state in the same window
export const [sessionId, setSessionId] = createSignal();
export const [serverIp, setServerIp] = createSignal();
export const [selfIp, setSelfIp] = createSignal();
export const [selfPort, setSelfPort] = createSignal();
export const [isConnected, setIsConnected] = createSignal(false);
export const [activity, setActivity] = createSignal(0);
export const [alerts, setAlerts] = createSignal();
export const [forwardingId, setForwardingId] = createSignal();

const[activityExceeded, setActivityExceeded] = createSignal(false);
const[prevConnected, setprevConnected] = createSignal(false);
const[forwardingExpiration, setForwardingExpiration] = createSignal(540);
var firstTime = true;

// a State object can be passed as a payload for tauri events for state management across windows
export interface State {
  selfIp: string,
  selfPort: number,
  sessionId: string,
  forwardingId: string,
  serverIp: string,
  isConnected: boolean,
  alerts: Array<Alert>,
  feedsystem: string,
  configs: Array<Config>,
  activeConfig: string,
  sequences: Array<Sequence>,
  calibrations: Map<string, number>,
  triggers: Array<Trigger>,
  abortStages: Array<AbortStage>,
  activeAbortStage: string
}

// interface for the server's authentication response
export interface AuthResponse {
  is_admin: boolean,
  session_id: string, 
}

// interface for the server's response to start forwarding
export interface PortResponse {
  target_id: string,
  seconds_to_expiration: number,
}

// interface to represent mappings
export interface Mapping {
  text_id: string,
  board_id: string,
  sensor_type: string,
  channel: number,
  computer: string,
  min: number,
  max: number,
  powered_threshold: number,
  normally_closed: any
}

// interface to represent Configurations
export interface Config {
  id: string,
  mappings: Mapping[]
}

// interface to represent a Sequence
export interface Sequence {
  name: string,
  configuration_id: string,
  script: string
}

export interface Trigger {
  name: string,
  condition: string,
  active: boolean,
  script: string
}

// interface representing the 'state' from the input stream
export interface StreamState {
  valve_states: object,
  sensor_readings: object,
  rolling: object,
  update_times: object,
  sequences_running: Array<string>,
  bms: BMS,
  ahrs: AHRS,
  reco: [RECO | undefined, RECO | undefined, RECO | undefined],
  gps: GPS | undefined,
  abort_stage: object
}

// interface to represent a sensor from stream data
export interface StreamSensor {
  value: number,
  unit: string
}

// interface to represent bus data (voltage + current)
export interface Bus {
  voltage: number,
  current: number
}

// interface to represent BMS data
export interface BMS {
  battery_bus: Bus,
  umbilical_bus: Bus,
  sam_power_bus: Bus,
  five_volt_rail: Bus,
  charger: number,
  e_stop: number,
  rbf_tag: number
}

export interface Vector {
  x: number,
  y: number,
  z: number
}

// interface to represent IMU data (accelerometer + gyroscope)
export interface IMU {
  accelerometer: Vector,
  gyroscope: Vector
}

// interface to represent AHRS data
export interface AHRS {
  imu: IMU,
  magnetometer: Vector,
  barometer: {
    pressure: number,
    temperature: number,
  },
  rail_3_3_v: Bus,
  rail_5_v: Bus,
}

// interface to represent RECO data for one MCU
export interface RECO {
  /** Quaternion representing vehicle attitude [w, x, y, z] */
  quaternion: [number, number, number, number],
  /** Position [longitude, latitude, altitude] in degrees and meters */
  lla_pos: [number, number, number],
  /** Velocity of vehicle [north, east, down] in m/s */
  velocity: [number, number, number],
  /** Gyroscope bias offset [x, y, z] */
  g_bias: [number, number, number],
  /** Accelerometer bias offset [x, y, z] */
  a_bias: [number, number, number],
  /** Gyro scale factor [x, y, z] */
  g_sf: [number, number, number],
  /** Acceleration scale factor [x, y, z] */
  a_sf: [number, number, number],
  /** Linear acceleration [x, y, z] in m/s² */
  lin_accel: [number, number, number],
  /** Angular rates (pitch, yaw, roll) in rad/s */
  angular_rate: [number, number, number],
  /** Magnetometer data [x, y, z] */
  mag_data: [number, number, number],
  /** Temperature in Kelvin */
  temperature: number,
  /** Pressure in Pa */
  pressure: number,
  /** Stage 1 enabled flag */
  stage1_enabled: boolean,
  /** Stage 2 enabled flag */
  stage2_enabled: boolean,
  /** VREF A stage 1 flag */
  vref_a_stage1: boolean,
  /** VREF A stage 2 flag */
  vref_a_stage2: boolean,
  /** VREF B stage 1 flag */
  vref_b_stage1: boolean,
  /** VREF B stage 2 flag */
  vref_b_stage2: boolean,
  /** VREF C stage 1 flag */
  vref_c_stage1: boolean,
  /** VREF C stage 2 flag */
  vref_c_stage2: boolean,
  /** VREF D stage 1 flag */
  vref_d_stage1: boolean,
  /** VREF D stage 2 flag */
  vref_d_stage2: boolean,
  /** VREF E stage 1-1 flag */
  vref_e_stage1_1: boolean,
  /** VREF E stage 1-2 flag */
  vref_e_stage1_2: boolean,
}

// interface to represent GPS data
export interface GPS {
  latitude_deg: number,
  longitude_deg: number,
  altitude_m: number,
  north_mps: number,
  east_mps: number,
  down_mps: number,
  timestamp_unix_ms: number | null,
  has_fix: boolean,
}

// Alert object
export interface Alert {
  time: string,
  agent: string,
  message: string,
}

// interface to represent mappings
export interface AbortStageMapping {
  valve_name: string,
  abort_stage: any,
  timer_to_abort: number
}

// interface to represent Configurations
export interface AbortStage {
  id: string,
  abort_condition: string,
  mappings: AbortStageMapping[]
}

// Agent enum
export enum Agent {
  GUI = 'GUI',
  SERVO = 'SERVO',
  FC = 'FC',
}

// on load initialize state and set local signals
console.log('loaded - comm');
invoke('initialize-state', {window: appWindow});
listen('state', (event) => {
  setServerIp((event.payload as State).serverIp);
  setIsConnected((event.payload as State).isConnected);
  setSessionId((event.payload as State).sessionId);
  setForwardingId((event.payload as State).forwardingId);
  setSelfIp((event.payload as State).selfIp);
  setSelfPort((event.payload as State).selfPort);
});

// clock for activity  
setInterval(() =>{
  setActivity(activity() as number + 10);
  if (document.getElementById('activity') != null) {
    document.getElementById('activity')!.style.color = activity() < ACTIVITY_WARN_THRESH? '#1DB55A':'#C53434';
  }
  if (document.getElementById('status') != null) {
    document.getElementById('status')!.style.color = isConnected()? '#1DB55A':'#C53434';
  }
}, 10);


// regex expression to validate ip address
const ipRegExp = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;

// list of hosts to check when connecting
const hostsToCheck = ['127.0.0.1', 'server-01.local', 'server-02.local']

// wrapper function to connect to the server
export async function connect(ip: string) {

  if (ip.length != 0) {
    const response = await getConfigs(ip);
    if (response instanceof Error) {
      return 'Could not connect';
    } else {
      return await afterConnect(ip);
    }
  } else {
    for (var i = 0; i < hostsToCheck.length; i++) {
      const response = await getConfigs(hostsToCheck[i]);
      console.log('response', response);
      if (!(response instanceof Error) || response instanceof SyntaxError) {
        return await afterConnect(hostsToCheck[i]);
      }
    }
  }
}

// function to set up state after connect
export async function afterConnect(ip:string) {
  console.log('after connect!');
  var result = 'Invalid IP';
  const isIpValid = true;
  if (isIpValid) {
    emit('activity', 0);
    setprevConnected(true);  
    //update state
    await invoke('update_session_id', {window: appWindow, value: /*(status as AuthResponse).session_id}*/ "session_id not in use"});
    await invoke('update_forwarding_id', {window: appWindow, value: "forwarding_id not in use"});
    await invoke('update_is_connected', {window: appWindow, value: true});
    await invoke('update_server_ip', {window: appWindow, value: ip});
    invoke('add_alert', {window: appWindow, 
      value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Connected to Servo"} as Alert 
    })
    result = '';
    var configs = await getConfigs(ip);
    var configMap = new Map(Object.entries(configs));
    var configArray = Array.from(configMap, ([name, value]) => ({'id': name, 'mappings': value }));
    invoke('update_configs', {window: appWindow, value: configArray});
    var abortStages = await getAbortStages(ip);
    const stages = (abortStages as { stages: Array<{ stage_name: string, abort_condition: string, valve_safe_states: Record<string, { desired_state: string, safing_timer: number }> }> }).stages;
    const abortStageArray = stages.map(stage => {
      const mappings: AbortStageMapping[] = Object.entries(stage.valve_safe_states).map(([valve_name, valveState]) => ({
        valve_name: valve_name,
        abort_stage: valveState.desired_state,
        timer_to_abort: valveState.safing_timer
      }));
      return {
        id: stage.stage_name,
        abort_condition: stage.abort_condition,
        mappings: mappings
      } as AbortStage;
    });
    invoke('update_abort_stages', {window: appWindow, value: abortStageArray});
    const sequences = await getSequences(ip); 
    const sequenceMap = sequences as object;
    const sequenceArray = sequenceMap['sequences' as keyof typeof sequenceMap];
    invoke('update_sequences', {window: appWindow, value: sequenceArray});
    const triggers = (await getTriggers(ip)) as Array<Trigger>;
    invoke('update_triggers', {window: appWindow, value: triggers});
    emit('open_stream', ip);
  }
  return result;
}

// function to receive configurations from server
export async function getConfigs(ip: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/mappings`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
    });
    return await response.json();
  } catch(e) {
    return e;
  }
} 

// function to send the currently active config to server
export async function sendActiveConfig(ip: string, config: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/active-configuration`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'POST',
      body: JSON.stringify({'configuration_id': config}),
    });
    console.log('sent active config to server');
    return await response.json();
  } catch(e) {
    return e;
  }
}

// sends a new or updated config to server
export async function sendConfig(ip: string, config: Config): Promise<Response> {
  const regex = /"(-|)([0-9]+(?:\.[0-9]+)?)"/g ;
  //console.log(JSON.stringify({'configuration_id': config.id, 'mappings': config.mappings}).replace(regex, '$1$2').replace("NaN", "null"))
  const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/mappings`, {
    headers: new Headers({ 'Content-Type': 'application/json'}),
    method: 'POST',
    body: JSON.stringify({'configuration_id': config.id, 'mappings': config.mappings}).replace(regex, '$1$2').replace("NaN", "null"),
  });
  console.log('sent config to server:', JSON.stringify({'configuration_id': config.id, 'mappings': config.mappings}).replace(regex, '$1$2'));
  return response;
}

// deletes a config from the server
export async function deleteConfig(ip: string, configId: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/mappings`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'DELETE',
      body: JSON.stringify({'configuration_id': configId}),
    });
    console.log('deleted config from server');
    return response;
  } catch (e) {
    return e;
  }
}

// function to receive abort stages from server
export async function getAbortStages(ip: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/abort-config`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
    });
    return await response.json();
  } catch(e) {
    return e;
  }
} 

// function to send the currently active abort stage to server
export async function sendActiveAbortStage(ip: string, abortStage: string) {
  // try {
  //   const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/active-abort-stage`, {
  //     headers: new Headers({ 'Content-Type': 'application/json'}),
  //     method: 'POST',
  //     body: JSON.stringify({'stage_name': abortStage}),
  //   });
  //   console.log('sent active abort stage to server');
  //   return await response.json();
  // } catch(e) {
  //   return e;
  // }
  return Promise.resolve(new Response().json());
}

// sends a new or updated abort stage to server
export async function sendAbortStage(ip: string, abortStage: AbortStage): Promise<Response> {
  const regex = /"(-|)([0-9]+(?:\.[0-9]+)?)"/g ;
  
  // transforms mappings into valve_safe_states hashmap
  const valveSafeStates: Record<string, { desired_state: string, safing_timer: number }> = {};
  for (const mapping of abortStage.mappings) {
    if (mapping.valve_name && mapping.abort_stage !== null && !isNaN(mapping.timer_to_abort)) {
      valveSafeStates[mapping.valve_name] = {
        desired_state: mapping.abort_stage, // "open" or "closed"
        safing_timer: mapping.timer_to_abort
      };
    }
  }
  
  const requestBody = {
    stage_name: abortStage.id,
    abort_condition: abortStage.abort_condition,
    valve_safe_states: valveSafeStates
  };
  
  const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/abort-config`, {
    headers: new Headers({ 'Content-Type': 'application/json'}),
    method: 'PUT',
    body: JSON.stringify(requestBody).replace(regex, '$1$2').replace("NaN", "null"),
  });
  console.log('sent abort stage to server:', JSON.stringify(requestBody).replace(regex, '$1$2'));
  return response;
}

// deletes an abort stage from the server
export async function deleteAbortStage(ip: string, abortStageId: string): Promise<Response> {
  try {
    console.log('abortStageId:', abortStageId);
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/abort-config`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'DELETE',
      body: JSON.stringify({'name': abortStageId}),
    });
    console.log('deleted abort stage from server');
    return response;
  } catch (e) {
    console.log('error deleting abort stage:', e);
    throw e;
  }
}

// function to run an abort stage
export async function runAbortStage(ip: string, name: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/set-stage`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'PUT',
      body: JSON.stringify({'stage_name': name}),
    });
    console.log('sent abort stage to server to run');

    if (!response.ok) {
      console.log("http fail");
      return { success: false, error: `HTTP ${response.status}`, body: await response.text() };
    }

    console.log("success");
    
    return { success: true, data: response };
  } catch (e) {
    console.log("didn't reach network");
    return { success: false, error: e };
  }
}

// sends a sequence to the server
export async function sendSequence(ip: string, name: string, sequence: string, config: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/sequence`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'PUT',
      body: JSON.stringify({
        'name': name,
        'configuration_id': config,
        'script': sequence
      }),
    });
    console.log('sent sequence to server');
    return response;
  } catch(e) {
    return e;
  }
}

// function to receive sequences from the sever
export async function getSequences(ip: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/sequence`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
    });
    return await response.json();
  } catch(e) {
    return e;
  }
}

// function to run a sequence
export async function runSequence(ip: string, name: string, override: boolean) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/run-sequence`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'POST',
      body: JSON.stringify({
        'name': name,
        'force': override
      }),
    });
    console.log('sent sequence to server to run');
    return await response.json();
  } catch(e) {
    return e;
  }
}

// function to get triggers
export async function getTriggers(ip: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/trigger`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
    });
    return await response.json();
  } catch(e) {
    return e;
  }
}

// function to send a trigger
export async function sendTrigger(ip: string, name: string, trigger: string, condition: string, active: boolean) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/trigger`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'PUT',
      body: JSON.stringify({
        'name': name,
        'condition': condition,
        'script': trigger,
        'active': active
      }),
    });
    console.log('sent trigger to server');
    return response;
  } catch(e) {
    return e;
  }
}

// function to calibrate sensors
export async function sendCalibrate(ip: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/calibrate`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'POST',
    });
    console.log('sent calibration command');
    return await response.json();
  } catch(e) {
    return e;
  }
}

// function to send system-wide abort
export async function sendAbort(ip: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/abort`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'POST',
    });
    console.log('sent abort command');
    return await response.json();
  } catch(e) {
    return e;
  }
}

// function to stop an individual sequence
export async function stopSequence(ip: string, name: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/stop-sequence`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'POST',
      body: JSON.stringify({
        'name': name
      }),
    });
    console.log('sent command to stop sequence');
    return await response.json();
  } catch(e) {
    return e;
  }
}


// function to open a stream to receive data on
export async function openStream(ip: string) {
  try {
    const socket = new WebSocket(`ws://${ip}:${SERVER_PORT}/data/forward`);
    socket.onopen = async (event) => {
      if (!firstTime) {
        await invoke('update_is_connected', {window: appWindow, value: true});
        invoke('add_alert', {window: appWindow, 
          value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Reconnected to Servo"} as Alert 
        });
      }
      firstTime = false;
    }
    socket.onmessage = async (event) => {
      try {
        const data = event.data.toString();
        const parsed_data = await JSON.parse(data) as StreamState;
        // console.log(parsed_data);
        emit('device_update', parsed_data);
        emit('activity', 0);
      } catch (e) {
        console.log('could not parse data or equivalent:', e);
      }
    };
    socket.onclose = async (event) => {
      console.log('closed:', event.wasClean, event);
      await invoke('update_is_connected', {window: appWindow, value: false});
      if (!event.wasClean) {
        invoke('add_alert', {window: appWindow, 
          value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Attempting to reconnect..."} as Alert 
        });
        socket.close();
        console.log('connection lost. attempting to reconnect..');
        emit('open_stream', ip);
      }
    };
    // socket.onerror = async (event) => {
    //   console.log('closed with error:', event);
    //   await invoke('update_is_connected', {window: appWindow, value: false});
    //   invoke('add_alert', {window: appWindow, 
    //     value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Lost Connection to Servo"} as Alert 
    //   });
    //   socket.close();
    // }
  } catch(e) {
    console.log("couldn't open socket!");
    console.log('attempting to reconnect..');
    emit('open_stream', ip);
  }
}
