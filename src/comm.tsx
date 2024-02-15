import { emit, listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { createSignal } from "solid-js";
import { ACTIVITY_WARN_THRESH, DISCONNECT_ACTIVITY_THRESH, SERVER_PORT } from "./appdata";
import { appWindow } from '@tauri-apps/api/window';
import { Buffer } from 'buffer';

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

// a State object can be passed as a payload for tauri events for state management across windows
export interface State {
  selfIp: string,
  selfPort: number,
  sessionId: string,
  forwardingId: string,
  serverIp: string,
  isConnected: boolean,
  //activity: number,
  alerts: Array<Alert>,
  feedsystem: string,
  configs: Array<Config>,
  activeConfig: string,
  sequences: Array<Sequence>,
  calibrations: Map<string, number>
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
  channel_type: string,
  channel: number,
  computer: string,
  min: number,
  max: number,
  connected_threshold: number,
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

// interface representing the 'state' from the input stream
export interface StreamState {
  valve_states: object,
  sensor_readings: object,
  update_times: object
}

export interface StreamSensor {
  value: number,
  unit: string
}

// Alert object
export interface Alert {
  time: string,
  agent: string,
  message: string,
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
  // checking if connected to network
  if (activity() % 100 == 0) {
    //invoke('update_self_ip', {window: appWindow});
  }
}, 10);


// regex expression to validate ip address
const ipRegExp = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;

const hostsToCheck = ['127.0.0.1', 'Jeffs-Macbook-Pro.local', 'server-01.local', 'server-02.local']

// wrapper function to connect to the server
export async function connect(ip: string) {

  for (var i = 0; i < hostsToCheck.length; i++) {
    const reader = (await checkStream(hostsToCheck[i])) as ReadableStreamDefaultReader;
    console.log('reader: ', reader);
    if (!(reader instanceof Error) || reader instanceof SyntaxError) {
      return await afterConnect(hostsToCheck[i]);
    }
  }

  if (!ipRegExp.test(ip)) {
    return 'Invalid IP';
  }
  const reader = (await checkStream(ip)) as ReadableStreamDefaultReader;
  if (reader instanceof Error) {
    return 'Could not connect';
  } else {
    return await afterConnect(ip);
  }
}

// function to set up state after connect
export async function afterConnect(ip:string) {
  console.log('after connect!');
  var result = 'Invalid IP';
  const isIpValid = true;
  if (isIpValid) {
    // send the authentication request
    // var status = await sendAuthReq(ip, {'username': username, 'password': password});
    // if (status instanceof TypeError) {
    //   result = 'Connection refused / timeout';
    // } else if (status instanceof SyntaxError) {
    //   result = 'Unauthorized'
    // } else if (status instanceof Error) {
    //   result = 'Something went wrong'
    // } else {
      // set the session id, server ip and connection status
      emit('activity', 0);
      // setActivityExceeded(false);
      setprevConnected(true);  
      //console.log((status as AuthResponse).session_id);
      await invoke('update_session_id', {window: appWindow, value: /*(status as AuthResponse).session_id}*/ "session_id not in use"});
      await invoke('update_forwarding_id', {window: appWindow, value: "forwarding_id not in use"});
      await invoke('update_is_connected', {window: appWindow, value: true});
      await invoke('update_server_ip', {window: appWindow, value: ip});
      invoke('add_alert', {window: appWindow, 
        value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Connected to Servo"} as Alert 
      })
      result = '';

      // start forwarding session
      //var port = (await sendPort(ip, selfPort() as number)) as PortResponse;
      // if (!(port instanceof Error)) {
      //   await invoke('update_forwarding_id', {window: appWindow, value: port.target_id});
      //   setForwardingExpiration(port.seconds_to_expiration);
      //   // start forwarding renewing process in the background
      //   startRenewForwarding(ip, forwardingId() as string, (forwardingExpiration()-60)*1000);
      // }
      var configs = await getConfigs(ip);
      var configMap = new Map(Object.entries(configs));
      var configArray = Array.from(configMap, ([name, value]) => ({'id': name, 'mappings': value }));
      invoke('update_configs', {window: appWindow, value: configArray});
      const sequences = await getSequences(ip); 
      const sequenceMap = sequences as object;
      const sequenceArray = sequenceMap['sequences' as keyof typeof sequenceMap];
      invoke('update_sequences', {window: appWindow, value: sequenceArray});
      emit('open_stream', ip);
    //}
  }
  return result;
}

// function to receive configurations from server
export async function getConfigs(ip: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/mappings`);
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
export async function sendConfig(ip: string, config: Config) {
  const regex = /"(-|)([0-9]+(?:\.[0-9]+)?)"/g ;
  console.log(JSON.stringify({'configuration_id': config.id, 'mappings': config.mappings}).replace(regex, '$1$2').replace("NaN", "null"))
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/mappings`, {
      headers: new Headers({ 'Content-Type': 'application/json'}),
      method: 'POST',
      body: JSON.stringify({'configuration_id': config.id, 'mappings': config.mappings}).replace(regex, '$1$2').replace("NaN", "null"),
    });
    console.log('sent config to server:', JSON.stringify({'configuration_id': config.id, 'mappings': config.mappings}).replace(regex, '$1$2'));
    return await response.json();
  } catch(e) {
    return e;
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
    return await response.json();
  } catch(e) {
    return e;
  }
}

// function to receive sequences from the sever
export async function getSequences(ip: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/operator/sequence`);
    return await response.json();
  } catch(e) {
    return e;
  }
}

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

// function to open a stream to receive data on
export async function openStream(ip: string) {
  var firstTime = true;
  while (true) {  
    try {
      const response = await fetch(`http://${ip}:${SERVER_PORT}/data/forward`);
      console.log(response);
      const reader = response.body?.getReader();
      if (!firstTime) {
        await invoke('update_is_connected', {window: appWindow, value: true});
        invoke('add_alert', {window: appWindow, 
          value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Reconnected to Servo"} as Alert 
        });
      }
      await updateData(reader!);
      firstTime = false;
    } catch(e) {

    }
    console.log('attempting to reconnect..');
  }
}

export async function checkStream(ip: string) {
  try {
    const response = await fetch(`http://${ip}:${SERVER_PORT}/data/forward`);
    console.log(response);
    const reader = response.body?.getReader();
    return reader;
  } catch(e) {
    return e;
  }
}

// updates sensor and valve data throughout the GUI from the stream
export async function updateData(reader: ReadableStreamDefaultReader) {
  while(true) {
    try {
      const { done, value } = await reader.read();
      const data = Buffer.from(value).toString();
      var parsed_data = await JSON.parse(data) as StreamState;
      emit('device_update', parsed_data);
      emit('activity', 0);
      if (done) {
        console.log('disconnected!');
        return;
      }
    } catch (e) {
      console.log(e);
      console.log('connection severed!');
      await invoke('update_is_connected', {window: appWindow, value: false});
      invoke('add_alert', {window: appWindow, 
        value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Lost Connection to Servo"} as Alert 
      });
      break;
    }
  }
}
