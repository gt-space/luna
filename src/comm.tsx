import { emit, listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { createSignal } from "solid-js";
import { ACTIVITY_WARN_THRESH, DISCONNECT_ACTIVITY_THRESH, SERVER_PORT } from "./appdata";
import { appWindow } from '@tauri-apps/api/window';

// signals work well for updating state in the same window
export const [sessionId, setSessionId] = createSignal();
export const [serverIp, setServerIp] = createSignal();
export const [selfIp, setSelfIp] = createSignal();
export const [selfPort, setSelfPort] = createSignal();
export const [isConnected, setIsConnected] = createSignal();
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
}

// interface for the server's authentication response
interface AuthResponse {
  is_admin: boolean,
  session_id: string, 
}

// interface for the server's response to start forwarding
interface PortResponse {
  target_id: string,
  seconds_to_expiration: number,
}

// alert object
export interface Alert {
  time: string,
  agent: string,
  message: string,
}

// agent enum
export enum Agent {
  GUI = 'GUI',
  SERVO = 'SERVO',
  FC = 'FC',
} 


console.log('loaded - comm');

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
    invoke('update_self_ip', {window: appWindow});
  }
  if (activity() > DISCONNECT_ACTIVITY_THRESH && !activityExceeded()) {
    console.log('disconnected')
    invoke('update_is_connected', {window: appWindow, value: false});
    if (prevConnected()) {
      invoke('add_alert', {window: appWindow, 
        value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Disconnected from Servo"} as Alert 
      })
    } 
    setActivityExceeded(true);
  }
}, 10);


// regex expression to validate ip address
const ipRegExp = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;


// sends and authentication request to the server
export async function sendAuthReq(ip: string, authReq: object) {
  try {
    console.log(`http://${ip}:${SERVER_PORT}/auth`);
    const response = await fetch(`http://${ip}:${SERVER_PORT}/auth`, {
    headers: new Headers({ 'Content-Type': 'application/json' }),
    method: 'POST',
    body: JSON.stringify(authReq),
    });
    return await response.json() as AuthResponse;
  } catch(e) {
    return e;
  }
}

// sends port to server
export async function sendPort(ip: string, port: number) {
  try {
    console.log(port);
    console.log(`http://${ip}:${SERVER_PORT}/data/forward`);
    const response = await fetch(`http://${ip}:${SERVER_PORT}/data/forward`, {
    headers: new Headers({ 'Content-Type': 'application/json', 'Authorization': `Bearer ${sessionId() as string}` }),
    method: 'POST',
    body: JSON.stringify({'port': port}),
    });
    return await response.json() as PortResponse;
  } catch(e) {
    return e;
  }
}

// renews forwarding session
async function startRenewForwarding(ip: string, id: string, expiration: number) {
  setInterval(async () => {
    console.log(id);
    try {
      const response = await fetch(`http://${ip}:${SERVER_PORT}/data/renew-forward`, {
      headers: new Headers({ 'Content-Type': 'application/json', 'Authorization': `Bearer ${sessionId() as string}` }),
      method: 'POST',
      body: JSON.stringify({'target_id': id}),
    });
    console.log('renewed forwarding session');
    } catch (e) {
      console.log(e);      
    }
  }, expiration)
}

// starts receieving data from the backend
async function startReceievingData() {
  var buffer = new Array(2000).fill(0);
  while (true){
    console.log("receiving data:");
    await invoke('receive_data', {window: appWindow, buf: buffer}).then((data) =>
      console.log(data)      
    ).catch((e) => console.log(e));
    emit('activity', 0);
    setActivityExceeded(false);
    if (!isConnected()) {
      invoke('update_is_connected', {window: appWindow, value: true});
      invoke('add_alert', {window: appWindow, 
        value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Reconnected to Servo"} as Alert 
      })
    }
  }
}


// wrapper function to connect to the server
export async function connect(ip: string, username: string, password: string) {

  // validating the ip address
  const isIpValid = ipRegExp.test(ip);
  var result = 'Invalid IP';
  if (isIpValid) {

    // send the authentication request
    var status = await sendAuthReq(ip, {'username': username, 'password': password});
    if (status instanceof TypeError) {
      result = 'Connection refused / timeout';
    } else if (status instanceof SyntaxError) {
      result = 'Unauthorized'
    } else if (status instanceof Error) {
      result = 'Something went wrong'
    } else {

      // set the session id, server ip and connection status
      emit('activity', 0);
      setActivityExceeded(false);
      setprevConnected(true);  
      console.log((status as AuthResponse).session_id);
      await invoke('update_session_id', {window: appWindow, value: (status as AuthResponse).session_id})
      await invoke('update_is_connected', {window: appWindow, value: true});
      await invoke('update_server_ip', {window: appWindow, value: ip})
      invoke('add_alert', {window: appWindow, 
        value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Connected to Servo"} as Alert 
      })
      result = '';

      // start forwarding session
      var port = (await sendPort(ip, selfPort() as number)) as PortResponse;
      if (!(port instanceof Error)) {
        await invoke('update_forwarding_id', {window: appWindow, value: port.target_id});
        setForwardingExpiration(port.seconds_to_expiration);
        // start forwarding renewing process in the background
        startRenewForwarding(ip, forwardingId() as string, (forwardingExpiration()-60)*1000);
      }
      console.log(port.target_id);
      startReceievingData();
    }
  }
  return result;
}

