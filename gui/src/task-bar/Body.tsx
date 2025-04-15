import { Component, createSignal, For} from "solid-js";
import Scrollbars from 'solid-custom-scrollbars'
import { invoke } from '@tauri-apps/api/tauri'
import { emit, listen } from "@tauri-apps/api/event";
import { Alert, alerts, StreamState } from "../comm";
// import { DISCONNECT_ACTIVITY_THRESH } from "../appdata";

const [devices, setDevices] = createSignal<{ 
  name: string; 
  lastUpdate: number; 
  lastChangedAt:number; 
  lastChange: number; 
  isConnected: boolean }[]>([]);
const DISCONNECT_THRESH = 5;

listen('device_update', (event) => {
  // console.log(event.payload)
  // const connected_devices = (event.payload as StreamState).rolling;
  // const deviceEntries = Object.entries(connected_devices).map(([name, data]: [string, any]) => {
  //   const lastUpdate = data.time_since_last_update;
  //   const isConnected = lastUpdate <= 5;
  //   return { name, lastUpdate, isConnected };
  // });

  // setDevices(deviceEntries);
  const connected_devices = (event.payload as StreamState).rolling;
  const currentDevices = devices(); 
  const now = Date.now();

  const deviceEntries = Object.entries(connected_devices).map(([name, data]: [string, any]) => {
    const newLastUpdate = data.time_since_last_update;

    const existing = currentDevices.find(d => d.name === name);
    const lastChangedAt = (existing && existing.lastUpdate !== newLastUpdate)
      ? now
      : existing?.lastChangedAt ?? now;

    const lastChange = (now - lastChangedAt) / 1000; // in seconds

    const isConnected = newLastUpdate < DISCONNECT_THRESH && lastChange < DISCONNECT_THRESH;

    return {
      name,
      lastUpdate: newLastUpdate,
      lastChangedAt,
      lastChange,
      isConnected
    };
  });

  setDevices(deviceEntries);
  console.log(deviceEntries)
});


const Body: Component = (props) => {
  return <div class="taskbar-body">
    <div class="taskbar-body-item">
      System Overview
    </div>
    <div class="taskbar-body-item">
      Alerts
    </div>
    <div class="taskbar-body-item">
      <div class="scrollable-container">
      <For each={devices().filter(d => d.isConnected)}>{(device, i) =>
        <div>
          [{device.name}]: <span style={{ color: '#1DB55A' }}> CONNECTED </span>
          (last update: {device.lastUpdate.toFixed(5)})
        </div>
      }</For>
      </div>
    </div>
    <div class="taskbar-body-item">
      <div class="scrollable-container">
        <Scrollbars>
          <For each={alerts() as Alert[]}>{(alert, i) =>
            <div>
              {`[${alert.time}] [${alert.agent}]: ${alert.message}`}
              {i() == 0 ? <div style={"height: 5px"}></div>:<div></div>}            
            </div>
          }</For>
        </Scrollbars> 
      </div>
    </div>
  </div>
}

export default Body;