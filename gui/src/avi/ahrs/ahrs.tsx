import { For, createEffect, createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { Config, Sequence, State, runSequence, serverIp, StreamState, Bus, AHRS as AHRS_struct, Vector } from "../../comm";
import { enableCommand, disableCommand } from "../../commands";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [activeBoards, setActiveBoards] = createSignal();
const [ahrsData, setAhrsData] = createSignal({
  barometer: {pressure: 0, temperature: 0},
  imu: {
    accelerometer: {x: 0, y: 0, z: 0},
    gyroscope: {x: 0, y: 0, z: 0}
  },
  magnetometer: {x: 0, y: 0, z: 0},
  rail_3_3_v: {voltage: 0, current: 0},
  rail_5_v: {voltage: 0, current: 0},
} as AHRS_struct);
// listens to device updates and updates the values of AHRS values accordingly for display
listen('device_update', (event) => {
  // get sensor data
  const ahrs_object = (event.payload as StreamState).ahrs;
  console.log(event.payload);
  console.log(ahrs_object)
  setAhrsData(ahrs_object);
});

listen('state', (event) => {
  console.log(event.windowLabel);
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
});

invoke('initialize_state', {window: appWindow});

function AHRS() {
  return <div class="window-template">
  <div style="height: 60px">
    <GeneralTitleBar name="AHRS"/>
  </div>
  <div class="ahrs-view">
    <div class="ahrs-horizontal-container">
      <div class="ahrs-data-container">
        <div class="section-title" style={{"text-decoration": 'underline'}}> IMU </div>
        <div class="column-title-row">
            <div class="column-title" style={{"font-size": "16px"}}> Variables </div>
            <div class="column-title" style={{"font-size": "16px"}}> Values </div>
          </div>
          <div class="ahrs-data-row-container">
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Accelerometer: x </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).imu.accelerometer.x).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Accelerometer: y </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).imu.accelerometer.y).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Accelerometer: z </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).imu.accelerometer.z).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Gyroscope: x </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).imu.gyroscope.x).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Gyroscope: y </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).imu.gyroscope.y).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Gyroscope: z </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).imu.gyroscope.z).toFixed(4)} </div>
            </div>
          </div>
      </div>

      <div class="ahrs-data-container">
        <div class="section-title" style={{"text-decoration": 'underline'}}> Barometer </div>
        <div class="column-title-row">
            <div class="column-title" style={{"font-size": "16px"}}> Variables </div>
            <div class="column-title" style={{"font-size": "16px"}}> Values </div>
          </div>
          <div class="ahrs-data-row-container">
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Barometer: Pressure </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).barometer.pressure).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Barometer: Temperature </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).barometer.temperature).toFixed(4)} </div>
            </div>
          </div>
      </div>
    </div>

    <div class="ahrs-horizontal-container">
      <div class="ahrs-data-container">
        <div class="section-title" style={{"text-decoration": 'underline'}}> Magnetometer </div>
        <div class="column-title-row">
            <div class="column-title" style={{"font-size": "16px"}}> Variables </div>
            <div class="column-title" style={{"font-size": "16px"}}> Values </div>
          </div>
          <div class="ahrs-data-row-container">
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Magnetometer: x </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).magnetometer.x).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Magnetometer: y </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).magnetometer.y).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> Magnetometer: z </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).magnetometer.z).toFixed(4)} </div>
            </div>
          </div>
      </div>

      <div class="ahrs-data-container">
        <div class="section-title" style={{"text-decoration": 'underline'}}> Volt Rails </div>
        <div class="column-title-row">
            <div class="column-title" style={{"font-size": "16px"}}> Variables </div>
            <div class="column-title" style={{"font-size": "16px"}}> Values </div>
          </div>
          <div class="ahrs-data-row-container">
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> 5V Rail Voltage </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).rail_5_v.voltage).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> 5V Rail Current </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).rail_5_v.current).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> 3.3V Rail Voltage </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).rail_3_3_v.voltage).toFixed(4)} </div>
            </div>
            <div class="ahrs-data-row">
              <div class="ahrs-data-variable"> 3.3V Rail Current </div>
              <div class="ahrs-data-value"> {((ahrsData() as AHRS_struct).rail_3_3_v.current).toFixed(4)} </div>
            </div>
          </div>
      </div>
    </div>
    <div class="ahrs-data-container-camera">
      <div> Camera: </div>
      <button class="camera-button-en" onClick={() => enableCommand("ahrs", "camera")}>Camera Enable</button>
      <button class="camera-button-en" onClick={() => disableCommand("ahrs", "camera")} style={{"background-color": '#C53434'}}>Camera Disable</button>
    </div>
  </div>
  <div>
    <Footer/>
  </div>
</div>
}

export default AHRS;
