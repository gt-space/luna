import { For, createEffect, createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { Config, Sequence, State, runSequence, serverIp, StreamState, Bus, FcSensors as FcSensors_struct, Vector } from "../../comm";
import { enableCommand, disableCommand } from "../../commands";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [activeBoards, setActiveBoards] = createSignal();
const [fcSensorsData, setFcSensorsData] = createSignal({
  barometer: { pressure: 0, temperature: 0 },
  imu: {
    accelerometer: { x: 0, y: 0, z: 0 },
    gyroscope: { x: 0, y: 0, z: 0 }
  },
  magnetometer: { x: 0, y: 0, z: 0 },
  rail_3v3: { voltage: 0, current: 0 },
  rail_5v: { voltage: 0, current: 0 },
} as FcSensors_struct);

listen('device_update', (event) => {
  const fc_sensors_object = (event.payload as StreamState).fc_sensors;
  console.log(event.payload);
  console.log(fc_sensors_object)
  setFcSensorsData(fc_sensors_object);
});

listen('state', (event) => {
  console.log(event.windowLabel);
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
});

invoke('initialize_state', { window: appWindow });

function FcSensors() {
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="FC Sensors" />
    </div>
    <div class="fc-sensors-view">
      <div class="fc-sensors-horizontal-container">
        <div class="fc-sensors-data-container">
          <div class="section-title" style={{ "text-decoration": 'underline' }}> IMU </div>
          <div class="column-title-row">
            <div class="column-title" style={{ "font-size": "16px" }}> Variables </div>
            <div class="column-title" style={{ "font-size": "16px" }}> Values </div>
          </div>
          <div class="fc-sensors-data-row-container">
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Accelerometer: x </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.accelerometer.x).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Accelerometer: y </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.accelerometer.y).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Accelerometer: z </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.accelerometer.z).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Gyroscope: x </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.gyroscope.x).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Gyroscope: y </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.gyroscope.y).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Gyroscope: z </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.gyroscope.z).toFixed(4)} </div>
            </div>
          </div>
        </div>

        <div class="fc-sensors-data-container">
          <div class="section-title" style={{ "text-decoration": 'underline' }}> Barometer </div>
          <div class="column-title-row">
            <div class="column-title" style={{ "font-size": "16px" }}> Variables </div>
            <div class="column-title" style={{ "font-size": "16px" }}> Values </div>
          </div>
          <div class="fc-sensors-data-row-container">
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Barometer: Pressure </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).barometer.pressure).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Barometer: Temperature </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).barometer.temperature).toFixed(4)} </div>
            </div>
          </div>
        </div>
      </div>

      <div class="fc-sensors-horizontal-container">
        <div class="fc-sensors-data-container">
          <div class="section-title" style={{ "text-decoration": 'underline' }}> Magnetometer </div>
          <div class="column-title-row">
            <div class="column-title" style={{ "font-size": "16px" }}> Variables </div>
            <div class="column-title" style={{ "font-size": "16px" }}> Values </div>
          </div>
          <div class="fc-sensors-data-row-container">
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Magnetometer: x </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).magnetometer.x).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Magnetometer: y </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).magnetometer.y).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Magnetometer: z </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).magnetometer.z).toFixed(4)} </div>
            </div>
          </div>
        </div>

        <div class="fc-sensors-data-container">
          <div class="section-title" style={{ "text-decoration": 'underline' }}> Volt Rails </div>
          <div class="column-title-row">
            <div class="column-title" style={{ "font-size": "16px" }}> Variables </div>
            <div class="column-title" style={{ "font-size": "16px" }}> Values </div>
          </div>
          <div class="fc-sensors-data-row-container">
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> 5V Rail Voltage </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).rail_5v.voltage).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> 5V Rail Current </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).rail_5v.current).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> 3.3V Rail Voltage </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).rail_3v3.voltage).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> 3.3V Rail Current </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).rail_3v3.current).toFixed(4)} </div>
            </div>
          </div>
        </div>
      </div>
      <div class="fc-sensors-data-container-camera">
        <div> Camera: </div>
        <button class="camera-button-en" onClick={() => enableCommand("fc_sensors", "camera")}>Camera Enable</button>
        <button class="camera-button-en" onClick={() => disableCommand("fc_sensors", "camera")} style={{ "background-color": '#C53434' }}>Camera Disable</button>
      </div>
    </div>
    <div>
      <Footer />
    </div>
  </div>
}

export default FcSensors;
