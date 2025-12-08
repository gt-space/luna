import { createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { AHRS as AHRS_struct } from "../../comm";
import { enableCommand, disableCommand } from "../../commands";

const [telData, setTelData] = createSignal({
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
});

invoke('initialize_state', {window: appWindow});

function TEL() {
  return <div class="window-template">
  <div style="height: 60px">
    <GeneralTitleBar name="TEL"/>
  </div>
  <div class="tel-view">
    <div class="tel-data-container-button">
      <button class="tel-button-en" onClick={() => enableCommand("tel", "camera")}>Enable/Disable TEL Link</button>
      <button class="tel-button-en" onClick={() => disableCommand("tel", "camera")} style={{"background-color": '#346cc5ff'}}>Switch to Umbilical/TEL</button>
    </div>
    <div class="tel-data-container">
      <div class="column-title-row">
        <div class="column-title" style={{"font-size": "16px"}}> Variables </div>
        <div class="column-title" style={{"font-size": "16px"}}> Values </div>
      </div>
      <div class="tel-data-row-container">
        <div class="tel-data-row">
          <div class="tel-data-variable"> Signal Strength (RSSI): </div>
          <div class="tel-data-value"> 0.0000 </div>
        </div>
          <div class="tel-data-row">
          <div class="tel-data-variable"> Packets Loss: </div>
          <div class="tel-data-value"> 0.0% </div>
        </div>
          <div class="tel-data-row">
          <div class="tel-data-variable"> Packet Errors: </div>
          <div class="tel-data-value"> 0 </div>
        </div>
        <div class="tel-data-row">
          <div class="tel-data-variable"> Packets Received: </div>
          <div class="tel-data-value"> 0 </div>
        </div>
        <div class="tel-data-row">
          <div class="tel-data-variable"> Packets Dropped: </div>
          <div class="tel-data-value"> 0 </div>
        </div>
        <div class="tel-data-row">
          <div class="tel-data-variable"> 2.4 GHz PA Temp: </div>
          <div class="tel-data-value"> 0.0 °C </div>
        </div>
        <div class="tel-data-row">
          <div class="tel-data-variable"> 915 MHz PA Temp: </div>
          <div class="tel-data-value"> 0.0 °C </div>
        </div>
        <div class="tel-data-row">
          <div class="tel-data-variable"> 12V: </div>
          <div class="tel-data-row-multiple-values">
            <div class="tel-data-value"> 0.00 V </div>
            <div class="tel-data-value"> 0.00 A </div>
          </div>
        </div>
        <div class="tel-data-row">
          <div class="tel-data-variable"> 5V: </div>
          <div class="tel-data-row-multiple-values">
            <div class="tel-data-value"> 0.00 V </div>
            <div class="tel-data-value"> 0.00 A </div>
          </div>
        </div>
        <div class="tel-data-row">
          <div class="tel-data-variable"> 3.3V: </div>
          <div class="tel-data-row-multiple-values">
            <div class="tel-data-value"> 0.00 V </div>
            <div class="tel-data-value"> 0.00 A </div>
          </div>
        </div>
      </div>
    </div>
  </div>
  <div>
    <Footer/>
  </div>
</div>
}

export default TEL;
