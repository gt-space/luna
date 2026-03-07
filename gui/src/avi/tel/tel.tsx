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

const [isUmbilicalActive, setIsUmbilicalActive] = createSignal(true);


// listens to device updates and updates the values of AHRS values accordingly for display
listen('device_update', (event) => {
  // get sensor data
});

invoke('initialize_state', {window: appWindow});

function switchTelSource(activeUmbilical: boolean) {
  // switch tel source here
  if (activeUmbilical != isUmbilicalActive()){
    setIsUmbilicalActive(activeUmbilical);
  }
}

function TEL() {
  return <div class="window-template">
  <div style="height: 60px">
    <GeneralTitleBar name="Telemetry"/>
  </div>
  <div class="tel-view">
    <div class="tel-data-container-tel-source">
      <div class="tel-source-title"> Telemetry Source </div>
      <div class="tel-button-row">
        <div class="tel-button-wrapper">
          <button class="tel-button-en" onClick={() => switchTelSource(true)} style={{"background-color": isUmbilicalActive() ? '#22873D' : '#C53434'}}>Umbilical</button>
          <div style={{"color": isUmbilicalActive() ? '#1ce852' : 'transparent'}}> Active </div>
        </div>
        <div  class="tel-button-wrapper">
          <button class="tel-button-en" onClick={() => switchTelSource(false)} style={{"background-color": !isUmbilicalActive() ? '#22873D' : '#C53434'}}>Radio</button>
          <div style={{"color": !isUmbilicalActive() ? '#1ce852' : 'transparent'}}> Active </div>
        </div>
      </div>
    </div>
    <div class="tel-data-container-row">
      {telDataContainer(true)}
      {telDataContainer(false)}
    </div>
  </div>
  <div>
    <Footer/>
  </div>
</div>
}

function telDataContainer(isUmbilicalStats: boolean) {
  const title = isUmbilicalStats ? "Umbilical Statistics" : "Tel Statistics";
  return <div class="tel-data-container">
      <div class="column-title-row">
        <div class="column-title" style={{"font-size": "16px"}}> {title} </div>
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
      </div>
    </div>;
}

export default TEL;
