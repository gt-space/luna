import { For, createEffect, createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { Config, Sequence, State, runSequence, serverIp, StreamState } from "../../comm";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [activeBoards, setActiveBoards] = createSignal();

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
      <div class="ahrs-camera-buttons">
        <button class="camera-button-en">Camera Enable</button>
        <button class="camera-button-en" style={{"background-color": '#C53434'}}>Camera Disable</button>
      </div>
      <div class="ahrs-view-container">
        <div class="ahrs-view-left">
          <div class="ahrs-data-container">
            <div class="section-title" style={{"text-decoration": 'underline'}}> Data Display </div>
            <div class="column-title-row">
                <div class="column-title" style={{"font-size": "16px"}}> Variables </div>
                <div class="column-title" style={{"font-size": "16px"}}> Values </div>
              </div>
              {/* Change to iteratively display ahrs data variables and values once backend array is implemented */}
              <div class="ahrs-data-row-container">
                <div class="ahrs-data-row">
                  <div class="ahrs-data-variable"> Variable 1 </div>
                  <div class="ahrs-data-value"> Value 1 </div>
                </div>
                <div class="ahrs-data-row">
                  <div class="ahrs-data-variable"> Variable 2 </div>
                  <div class="ahrs-data-value"> Value 2 </div>
                </div>
                <div class="ahrs-data-row">
                  <div class="ahrs-data-variable"> Variable 3 </div>
                  <div class="ahrs-data-value"> Value 3 </div>
                </div>
                <div class="ahrs-data-row">
                  <div class="ahrs-data-variable"> Variable 4 </div>
                  <div class="ahrs-data-value"> Value 4 </div>
                </div>
                <div class="ahrs-data-row">
                  <div class="ahrs-data-variable"> Variable 5 </div>
                  <div class="ahrs-data-value"> Value 5 </div>
                </div>
              </div>
          </div>
          <div class="imu-container">
            <div class="section-title" style={{"text-decoration": 'underline'}}> IMU </div>
          </div>
        </div>
        <div class="ahrs-view-right">
          <div class="baro-container">
            <div class="section-title" style={{"text-decoration": 'underline'}}> Barometer </div>   
          </div>
          <div class="mag-container">
            <div class="section-title" style={{"text-decoration": 'underline'}}> Magnetometer </div>
          </div>
        </div>
      </div>
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default AHRS;