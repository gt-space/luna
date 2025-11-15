import { createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { State, BMS as BMS_struct, Bus, sendCameraAction, sendArmLugsAction, sendDetonateLugsAction } from "../../comm";


const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [activeBoards, setActiveBoards] = createSignal();
const [serverIp, setServerIp] = createSignal("");
const [bmsData, setBmsData] = createSignal({
  battery_bus: {voltage: 0, current: 0} as Bus,
  umbilical_bus: {voltage: 0, current: 0} as Bus,
  sam_power_bus: {voltage: 0, current: 0} as Bus,
  five_volt_rail: {voltage: 0, current: 0} as Bus,
  charger: 0,
  e_stop: 0,
  rbf_tag: 0
} as BMS_struct);

listen('state', (event) => {
  console.log(event.windowLabel);
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
  setServerIp((event.payload as State).serverIp);
  console.log(serverIp());
});

invoke('initialize_state', {window: appWindow});

function SAM() {
    const label = appWindow.label.toLowerCase();
    const isFlightSam = label.startsWith("sam-2") || label.startsWith("sam-3");

    return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name={appWindow.label}/>
    </div>
    <div class="sam-view">
      {isFlightSam && (
      <div class="sam-section-en" id="enable">
          <div class="section-title"> ENABLE </div>
          <button class="sam-button-en" onClick={() => sendCameraAction(serverIp(), true)}> CAMERA </button>
          <button class="sam-button-en" onClick={() => sendArmLugsAction(serverIp(), true)}> LAUNCH LUG ARM </button>
          <button class="sam-button-en" onClick={() => sendDetonateLugsAction(serverIp(), true)}> LAUNCH LUG DETONATE </button>
      </div>
      <div class="sam-section-en" id="disable">
          <div class="section-title"> DISABLE </div>
          <button class="sam-button-en" style={{"background-color": '#C53434'}} onClick={() => sendCameraAction(serverIp(), false)}> CAMERA </button>
          <button class="sam-button-en" style={{"background-color": '#C53434'}} onClick={() => sendArmLugsAction(serverIp(), false)}> LAUNCH LUG DISARM </button>
          <button class="sam-button-en" style={{"background-color": '#C53434'}} onClick={() => sendDetonateLugsAction(serverIp(), false)}> LAUNCH LUG DE-DETONATE </button>
      </div>
      )}
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default SAM;