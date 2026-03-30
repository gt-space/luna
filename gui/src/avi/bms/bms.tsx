import { createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { State, StreamState, BMS as BMS_struct, Bus } from "../../comm";
import { enableCommand, disableCommand } from "../../commands";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [activeBoards, setActiveBoards] = createSignal();
const [bmsData, setBmsData] = createSignal({
  battery_bus: {voltage: 0, current: 0} as Bus,
  umbilical_bus: {voltage: 0, current: 0} as Bus,
  sam_power_bus: {voltage: 0, current: 0} as Bus,
  ethernet_bus: {voltage: 0, current: 0} as Bus,
  tel_bus: {voltage: 0, current: 0} as Bus,
  fcb_bus: {voltage: 0, current: 0} as Bus,
  five_volt_rail: {voltage: 0, current: 0} as Bus,
  charger: 0,
  chassis: 0,
  e_stop: 0,
  rbf_tag: 0,
  reco_load_switch_1: 0,
  reco_load_switch_2: 0,
} as BMS_struct);

// listens to device updates and updates the values of BMS values accordingly for display
listen('device_update', (event) => {
  // get sensor data
  const bms_object = (event.payload as StreamState).bms;
  console.log(bms_object)
  setBmsData(bms_object);
});


listen('state', (event) => {
  console.log(event.windowLabel);
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
});

invoke('initialize_state', {window: appWindow});

function BMS() {
  const d = () => bmsData() as BMS_struct;

  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="BMS"/>
    </div>
    <div class="bms-view">
      <div class="bms-section-en" id="enable">
          <div class="section-title"> ENABLE </div>
          <button class="bms-button-en" onClick={() => enableCommand("bms", "battery_ls")}> BATTERY POWER </button>
          <button class="bms-button-en" onClick={() => enableCommand("bms", "charge")}> BATTERY CHARGER </button>
          <button class="bms-button-en" onClick={() => enableCommand("bms", "sam_ls")}> SAM POWER </button>
          <button class="bms-button-en" onClick={() => enableCommand("bms", "tel_ls")}> TEL POWER </button>
          <button class="bms-button-en" onClick={() => enableCommand("bms", "estop")}> ESTOP RESET </button>
      </div>
      <div class="bms-section-en" id="disable">
          <div class="section-title"> DISABLE </div>
          <button class="bms-button-en" style={{"background-color": '#C53434'}} onClick={() => disableCommand("bms", "battery_ls")}> BATTERY POWER </button>
          <button class="bms-button-en" style={{"background-color": '#C53434'}} onClick={() => disableCommand("bms", "charge")}> BATTERY CHARGER </button>
          {/* <button class="bms-button-en" style={{"background-color": '#C53434'}} onClick={() => disableCommand("bms", "estop")}> EStop R </button> */}
          <button class="bms-button-en" style={{"background-color": '#C53434'}} onClick={() => disableCommand("bms", "sam_ls")}> SAM POWER </button>
          <button class="bms-button-en" style={{"background-color": '#C53434'}} onClick={() => disableCommand("bms", "tel_ls")}> TEL POWER </button>
      </div>
      <div class="bms-section" id="data">
          <div class="section-title"> DATA DISPLAY </div>
            {/* DATA content here */}
            <div class="adc-data-section">
              <div class="section-title" style={{"text-decoration": 'underline'}}> ADC Data </div>
              <div class="bms-data-groups">
                <div class="bms-data-group">
                  <div class="bms-data-group-title">Battery</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Bus current</div>
                    <div class="adc-data-value">{(d().battery_bus as Bus).current.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Bus voltage</div>
                    <div class="adc-data-value">{(d().battery_bus as Bus).voltage.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Charger</div>
                    <div class="adc-data-value">{d().charger.toFixed(4)}</div>
                  </div>
                </div>
                <div class="bms-data-group">
                  <div class="bms-data-group-title">UMB bus</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Current</div>
                    <div class="adc-data-value">{(d().umbilical_bus as Bus).current.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Voltage</div>
                    <div class="adc-data-value">{(d().umbilical_bus as Bus).voltage.toFixed(4)}</div>
                  </div>
                </div>
                <div class="bms-data-group">
                  <div class="bms-data-group-title">5V rail</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Current</div>
                    <div class="adc-data-value">{(d().five_volt_rail as Bus).current.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Voltage</div>
                    <div class="adc-data-value">{(d().five_volt_rail as Bus).voltage.toFixed(4)}</div>
                  </div>
                </div>
                <div class="bms-data-group">
                  <div class="bms-data-group-title">RECO</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Load switch 1</div>
                    <div class="adc-data-value">{d().reco_load_switch_1.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Load switch 2</div>
                    <div class="adc-data-value">{d().reco_load_switch_2.toFixed(4)}</div>
                  </div>
                </div>
                <div class="bms-data-group">
                  <div class="bms-data-group-title">SAM power bus</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Current</div>
                    <div class="adc-data-value">{(d().sam_power_bus as Bus).current.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Voltage</div>
                    <div class="adc-data-value">{(d().sam_power_bus as Bus).voltage.toFixed(4)}</div>
                  </div>
                </div>
                <div class="bms-data-group">
                  <div class="bms-data-group-title">FC bus</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Current</div>
                    <div class="adc-data-value">{(d().fcb_bus as Bus).current.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Voltage</div>
                    <div class="adc-data-value">{(d().fcb_bus as Bus).voltage.toFixed(4)}</div>
                  </div>
                </div>
                <div class="bms-data-group">
                  <div class="bms-data-group-title">TEL bus</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Current</div>
                    <div class="adc-data-value">{(d().tel_bus as Bus).current.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Voltage</div>
                    <div class="adc-data-value">{(d().tel_bus as Bus).voltage.toFixed(4)}</div>
                  </div>
                </div>
                <div class="bms-data-group">
                  <div class="bms-data-group-title">Ethernet bus</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Current</div>
                    <div class="adc-data-value">{(d().ethernet_bus as Bus).current.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Voltage</div>
                    <div class="adc-data-value">{(d().ethernet_bus as Bus).voltage.toFixed(4)}</div>
                  </div>
                </div>
                <div class="bms-data-group">
                  <div class="bms-data-group-title">ESTOP / RBF</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">E-stop</div>
                    <div class="adc-data-value">{d().e_stop.toFixed(4)}</div>
                  </div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">RBF tag</div>
                    <div class="adc-data-value">{d().rbf_tag.toFixed(4)}</div>
                  </div>
                </div>
                <div class="bms-data-group">
                  <div class="bms-data-group-title">Miscellaneous</div>
                  <div class="adc-data-row">
                    <div class="adc-data-variable">Chassis</div>
                    <div class="adc-data-value">{d().chassis.toFixed(4)}</div>
                  </div>
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

export default BMS;