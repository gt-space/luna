import { For, createEffect, createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { Config, Sequence, State, runSequence, serverIp, StreamState, BMS as BMS_struct, Bus } from "../../comm";
import { Valve } from "../../devices";
import { enableCommand, disableCommand } from "../../commands";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [activeBoards, setActiveBoards] = createSignal();
const [bmsData, setBmsData] = createSignal({
  battery_bus: {voltage: 0, current: 0} as Bus,
  umbilical_bus: {voltage: 0, current: 0} as Bus,
  sam_power_bus: {voltage: 0, current: 0} as Bus,
  five_volt_rail: {voltage: 0, current: 0} as Bus,
  charger: 0,
  e_stop: 0,
  rbf_tag: 0
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
    return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="BMS"/>
    </div>
    <div class="bms-view">
      <div class="bms-section-en" id="enable">
          <div class="section-title"> ENABLE </div>
          <button class="bms-button-en" onClick={() => enableCommand("bms", "battery_ls")}> BMS </button>
          <button class="bms-button-en" onClick={() => enableCommand("bms", "charge")}> Battery </button>
          <button class="bms-button-en" onClick={() => enableCommand("bms", "estop")}> EStop R </button>
          <button class="bms-button-en" onClick={() => enableCommand("bms", "sam_ls")}> Balance </button>
      </div>
      <div class="bms-section-en" id="disable">
          <div class="section-title"> DISABLE </div>
          <button class="bms-button-en" style={{"background-color": '#C53434'}} onClick={() => disableCommand("bms", "battery_ls")}> BMS </button>
          <button class="bms-button-en" style={{"background-color": '#C53434'}} onClick={() => disableCommand("bms", "charge")}> Battery </button>
          <button class="bms-button-en" style={{"background-color": '#C53434'}} onClick={() => disableCommand("bms", "estop")}> EStop R </button>
          <button class="bms-button-en" style={{"background-color": '#C53434'}} onClick={() => disableCommand("bms", "sam_ls")}> Balance </button>
      </div>
      <div class="bms-section" id="data">
          <div class="section-title"> DATA DISPLAY </div>
            {/* DATA content here */}
            <div class="adc-data-section">
              <div class="section-title" style={{"text-decoration": 'underline'}}> ADC Data </div>
              <div class="column-title-row">
                <div class="column-title" style={{"font-size": "16px"}}> Variables </div>
                <div class="column-title" style={{"font-size": "16px"}}> Values </div>
              </div>
              {/* Change to iteratively display ADC data variables and values once backend array is implemented */}
              <div class="adc-data-row-container">
                <div class="adc-data-row">
                  <div class="adc-data-variable"> Battery Bus current </div>
                  <div class="adc-data-value"> {((bmsData() as BMS_struct).battery_bus as Bus).current} </div>
                </div>
                <div class="adc-data-row">
                  <div class="adc-data-variable"> Battery Bus Voltage </div>
                  <div class="adc-data-value"> {((bmsData() as BMS_struct).battery_bus as Bus).current} </div>
                </div>
                <div class="adc-data-row">
                  <div class="adc-data-variable"> Variable 3 </div>
                  <div class="adc-data-value"> Value 3 </div>
                </div>
                <div class="adc-data-row">
                  <div class="adc-data-variable"> Variable 4 </div>
                  <div class="adc-data-value"> Value 4 </div>
                </div>
                <div class="adc-data-row">
                  <div class="adc-data-variable"> Variable 5 </div>
                  <div class="adc-data-value"> Value 5 </div>
                </div>
              </div>
            </div>
            <div class="state-section">
              <div class="section-title" style={{"text-decoration": 'underline'}}> States </div>
              <div class="column-title-row">
                <div class="column-title" style={{"font-size": "16px"}}> Smth? </div>
                <div class="column-title" style={{"font-size": "16px"}}> States </div>
              </div>
              {/* Change to iteratively display state variables and values once backend array is implemented */}
              <div class="state-row-container">
                <div class="state-row">
                  <div class="state-variable"> State 1 </div>
                  <div class="state-value"> Value 1 </div>
                </div>
                <div class="state-row">
                  <div class="state-variable"> State 2 </div>
                  <div class="state-value"> Value 2 </div>
                </div>
                <div class="state-row">
                  <div class="state-variable"> State 3 </div>
                  <div class="state-value"> Value 3 </div>
                </div>
                <div class="state-row">
                  <div class="state-variable"> State 4 </div>
                  <div class="state-value"> Value 4 </div>
                </div>
              </div>
            </div>
            <div class="cell-voltages-section">
              <div class="section-title" style={{"text-decoration": 'underline'}}> Cell Voltages </div>
              <div class="column-title-row">
                <div class="column-title" style={{"font-size": "16px"}}> Cell </div>
                <div class="column-title" style={{"font-size": "16px"}}> Voltage </div>
              </div>
              {/* Change to iteratively display cell voltage variables and values once backend array is implemented */}
              <div class="cell-voltages-row-container">
                <div class="cell-voltage-row">
                  <div class="cell-voltage-variable"> State 1 </div>
                  <div class="cell-voltage-value"> Value 1 </div>
                </div>
                <div class="cell-voltage-row">
                  <div class="cell-voltage-variable"> State 2 </div>
                  <div class="cell-voltage-value"> Value 2 </div>
                </div>
                <div class="cell-voltage-row">
                  <div class="cell-voltage-variable"> State 3 </div>
                  <div class="cell-voltage-value"> Value 3 </div>
                </div>
                <div class="cell-voltage-row">
                  <div class="cell-voltage-variable"> State 4 </div>
                  <div class="cell-voltage-value"> Value 4 </div>
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