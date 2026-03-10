import { createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { StreamState, Igniter as Igniter_struct, Bus } from "../../comm";

const [igniterAData, setIgniterAData] = createSignal({
  p5v0_rail: {voltage: 0, current: 0} as Bus,
  config_rail: {voltage: 0, current: 0} as Bus,
  p24v0_rail: {voltage: 0, current: 0} as Bus,
  cv_buses: [{voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}] as [Bus, Bus, Bus],
  cc_buses: [{voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}] as [Bus, Bus, Bus],
  continuity: [0, 0, 0] as [number, number, number],
  rbf: 0,
} as Igniter_struct);

const [igniterBData, setIgniterBData] = createSignal({
  p5v0_rail: {voltage: 0, current: 0} as Bus,
  config_rail: {voltage: 0, current: 0} as Bus,
  p24v0_rail: {voltage: 0, current: 0} as Bus,
  cv_buses: [{voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}] as [Bus, Bus, Bus],
  cc_buses: [{voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}] as [Bus, Bus, Bus],
  continuity: [0, 0, 0] as [number, number, number],
  rbf: 0,
} as Igniter_struct);

// listens to device updates and updates the values of BMS values accordingly for display
listen('device_update', (event) => {
  // get igniter a data
  const ingiter_a = (event.payload as StreamState).igniter_a;
  console.log(ingiter_a)
  setIgniterAData(ingiter_a);

  // get igniter b data
  const igniter_b = (event.payload as StreamState).igniter_b;
  console.log(igniter_b)
  setIgniterBData(igniter_b);
});

invoke('initialize_state', {window: appWindow});

function IGNITER() {
    return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="IGNITER"/>
    </div>
    <div class="igniter-view">
        {igniterData(true)}
        {igniterData(false)}
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

function igniterData(isDeviceA: boolean) {
  let title = "Device A";
  let data = igniterAData();
  if (!isDeviceA) {
    title = "Device B";
    data = igniterBData();
  }
  let cv_buses = (data as Igniter_struct).cv_buses as [Bus, Bus, Bus];
  let cc_buses = (data as Igniter_struct).cc_buses as [Bus, Bus, Bus];
  return <div class="igniter-section" id="data">
            <div class="title"> {title} </div>
            <div class="igniter-data-section-triple">
              <div class="column-title-row">
                <div class="column-title" style={{"font-size": "16px"}}> Variables </div>
                <div class="column-title" style={{"font-size": "16px"}}> Voltage </div>
                <div class="column-title" style={{"font-size": "16px"}}> Current </div>
              </div>
              <div class="igniter-data-row-container">
                {igniterTripleDataRow("5v0 Rail", ((data as Igniter_struct).p5v0_rail as Bus).voltage.toFixed(4), ((data as Igniter_struct).p5v0_rail as Bus).current.toFixed(4))}
                {igniterTripleDataRow("24v0 Rail", ((data as Igniter_struct).p24v0_rail as Bus).voltage.toFixed(4), ((data as Igniter_struct).p24v0_rail as Bus).current.toFixed(4))}
                {igniterTripleDataRow("Config Rail", ((data as Igniter_struct).config_rail as Bus).voltage.toFixed(4), ((data as Igniter_struct).config_rail as Bus).current.toFixed(4))}
                
                {/* Constant Voltage Buses */}
                {igniterTripleDataRow("Const Volt Ch 1", cv_buses[0].voltage.toFixed(4), cv_buses[0].current.toFixed(4))}
                {igniterTripleDataRow("Const Volt Ch 2", cv_buses[1].voltage.toFixed(4), cv_buses[1].current.toFixed(4))}
                {igniterTripleDataRow("Const Volt Ch 3", cv_buses[2].voltage.toFixed(4), cv_buses[2].current.toFixed(4))}

                {/* Constant Current Buses */}
                {igniterTripleDataRow("Const Cur Ch 1", cc_buses[0].voltage.toFixed(4), cc_buses[0].current.toFixed(4))}
                {igniterTripleDataRow("Const Cur Ch 2", cc_buses[1].voltage.toFixed(4), cc_buses[1].current.toFixed(4))}
                {igniterTripleDataRow("Const Cur Ch 3", cc_buses[2].voltage.toFixed(4), cc_buses[2].current.toFixed(4))}
              </div>
            </div>
            <div class="igniter-data-section">
              <div class="column-title-row">
                <div class="column-title" style={{"font-size": "16px"}}> Variables </div>
                <div class="column-title" style={{"font-size": "16px"}}> Values </div>
              </div>
              <div class="igniter-data-row-container">
                {igniterDataRow("RBF Voltage", ((data as Igniter_struct).rbf).toFixed(4))}
                {igniterDataRow("Continuity Ch 1", ((data as Igniter_struct).continuity)[0].toFixed(4))}
                {igniterDataRow("Continuity Ch 2", ((data as Igniter_struct).continuity)[1].toFixed(4))}
                {igniterDataRow("Continuity Ch 3", ((data as Igniter_struct).continuity)[2].toFixed(4))}
              </div>
            </div>
          </div>
}

function igniterTripleDataRow(name: string, voltage: string, current: string) {
  return <div class="igniter-data-row-triple">
            <div class="igniter-data-variable"> {name} </div>
            <div class="igniter-data-value"> {voltage} </div>
            <div class="igniter-data-value"> {current} </div>
          </div>
}

function igniterDataRow(name: string, data: string) {
  return <div class="igniter-data-row">
            <div class="igniter-data-variable"> {name} </div>
            <div class="igniter-data-value"> {data} </div>
          </div>
}

export default IGNITER;