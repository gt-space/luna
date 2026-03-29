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
  channels: [{voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}] as [Bus, Bus, Bus, Bus, Bus, Bus],
  continuity: [0, 0, 0, 0, 0, 0] as [number, number, number, number, number, number],
  cc_faults: [0, 0, 0] as [number, number, number],
  rbf: 0,
} as Igniter_struct);

const [igniterBData, setIgniterBData] = createSignal({
  p5v0_rail: {voltage: 0, current: 0} as Bus,
  config_rail: {voltage: 0, current: 0} as Bus,
  p24v0_rail: {voltage: 0, current: 0} as Bus,
  channels: [{voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}, {voltage: 0, current: 0}] as [Bus, Bus, Bus, Bus, Bus, Bus],
  continuity: [0, 0, 0, 0, 0, 0] as [number, number, number, number, number, number],
  cc_faults: [0, 0, 0] as [number, number, number],
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
  return <div class="igniter-section" id="data">
            <div class="title"> {title} </div>
            <div class="igniter-data-section-triple">
              <div class="column-title-row">
                <div class="column-title" style={{"font-size": "14px"}}> Rail </div>
                <div class="column-title" style={{"font-size": "14px"}}> Voltage </div>
                <div class="column-title" style={{"font-size": "14px"}}> Current </div>
              </div>
              <div class="igniter-data-row-container">
                {igniterTripleDataRow("5v0 Rail", ((data as Igniter_struct).p5v0_rail as Bus).voltage.toFixed(4), ((data as Igniter_struct).p5v0_rail as Bus).current.toFixed(4))}
                {igniterTripleDataRow("24v0 Rail", ((data as Igniter_struct).p24v0_rail as Bus).voltage.toFixed(4), ((data as Igniter_struct).p24v0_rail as Bus).current.toFixed(4))}
                {igniterTripleDataRow("Config Rail", ((data as Igniter_struct).config_rail as Bus).voltage.toFixed(4), ((data as Igniter_struct).config_rail as Bus).current.toFixed(4))}
              </div>
            </div>
            <div class="igniter-data-section-quad">
              <div class="column-title-row">
                <div class="column-title" style={{"font-size": "14px"}}> Channel </div>
                <div class="column-title" style={{"font-size": "14px"}}> Voltage </div>
                <div class="column-title" style={{"font-size": "14px"}}> Current </div>
                <div class="column-title" style={{"font-size": "14px"}}> Continuity </div>
              </div>
              <div class="igniter-data-row-container">
                {igniterQuadDataRow(
                  "Channel 1",
                  ((data as Igniter_struct).channels)[0].voltage.toFixed(4),
                  ((data as Igniter_struct).channels)[0].current.toFixed(4),
                  ((data as Igniter_struct).continuity)[0].toFixed(4)
                )}
                {igniterQuadDataRow(
                  "Channel 2",
                  ((data as Igniter_struct).channels)[1].voltage.toFixed(4),
                  ((data as Igniter_struct).channels)[1].current.toFixed(4),
                  ((data as Igniter_struct).continuity)[1].toFixed(4)
                )}
                {igniterQuadDataRow(
                  "Channel 3",
                  ((data as Igniter_struct).channels)[2].voltage.toFixed(4),
                  ((data as Igniter_struct).channels)[2].current.toFixed(4),
                  ((data as Igniter_struct).continuity)[2].toFixed(4)
                )}
                {igniterQuadDataRow(
                  "Channel 4",
                  ((data as Igniter_struct).channels)[3].voltage.toFixed(4),
                  ((data as Igniter_struct).channels)[3].current.toFixed(4),
                  ((data as Igniter_struct).continuity)[3].toFixed(4)
                )}
                {igniterQuadDataRow(
                  "Channel 5",
                  ((data as Igniter_struct).channels)[4].voltage.toFixed(4),
                  ((data as Igniter_struct).channels)[4].current.toFixed(4),
                  ((data as Igniter_struct).continuity)[4].toFixed(4)
                )}
                {igniterQuadDataRow(
                  "Channel 6",
                  ((data as Igniter_struct).channels)[5].voltage.toFixed(4),
                  ((data as Igniter_struct).channels)[5].current.toFixed(4),
                  ((data as Igniter_struct).continuity)[5].toFixed(4)
                )}
              </div>
            </div>
            <div class="igniter-data-section">
              <div class="column-title-row">
                <div class="column-title" style={{"font-size": "14px"}}> Fault / RBF </div>
                <div class="column-title" style={{"font-size": "14px"}}> Values </div>
              </div>
              <div class="igniter-data-row-container">
                {igniterDataRow("RBF", Number(((data as Igniter_struct).rbf)).toFixed(0))}
                {igniterDataRow("nCC Fault Ch 4", (((data as Igniter_struct).cc_faults)[0] as number).toFixed(0))}
                {igniterDataRow("nCC Fault Ch 5", (((data as Igniter_struct).cc_faults)[1] as number).toFixed(0))}
                {igniterDataRow("nCC Fault Ch 6", (((data as Igniter_struct).cc_faults)[2] as number).toFixed(0))}
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

function igniterQuadDataRow(name: string, voltage: string, current: string, continuity: string) {
  return <div class="igniter-data-row-quad">
            <div class="igniter-data-variable"> {name} </div>
            <div class="igniter-data-value"> {voltage} </div>
            <div class="igniter-data-value"> {current} </div>
            <div class="igniter-data-value"> {continuity} </div>
          </div>
}

function igniterDataRow(name: string, data: string) {
  return <div class="igniter-data-row">
            <div class="igniter-data-variable"> {name} </div>
            <div class="igniter-data-value"> {data} </div>
          </div>
}

export default IGNITER;