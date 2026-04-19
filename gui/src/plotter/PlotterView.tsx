import { Component, For, Setter, createEffect, createSignal } from "solid-js";
import ChartComponent from "./Chart";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { Config, FCSensors, GPS, Mapping, RECO, State, StreamSensor, StreamState } from "../comm";

const FC_TEMPERATURE_PLOT_ID = "FC_Temperature_C";

function fcSensorsPlotMappings(): Mapping[] {
  return [{
    text_id: FC_TEMPERATURE_PLOT_ID,
    board_id: "fc",
    sensor_type: "fc_plot",
    channel: 0,
    computer: "",
    min: 0,
    max: 0,
    powered_threshold: 0,
    normally_closed: null,
  }];
}

function fcSensorsPlotValue(plotId: string, fc: FCSensors | undefined): number | undefined {
  if (plotId !== FC_TEMPERATURE_PLOT_ID || fc === undefined) return undefined;
  return fc.temperature;
}

/** Single stream GPS series; `StreamState.gps.altitude_m` (meters). */
const GPS_ALTITUDE_PLOT_ID = "GPS_Altitude_m";

function gpsPlotMappings(): Mapping[] {
  return [
    {
      text_id: GPS_ALTITUDE_PLOT_ID,
      board_id: "gps",
      sensor_type: "gps_plot",
      channel: 0,
      computer: "",
      min: 0,
      max: 0,
      powered_threshold: 0,
      normally_closed: null,
    },
  ];
}

function gpsPlotValue(plotId: string, gps: GPS | undefined): number | undefined {
  if (plotId !== GPS_ALTITUDE_PLOT_ID || gps === undefined) {
    return undefined;
  }
  return gps.altitude_m;
}

const RECO_MCU_LETTERS = ["A", "B", "C"] as const;

/** Stable plot ids (valid HTML canvas ids); values come from StreamState.reco on device_update. */
const RECO_SCALAR_FIELDS = [
  "LinAccel_X",
  "LinAccel_Y",
  "LinAccel_Z",
  "Gyro_X",
  "Gyro_Y",
  "Gyro_Z",
  "Baro_Pressure",
  "Baro_Temp",
  "Baro_FadingMemory",
  "EKF_Lon",
  "EKF_Lat",
  "EKF_Alt",
  "EKF_VN",
  "EKF_VE",
  "EKF_VD",
] as const;

function recoPlotMappings(): Mapping[] {
  const out: Mapping[] = [];
  for (const letter of RECO_MCU_LETTERS) {
    for (const field of RECO_SCALAR_FIELDS) {
      out.push({
        text_id: `RECO_${letter}_${field}`,
        board_id: "reco",
        sensor_type: "reco_plot",
        channel: 0,
        computer: "",
        min: 0,
        max: 0,
        powered_threshold: 0,
        normally_closed: null,
      });
    }
  }
  return out;
}

function recoPlotValue(
  plotId: string,
  reco: [RECO | undefined, RECO | undefined, RECO | undefined],
): number | undefined {
  const m = /^RECO_([ABC])_(.+)$/.exec(plotId);
  if (!m) return undefined;
  const mcuIdx = m[1] === "A" ? 0 : m[1] === "B" ? 1 : 2;
  const field = m[2];
  const r = reco[mcuIdx];
  if (!r) return undefined;
  switch (field) {
    case "LinAccel_X":
      return r.lin_accel[0];
    case "LinAccel_Y":
      return r.lin_accel[1];
    case "LinAccel_Z":
      return r.lin_accel[2];
    case "Gyro_X":
      return r.angular_rate[0];
    case "Gyro_Y":
      return r.angular_rate[1];
    case "Gyro_Z":
      return r.angular_rate[2];
    case "Baro_Pressure":
      return r.pressure;
    case "Baro_Temp":
      return r.temperature;
    case "Baro_FadingMemory":
      return r.fading_memory_baro;
    case "EKF_Lon":
      return r.lla_pos[0];
    case "EKF_Lat":
      return r.lla_pos[1];
    case "EKF_Alt":
      return r.lla_pos[2];
    case "EKF_VN":
      return r.velocity[0];
    case "EKF_VE":
      return r.velocity[1];
    case "EKF_VD":
      return r.velocity[2];
    default:
      return undefined;
  }
}

export const [plotterValues, setPlotterValues] = createSignal(new Array(10));
export const [levels, setlevels] = createSignal(new Map<string, number>([]));
const [plotterDevices, setPlotterDevices] = createSignal(new Array);
const [deviceOptions, setDeviceOptions] = createSignal(new Array);

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();

listen('state', (event) => {
    setConfigurations((event.payload as State).configs);
    setActiveConfig((event.payload as State).activeConfig);
    const mappings = (configurations() as Config[]).filter((conf) => {return conf.id == activeConfig() as string})[0].mappings;
    var newMappings = [];
    for (var i = 0; i < mappings.length; i++) {
        if (mappings[i].sensor_type === 'valve') {
            var voltageMapping = structuredClone(mappings[i]);
            var currentMapping = structuredClone(mappings[i]);
            voltageMapping.text_id+='_V';
            currentMapping.text_id+='_I';
            newMappings.push(voltageMapping);
            newMappings.push(currentMapping);
        } else {
            newMappings.push(mappings[i]);
        }
    }
    setDeviceOptions(
      [...newMappings, ...recoPlotMappings(), ...gpsPlotMappings(), ...fcSensorsPlotMappings()].sort((a, b) =>
        a.text_id.localeCompare(b.text_id),
      ),
    );
    console.log(newMappings);
});

invoke('initialize_state', {window: appWindow});

// listens to device updates and updates the values of sensors and valves accordingly for display
listen('device_update', (event) => {
    // getting data
    const payload = event.payload as StreamState;
    const sensor_object = payload.sensor_readings;
    const valve_object = payload.valve_states;
    const reco = payload.reco;
    const gps = payload.gps;
    const fc_sensors = payload.fc_sensors;
    var sensorDevices = Object.keys(sensor_object).map((key) => [key, sensor_object[key as keyof typeof sensor_object] as StreamSensor]);
    //console.log(sensorDevices);
    var valveDevices = Object.keys(valve_object).map((key) => [key, valve_object[key as keyof typeof valve_object]]);
    
    // updating all sensors
    sensorDevices.forEach(async (device) => {
        var index = (plotterDevices() as Array<{id: string, board_id: string, channel: Number, value: number}>)
        .findIndex(item => (item.id === device[0] as string));
        var new_values = [...plotterValues()];
        new_values[index] = (device[1] as StreamSensor).value;
        //console.log((device[1] as StreamSensor).value);
        setPlotterValues(new_values);
      });
    //console.log(plotterValues());
    
    // updating all valves
    valveDevices.forEach(async (device) => {
        var index = (plotterDevices() as Array<{id: string, board_id: string, channel: number, value: number}>)
        .findIndex(item => (item.id === device[0] as string));
        var new_values = [...plotterValues()];
        // A '1' means valve is open, '0' means it is closed.
        switch (device[1]) {
            case "open":
                new_values[index] = 1;
                break
            case "closed":
                new_values[index] = 0;
                break
        }
        setPlotterValues(new_values);
    });

    const devices = plotterDevices() as Array<{
      id: string;
      board_id: string;
      channel: Number;
      value: number;
    }>;
    let streamValues = [...plotterValues()];
    let streamChanged = false;
    for (let i = 0; i < devices.length; i++) {
      const id = devices[i].id;
      const fromReco = recoPlotValue(id, reco);
      const fromGps = gpsPlotValue(id, gps);
      const fromFc = fcSensorsPlotValue(id, fc_sensors);
      const v = fromReco !== undefined ? fromReco : fromGps !== undefined ? fromGps : fromFc;
      if (v !== undefined && Number.isFinite(v)) {
        streamValues[i] = v;
        streamChanged = true;
      }
    }
    if (streamChanged) {
      setPlotterValues(streamValues);
    }
});

function openDropdown() {
    console.log("opening dropdown");
    var button = document.getElementById("plotsbutton")!;
    var dropdownContent = document.getElementById("plotterdropdown")!;
    dropdownContent.style.display = "flex";
}

function closeDropdown(evt:MouseEvent) {
    var button = document.getElementById("plotsbutton")!;
    var dropdownContent = document.getElementById("plotterdropdown")!;
    if (evt.target != button) {
        dropdownContent.style.display = "none";
    }
}

function addPlotterDevice(mapping: Mapping) {
    var newPlotterDevices = [...plotterDevices() as Array<{id: string, board_id: string, channel: number, value: number}>];
    var indexToRemove = -1;
    for (var i = 0; i < plotterDevices().length; i++) {
        if (plotterDevices()[i].id === mapping.text_id) {
            indexToRemove = i;
            break;
        }
    }
    if (indexToRemove != -1) {
        console.log('deleting...');
        newPlotterDevices.splice(indexToRemove, 1);
        setPlotterDevices(newPlotterDevices);
        return;
    }
    newPlotterDevices.push({
        id: mapping.text_id,
        board_id: mapping.board_id,
        channel: mapping.channel,
        value: NaN
    });
    setPlotterDevices(newPlotterDevices);
}

async function addLevel() {
    var deviceName = (document.getElementById("leveldropdown")! as HTMLSelectElement).value;
    var level = (document.getElementById("levelinput")! as HTMLInputElement).value;
    var newLevels = structuredClone(levels());
    if (level.length == 0) {
        if (levels().has(deviceName)) {
            newLevels.delete(deviceName);
            setlevels(newLevels);
        }
        return;
    }
    if (!isNaN(parseFloat(level))) {
        newLevels.set(deviceName, parseFloat(level));
    } 
    setlevels(newLevels);
    console.log(levels());
}

document.addEventListener("click", (evt) => closeDropdown(evt));

const PlotterView: Component = (props) => {
    return <div style={{display: "grid", "grid-template-rows": "50px 1fr", height: "100%"}}>
        <div style={{display: "flex", margin: "10px", "margin-left": "20px", "margin-bottom": "0px", "align-items": "center"}}>
            <div id="plotsbutton" class="addplotsbutton" onClick={() => {openDropdown()}}>
                Add/remove plots
            </div>
            <div id="plotterdropdown" class="plotterdropdowncontent">
                {deviceOptions().length != 0? <For each={deviceOptions() as Mapping[]}>{(mapping, i) =>
                    <div class="plotterdropdownitem" onClick={() => addPlotterDevice(mapping)}>{mapping.text_id}</div>
                }</For>:<div class="plotterdropdownitem">There is no active config rip</div>
                }
            </div>
            <div style={{"margin-left": "20px", "margin-right": "5px"}}>
                Add plot levels: 
            </div>
            <select id="leveldropdown"class="feedsystem-config-dropdown" style={{width: "100px"}}>
            <For each={deviceOptions() as Mapping[]}>{(device, i) => 
                <option style={{color: "black"}}>{device.text_id}</option>}                
            </For>
            </select>
            <input type="text" id="levelinput" placeholder="Level" class="level-textfield"></input>
            <button class="submit-feedsystem-button" onClick={addLevel}>Add</button>
        </div>
        <div class="plotter-view-section">
            <For each={plotterDevices() as Array<{id: string, board_id: Number, channel: Number, value: number}>}>{(device, i) =>
                <div style={{margin: '5px'}}><ChartComponent id={device.id} index={i()}  /></div>
            }</For>
        </div>
    </div> 
  }
  
  export default PlotterView;