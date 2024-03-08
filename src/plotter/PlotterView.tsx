import { Component, For, Setter, createEffect, createSignal } from "solid-js";
import ChartComponent from "./Chart";
import { listen } from "@tauri-apps/api/event";
import { Config, Mapping, State, StreamSensor, StreamState } from "../comm";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";

export const [plotterValues, setPlotterValues] = createSignal(new Array(10));
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
    setDeviceOptions(newMappings);
    console.log(newMappings);
});

invoke('initialize_state', {window: appWindow});

// listens to device updates and updates the values of sensors and valves accordingly for display
listen('device_update', (event) => {
    // getting data
    const sensor_object = (event.payload as StreamState).sensor_readings;
    const valve_object = (event.payload as StreamState).valve_states;
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
        </div>
        <div class="plotter-view-section">
            <For each={plotterDevices() as Array<{id: string, board_id: Number, channel: Number, value: number}>}>{(device, i) =>
                <div style={{margin: '5px'}}><ChartComponent id={device.id} index={i()} /></div>
            }</For>
        </div>
    </div> 
  }
  
  export default PlotterView;