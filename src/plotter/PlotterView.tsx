import { Component, For, Setter, createSignal } from "solid-js";
import ChartComponent from "./Chart";
import { listen } from "@tauri-apps/api/event";
import { plotterDevices, setPlotterDevices } from "./Plotter";
import Scrollbars from "solid-custom-scrollbars";
import { StreamSensor, StreamState } from "../comm";

export const [plotterValues, setPlotterValues] = createSignal(new Array(10));

// listens to device updates and updates the values of sensors and valves accordingly for display
listen('device_update', (event) => {
    // getting data
    const sensor_object = (event.payload as StreamState).sensor_readings;
    const valve_object = (event.payload as StreamState).valve_states;
    var sensorDevices = Object.keys(sensor_object).map((key) => [key, sensor_object[key as keyof typeof sensor_object] as StreamSensor]);
    console.log(sensorDevices);
    var valveDevices = Object.keys(valve_object).map((key) => [key, valve_object[key as keyof typeof valve_object]]);
    
    // updating all sensors
    sensorDevices.forEach(async (device) => {
        var index = (plotterDevices() as Array<{id: string, board_id: Number, channel: Number, value: number}>)
        .findIndex(item => (item.id === device[0] as string));
        var new_values = [...plotterValues()];
        new_values[index] = (device[1] as StreamSensor).value;
        console.log((device[1] as StreamSensor).value);
        setPlotterValues(new_values);
      });
    console.log(plotterValues());
    
    // updating all valves
    valveDevices.forEach(async (device) => {
        var index = (plotterDevices() as Array<{id: string, board_id: Number, channel: Number, value: number}>)
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

const PlotterView: Component = (props) => {
    return <div class="plotter-view-section">
        <For each={plotterDevices() as Array<{id: string, board_id: Number, channel: Number, value: number}>}>{(device, i) =>
              <div style={{margin: '5px'}}><ChartComponent id={device.id} index={i()} /></div>
        }</For>
    </div>
  }
  
  export default PlotterView;