import { Component, For, Setter, createSignal } from "solid-js";
import ChartComponent from "./Chart";
import { listen } from "@tauri-apps/api/event";
import { GenericDevice } from "../devices";
import { plotterDevices, setPlotterDevices } from "./Plotter";
import Scrollbars from "solid-custom-scrollbars";

export const [plotterValues, setPlotterValues] = createSignal(new Array(10));

listen('device_update', (event) => {
    var devices = event.payload as Array<GenericDevice>;
    devices.forEach(async (device) => {
        var index = (plotterDevices() as Array<{id: string, board_id: Number, channel: Number, value: number}>)
        .findIndex(item => (item.board_id === device.board_id && item.channel === device.channel));
        var new_values = [...plotterValues()];
        new_values[index] = device.floatValue;
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