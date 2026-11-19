import { Component, Show } from "solid-js";
import DragAndDrop from "../general-components/DragAndDrop";
import { Device } from "../devices";
import { debugMode } from "./Sensors";

function SensorRow(name: string, value: number, unit: string, offset: number, min: number, max: number, board_id: string, channel: number, sensor_type: string) {
  return <div class='sensor-row'>
    <div style="flex: 1; display: flex; justify-content: center; border-right-style:solid;
      border-right-color: #737373; border-right-width: 1px">
      {name}
    </div>
    <div style="flex: 1; display: flex;">
      <div style="flex: 2; display: flex; justify-content: right; padding-right: 5px; font-family: monospace" >{(value as number).toFixed(4)}</div>
      <div style="flex: 1; font-family: monospace">{unit}</div>
    </div>
    <div style={{flex: debugMode() ? 2 : 1, display: "flex", "justify-content": "center",
      "border-left": "1px solid #737373"}}>
      <div style="flex: 1; display: flex; flex-direction: column; text-align: center; font-size: 10px;">
        <div>
          Range: {Number.isFinite(min) && Number.isFinite(max) ? `${min} ${unit} - ${max} ${unit}` : "N/A"}
        </div>
        <Show when={debugMode()}>
          <div>Offset: {offset}</div>
        </Show>
      </div>
      <Show when={debugMode()}>
        <div style="flex: 1; display: flex; flex-direction: column; text-align: center; font-size: 10px;">
          <div>Board: {board_id}</div>
          <div>Channel: {sensor_type.toUpperCase()} {channel}</div>
        </div>
      </Show>
    </div>
  </div>
}

const SortableSensorView: Component<{sensors: Device[]}> = (props) => {
  return <div class="sortable-sensor-view">
    <DragAndDrop sensors={props.sensors} row={SensorRow}/>
  </div>
}

export default SortableSensorView;
