import { Component } from "solid-js";
import DragAndDrop from "../general-components/DragAndDrop";
import { Device } from "../devices";

function SensorRow(name: string, value: number, unit: string, offset: number) {
  return <div class='sensor-row'>
    <div style="flex: 1; display: flex; justify-content: center; border-right-style:solid; 
      border-right-color: #737373; border-right-width: 1px">
      {name}
    </div>
    <div style="flex: 1; display: flex; justify-content: center; border-right-style:solid; 
      border-right-color: #737373; border-right-width: 1px; color: #1DB55A"> 
      Connected
    </div>
    <div style="flex: 1; display: flex">
      <div style="flex: 2; display: flex; justify-content: right; padding-right: 5px; font-family: monospace" >{(value as number).toFixed(4)}</div>
      <div style="flex: 1; font-family: monospace; padding-right: 5px">{unit}</div>
    </div>
    <div style="flex 1; font-size: 10px">
      Offset: {offset}
    </div>
  </div>
}

const SortableSensorView: Component<{sensors: Device[]}> = (props) => {
  return <div class="sortable-sensor-view">
    <DragAndDrop sensors={props.sensors} row={SensorRow}/>
  </div>
}

export default SortableSensorView;