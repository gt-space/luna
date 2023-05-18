import { Component } from "solid-js";
import DragAndDrop from "../general-components/DragAndDrop";
import { Sensor } from "../devices";

function SensorRow(name: string, value: number, unit: string) {
  return <div class='sensor-row'>
    <div style="flex: 2; display: flex; justify-content: center; border-right-style:solid; 
      border-right-color: #737373; border-right-width: 1px">
      {name}
    </div>
    <div style="flex: 2; display: flex; justify-content: center; border-right-style:solid; 
      border-right-color: #737373; border-right-width: 1px; color: #1DB55A"> 
      Connected
    </div>
    <div style="flex: 2; display: flex">
      <div style="flex: 1; display: flex; justify-content: center" >{value}</div>
      <div style="flex: 1">{unit}</div>
    </div>
    <div style="flex 1">
      <button class="open-plotter-button">Open Plotter</button>
    </div>
  </div>
}

const SortableSensorView: Component<{sensors: Sensor[]}> = (props) => {
  return <div class="sortable-sensor-view">
    <DragAndDrop sensors={props.sensors} row={SensorRow}/>
  </div>
}

export default SortableSensorView;