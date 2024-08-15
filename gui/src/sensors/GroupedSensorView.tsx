import { Component, For } from "solid-js";
import { Device } from "../devices";

function row(name: string, value: number, unit: string) {
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
      <div style="flex: 1; display: flex; justify-content: center" >{(value as number).toFixed(4)}</div>
      <div style="flex: 1">{unit}</div>
    </div>
    <div style="flex 1">
      <button class="open-plotter-button">Open Plotter</button>
    </div>
  </div>
}

const GroupedSensorView: Component<{type: string, sensors: Device[], color: string}> = (props) => {
  return <div style="display: flex; flex-direction: column">
    <div id="sectionTitle" style={{"text-align": "center", color: props.color}}>
      {props.type}
    </div>
    <div style={{"border-color": props.color, "border-style":"solid", "border-width": "2px", "border-radius": "5px",
      padding: "10px", "background-color": "#333333"}}>
    <For each={props.sensors}>
      {(item) => row(item.name, item.value, item.unit)}
    </For>
    </div>
  </div>
}

export default GroupedSensorView;