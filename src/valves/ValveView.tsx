import { Component, For, Setter, createSignal } from "solid-js";
import { emit, listen } from "@tauri-apps/api/event";
import { Valve } from "../devices";


function toggleValve(index: number) {
  emit('valveUpdate', index);
}


function ValveRow(valves: Valve[], index: number) {
  let openColor: string;
  let feedbackColor: string;
  let valve = valves.at(index)!;
  if (valve.open) {
    openColor = '#22873D';
  } else {
    openColor = '#C53434';
  }
  if (valve.feedback) {
    feedbackColor = '#22873D';
  } else {
    feedbackColor = '#C53434';
  }
  return <div class='valve-row'>
    <div style="flex: 2; display: flex; justify-content: center;">
      {valve.name}
    </div>
    <div style="width: 1px; height: 50px; border-right-style:solid; 
      border-right-color: #737373; border-right-width: 1px"></div>
    <button class="valve-button" style={{"background-color": openColor}} onClick={() => toggleValve(index)}> 
      {valve.open? 'Opened':'Closed'}
    </button>
    <div style="width: 1px; height: 50px; border-right-style:solid; 
      border-right-color: #737373; border-right-width: 1px"></div>
    <div style="flex: 2; display: flex">
      <div style={{'flex': 1, 'display': 'flex', 'justify-content': 'center', 'align-items': 'center', 'margin': '10px', 'height': '40px', "background-color": feedbackColor}} >Feedback</div>
    </div>
    <div style="flex 1">
      <button class="open-plotter-button">Open Plotter</button>
    </div>
</div>
}

const ValveView: Component<{valves: Valve[]}> = (props) => {
  const [valveList, setValveList] = createSignal(props.valves);
  return <div class="valve-view-section">
    <For each={valveList()}>{(valve, i) =>
      ValveRow(valveList(), i())
      }
    </For>
  </div>
}

export default ValveView;