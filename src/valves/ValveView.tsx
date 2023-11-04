import { Component, For, Setter, createSignal } from "solid-js";
import { emit, listen } from "@tauri-apps/api/event";
import { Valve } from "../devices";
import { closeValve, openValve } from "../commands";
import { valves } from "./Valves";

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
    <div style="width: 10px; height: 50px; border-left-style:solid; 
      border-left-color: #737373; border-left-width: 1px"></div>
    <button class="valve-button" style={{"background-color": '#22873D'}} onClick={() => openValve(valves.at(index)!.name)}> 
      Open
    </button>
    <button class="valve-button" style={{"background-color": '#C53434'}} onClick={() => closeValve(valves.at(index)!.name)}> 
      Close
    </button>
    <div style="width: 10px; height: 50px; border-right-style:solid; 
      border-right-color: #737373; border-right-width: 1px"></div>
    <div style="flex: 2; display: flex">
      <div style={{'flex': 1, 'display': 'flex', 'justify-content': 'center', 'align-items': 'center', 'margin': '10px', 'height': '40px', 'padding': '5px',"background-color": feedbackColor}} >Feedback</div>
    </div>
    <div style="flex 1">
      <button class="open-plotter-button">Open Plotter</button>
    </div>
</div>
}

const ValveView: Component<{valves: Valve[]}> = (props) => {
  console.log('inner valve', props.valves);
  //const [valveList, setValveList] = createSignal(props.valves);
  return <div class="valve-view-section">
    <For each={valves()}>{(valve, i) =>
      ValveRow(valves(), i())
      }
    </For>
  </div>
}

export default ValveView;