import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import { emit, listen } from "@tauri-apps/api/event";
import { createSignal, For} from "solid-js";
import { Valve } from "../devices";
import { closeValve, openValve } from "../commands";
import { Config, Mapping, Sequence, State, StreamState, runSequence, serverIp } from "../comm";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [sequences, setSequences] = createSignal();
const [override, setOverride] = createSignal(false);
const [seqButtonLabel, setSeqButtonLabel] = createSignal('Start Sequence');
const [seqRunning, setSeqRunning] = createSignal(false);
const [deviceOptions, setDeviceOptions] = createSignal(new Array);
export const [valves, setValves] = createSignal(new Array<Valve>);

listen('device_update', (event) => {
  const valve_object = (event.payload as StreamState).valve_states;
  var valveDevices = Object.keys(valve_object).map((key) => [key, valve_object[key as keyof typeof valve_object]]);
  console.log(valveDevices);
  // updating all valves
  valveDevices.forEach(async (device) => {
    var index = valves().findIndex(item => (item.name === device[0] as string));
    var new_valves = [...valves()];
    console.log(device[1]);
    var valveStates = (device[1] as unknown as object);
    new_valves[index].commanded = valveStates['commanded' as keyof typeof valveStates];
    new_valves[index].actual = valveStates['actual' as keyof typeof valveStates];
    setValves(new_valves);
});
});


listen('state', (event) => {
  console.log(event.windowLabel);
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
  setSequences((event.payload as State).sequences);
  console.log(activeConfig());
  console.log(configurations() as Config[]);
  var activeconfmappings = (configurations() as Config[]).filter((conf) => {return conf.id == activeConfig() as string})[0];
  var vlvs = new Array;
  var deviceOptions = new Array;
  console.log(activeconfmappings);
  for (const mapping of activeconfmappings.mappings) {
    if (mapping.sensor_type === 'valve') {
      deviceOptions.push(mapping);
      // vlvs.push(
      //   {
      //     name: mapping.text_id,
      //     group: 'Fuel',
      //     board_id: mapping.board_id,
      //     sensor_type: mapping.sensor_type,
      //     channel: mapping.channel,
      //     commanded: 'closed',
      //     actual: 'disconnected'
      //   } as Valve,
      // )
    }
  }
  setDeviceOptions(deviceOptions);
  //setValves(vlvs);
  //console.log('valves', valves());
});

invoke('initialize_state', {window: appWindow});

function toggleSequenceButton() {
  var button = document.getElementById("sequencebutton")!;
  const seqDropdown = document.getElementById("sequenceselect")! as HTMLSelectElement;
  console.log(seqDropdown);
  if (seqRunning()) {
    setSeqButtonLabel('Start Sequence');
    setSeqRunning(false);
    button.style.backgroundColor = "#015878"
    button.style.setProperty('seqButtonBackgroundColor',  '#00425a!important');
  } else {
    runSequence(serverIp() as string, seqDropdown.value, override());
    setSeqButtonLabel('Abort Sequence');
    setSeqRunning(true);
    button.style.backgroundColor = "#C53434"
  }
}

function stateToColor(state: string) {
  switch (state) {
    case "open":
      return "#22873D";
    case "closed":
      return "#C53434";
    case "disconnected":
      return "#737373";
    case "undetermined":
      return "#015878";
    case "fault":
      return "#CC9A13";
  }
}

function openDropdown() {
  console.log("opening dropdown");
  var button = document.getElementById("valvebutton")!;
  var dropdownContent = document.getElementById("valvedropdown")!;
  dropdownContent.style.display = "flex";
}

function closeDropdown(evt:MouseEvent) {
  var button = document.getElementById("valvebutton")!;
  var dropdownContent = document.getElementById("valvedropdown")!;
  if (evt.target != button) {
      dropdownContent.style.display = "none";
  }
}

function addValveDevice(mapping: Mapping) {
  var newValves = [...valves() as Valve[]]
  var indexToRemove = -1;
  for (var i = 0; i < valves().length; i++) {
      if (valves()[i].name === mapping.text_id) {
          indexToRemove = i;
          break;
      }
  }
  if (indexToRemove != -1) {
      console.log('deleting...');
      newValves.splice(indexToRemove, 1);
      setValves(newValves);
      return;
  }
  newValves.push(
    {
      name: mapping.text_id,
      group: 'Fuel',
      board_id: mapping.board_id,
      sensor_type: mapping.sensor_type,
      channel: mapping.channel,
      commanded: 'closed',
      actual: 'disconnected'
    } as Valve,
  )
  setValves(newValves);
}

document.addEventListener("click", (evt) => closeDropdown(evt));

function Valves() {
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="Valves"/>
    </div>
    <div class='valve-view'>
      <div style={{display: "flex"}}>
        <div id='valvebutton' class='addvalvebutton' onClick={openDropdown}>Add/Remove Valves</div>
          <div id="valvedropdown" class="valvedropdowncontent">
            {deviceOptions().length != 0? <For each={deviceOptions() as Mapping[]}>{(mapping, i) =>
                <div class="valvedropdownitem" onClick={() => addValveDevice(mapping)}>{mapping.text_id}</div>
            }</For>:<div class="valvedropdownitem">no valves or active config rip</div>
            }
          </div>
      </div>
      <div class="valve-view-section">
        <div style={{display: "grid", "grid-template-columns": "4fr 10fr 220px"}}>
            <div style={{"text-align": "center"}}>Name</div>
            <div style={{"text-align": "center"}}>Actions</div>
            <div style={{display: "flex"}}>
              <div style={{"text-align": "center", flex: 1, "margin-left": "5px"}}>Commanded</div>
              <div style={{"text-align": "center", flex: 1}}>Actual</div>
            </div>
            
        </div>
        <For each={valves()}>{(valve, i) =>
          <div class='valve-row'>
          <div style="flex: 2; display: flex; justify-content: center;">
            {valves()[i()].name}
          </div>
          <div style="width: 10px; height: 30px; border-left-style:solid; 
            border-left-color: #737373; border-left-width: 1px"></div>
          <button class="valve-button" style={{"background-color": '#22873D'}} onClick={() => openValve(valves()[i()].name)}> 
            Open
          </button>
          <button class="valve-button" style={{"background-color": '#C53434'}} onClick={() => closeValve(valves()[i()].name)}> 
            Close
          </button>
          <div style="width: 10px; height: 30px; border-right-style:solid; 
            border-right-color: #737373; border-right-width: 1px"></div>
          <div style={{'display': 'flex', 'justify-content': 'center', 'align-items': 'center', 'margin-left': '10px', 'width': '90px', 'height': '10px', 'padding': '5px',"background-color": stateToColor(valves()[i()].commanded)}} >
            {valves()[i()].commanded.charAt(0).toUpperCase()+valves()[i()].commanded.substring(1)}
          </div>
          <div style={{'display': 'flex', 'justify-content': 'center', 'align-items': 'center', 'margin-left': '10px', 'width': '90px', 'height': '10px', 'padding': '5px',"background-color": stateToColor(valves()[i()].actual)}} >
            {valves()[i()].actual.charAt(0).toUpperCase()+valves()[i()].actual.substring(1)}
          </div>
        </div>}
        </For>
      </div>
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default Valves;