import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import { emit, listen } from "@tauri-apps/api/event";
import { createSignal, For} from "solid-js";
import { Valve } from "../devices";
import { closeValve, openValve } from "../commands";
import { Config, Sequence, State, StreamState, runSequence, serverIp } from "../comm";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [sequences, setSequences] = createSignal();
const [override, setOverride] = createSignal(false);
const [seqButtonLabel, setSeqButtonLabel] = createSignal('Start Sequence');
const [seqRunning, setSeqRunning] = createSignal(false);
export const [valves, setValves] = createSignal(new Array<Valve>);

invoke('initialize_state', {window: appWindow});

listen('device_update', (event) => {
  const valve_object = (event.payload as StreamState).valve_states;
  var valveDevices = Object.keys(valve_object).map((key) => [key, valve_object[key as keyof typeof valve_object]]);
  // updating all valves
  valveDevices.forEach(async (device) => {
    var index = valves().findIndex(item => (item.name === device[0] as string));
    var new_valves = [...valves()];
    switch (device[1]) {
      case "open":
        new_valves[index].connected = true;
        new_valves[index].open = true;
        new_valves[index].feedback = true;
        break;
      case "closed":
        new_valves[index].connected = true;
        new_valves[index].open = false;
        new_valves[index].feedback = false;
        break;
      case "disconnected":
        new_valves[index].connected = false;
        break;
      case "commanded_closed":
        new_valves[index].connected = true;
        new_valves[index].open = false;
        new_valves[index].feedback = true;
        break;
      case "commanded_open":
        new_valves[index].connected = true;
        new_valves[index].open = true;
        new_valves[index].feedback = false;
        break;
    }
    setValves(new_valves);
    console.log(valves());
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
  console.log(activeconfmappings);
  for (const mapping of activeconfmappings.mappings) {
    if (mapping.channel_type === 'valve_voltage' || mapping.channel_type === "valve_current") {
      vlvs.push(
        {
          name: mapping.text_id,
          group: 'Fuel',
          board_id: mapping.board_id,
          channel_type: mapping.channel_type,
          channel: mapping.channel,
          open: false,
          feedback: false,
          connected: false,
        } as Valve,
      )
    }
  }
  setValves(vlvs);
  console.log('valves', valves());
});

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

function Valves() {
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="Valves"/>
    </div>
    <div class='valve-view'>
      <div class='sequences-panel'>
        <select
          id="sequenceselect"
          class="sequences-dropdown"
          onChange={(e) => {
            console.log(e?.target.className);
          }}
        >
          <For each={sequences() as Sequence[]}>{(sequence, i) =>
              <option class="seq-dropdown-item" value={sequence.name}>{sequence.name}</option>
            }</For>
        </select>
        <div style={{flex: 1, padding: '5px'}}>
          <button class='toggle-sequence-button' id="sequencebutton" onClick={toggleSequenceButton}>
            {seqButtonLabel()}
          </button>
        </div>
      </div>
      <div class="valve-view-section">
        <For each={valves()}>{(valve, i) =>
          <div class='valve-row'>
          <div style="flex: 2; display: flex; justify-content: center;">
            {valves()[i()].name}
          </div>
          <div style="width: 10px; height: 20px; border-left-style:solid; 
            border-left-color: #737373; border-left-width: 1px"></div>
          <button class="valve-button" style={{"background-color": '#22873D'}} onClick={() => openValve(valves()[i()].name)}> 
            Open
          </button>
          <button class="valve-button" style={{"background-color": '#C53434'}} onClick={() => closeValve(valves()[i()].name)}> 
            Close
          </button>
          <div style="width: 10px; height: 20px; border-right-style:solid; 
            border-right-color: #737373; border-right-width: 1px"></div>
          <div style={{'display': 'flex', 'justify-content': 'center', 'align-items': 'center', 'margin-left': '10px', 'width': '90px', 'height': '10px', 'padding': '5px',"background-color": valves()[i()].open? "#22873D": "#C53434"}} >Commanded</div>
          <div style={{'display': 'flex', 'justify-content': 'center', 'align-items': 'center', 'margin-left': '10px', 'width': '90px', 'height': '10px', 'padding': '5px',"background-color": valves()[i()].connected? (valves()[i()].feedback? "#22873D": "#C53434"): "#737373"}} >Actual</div>
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