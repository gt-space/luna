import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import { Select } from "@thisbeyond/solid-select";
import { emit, listen } from "@tauri-apps/api/event";
import { createSignal, For} from "solid-js";
import ValveView from "./ValveView";
import { Valve } from "../devices";
import { closeValve, openValve } from "../commands";

function Valves() {
  const [valves, setValves] = createSignal(
    [
      {
        name: 'Valve1',
        group: 'Fuel',
        board_id: 0,
        channel_id: "Valve",
        node_id: 1,
        open: false,
        feedback: false,
      },
      {
        name: 'Valve2',
        group: 'Fuel',
        board_id: 0,
        channel_id: "Valve",
        node_id: 1,
        open: false,
        feedback: false,
      },
      {
        name: 'Valve3',
        group: 'Fuel',
        board_id: 0,
        channel_id: "Valve",
        node_id: 1,
        open: false,
        feedback: false,
      },
      {
        name: 'Valve4',
        group: 'Fuel',
        board_id: 0,
        channel_id: "Valve",
        node_id: 1,
        open: false,
        feedback: false,
      },
    ]
  );
  
  listen('valveUpdate', async (event) => {
    let valvelist: Valve[] = valves() as Valve[];
    let valve = valvelist.at(event.payload as number)!
    console.log(valve);
    if (valve.open) {
      console.log('sending command to close');
      await closeValve(valve.name);
      valve.open = false;
    } else {
      console.log('sending command to open');
      await openValve(valve.name);
      valve.open = true;
    }
    setValves(valvelist);
    console.log(valves());
  });

  const [seqButtonLabel, setSeqButtonLabel] = createSignal('Start Sequence');
  const [seqRunning, setSeqRunning] = createSignal(false);
  function toggleSequenceButton() {
    var button = document.getElementById("sequencebutton")!;
    if (seqRunning()) {
      setSeqButtonLabel('Start Sequence');
      setSeqRunning(false);
      button.style.backgroundColor = "#015878"
      button.style.setProperty('seqButtonBackgroundColor',  '#00425a!important');
    } else {
      setSeqButtonLabel('Abort Sequence');
      setSeqRunning(true);
      button.style.backgroundColor = "#C53434"
    }
  }

  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="Valves"/>
    </div>
    <div class='valve-view'>
      <div class='sequences-panel'>
        <select
          class="sequences-dropdown"
          onChange={(e) => {
            console.log(e?.target.className);
          }}
        >
          <option class="seq-dropdown-item" value="seq1">Sequence 1</option>
          <option class="seq-dropdown-item" value="seq2">Sequence 2</option>
          <option class="seq-dropdown-item" value="seq3">Sequence 3</option>
          <option class="seq-dropdown-item" value="seq4">Sequence 4</option>
          <option class="seq-dropdown-item" value="seq5">Sequence 5</option>
          <option class="seq-dropdown-item" value="seq6">Sequence 6</option>
        </select>
        <div style={{flex: 1, padding: '5px'}}>
          <button class='toggle-sequence-button' id="sequencebutton" onClick={toggleSequenceButton}>
            {seqButtonLabel()}
          </button>
        </div>
      </div>
      <ValveView valves={valves() as Valve[]}/>
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default Valves;