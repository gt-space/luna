import { For, createSignal, onCleanup, onMount } from "solid-js";
import { GeneralTitleBar } from "../general-components/TitleBar";
import { AbortStage, State, serverIp, runAbortStage, AbortStageMapping, getAbortStages, StreamState } from "../comm";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import Footer from "../general-components/Footer";

const [abortStages, setAbortStages] = createSignal();
const [activeAbortStage, setActiveAbortStage] = createSignal("");
const [dispatchFeedback, setDispatchFeedback] = createSignal("");
const [feedbackColor, setFeedbackColor] = createSignal("black");

listen('state', async (event) => {
  setAbortStages((event.payload as State).abortStages);
  setActiveAbortStage((event.payload as State).activeAbortStage);
});
  
invoke('initialize_state', {window: appWindow});



listen('device_update', (event) => {
  const rawAbortStage = (event.payload as StreamState).abort_stage;
  const abortStage = typeof rawAbortStage === "string"
    ? JSON.parse(rawAbortStage)
    : rawAbortStage;
  setActiveAbortStage(abortStage?.name);
})

async function dispatchAbortStage() {
  const stageDropdown = document.getElementById("stageselect")! as HTMLSelectElement;
  console.log(stageDropdown.value);
  const result = await runAbortStage(serverIp() as string, stageDropdown.value);

  if (result.success) {
    setFeedbackColor("green");
    setDispatchFeedback("Successfully Dispatched: " + stageDropdown.value);
    await new Promise(resolve => setTimeout(resolve, 1500));
    setDispatchFeedback("");
  } else {
    setFeedbackColor("red");
    setDispatchFeedback("Dispatch FAILED: " + JSON.stringify(result.error));
    await new Promise(resolve => setTimeout(resolve, 1500));
    setDispatchFeedback("");
  }
}

function AbortStages() {
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="Abort Stages"/>
    </div>
    <div class='sequences-view'>
      <div class='sequences-panel'>
        <select
          id="stageselect"
          class="sequences-dropdown"
          onChange={(e) => {
            console.log(e?.target.className);
          }}
        >
          <For each={abortStages() as AbortStage[]}>{(stage, i) =>
              <option class="seq-dropdown-item" value={stage.id}>{stage.id}</option>
            }
          </For>
        </select>
        <div style={{flex: 1, padding: '5px'}}>
          <button class='toggle-sequence-button' id="abortstagebutton" onClick={dispatchAbortStage}>
            Dispatch Abort Stage
          </button>
        </div>
      </div>
      <div
      style={{
        "text-align": "center",
        "font-size": "14px",
        color: feedbackColor(),
        "margin-top": "10px",
        "align-self": "center"
      }}
    >
      {dispatchFeedback()}
    </div> 
    <div style={{"margin-top": "5px", "text-align": "center", width: "100%"}}>Current Abort Stage:</div>
    <div class="abort-dispatch-view-section">
      {activeAbortStage()}
    </div>
    </div>
    <div>
      <Footer/>
    </div>
  </div>
}
  
export default AbortStages;