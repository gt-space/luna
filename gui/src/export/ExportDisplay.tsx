import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/core";
import { getCurrentWebviewWindow } from "@tauri-apps/api/webviewWindow";
import { createSignal } from "solid-js";
import { exportLog } from "../exportData";
const appWindow = getCurrentWebviewWindow()

listen('state', (event) => {

});

invoke('initialize_state', { window: appWindow });

async function exportData(startTime: Date) {
  const endTime = new Date();
  exportLog(startTime.toDateString(), endTime.toDateString(), servo_output_path, all)
}

function ExportDisplay() {
  const [beginRecordingTimestamp, setBeginRecordingTimestamp] = createSignal(null);
  const [recording, setRecording] = createSignal(false);

  if (recording()) {
    return <div class="window-template">
      <div style="height: 60px">
        <GeneralTitleBar name="EXPORT" />
      </div>
      <div class="export-view">
        <div style={{ width: "100%", display: "flex", "justify-content": "center" }}>
          <button class="sam-button" onClick={() => {
            setRecording(false);
            
          }}> End Recording </button></div>
      </div>
      <div>
        <Footer />
      </div>
    </div>
  }
  
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="EXPORT" />
    </div>
    <div class="export-view">
      <div style={{ width: "100%", display: "flex", "justify-content": "center" }}>
        <button class="sam-button" onClick={() => setRecording(true)}> Begin Recording </button></div>
    </div>
    <div>
      <Footer />
    </div>
  </div>
}

export default ExportDisplay;
