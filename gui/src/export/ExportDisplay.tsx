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

async function exportData(startTime: string, entTime: string) {
  exportLog(startTime, entTime, false);
}

function ExportDisplay() {
  const [exportStartTimestamp, setExportStartTimestamp] = createSignal("12:00");
  const [exportEndTimestamp, setExportEndTimestamp] = createSignal("13:00");
  
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="EXPORT" />
    </div>
    <div class="export-view">
      <div style={{ width: "100%", margin: "10px", gap: "20px", display: "flex", "justify-content": "center" }}>
        <input type="time" value={exportStartTimestamp()} onChange={(e) => setExportStartTimestamp(e.target.value)} />
        <input type="time" value={exportEndTimestamp()} onChange={(e) => setExportEndTimestamp(e.target.value)} />
      </div>
      <div style={{ width: "100%", display: "flex", "justify-content": "center" }}>
        <button class="sam-button" onClick={() => exportData(exportStartTimestamp(), exportEndTimestamp())}> Export to CSV </button></div>
    </div>
    <div>
      <Footer />
    </div>
  </div>
}

export default ExportDisplay;
