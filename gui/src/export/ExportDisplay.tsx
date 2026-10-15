import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/core";
import { getCurrentWebviewWindow } from "@tauri-apps/api/webviewWindow";
import { createSignal } from "solid-js";
import { sendExportLogCommand } from "../exportData";
import { save, } from "@tauri-apps/plugin-dialog";
import { writeTextFile } from "@tauri-apps/plugin-fs";

const appWindow = getCurrentWebviewWindow()

listen('state', (event) => {

});

invoke('initialize_state', { window: appWindow });

function ExportDisplay() {
  const [exportStartTimestamp, setExportStartTimestamp] = createSignal(new Date());
  const [exportEndTimestamp, setExportEndTimestamp] = createSignal(new Date());

  function formatExportTimestamp(timestamp: Date): string {
    console.log(timestamp);
    const year = timestamp.getUTCFullYear();
    const day = timestamp.getDay();
    const month = timestamp.getMonth() + 1;
    const hours = timestamp.getHours();
    const minutes = timestamp.getMinutes();

    //IMPORTANT: the format is YYYY-MM-DD HH:MM according to the YJSP avionics wiki for servo
    const result = `${month}/${day}/${year} ${hours}:${minutes}`;
    return result;
  }

  function formatForInput(dateObj: Date): string {
    if (!dateObj || isNaN(dateObj.getTime())) return "";
    
    const pad = (n: number) => String(n).padStart(2, '0');
    const YYYY = dateObj.getFullYear();
    const MM = pad(dateObj.getMonth() + 1);
    const DD = pad(dateObj.getDate());
    const hh = pad(dateObj.getHours());
    const mm = pad(dateObj.getMinutes());
    
    return `${YYYY}-${MM}-${DD}T${hh}:${mm}`;
  };

  async function exportData(startTime: Date, endTime: Date) {
    try {
      const result = await sendExportLogCommand({
        "start_time": formatExportTimestamp(startTime),
        "end_time": formatExportTimestamp(endTime),
        "all": false,
      })

      const saveFilePath = await save({
        defaultPath: `data_export_${startTime.getTime()}_${endTime.getTime()}.csv`,
        filters: [
          { name: `data_export_${startTime.getTime()}_${endTime.getTime()}`, extensions: ["csv"] }
        ]
      })

      if (saveFilePath) {
        await writeTextFile(saveFilePath, result);
      } else {
        console.error("could not create file");
      }
      
    } catch (e) {
      console.error(e)
    }
  }
  
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="EXPORT" />
    </div>
    <div class="export-view">
      <div style={{ width: "100%", margin: "10px", gap: "20px", display: "flex", "justify-content": "center" }}>
        <input type="datetime-local" value={formatForInput(exportStartTimestamp())} onChange={(e) => setExportStartTimestamp(new Date(e.target.value))} />
        <input type="datetime-local" value={formatForInput(exportEndTimestamp())} onChange={(e) => setExportEndTimestamp(new Date(e.target.value))} />
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
