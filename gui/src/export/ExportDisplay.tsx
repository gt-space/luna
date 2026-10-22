import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/core";
import { getCurrentWebviewWindow } from "@tauri-apps/api/webviewWindow";
import { createSignal } from "solid-js";
import { save, } from "@tauri-apps/plugin-dialog";
import { writeTextFile } from "@tauri-apps/plugin-fs";

const appWindow = getCurrentWebviewWindow()

listen('state', (event) => {

});

invoke('initialize_state', { window: appWindow });

import { SERVER_PORT } from "./../appdata";
import { serverIp, sessionId } from "./../comm";

// sends a generic command to the servers
export async function sendExportLogCommand(body: object): Promise<string> {
  const response = await fetch(`http://${serverIp()}:${SERVER_PORT}/operator/export-log`, {
    headers: new Headers({
      'Authorization': sessionId() as string,
      'Content-Type': 'application/json;charset=utf-8'
    }),
    method: 'POST',
    body: JSON.stringify(body),
  });
  return response.text();
}

function ExportDisplay() {
  const [exportStartTimestamp, setExportStartTimestamp] = createSignal(new Date());
  const [exportEndTimestamp, setExportEndTimestamp] = createSignal(new Date());
  const [exportAll, setExportAll] = createSignal(false);
  const [awaitingExport, setAwaitingExport] = createSignal(false);

  function formatForServoTimestampExport(timestamp: Date): string {
    console.log(timestamp);
    const year = timestamp.getUTCFullYear();
    const day = timestamp.getDate();
    const month = timestamp.getMonth() + 1;
    const hours = timestamp.getHours();
    const minutes = timestamp.getMinutes();

    //IMPORTANT: the format is YYYY-MM-DD HH:MM according to the YJSP avionics wiki for servo
    const result = `${month}/${day}/${year} ${hours}:${minutes}`;
    return result;
  }

  function handleDateTimeChangeDate(value: string, setDateSignal: (d: Date) => void): void {
    const [year, month, dat] = value.split('-').map(Number);
    const parsedDate = new Date(year, month - 1, dat);

    if (!isNaN(parsedDate.getTime())) {
      setDateSignal(parsedDate);
    }
  }

  function handleDateTimeChangeTime(value: string, getDateSignal: () => Date): void {
    const hh = parseInt(value.split(':')[0]);
    const mm = parseInt(value.split(':')[1]);
    getDateSignal().setHours(hh, mm, 0, 0);
  }

  function formatForDateInput(dateObj: Date): string {
    if (!dateObj || isNaN(dateObj.getTime())) return "";
    
    const pad = (n: number) => String(n).padStart(2, '0');
    const YYYY = dateObj.getFullYear();
    const MM = pad(dateObj.getMonth() + 1);
    const DD = pad(dateObj.getDate());
    
    return `${YYYY}-${MM}-${DD}`;
  };

  function formatForTimeInput(dateObj: Date): string {
    if (!dateObj || isNaN(dateObj.getTime())) return "";
    
    const pad = (n: number) => String(n).padStart(2, '0');
    const hh = pad(dateObj.getHours());
    const mm = pad(dateObj.getMinutes());
    
    return `${hh}:${mm}`;
  };

  async function exportData(startTime: Date, endTime: Date) {
    try {
      setAwaitingExport(true);
      
      const result = await sendExportLogCommand({
        "start_time": formatForServoTimestampExport(startTime),
        "end_time": formatForServoTimestampExport(endTime),
        "all": exportAll(),
      })

      setAwaitingExport(false);
      
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
      <div style={{ width: "100%", gap: "20px", display: "flex", "justify-content": "center" }}>
        <div style={{ display: "flex", gap: "10px" }}>
          <input type="date" value={formatForDateInput(exportStartTimestamp())} onChange={(e) => handleDateTimeChangeDate(e.target.value, setExportStartTimestamp)} />
          <input type="time" value={formatForTimeInput(exportStartTimestamp())} onChange={(e) => handleDateTimeChangeTime(e.target.value, exportStartTimestamp)} />
        </div>
        <div style={{ display: "flex", gap: "10px" }}>
          <input type="date" value={formatForDateInput(exportEndTimestamp())} onChange={(e) => handleDateTimeChangeDate(e.target.value, setExportEndTimestamp)} />
          <input type="time" value={formatForTimeInput(exportEndTimestamp())} onChange={(e) => handleDateTimeChangeTime(e.target.value, exportEndTimestamp)} />
        </div>
      </div>
      <div style={{ width: "100%", display: "flex", "justify-content": "center" }}>
        <button disabled={awaitingExport()} class="sam-button" onClick={() => exportData(exportStartTimestamp(), exportEndTimestamp())}>{awaitingExport() ? "Exporting..." : "Export to CSV"}</button>
      </div>
    </div>
    <div>
      <Footer />
    </div>
  </div>
}

export default ExportDisplay;
