import { SERVER_PORT } from "./appdata";
import { serverIp, sessionId } from "./comm";

// sends a generic command to the servers
export async function sendExportLogCommand(body: object) {
  console.log(serverIp());
  console.log(sessionId())
  try {
    const response = await fetch(`http://${serverIp()}:${SERVER_PORT}/operator/export-log`, {
      headers: new Headers({
        'Authorization': sessionId() as string,
        'Content-Type': 'application/json;charset=utf-8'
      }),
      method: 'POST',
      body: JSON.stringify(body),
    });
    console.log(response);
    return response.json();
  } catch(e) {
    return e;
  }
}

// command to export flight log from servo

export async function exportLog(start_time: String, end_time: String, servo_output_path: String, all: boolean) {
  try {
    await sendExportLogCommand({
      "start_time": start_time,
      "end_time": end_time,
      "servo_output_path": servo_output_path,
      "all": all
    });
  } catch(e) {
    console.log(e);
  }
}
