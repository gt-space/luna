import { SERVER_PORT } from "./appdata";
import { serverIp, sessionId } from "./comm";

// sends a generic command to the servers
export async function sendCommand(command: object) {
  console.log(serverIp());
  console.log(sessionId())
  try {
    const response = await fetch(`http://${serverIp()}:${SERVER_PORT}/operator/bms_command`, {
      headers: new Headers({
        'Authorization': sessionId() as string,
        'Content-Type': 'application/json;charset=utf-8' 
      }),
      method: 'POST',
      body: JSON.stringify(command),
    });
    console.log(response);
    return response.json();
  } catch(e) {
    return e;
  }
}


export async function openBMS(name: string) {
  try {
    await sendCommand({
      "command": "click_valve",
      "target": name,
      "state": "enable"
    })
  } catch(e) {
    console.log(e);
  }
}

export async function closeBMS(name: string) {
  try {
    await sendCommand({
      "command": "click_valve",
      "target": name,
      "state": "disable"
    })
  } catch(e) {
    console.log(e);
  }
}
