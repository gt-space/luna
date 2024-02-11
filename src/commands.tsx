import { SERVER_PORT } from "./appdata";
import { serverIp, sessionId } from "./comm";

// sends a generic command to the servers
export async function sendCommand(command: object) {
  console.log(serverIp());
  console.log(sessionId())
  try {
    const response = await fetch(`http://${serverIp()}:${SERVER_PORT}/operator/command`, {
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

// command to turn on LED
export async function turnOnLED() {
  try {
    await sendCommand({
      "command": "set_led",
      "target": "led0",
      "state": "on"
    });
  } catch(e) {
    console.log(e);
  }
}

// command to turn off LED
export async function turnOffLED() {
  try {
    await sendCommand({
      "command": "set_led",
      "target": "led0",
      "state": "off"
    });
  } catch(e) {
    console.log(e);
  }
}

// command to click valve open
export async function openValve(name: string) {
  try {
    await sendCommand({
      "command": "click_valve",
      "target": name,
      "state": "open"
    })
  } catch(e) {
    console.log(e);
  }
}

// command to click valave close
export async function closeValve(name: string) {
  try {
    await sendCommand({
      "command": "click_valve",
      "target": name,
      "state": "closed"
    })
  } catch(e) {
    console.log(e);
  }
}
