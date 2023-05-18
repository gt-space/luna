import { SERVER_PORT } from "./appdata";
import { serverIp, sessionId } from "./comm";

export enum Channel {
  LED,
  VALVE,
  TC,
  PT,
  GPIO,
}

export enum Command {
  OFF,
  ON,
}

export async function sendCommand(board: number, channel: Channel, node: number, command: Command) {
  console.log(serverIp());
  console.log(sessionId())
  try {
    const response = await fetch(`http://${serverIp()}:${SERVER_PORT}/commands`, {
      headers: new Headers({
        'Authorization': sessionId() as string
      }),
      method: 'POST',
      body: JSON.stringify({
        'board': board,
        'channel': channel,
        'node_id': node,
        'command': command,
      }),
    });
    console.log(response);
    return response.json();
  } catch(e) {
    return e;
  }
}

export async function turnOnLED() {
  try {
    await sendCommand(0, Channel.LED, 0, Command.ON);
  } catch(e) {
    console.log(e);
  }
}

export async function turnOffLED() {
  try {
    await sendCommand(0, Channel.LED, 0, Command.OFF);
  } catch(e) {
    console.log(e);
  }
}
