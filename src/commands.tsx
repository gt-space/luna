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
  ON,
  OFF,
}

export async function sendCommand(board: number, channel: Channel, node: number, command: Command) {
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
  await sendCommand(0, Channel.LED, 0, Command.ON);
}

export async function turnOffLED() {
  await sendCommand(0, Channel.LED, 0, Command.OFF);
}
