export interface Device {
  name: string,
  group: string,
  board_id: string,
  channel_type: string,
  channel: number,
  unit: string,
  value: number,
  offset: number,
}

export interface Valve {
  name: string,
  group: string,
  board_id: string,
  channel_type: string,
  channel: number,
  open: boolean,
  feedback: boolean,
  connected: boolean,
}