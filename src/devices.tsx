export interface Device {
  name: string,
  group: string,
  board_id: number,
  channel_type: string,
  channel: number,
  unit: string,
  value: number,
}

export interface Valve {
  name: string,
  group: string,
  board_id: number,
  channel_type: "Valve",
  channel: number,
  open: boolean,
  feedback: boolean,
}

export interface GenericDevice {
  seconds: number,
  nanos: number,
  micros: number,
  floatValue: number,
  boolValue: boolean,
  kind: number,
  board_id: number, 
  channel: number
}