export interface Sensor {
  name: string,
  group: string,
  board_id: number,
  channel_id: string,
  node_id: number,
  unit: string,
  value: number,
}

export interface Valve {
  name: string,
  group: string,
  board_id: number,
  channel_id: "Valve",
  node_id: number,
  open: boolean,
  feedback: boolean,
}