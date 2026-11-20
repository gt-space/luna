export const PAGE_SIZE = 50;

export interface Source {
  path: string;
  name: string;
  rows: number;
  firstTimestamp: number;
  lastTimestamp: number;
  gapSeconds: number | null;
  warning: string | null;
}

export interface Dataset {
  id: number;
  columns: string[];
  sources: Source[];
  kind: "VehicleState" | "Imu";
  rows: number;
  firstTimestamp: number;
  lastTimestamp: number;
  notices: string[];
}

export interface Progress {
  label: string;
  done: number;
  total: number;
}

export interface Endpoint {
  host: string;
  username: string;
  port: number;
  folder: string;
}

export type ConnectStatus = { status: "verify"; fingerprint: string } | { status: "password" };

export interface Listing {
  path: string;
  entries: { name: string; directory: boolean; bytes: number }[];
}

export interface DownloadReport {
  files: string[];
  notices: string[];
}

export function utcTime(timestamp: number) {
  const date = new Date(timestamp * 1000);
  return Number.isNaN(date.getTime()) ? String(timestamp) : date.toISOString().replace("T", " ").replace("Z", " UTC");
}
