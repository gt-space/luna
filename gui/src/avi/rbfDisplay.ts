/** SAM RBF: 0 = installed, 1 = removed */
export function formatSamRbf(value: number): string {
  if (value === 0) return "Installed";
  if (value === 1) return "Removed";
  return String(value);
}

/** BMS RBF tag: 1 = installed, 0 = removed. Non-binary values shown numerically. */
export function formatBmsRbf(value: number): string {
  if (value === 0) return "Removed";
  if (value === 1) return "Installed";
  return value.toFixed(4);
}

/** RECO RBF: 1 = installed, 0 = removed */
export function formatRecoRbf(value: number): string {
  if (value === 0) return "Removed";
  if (value === 1) return "Installed";
  return String(value);
}

/** E-stop: 1 = engaged, 0 = disengaged */
export function formatEstop(value: number): string {
  if (value === 0) return "Disengaged";
  if (value === 1) return "Engaged";
  return value.toFixed(4);
}

const RBF_STATUS_GREEN = "#7dd3a0";
const RBF_STATUS_RED = "#e57373";

/** Text color for RBF / E-stop status labels shown in the GUI. */
export function rbfStatusTextColor(label: string): string | undefined {
  const k = label.toLowerCase();
  if (k === "engaged" || k === "installed") return RBF_STATUS_GREEN;
  if (k === "disengaged" || k === "removed") return RBF_STATUS_RED;
  return undefined;
}