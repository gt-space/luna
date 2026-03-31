/** SAM RBF: 1 = installed, 0 = removed */
export function formatSamRbf(value: number): string {
  if (value === 0) return "Removed";
  if (value === 1) return "Installed";
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