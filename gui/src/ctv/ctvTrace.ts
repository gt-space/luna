// What the CTV page plots, and the buffer that holds it. Pure logic, no Tauri or
// DOM, so it is tested on its own (tests/ctv-trace.spec.ts).
//
// The flight computer sends the control periods in batches with its status:
// `status.samples`. Telemetry frames come faster than the status does, and a
// sample is sent once, but the buffer still drops any it has already seen.
import type { CtvNav, CtvSample, CtvStatus } from "../comm";

/** Samples kept: five minutes at the 100 Hz control rate. */
export const MAX_SAMPLES = 30000;

/** Samples this far older than the newest one taken mean the flight computer's clock started over. */
export const RESTART_BACKWARDS_S = 1;
/** Seconds of history shown while there is no flight in the buffer. */
export const IDLE_WINDOW_S = 60;
/** Seconds of the pad shown before the start of a flight. */
export const PREROLL_S = 2;
/** The GPS gate: an innovation with a normalized square above it is rejected. */
export const GPS_GATE_NIS = 16.8;

/** A plotted point. `y` is null where the value was not available (a gap). */
export interface Point {
  x: number;
  y: number | null;
}

/** The filter's state at the time of a status frame (about 10 Hz). */
export interface NavPoint {
  time_s: number;
  nav: CtvNav;
}

/** What a chart is drawn from: the samples and filter points in view, and the time zero. */
export interface View {
  samples: CtvSample[];
  nav: NavPoint[];
  /** The flight computer's clock at x = 0 on the charts. */
  t0: number;
  /** Whether t0 is the start of a flight (otherwise the newest sample, with x going back from it). */
  inFlight: boolean;
}

const finite = (v: number | null | undefined): number | null =>
  typeof v === "number" && Number.isFinite(v) ? v : null;

/** Groups of values in a sample, by the bit that says they are valid (common::comm::ctv::CtvSample). */
export const ESTIMATE = 1;
export const CONTROLLER = 2;
export const COMMAND = 4;

const gap = (n: number): number[] => Array<number>(n).fill(Number.NaN);

/**
 * The flight computer sends zeros for what it does not have, with a flag (NaN
 * would not survive every format the status goes through). The plots want gaps:
 * a group that is not flagged becomes NaN here.
 */
export function normalizeSample(sample: CtvSample): CtvSample {
  const has = (bit: number) => (sample.available & bit) !== 0;

  return {
    ...sample,
    position_ned_m: has(ESTIMATE) ? sample.position_ned_m : gap(3),
    velocity_ned_mps: has(ESTIMATE) ? sample.velocity_ned_mps : gap(3),
    body_rate_dps: has(ESTIMATE) ? sample.body_rate_dps : gap(3),
    tilt_deg: has(ESTIMATE) ? sample.tilt_deg : Number.NaN,
    pad_position_m: has(CONTROLLER) ? sample.pad_position_m : gap(3),
    pad_velocity_mps: has(CONTROLLER) ? sample.pad_velocity_mps : gap(3),
    reference_position_m: has(CONTROLLER) ? sample.reference_position_m : gap(3),
    reference_velocity_mps: has(CONTROLLER) ? sample.reference_velocity_mps : gap(3),
    command: has(COMMAND) ? sample.command : gap(4),
  };
}

/** The filter's state with what it does not have as gaps. */
export function normalizeNav(nav: CtvNav): CtvNav {
  return {
    ...nav,
    position_std_m: nav.has_estimate ? nav.position_std_m : gap(3),
    velocity_std_mps: nav.has_estimate ? nav.velocity_std_mps : gap(3),
    tilt_std_deg: nav.has_estimate ? nav.tilt_std_deg : Number.NaN,
    heading_std_deg: nav.has_estimate ? nav.heading_std_deg : Number.NaN,
    gps_nis: nav.has_gps_nis ? nav.gps_nis : Number.NaN,
  };
}

export class TraceBuffer {
  samples: CtvSample[] = [];
  nav: NavPoint[] = [];
  /** The flight computer's clock at the mission start of the latest flight in the buffer. */
  flightStart: number | null = null;
  private lastTime = -Infinity;

  /** Takes the samples a status frame carries. Returns how many were new. */
  append(status: CtvStatus): number {
    // The flight computer's clock starts again from zero when it restarts, while this window
    // lives on: samples far older than the newest one taken are a new run, not repeats. (Servo
    // re-sends the last frame, whose samples are never older than what was already taken.) A frame
    // with no samples says nothing about the clock, and must leave the history alone.
    const times = (status.samples ?? []).map((s) => s.time_s);
    if (times.length > 0 && Math.max(...times) < this.lastTime - RESTART_BACKWARDS_S) {
      this.clear();
    }

    const incoming = (status.samples ?? [])
      .filter((s) => s.time_s > this.lastTime)
      .map(normalizeSample);

    for (const sample of incoming) {
      this.samples.push(sample);
      this.lastTime = sample.time_s;

      // the mission clock restarting is a new flight
      if (sample.mission_s > 0) {
        const start = sample.time_s - sample.mission_s;
        if (this.flightStart === null || Math.abs(start - this.flightStart) > 1) {
          this.flightStart = start;
        }
      }
    }

    if (incoming.length > 0 && status.nav) {
      this.nav.push({ time_s: incoming[incoming.length - 1].time_s, nav: normalizeNav(status.nav) });
    }

    if (this.samples.length > MAX_SAMPLES) {
      this.samples.splice(0, this.samples.length - MAX_SAMPLES);
    }
    if (this.nav.length > MAX_SAMPLES / 10) {
      this.nav.splice(0, this.nav.length - MAX_SAMPLES / 10);
    }

    return incoming.length;
  }

  clear(): void {
    this.samples = [];
    this.nav = [];
    this.flightStart = null;
    this.lastTime = -Infinity;
  }

  /** The newest sample's time, or null if there is none. */
  latestTime(): number | null {
    return this.samples.length > 0 ? this.samples[this.samples.length - 1].time_s : null;
  }

  /**
   * What to draw: the latest flight (with a little of the pad before it) when
   * the buffer holds one, otherwise the last minute, x counting back from now.
   */
  view(): View {
    const latest = this.latestTime();

    if (latest === null) {
      return { samples: [], nav: [], t0: 0, inFlight: false };
    }

    if (this.flightStart !== null) {
      const from = this.flightStart - PREROLL_S;

      return {
        samples: this.samples.filter((s) => s.time_s >= from),
        nav: this.nav.filter((n) => n.time_s >= from),
        t0: this.flightStart,
        inFlight: true,
      };
    }

    const from = latest - IDLE_WINDOW_S;

    return {
      samples: this.samples.filter((s) => s.time_s >= from),
      nav: this.nav.filter((n) => n.time_s >= from),
      t0: latest,
      inFlight: false,
    };
  }
}

/** One value of every sample as a series against time since t0, gaps where it is NaN. */
export function series(view: View, pick: (s: CtvSample) => number): Point[] {
  return view.samples.map((s) => ({ x: s.time_s - view.t0, y: finite(pick(s)) }));
}

/** One value of every filter point as a series against time since t0. */
export function navSeries(view: View, pick: (n: CtvNav) => number): Point[] {
  return view.nav.map((n) => ({ x: n.time_s - view.t0, y: finite(pick(n.nav)) }));
}

const neg = (v: number) => -v;
const hypot2 = (a: number, b: number) => Math.hypot(a, b);

/** Every series the page draws, by name. */
export const PICK = {
  // the estimate, in the navigation frame: north, east, up
  estNorth: (s: CtvSample) => s.position_ned_m[0],
  estEast: (s: CtvSample) => s.position_ned_m[1],
  estUp: (s: CtvSample) => neg(s.position_ned_m[2]),
  velNorth: (s: CtvSample) => s.velocity_ned_mps[0],
  velEast: (s: CtvSample) => s.velocity_ned_mps[1],
  velUp: (s: CtvSample) => neg(s.velocity_ned_mps[2]),
  tilt: (s: CtvSample) => s.tilt_deg,
  rateP: (s: CtvSample) => s.body_rate_dps[0],
  rateQ: (s: CtvSample) => s.body_rate_dps[1],
  rateR: (s: CtvSample) => s.body_rate_dps[2],

  // what the controller was given and what the schedule asked, pad frame
  altitude: (s: CtvSample) => s.pad_position_m[2],
  referenceAltitude: (s: CtvSample) => s.reference_position_m[2],
  northError: (s: CtvSample) => s.pad_position_m[0] - s.reference_position_m[0],
  eastError: (s: CtvSample) => s.pad_position_m[1] - s.reference_position_m[1],
  upError: (s: CtvSample) => s.pad_position_m[2] - s.reference_position_m[2],
  horizontalError: (s: CtvSample) =>
    hypot2(s.pad_position_m[0] - s.reference_position_m[0], s.pad_position_m[1] - s.reference_position_m[1]),
  velNorthError: (s: CtvSample) => s.pad_velocity_mps[0] - s.reference_velocity_mps[0],
  velEastError: (s: CtvSample) => s.pad_velocity_mps[1] - s.reference_velocity_mps[1],
  velUpError: (s: CtvSample) => s.pad_velocity_mps[2] - s.reference_velocity_mps[2],

  // what the controller asked for
  thrust: (s: CtvSample) => s.command[0],
  tvcPitch: (s: CtvSample) => s.command[1],
  tvcYaw: (s: CtvSample) => s.command[2],
  rollTorque: (s: CtvSample) => s.command[3],
};

/** The filter's uncertainty, from the status frames. */
export const NAV_PICK = {
  positionHorizontal: (n: CtvNav) => hypot2(n.position_std_m[0], n.position_std_m[1]),
  positionVertical: (n: CtvNav) => n.position_std_m[2],
  velocityHorizontal: (n: CtvNav) => hypot2(n.velocity_std_mps[0], n.velocity_std_mps[1]),
  velocityVertical: (n: CtvNav) => n.velocity_std_mps[2],
  tiltStd: (n: CtvNav) => n.tilt_std_deg,
  headingStd: (n: CtvNav) => n.heading_std_deg,
  gpsNis: (n: CtvNav) => n.gps_nis,
};

/** A horizontal line across the view, for a limit. */
export function constant(view: View, value: number): Point[] {
  if (view.samples.length === 0) return [];

  const first = view.samples[0].time_s - view.t0;
  const last = view.samples[view.samples.length - 1].time_s - view.t0;

  return [
    { x: first, y: value },
    { x: last, y: value },
  ];
}

export type SensorResult = "taken" | "rejected" | "none";

export interface SensorBadge {
  name: string;
  result: SensorResult;
}

/** The filter's latest result for each sensor: barometer, GPS, magnetometer, then the four LiDARs. */
export function sensorBadges(nav: CtvNav | undefined): SensorBadge[] {
  const names = ["Baro", "GPS", "Mag", "LiDAR 1", "LiDAR 2", "LiDAR 3", "LiDAR 4"];
  const results = nav?.sensor_results ?? [];

  return names.map((name, i) => ({
    name,
    result: results[i] === 1 ? "taken" : results[i] === 0 ? "rejected" : "none",
  }));
}

/** The range of the x axis: the flight's schedule while flying, otherwise what is in view. */
export function xRange(view: View, scheduleEnd: number): { min: number; max: number } {
  if (view.inFlight) {
    const last = view.samples.length > 0 ? view.samples[view.samples.length - 1].time_s - view.t0 : 0;

    return { min: -PREROLL_S, max: Math.max(scheduleEnd, last) };
  }

  return { min: -IDLE_WINDOW_S, max: 0 };
}
