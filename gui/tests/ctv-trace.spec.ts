// Logic tests for what the CTV page plots. No browser or Tauri needed:
//   npx playwright test tests/ctv-trace.spec.ts
import { test, expect } from '@playwright/test';
import type { CtvNav, CtvSample, CtvStatus } from '../src/comm';
import {
  COMMAND,
  CONTROLLER,
  ESTIMATE,
  GPS_GATE_NIS,
  IDLE_WINDOW_S,
  MAX_SAMPLES,
  NAV_PICK,
  PICK,
  PREROLL_S,
  TraceBuffer,
  constant,
  navSeries,
  sensorBadges,
  series,
  xRange,
} from '../src/ctv/ctvTrace';

/** A sample as the flight computer sends it: the estimate only, zeros for the rest. */
const sample = (time_s: number, overrides: Partial<CtvSample> = {}): CtvSample => ({
  available: ESTIMATE,
  time_s,
  mission_s: 0,
  position_ned_m: [0, 0, 0],
  velocity_ned_mps: [0, 0, 0],
  body_rate_dps: [0, 0, 0],
  tilt_deg: 0,
  pad_position_m: [0, 0, 0],
  pad_velocity_mps: [0, 0, 0],
  reference_position_m: [0, 0, 0],
  reference_velocity_mps: [0, 0, 0],
  command: [0, 0, 0, 0],
  ...overrides,
});

const nav = (overrides: Partial<CtvNav> = {}): CtvNav => ({
  health: 'Healthy',
  has_estimate: true,
  has_gps_nis: true,
  position_std_m: [0.02, 0.03, 0.01],
  velocity_std_mps: [0.05, 0.06, 0.04],
  tilt_std_deg: 0.5,
  heading_std_deg: 1,
  sensor_results: [1, 1, 1, -1, -1, -1, -1],
  gps_nis: 3,
  ...overrides,
});

const status = (samples: CtvSample[], navState: CtvNav = nav()): CtvStatus => ({
  mode: 'Idle',
  mission_time_s: 0,
  schedule_time_s: 0,
  schedule_end_s: 21,
  altitude_m: 0,
  tilt_deg: 0,
  speed_mps: 0,
  ec_connected: true,
  ec_state: 'Disarmed',
  ec_safing_reason: '',
  fault: '',
  last_event: '',
  gains: '',
  missed_ticks: 0,
  preflight: [],
  can_start: false,
  nav: navState,
  tvc_limit_deg: 8,
  samples,
});

/** Samples every 10 ms from `from` for `count` periods. */
const run = (from: number, count: number, make: (t: number) => Partial<CtvSample> = () => ({})) =>
  Array.from({ length: count }, (_, i) => {
    const t = from + i * 0.01;
    return sample(t, make(t));
  });

test.describe('the buffer', () => {
  test('takes each sample once, however often a status is repeated', () => {
    const buffer = new TraceBuffer();
    const batch = run(1, 10);

    expect(buffer.append(status(batch))).toBe(10);
    expect(buffer.append(status(batch))).toBe(0);
    expect(buffer.append(status(batch.slice(5).concat(run(1.1, 5))))).toBe(5);
    expect(buffer.samples.length).toBe(15);
  });

  test('a restarted flight computer, whose clock starts again from zero, is taken from its first sample', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(500, 100, (t) => ({ mission_s: t - 500 + 0.01 }))));
    expect(buffer.samples.length).toBe(100);
    expect(buffer.flightStart).not.toBeNull();

    // The new process counts from zero: its samples are not older repeats of the last run's
    const after = run(0.5, 10);
    expect(buffer.append(status(after))).toBe(10);
    expect(buffer.samples.length).toBe(10);
    expect(buffer.latestTime()).toBeCloseTo(0.59);
    expect(buffer.flightStart).toBeNull();

    // and it carries on as before: repeats are still taken once
    expect(buffer.append(status(after))).toBe(0);
    expect(buffer.append(status(run(0.6, 5)))).toBe(5);
  });

  test('a repeated or slightly older window is not a restart', () => {
    const buffer = new TraceBuffer();
    const batch = run(100, 100);

    buffer.append(status(batch));
    expect(buffer.append(status(batch.slice(0, 50)))).toBe(0);
    expect(buffer.samples.length).toBe(100);
  });

  test('a status with no samples adds nothing, and an old flight computer sends none', () => {
    const buffer = new TraceBuffer();
    const old = status([]);

    delete old.samples;
    expect(buffer.append(status([]))).toBe(0);
    expect(buffer.append(old)).toBe(0);
    expect(buffer.view().samples).toEqual([]);
  });

  test('a status with no samples leaves the history already taken alone', () => {
    const buffer = new TraceBuffer();
    const batch = run(100, 100);
    const old = status([]);

    delete old.samples;
    buffer.append(status(batch, nav({ health: 'Healthy' })));

    // an empty frame (no new samples yet, or an old flight computer) is not a restart
    expect(buffer.append(status([]))).toBe(0);
    expect(buffer.append(old)).toBe(0);
    expect(buffer.samples.length).toBe(100);
    expect(buffer.nav.length).toBe(1);
    expect(buffer.latestTime()).toBeCloseTo(100.99);

    // and the run carries on from where it was
    expect(buffer.append(status(run(101, 5)))).toBe(5);
    expect(buffer.samples.length).toBe(105);
  });

  test('keeps the filter state of each status, at the time of its last sample', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(1, 10), nav({ health: 'Leveling' })));
    buffer.append(status(run(1.1, 10), nav({ health: 'Healthy' })));

    expect(buffer.nav.map((n) => n.nav.health)).toEqual(['Leveling', 'Healthy']);
    expect(buffer.nav[0].time_s).toBeCloseTo(1.09, 6);
  });

  test('holds at most five minutes of samples', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(0, MAX_SAMPLES + 500)));
    expect(buffer.samples.length).toBe(MAX_SAMPLES);
    // the newest are the ones kept
    expect(buffer.latestTime()).toBeCloseTo((MAX_SAMPLES + 499) * 0.01, 6);
  });

  test('clearing empties it, and what was seen can be taken again', () => {
    const buffer = new TraceBuffer();
    const batch = run(1, 10);

    buffer.append(status(batch));
    buffer.clear();
    expect(buffer.view().samples).toEqual([]);
    expect(buffer.append(status(batch))).toBe(10);
  });
});

test.describe('the view', () => {
  test('before any flight it is the last minute, counting back from now', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(0, 12000)));

    const view = buffer.view();

    expect(view.inFlight).toBe(false);
    expect(view.t0).toBeCloseTo(119.99, 6);
    expect(view.samples[0].time_s).toBeGreaterThanOrEqual(119.99 - IDLE_WINDOW_S);
    expect(view.samples.length).toBeLessThanOrEqual(IDLE_WINDOW_S * 100 + 1);
    expect(series(view, PICK.tilt).at(-1)?.x).toBe(0);
    expect(xRange(view, 21)).toEqual({ min: -IDLE_WINDOW_S, max: 0 });
  });

  test('in a flight it is the flight, with x counted from the mission start', () => {
    const buffer = new TraceBuffer();

    // 10 s on the pad, then a flight that has been going for 3 s
    buffer.append(status(run(100, 1000)));
    buffer.append(status(run(110, 300, (t) => ({ mission_s: t - 110 + 0.01 }))));

    const view = buffer.view();

    expect(view.inFlight).toBe(true);
    // the mission started 0.01 s before the first sample with a clock
    expect(view.t0).toBeCloseTo(109.99, 3);
    // a little of the pad before it, none of the earlier pad
    expect(view.samples[0].time_s).toBeGreaterThanOrEqual(109.99 - PREROLL_S - 1e-9);
    const first = series(view, PICK.tilt)[0].x;

    expect(first).toBeGreaterThanOrEqual(-PREROLL_S - 1e-9);
    expect(first).toBeLessThan(-PREROLL_S + 0.02);
    expect(xRange(view, 21)).toEqual({ min: -PREROLL_S, max: 21 });
  });

  test('the axis grows past the schedule if the flight runs over it', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(10, 3000, (t) => ({ mission_s: t - 10 + 0.01 }))));
    expect(xRange(buffer.view(), 21).max).toBeCloseTo(30, 3);
  });

  test('a second flight replaces the first when the mission clock starts over', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(10, 500, (t) => ({ mission_s: t - 10 + 0.01 }))));
    expect(buffer.flightStart).toBeCloseTo(9.99, 3);

    buffer.append(status(run(40, 300, (t) => ({ mission_s: t - 40 + 0.01 }))));
    expect(buffer.flightStart).toBeCloseTo(39.99, 3);
    expect(buffer.view().t0).toBeCloseTo(39.99, 3);
    expect(buffer.view().samples[0].time_s).toBeGreaterThanOrEqual(39.99 - PREROLL_S - 1e-9);
  });

  test('an abort that moves the schedule does not move the flight start', () => {
    const buffer = new TraceBuffer();

    // the mission clock is the one that counts, and it runs on
    buffer.append(status(run(10, 500, (t) => ({ mission_s: t - 10 + 0.01 }))));
    buffer.append(status(run(15, 500, (t) => ({ mission_s: t - 10 + 0.01 }))));
    expect(buffer.flightStart).toBeCloseTo(9.99, 3);
  });
});

test.describe('the series', () => {
  test('a value that is not there is a gap, not zero', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(1, 3)));

    const points = series(buffer.view(), PICK.altitude);

    expect(points.map((p) => p.y)).toEqual([null, null, null]);
  });

  test('what the flight computer says it does not have is a gap, whatever its zeros say', () => {
    const buffer = new TraceBuffer();
    const s = run(1, 3);

    // the estimate is there in all three; the command in the last only
    s[2].available = ESTIMATE | COMMAND;
    s[2].command = [12.5, 1, -1, 0];
    buffer.append(status(s));

    const view = buffer.view();

    expect(series(view, PICK.thrust).map((p) => p.y)).toEqual([null, null, 12.5]);
    expect(series(view, PICK.tilt).map((p) => p.y)).toEqual([0, 0, 0]);
    // and a sample with no estimate has no estimate to plot
    const bare = new TraceBuffer();

    bare.append(status([sample(1, { available: 0, tilt_deg: 7 })]));
    expect(series(bare.view(), PICK.tilt)[0].y).toBeNull();
  });

  test('a null that reaches the page anyway is still a gap', () => {
    const buffer = new TraceBuffer();
    const s = run(1, 2);

    (s[0].command as unknown as (number | null)[])[0] = null;
    s[0].available = ESTIMATE | COMMAND;
    s[1].available = ESTIMATE | COMMAND;
    s[1].command[0] = 12.5;
    buffer.append(status(s));

    expect(series(buffer.view(), PICK.thrust).map((p) => p.y)).toEqual([null, 12.5]);
  });

  test('the estimate is plotted up, though the navigation frame is down', () => {
    const buffer = new TraceBuffer();

    buffer.append(
      status(run(1, 1, () => ({ position_ned_m: [1, 2, -3], velocity_ned_mps: [0, 0, -0.5] }))),
    );

    const view = buffer.view();

    expect(series(view, PICK.estNorth)[0].y).toBe(1);
    expect(series(view, PICK.estEast)[0].y).toBe(2);
    expect(series(view, PICK.estUp)[0].y).toBe(3);
    expect(series(view, PICK.velUp)[0].y).toBe(0.5);
  });

  test('errors are what the controller was given minus the schedule', () => {
    const buffer = new TraceBuffer();

    buffer.append(
      status(
        run(1, 1, () => ({
          available: ESTIMATE | CONTROLLER,
          pad_position_m: [0.3, -0.4, 2.1],
          reference_position_m: [0, 0, 2.0],
          pad_velocity_mps: [0.1, 0, 0.45],
          reference_velocity_mps: [0, 0, 0.5],
        })),
      ),
    );

    const view = buffer.view();

    expect(series(view, PICK.northError)[0].y).toBeCloseTo(0.3, 6);
    expect(series(view, PICK.eastError)[0].y).toBeCloseTo(-0.4, 6);
    expect(series(view, PICK.upError)[0].y).toBeCloseTo(0.1, 6);
    expect(series(view, PICK.horizontalError)[0].y).toBeCloseTo(0.5, 6);
    expect(series(view, PICK.velUpError)[0].y).toBeCloseTo(-0.05, 6);
  });

  test('the filter uncertainties combine the horizontal axes and keep the vertical', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(1, 2), nav({ position_std_m: [0.3, 0.4, 0.05], velocity_std_mps: [0.06, 0.08, 0.02] })));

    const view = buffer.view();

    expect(navSeries(view, NAV_PICK.positionHorizontal)[0].y).toBeCloseTo(0.5, 6);
    expect(navSeries(view, NAV_PICK.positionVertical)[0].y).toBeCloseTo(0.05, 6);
    expect(navSeries(view, NAV_PICK.velocityHorizontal)[0].y).toBeCloseTo(0.1, 6);
  });

  test('a filter with no estimate yet has no uncertainty to plot', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(1, 2), nav({ has_estimate: false, position_std_m: [0, 0, 0] })));
    expect(navSeries(buffer.view(), NAV_PICK.positionHorizontal)[0].y).toBeNull();
    expect(navSeries(buffer.view(), NAV_PICK.tiltStd)[0].y).toBeNull();
  });

  test('a GPS innovation that is not there is a gap, and the gate is a line across the view', () => {
    const buffer = new TraceBuffer();

    buffer.append(status(run(1, 2), nav({ has_gps_nis: false, gps_nis: 0 })));

    const view = buffer.view();

    expect(navSeries(view, NAV_PICK.gpsNis)[0].y).toBeNull();

    const gate = constant(view, GPS_GATE_NIS);

    expect(gate.map((p) => p.y)).toEqual([GPS_GATE_NIS, GPS_GATE_NIS]);
    expect(gate[0].x).toBeLessThan(gate[1].x);
    expect(constant({ ...view, samples: [] }, 1)).toEqual([]);
  });
});

test.describe('the sensor badges', () => {
  test('say taken, rejected or nothing yet, for each of the seven', () => {
    const badges = sensorBadges(nav({ sensor_results: [1, 0, -1, 1, 1, 0, -1] }));

    expect(badges.map((b) => `${b.name}: ${b.result}`)).toEqual([
      'Baro: taken',
      'GPS: rejected',
      'Mag: none',
      'LiDAR 1: taken',
      'LiDAR 2: taken',
      'LiDAR 3: rejected',
      'LiDAR 4: none',
    ]);
  });

  test('all say nothing without a status from the filter', () => {
    expect(sensorBadges(undefined).every((b) => b.result === 'none')).toBe(true);
    expect(sensorBadges(undefined).length).toBe(7);
  });
});
