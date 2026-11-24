// Logic tests for the CTV operations page. These need no browser or Tauri:
//   npx playwright test tests/ctv-logic.spec.ts
import { test, expect } from '@playwright/test';
import type { CtvStatus } from '../src/comm';
import {
  FlowContext,
  classifyTelemetry,
  commandTookEffect,
  deriveFlow,
  diffStatus,
  disabledReason,
  progressFraction,
} from '../src/ctv/ctvLogic';

const status = (overrides: Partial<CtvStatus> = {}): CtvStatus => ({
  mode: 'Idle',
  mission_time_s: 0,
  schedule_time_s: 0,
  schedule_end_s: 30,
  altitude_m: 0,
  tilt_deg: 0,
  speed_mps: 0,
  ec_connected: true,
  ec_state: 'Disarmed',
  ec_safing_reason: '',
  fault: '',
  last_event: '',
  gains: 'lqr @ abc1234',
  missed_ticks: 0,
  preflight: [
    { name: 'Navigation estimate', ok: true, detail: 'ok' },
    { name: 'EC link', ok: true, detail: 'ok' },
  ],
  can_start: true,
  ...overrides,
});

const live: FlowContext = { connected: true, fresh: true, sourceCarriesCtv: true };

test.describe('guided flow', () => {
  test('arm first when the EC is disarmed', () => {
    const flow = deriveFlow(status(), live);
    expect(flow.stage).toBe('arm');
    expect(flow.primary?.command).toBe('arm_ec');
    expect(flow.primary?.disabledReason).toBeUndefined();
  });

  test('arming is refused while the EC is not connected', () => {
    const flow = deriveFlow(status({ ec_connected: false }), live);
    expect(flow.primary?.command).toBe('arm_ec');
    expect(flow.primary?.disabledReason).toContain('not connected');
  });

  test('start flight needs every check to pass', () => {
    const blocked = status({
      ec_state: 'Armed',
      can_start: false,
      preflight: [{ name: 'Navigation estimate', ok: false, detail: 'no fix' }],
    });
    const flow = deriveFlow(blocked, live);
    expect(flow.stage).toBe('start');
    expect(flow.primary?.command).toBe('start_flight');
    expect(flow.primary?.disabledReason).toContain('Navigation estimate');

    const ready = deriveFlow(status({ ec_state: 'Armed' }), live);
    expect(ready.primary?.disabledReason).toBeUndefined();
    expect(ready.needsConfirm).toContain('start_flight');
    expect(ready.needsConfirm).toContain('terminate');
  });

  test('abort / land is the primary action in flight', () => {
    for (const mode of ['Flight'] as const) {
      const flow = deriveFlow(status({ mode, ec_state: 'Armed' }), live);
      expect(flow.stage).toBe('flying');
      expect(flow.primary?.command).toBe('abort_land');
      expect(flow.primary?.disabledReason).toBeUndefined();
    }
    const landing = deriveFlow(status({ mode: 'AbortLand', ec_state: 'Armed' }), live);
    expect(landing.primary?.disabledReason).toBe('Already landing.');
  });

  test('a safed vehicle offers clear safing, a landed one disarm', () => {
    const safed = deriveFlow(status({ mode: 'Safed', ec_state: 'Safing' }), live);
    expect(safed.primary?.command).toBe('clear_safing');
    expect(safed.primary?.disabledReason).toBeUndefined();

    const landed = deriveFlow(status({ mode: 'Landed', ec_state: 'Armed' }), live);
    expect(landed.primary?.command).toBe('disarm_ec');
  });

  test('after a normal landing (EC disarmed itself) the operator can arm again for another flight', () => {
    const landed = deriveFlow(status({ mode: 'Landed', ec_state: 'Disarmed' }), live);
    expect(landed.stage).toBe('landed');
    expect(landed.primary?.command).toBe('arm_ec');
    expect(landed.primary?.disabledReason).toBeUndefined();
    expect(landed.secondary.map((a) => a.command)).toContain('reinit_nav');

    // A start is not blocked by having landed (the flight computer starts a new flight), only by a safed EC
    expect(disabledReason('start_flight', status({ mode: 'Landed', ec_state: 'Armed', can_start: true }), live)).toBeUndefined();
    expect(disabledReason('start_flight', status({ mode: 'Safed', ec_state: 'Armed' }), live)).toContain('clear safing');
  });

  test('terminate is always offered, abort survives stale telemetry', () => {
    const idle = deriveFlow(status(), live);
    expect(idle.terminate.command).toBe('terminate');
    expect(idle.terminate.disabledReason).toBeUndefined();

    const stale = deriveFlow(status({ mode: 'Flight' }), { ...live, fresh: false });
    expect(stale.stage).toBe('unavailable');
    expect(stale.terminate.disabledReason).toBeUndefined();
    expect(stale.primary?.command).toBe('abort_land');
    expect(stale.primary?.disabledReason).toBeUndefined();
  });

  test('nothing can be sent without a Servo connection', () => {
    const offline = { ...live, connected: false };
    const flow = deriveFlow(status(), offline);
    expect(flow.primary).toBeNull();
    expect(flow.terminate.disabledReason).toContain('Not connected');
    expect(disabledReason('abort_land', status(), offline)).toContain('Not connected');
  });

  test('radio telemetry carries no CTV status', () => {
    const flow = deriveFlow(status(), { ...live, sourceCarriesCtv: false });
    expect(flow.stage).toBe('unavailable');
    expect(flow.primary).toBeNull();
    expect(flow.headline).toContain('Umbilical');
  });

  test('flight logic off gives no primary action', () => {
    const flow = deriveFlow(status({ mode: 'Off' }), live);
    expect(flow.primary).toBeNull();
  });

  test('disarm is refused in flight', () => {
    expect(disabledReason('disarm_ec', status({ mode: 'Flight', ec_state: 'Armed' }), live)).toContain('flying');
  });
});

test.describe('acknowledgement', () => {
  test('each command is acknowledged by its effect on the status', () => {
    expect(commandTookEffect('arm_ec', status({ ec_state: 'Armed' }))).toBe(true);
    expect(commandTookEffect('arm_ec', status())).toBe(false);
    expect(commandTookEffect('disarm_ec', status())).toBe(true);
    expect(commandTookEffect('start_flight', status({ mode: 'Armed' }))).toBe(false);
    expect(commandTookEffect('start_flight', status({ mode: 'Flight' }))).toBe(true);
    expect(commandTookEffect('abort_land', status({ mode: 'Flight' }))).toBe(false);
    expect(commandTookEffect('abort_land', status({ mode: 'AbortLand' }))).toBe(true);
    expect(commandTookEffect('terminate', status({ mode: 'Safed' }))).toBe(true);
    expect(commandTookEffect('terminate', status({ ec_state: 'Safing' }))).toBe(true);
    expect(commandTookEffect('clear_safing', status({ ec_state: 'Safing', mode: 'Safed' }))).toBe(false);
    expect(commandTookEffect('clear_safing', status({ ec_state: 'Disarmed' }))).toBe(true);
  });

  test('re-initializing navigation shows as the estimate levelling again', () => {
    const unhealthy = status({
      preflight: [{ name: 'Navigation estimate', ok: false, detail: 'Unhealthy("GPS rejected repeatedly")' }],
    });
    const leveling = status({
      preflight: [{ name: 'Navigation estimate', ok: false, detail: 'Leveling' }],
    });
    expect(commandTookEffect('reinit_nav', unhealthy)).toBe(false);
    expect(commandTookEffect('reinit_nav', leveling)).toBe(true);
  });
});

test.describe('re-initializing navigation', () => {
  test('is offered on the pad and after a flight, with the primary action unchanged', () => {
    const safed = deriveFlow(status({ mode: 'Safed', ec_state: 'Safing' }), live);
    expect(safed.primary?.command).toBe('clear_safing');
    expect(safed.secondary.map((a) => a.command)).toContain('reinit_nav');

    const landed = deriveFlow(status({ mode: 'Landed', ec_state: 'Armed' }), live);
    expect(landed.secondary.map((a) => a.command)).toContain('reinit_nav');

    const arm = deriveFlow(status(), live);
    expect(arm.primary?.command).toBe('arm_ec');
    expect(arm.secondary.map((a) => a.command)).toContain('reinit_nav');

    const armed = deriveFlow(status({ ec_state: 'Armed' }), live);
    expect(armed.secondary.map((a) => a.command)).toEqual(['disarm_ec', 'reinit_nav']);
  });

  test('is refused in flight, where the controller is using the estimate', () => {
    expect(disabledReason('reinit_nav', status({ mode: 'Flight', ec_state: 'Armed' }), live)).toContain('flying');
    expect(disabledReason('reinit_nav', status({ mode: 'AbortLand', ec_state: 'Armed' }), live)).toContain('flying');
    expect(deriveFlow(status({ mode: 'Flight', ec_state: 'Armed' }), live).secondary).toEqual([]);
  });

  test('is refused with no connection or no fresh telemetry like the other commands', () => {
    expect(disabledReason('reinit_nav', status(), { ...live, connected: false })).toContain('Not connected');
    expect(disabledReason('reinit_nav', status(), { ...live, fresh: false })).toContain('No fresh');
  });
});

test.describe('event log diff', () => {
  test('describes the starting point when there is no previous frame', () => {
    const drafts = diffStatus(null, status({ fault: 'NavLost (R4)', last_event: 'armed' }));
    expect(drafts.map((d) => d.kind)).toEqual(['mode', 'fault', 'event']);
  });

  test('records mode, event, fault and missed period changes once', () => {
    const before = status({ mode: 'Armed', ec_state: 'Armed', last_event: 'pad pose taken' });
    const after = status({
      mode: 'Flight',
      ec_state: 'Armed',
      last_event: 'start',
      fault: 'NavLost (R4)',
      missed_ticks: 2,
    });
    const text = diffStatus(before, after).map((d) => d.text);
    expect(text).toContain('Mode: Armed -> Flight');
    expect(text).toContain('start');
    expect(text).toContain('Fault latched: NavLost (R4)');
    expect(text).toContain('Missed control periods: 0 -> 2');
    expect(diffStatus(after, after)).toEqual([]);
  });

  test('does not log an event that went blank', () => {
    expect(diffStatus(status({ last_event: 'x' }), status({ last_event: '' }))).toEqual([]);
  });
});

test.describe('telemetry health and progress', () => {
  test('classifies freshness from servo transport stats', () => {
    expect(classifyTelemetry(null).kind).toBe('none');
    expect(classifyTelemetry({ time_since_update_ms: null, update_rate_hz: null }).kind).toBe('none');
    expect(classifyTelemetry({ time_since_update_ms: 120, update_rate_hz: 10 }).kind).toBe('live');
    expect(classifyTelemetry({ time_since_update_ms: 4000, update_rate_hz: 10 }).kind).toBe('stale');
  });

  test('progress is clamped and safe for a missing schedule', () => {
    expect(progressFraction(15, 30)).toBe(0.5);
    expect(progressFraction(45, 30)).toBe(1);
    expect(progressFraction(-1, 30)).toBe(0);
    expect(progressFraction(5, 0)).toBe(0);
  });
});
