// Pure logic for the CTV operations page: what the operator should do next,
// whether a command has taken effect, and what changed between two status
// frames. No Tauri or DOM access here so it can be tested on its own.
import type { CtvMode, CtvOperatorCommand, CtvStatus } from "../comm";

/** Telemetry older than this is treated as stale (servo re-sends the last frame at 10 Hz even when flight is silent). */
export const STALE_AFTER_MS = 1500;

/** How long a command may take to show up in the status before it is reported as having no effect. */
export const ACK_TIMEOUT_MS = 5000;

export type FlowStage =
  /** No usable CTV status: not connected to servo, no telemetry, or flight logic off. */
  | 'unavailable'
  | 'arm'
  | 'start'
  | 'flying'
  | 'safed'
  | 'landed';

export interface ActionSpec {
  command: CtvOperatorCommand,
  label: string,
  /** When set the button is disabled and this is shown as the reason. */
  disabledReason?: string,
}

export interface FlowContext {
  /** The GUI is connected to servo (control commands can be sent). */
  connected: boolean,
  /** CTV telemetry is arriving from the flight computer. */
  fresh: boolean,
  /** The selected telemetry source carries CTV status at all (radio does not). */
  sourceCarriesCtv: boolean,
}

export interface Flow {
  stage: FlowStage,
  /** One line telling the operator what to do next. */
  headline: string,
  primary: ActionSpec | null,
  secondary: ActionSpec[],
  terminate: ActionSpec,
  /** Commands that ask for an explicit confirmation step. */
  needsConfirm: CtvOperatorCommand[],
}

export const CONFIRMED_COMMANDS: CtvOperatorCommand[] = ['start_flight', 'terminate'];

export const COMMAND_LABELS: Record<CtvOperatorCommand, string> = {
  arm_ec: 'Arm EC',
  disarm_ec: 'Disarm EC',
  start_flight: 'Start flight',
  abort_land: 'Abort / Land',
  terminate: 'Terminate',
  clear_safing: 'Clear safing',
  reinit_nav: 'Re-initialize navigation',
};

export function ecIs(ctv: CtvStatus, state: string): boolean {
  return ctv.ec_state.trim().toLowerCase() === state;
}

export function isFlying(mode: CtvMode): boolean {
  return mode === 'Flight' || mode === 'AbortLand';
}

function failingChecks(ctv: CtvStatus): string[] {
  return ctv.preflight.filter((check) => !check.ok).map((check) => check.name);
}

/** The reason a command would be refused right now, or undefined if it would be sent. */
export function disabledReason(
  command: CtvOperatorCommand,
  ctv: CtvStatus,
  ctx: FlowContext,
): string | undefined {
  if (!ctx.connected) {
    return 'Not connected to Servo (connect from the System window).';
  }
  // Terminate and abort stay available whatever the status says: if the
  // status is wrong or missing they are the way out.
  if (command === 'terminate' || command === 'abort_land') {
    return undefined;
  }
  if (!ctx.sourceCarriesCtv) {
    return 'The selected telemetry source (radio) carries no CTV status.';
  }
  if (!ctx.fresh) {
    return 'No fresh CTV telemetry from the flight computer.';
  }
  if (ctv.mode === 'Off') {
    return 'Flight logic is off (flight computer was not started with --gnc-gains).';
  }

  switch (command) {
    case 'arm_ec':
      if (!ctv.ec_connected) return 'Engine controller is not connected.';
      if (isFlying(ctv.mode)) return 'Vehicle is flying.';
      if (ecIs(ctv, 'armed')) return 'EC is already armed.';
      if (ecIs(ctv, 'safing') || ctv.mode === 'Safed') return 'EC is safed: clear safing first.';
      return undefined;
    case 'disarm_ec':
      if (isFlying(ctv.mode)) return 'Vehicle is flying: use Abort / Land or Terminate.';
      if (ecIs(ctv, 'disarmed')) return 'EC is already disarmed.';
      return undefined;
    case 'start_flight':
      if (isFlying(ctv.mode)) return 'Already flying.';
      // After a landing the flight computer starts a new flight itself (arming the EC again takes it back to idle)
      if (ctv.mode === 'Safed') return 'EC is safed: clear safing before another start.';
      if (!ecIs(ctv, 'armed')) return 'Arm the EC first.';
      if (!ctv.can_start) {
        const failing = failingChecks(ctv);
        return failing.length > 0
          ? `Pre-flight checks failing: ${failing.join(', ')}.`
          : 'Flight computer reports the vehicle cannot start.';
      }
      return undefined;
    case 'clear_safing':
      if (!ecIs(ctv, 'safing') && ctv.mode !== 'Safed') return 'EC is not safed.';
      return undefined;
    case 'reinit_nav':
      if (isFlying(ctv.mode)) return 'Vehicle is flying: the controller is using the estimate.';
      return undefined;
  }
}

/** The pre-flight check on the navigation estimate, if the status has one. */
export function navigationCheck(ctv: CtvStatus) {
  return ctv.preflight.find((check) => check.name === 'Navigation estimate');
}

function spec(command: CtvOperatorCommand, ctv: CtvStatus, ctx: FlowContext, label?: string): ActionSpec {
  return {
    command,
    label: label ?? COMMAND_LABELS[command],
    disabledReason: disabledReason(command, ctv, ctx),
  };
}

/** Decides the one primary action, the secondary actions, and the headline for the current status. */
export function deriveFlow(ctv: CtvStatus, ctx: FlowContext): Flow {
  const terminate = spec('terminate', ctv, ctx, 'TERMINATE');
  const base = { terminate, needsConfirm: CONFIRMED_COMMANDS };

  if (!ctx.connected) {
    return { ...base, stage: 'unavailable', headline: 'Connect to Servo to operate the CTV.', primary: null, secondary: [] };
  }
  if (!ctx.sourceCarriesCtv) {
    return {
      ...base,
      stage: 'unavailable',
      headline: 'Switch to Umbilical telemetry: radio telemetry carries no CTV status.',
      primary: null,
      secondary: [],
    };
  }
  if (!ctx.fresh) {
    return {
      ...base,
      stage: 'unavailable',
      headline: 'Waiting for CTV telemetry from the flight computer.',
      // Abort remains reachable: the vehicle may be flying with a broken downlink.
      primary: spec('abort_land', ctv, ctx),
      secondary: [],
    };
  }
  if (ctv.mode === 'Off') {
    return { ...base, stage: 'unavailable', headline: 'CTV flight logic is off on the flight computer.', primary: null, secondary: [] };
  }

  if (ctv.mode === 'Flight') {
    return { ...base, stage: 'flying', headline: 'Flying the schedule. Abort / Land if anything looks wrong.', primary: spec('abort_land', ctv, ctx), secondary: [] };
  }
  if (ctv.mode === 'AbortLand') {
    return {
      ...base,
      stage: 'flying',
      headline: 'Aborting to land. Terminate cuts the engine now.',
      primary: { command: 'abort_land', label: 'Abort / Land', disabledReason: 'Already landing.' },
      secondary: [],
    };
  }
  if (ctv.mode === 'Safed' || ecIs(ctv, 'safing')) {
    return {
      ...base,
      stage: 'safed',
      headline: 'EC is safed. Clear safing to re-enable arming.',
      primary: spec('clear_safing', ctv, ctx),
      secondary: [spec('reinit_nav', ctv, ctx)],
    };
  }
  if (ctv.mode === 'Landed') {
    // The EC disarms itself on a normal landing; arming it again takes the flight logic back to idle,
    // so the next flight starts as the first one did.
    if (ecIs(ctv, 'disarmed')) {
      return {
        ...base,
        stage: 'landed',
        headline: 'Landed. Arm the EC to fly again, or re-initialize navigation if the vehicle was moved.',
        primary: spec('arm_ec', ctv, ctx, 'Arm EC for another flight'),
        secondary: [spec('reinit_nav', ctv, ctx)],
      };
    }
    return {
      ...base,
      stage: 'landed',
      headline: 'Landed. Disarm the EC.',
      primary: spec('disarm_ec', ctv, ctx),
      secondary: [spec('reinit_nav', ctv, ctx)],
    };
  }

  // Idle or Armed on the pad.
  if (!ecIs(ctv, 'armed')) {
    return {
      ...base,
      stage: 'arm',
      headline: 'Step 1: arm the engine controller.',
      primary: spec('arm_ec', ctv, ctx),
      secondary: [spec('reinit_nav', ctv, ctx)],
    };
  }
  return {
    ...base,
    stage: 'start',
    headline: ctv.can_start ? 'Ready. Start flight when the range is clear.' : 'EC armed. Waiting on pre-flight checks.',
    primary: spec('start_flight', ctv, ctx, 'START FLIGHT'),
    secondary: [spec('disarm_ec', ctv, ctx), spec('reinit_nav', ctv, ctx)],
  };
}

/** Whether the status shows the effect of a command having been carried out. */
export function commandTookEffect(command: CtvOperatorCommand, ctv: CtvStatus): boolean {
  switch (command) {
    case 'arm_ec':
      return ecIs(ctv, 'armed');
    case 'disarm_ec':
      return ecIs(ctv, 'disarmed');
    case 'start_flight':
      return ctv.mode === 'Flight' || ctv.mode === 'AbortLand' || ctv.mode === 'Landed';
    case 'abort_land':
      return ctv.mode === 'AbortLand' || ctv.mode === 'Landed' || ctv.mode === 'Safed';
    case 'terminate':
      return ctv.mode === 'Safed' || ecIs(ctv, 'safing');
    case 'clear_safing':
      return !ecIs(ctv, 'safing') && ctv.mode !== 'Safed';
    case 'reinit_nav':
      // the filter throws its state away and levels again, which the check reports
      return /leveling/i.test(navigationCheck(ctv)?.detail ?? '');
  }
}

export type LogKind = 'event' | 'mode' | 'fault' | 'ec' | 'missed' | 'operator' | 'system';

export interface LogDraft {
  kind: LogKind,
  text: string,
}

/** What changed between two status frames, as log lines. With no previous frame, describes the starting point. */
export function diffStatus(prev: CtvStatus | null, next: CtvStatus): LogDraft[] {
  const out: LogDraft[] = [];
  if (prev === null) {
    out.push({ kind: 'mode', text: `Mode at page open: ${next.mode}` });
    if (next.fault) out.push({ kind: 'fault', text: `Fault latched: ${next.fault}` });
    if (next.last_event) out.push({ kind: 'event', text: next.last_event });
    return out;
  }
  if (next.mode !== prev.mode) {
    out.push({ kind: 'mode', text: `Mode: ${prev.mode} -> ${next.mode}` });
  }
  if (next.fault !== prev.fault) {
    out.push({ kind: 'fault', text: next.fault ? `Fault latched: ${next.fault}` : 'Fault cleared' });
  }
  if (next.ec_state !== prev.ec_state) {
    out.push({ kind: 'ec', text: `EC: ${prev.ec_state || 'unknown'} -> ${next.ec_state || 'unknown'}` });
  }
  if (next.ec_safing_reason !== prev.ec_safing_reason && next.ec_safing_reason) {
    out.push({ kind: 'ec', text: `EC safing reason: ${next.ec_safing_reason}` });
  }
  if (next.ec_connected !== prev.ec_connected) {
    out.push({ kind: 'ec', text: next.ec_connected ? 'EC connected' : 'EC disconnected' });
  }
  if (next.last_event !== prev.last_event && next.last_event) {
    out.push({ kind: 'event', text: next.last_event });
  }
  if (next.missed_ticks > prev.missed_ticks) {
    out.push({ kind: 'missed', text: `Missed control periods: ${prev.missed_ticks} -> ${next.missed_ticks}` });
  }
  return out;
}

export type TelemetryHealth =
  | { kind: 'live', ageMs: number, rateHz: number | null }
  | { kind: 'stale', ageMs: number }
  | { kind: 'none' };

/** Classifies telemetry freshness from servo's transport stats (null stats means servo could not be asked). */
export function classifyTelemetry(
  stats: { time_since_update_ms: number | null, update_rate_hz: number | null } | null,
): TelemetryHealth {
  if (stats === null || stats.time_since_update_ms === null) {
    return { kind: 'none' };
  }
  if (stats.time_since_update_ms > STALE_AFTER_MS) {
    return { kind: 'stale', ageMs: stats.time_since_update_ms };
  }
  return { kind: 'live', ageMs: stats.time_since_update_ms, rateHz: stats.update_rate_hz };
}

/** Fraction (0..1) of the schedule elapsed, safe against a zero or missing end time. */
export function progressFraction(seconds: number, end: number): number {
  if (!(end > 0) || !Number.isFinite(seconds)) return 0;
  return Math.min(1, Math.max(0, seconds / end));
}
