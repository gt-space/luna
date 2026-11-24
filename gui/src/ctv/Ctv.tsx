import { For, Show, createEffect, createMemo, createSignal, onCleanup } from "solid-js";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import Footer from "../general-components/Footer";
import { PlotsCard } from "./CtvPlots";
import { TraceBuffer, View } from "./ctvTrace";
import { GeneralTitleBar } from "../general-components/TitleBar";
import {
  CtvMode,
  CtvOperatorCommand,
  CtvStatus,
  StreamState,
  TelemetrySourceStats,
  TelemetryStatsResponse,
  currentDataSource,
  getTelemetryStats,
  isConnected,
  selectTelemetrySource,
  sendCtvCommand,
  serverIp,
} from "../comm";
import {
  ACK_TIMEOUT_MS,
  ActionSpec,
  COMMAND_LABELS,
  CONFIRMED_COMMANDS,
  FlowContext,
  LogDraft,
  LogKind,
  classifyTelemetry,
  commandTookEffect,
  deriveFlow,
  diffStatus,
  ecIs,
  isFlying,
  progressFraction,
} from "./ctvLogic";

const EMPTY_STATUS: CtvStatus = {
  mode: "Off",
  mission_time_s: 0,
  schedule_time_s: 0,
  schedule_end_s: 0,
  altitude_m: 0,
  tilt_deg: 0,
  speed_mps: 0,
  ec_connected: false,
  ec_state: "",
  ec_safing_reason: "",
  fault: "",
  last_event: "",
  gains: "",
  missed_ticks: 0,
  preflight: [],
  can_start: false,
};

const MODE_COLORS: Record<CtvMode, string> = {
  Off: "#737373",
  Idle: "#015878",
  Armed: "#CC9A13",
  Flight: "#22873D",
  AbortLand: "#D2691E",
  Landed: "#2A7F8E",
  Safed: "#C53434",
};

const CONFIRM_WINDOW_MS = 6000;
const MISSED_WARN_MS = 15000;
const MAX_LOG_ENTRIES = 300;

interface LogEntry {
  id: number,
  at: Date,
  /** Mission time when the entry was made, if flying. */
  mission: number | null,
  kind: LogKind,
  text: string,
}

type CommandState = "pending" | "sent" | "acknowledged" | "failed";

interface CommandRecord {
  id: number,
  command: CtvOperatorCommand,
  state: CommandState,
  detail: string,
}

const [ctv, setCtv] = createSignal<CtvStatus>(EMPTY_STATUS);
// false until the first frame carrying ctv arrives, and again if frames stop carrying it
const [haveStatus, setHaveStatus] = createSignal(false);
const [log, setLog] = createSignal<LogEntry[]>([]);
const [lastCommand, setLastCommand] = createSignal<CommandRecord | null>(null);
const [confirming, setConfirming] = createSignal<CtvOperatorCommand | null>(null);
const [now, setNow] = createSignal(Date.now());
// missed control periods at first sight, and when they last went up
const [missedBaseline, setMissedBaseline] = createSignal<number | null>(null);
const [lastMissedIncrease, setLastMissedIncrease] = createSignal<number | null>(null);
const [stats, setStats] = createSignal<TelemetrySourceStats | null>(null);

let previous: CtvStatus | null = null;
let logCounter = 0;
let commandCounter = 0;
let confirmTimer: ReturnType<typeof setTimeout> | undefined;

function addLog(kind: LogKind, text: string, mission: number | null = null) {
  const entry: LogEntry = { id: logCounter++, at: new Date(), mission, kind, text };
  setLog((entries) => [entry, ...entries].slice(0, MAX_LOG_ENTRIES));
}

function addDrafts(drafts: LogDraft[], status: CtvStatus) {
  const mission = isFlying(status.mode) ? status.mission_time_s : null;
  // oldest first in the draft list, so the newest ends on top of the log
  for (const draft of drafts) {
    addLog(draft.kind, draft.text, mission);
  }
}

// The stream is opened by the task bar window and re-broadcast to every window.
// What the plots draw: the control periods the flight computer sends with its
// status, kept here, in the window only (like the event log).
const traceBuffer = new TraceBuffer();
const [plotView, setPlotView] = createSignal<View>(traceBuffer.view(), { equals: false });
let plotsDirty = false;

// redraw a few times a second, not on every frame
setInterval(() => {
  if (plotsDirty) {
    plotsDirty = false;
    setPlotView(traceBuffer.view());
  }
}, 250);

function clearPlots() {
  traceBuffer.clear();
  setPlotView(traceBuffer.view());
}

listen("device_update", (event) => {
  const next = (event.payload as StreamState).ctv;
  if (next === undefined || next === null) {
    setHaveStatus(false);
    return;
  }

  addDrafts(diffStatus(previous, next), next);
  if (previous !== null && next.missed_ticks > previous.missed_ticks) {
    setLastMissedIncrease(Date.now());
  }
  if (missedBaseline() === null) {
    setMissedBaseline(next.missed_ticks);
  }
  previous = next;
  setCtv(next);
  setHaveStatus(true);
  if (traceBuffer.append(next) > 0) {
    plotsDirty = true;
  }
});

invoke("initialize_state", { window: appWindow });

setInterval(() => setNow(Date.now()), 500);

// Servo re-sends the last vehicle state at 10 Hz even if the flight computer
// has gone silent, so frame arrival proves nothing. Servo's own transport
// stats say how old the last frame from the flight computer is.
const health = createMemo(() => {
  const all = stats();
  return classifyTelemetry(all);
});

const sourceCarriesCtv = () => currentDataSource() === "umbilical";

const flowContext = createMemo<FlowContext>(() => ({
  connected: isConnected() && !!serverIp(),
  fresh: haveStatus() && health().kind === "live",
  sourceCarriesCtv: sourceCarriesCtv(),
}));

const flow = createMemo(() => deriveFlow(ctv(), flowContext()));

// log telemetry loss and recovery (not the first connection)
let wasLive = false;
let lostLive = false;
createEffect(() => {
  const kind = health().kind;
  if (sourceCarriesCtv() && isConnected()) {
    if (kind === "live") {
      if (lostLive) addLog("system", "CTV telemetry restored");
      wasLive = true;
      lostLive = false;
    } else if (wasLive && !lostLive) {
      addLog("system", "CTV telemetry lost: no frames from the flight computer");
      lostLive = true;
    }
  }
});

// A command has taken effect when the status shows it.
createEffect(() => {
  const record = lastCommand();
  if (record === null || record.state !== "sent" || !haveStatus()) return;
  if (commandTookEffect(record.command, ctv())) {
    setLastCommand({ ...record, state: "acknowledged", detail: "Flight computer reports the change." });
    addLog("operator", `Acknowledged: ${COMMAND_LABELS[record.command]}`);
  }
});

// Drop a pending confirmation if the action stops being allowed.
createEffect(() => {
  const pending = confirming();
  if (pending === null) return;
  const available = [flow().primary, ...flow().secondary, flow().terminate].some(
    (spec) => spec !== null && spec.command === pending && spec.disabledReason === undefined,
  );
  if (!available) cancelConfirm();
});

function cancelConfirm() {
  clearTimeout(confirmTimer);
  setConfirming(null);
}

async function runCommand(command: CtvOperatorCommand) {
  cancelConfirm();
  const ip = serverIp() as string;
  const id = commandCounter++;
  const label = COMMAND_LABELS[command];

  setLastCommand({ id, command, state: "pending", detail: "Sending to Servo..." });
  addLog("operator", `Sent: ${label}`, isFlying(ctv().mode) ? ctv().mission_time_s : null);

  const result = await sendCtvCommand(ip, command);
  if (!result.ok) {
    setLastCommand({ id, command, state: "failed", detail: result.error ?? "Servo refused the command." });
    addLog("operator", `FAILED: ${label}: ${result.error ?? "refused"}`);
    return;
  }

  // The acknowledgement effect above moves it to "acknowledged".
  const current = lastCommand();
  if (current !== null && current.id === id && current.state === "pending") {
    setLastCommand({ id, command, state: "sent", detail: "Forwarded to the flight computer. Waiting for it to act..." });
  }

  setTimeout(() => {
    const record = lastCommand();
    if (record !== null && record.id === id && record.state === "sent") {
      const detail = `No effect seen within ${ACK_TIMEOUT_MS / 1000} s. Check the status before retrying.`;
      setLastCommand({ id, command, state: "failed", detail });
      addLog("operator", `No effect: ${label}`);
    }
  }, ACK_TIMEOUT_MS);
}

function onAction(spec: ActionSpec) {
  if (spec.disabledReason !== undefined) return;
  if (CONFIRMED_COMMANDS.includes(spec.command)) {
    clearTimeout(confirmTimer);
    setConfirming(spec.command);
    confirmTimer = setTimeout(() => setConfirming(null), CONFIRM_WINDOW_MS);
    return;
  }
  runCommand(spec.command);
}

type Tone = "primary" | "secondary" | "danger" | "caution";

function ActionButton(props: { spec: ActionSpec, tone: Tone, big?: boolean, testId: string }) {
  const disabled = () => props.spec.disabledReason !== undefined;
  const asking = () => confirming() === props.spec.command;

  return (
    <div class="ctv-action" title={props.spec.disabledReason ?? ""}>
      <Show
        when={asking()}
        fallback={
          <button
            data-testid={props.testId}
            class={`ctv-button ctv-button-${props.tone}${props.big ? " ctv-button-big" : ""}`}
            disabled={disabled()}
            onClick={() => onAction(props.spec)}
          >
            {props.spec.label}
          </button>
        }
      >
        <div class="ctv-confirm-row">
          <button
            data-testid={`${props.testId}-confirm`}
            class={`ctv-button ctv-button-${props.tone}${props.big ? " ctv-button-big" : ""} ctv-button-confirm`}
            onClick={() => runCommand(props.spec.command)}
          >
            Confirm: {props.spec.label}
          </button>
          <button class="ctv-button ctv-button-secondary" onClick={cancelConfirm}>
            Cancel
          </button>
        </div>
      </Show>
      <Show when={disabled()}>
        <div class="ctv-reason">{props.spec.disabledReason}</div>
      </Show>
    </div>
  );
}

function formatNumber(value: number, digits: number) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

const STEPS: { stage: string, label: string }[] = [
  { stage: "arm", label: "1 Arm EC" },
  { stage: "start", label: "2 Start" },
  { stage: "flying", label: "3 Flight" },
  { stage: "landed", label: "4 Landed" },
];

function stepState(stage: string): "done" | "current" | "todo" {
  const order = ["arm", "start", "flying", "landed"];
  const current = flow().stage === "safed" ? "landed" : flow().stage;
  const here = order.indexOf(current);
  const there = order.indexOf(stage);
  if (here < 0) return "todo";
  if (there < here) return "done";
  return there === here ? "current" : "todo";
}

function CommandStatus() {
  return (
    <Show when={lastCommand()}>
      {(record) => (
        <div class={`ctv-command-status ctv-command-${record().state}`} data-testid="ctv-command-status">
          <span class="ctv-command-name">{COMMAND_LABELS[record().command]}</span>
          <span class="ctv-command-state">{record().state}</span>
          <span class="ctv-command-detail">{record().detail}</span>
        </div>
      )}
    </Show>
  );
}

function Checklist() {
  return (
    <div class="ctv-card">
      <div class="ctv-card-title">Pre-flight checks</div>
      <div
        class={`ctv-ready ${
          !flowContext().fresh || isFlying(ctv().mode) ? "ctv-ready-none" : ctv().can_start ? "ctv-ready-yes" : "ctv-ready-no"
        }`}
        data-testid="ctv-ready"
      >
        {!flowContext().fresh ? "NO DATA" : isFlying(ctv().mode) ? "IN FLIGHT" : ctv().can_start ? "READY" : "NOT READY"}
      </div>
      <Show
        when={ctv().preflight.length > 0}
        fallback={<div class="ctv-muted">No checks reported yet.</div>}
      >
        <For each={ctv().preflight}>
          {(check) => (
            <div class={`ctv-check ${flowContext().fresh ? "" : "ctv-dim"}`} data-testid="ctv-check">
              <span class={`ctv-dot ${check.ok ? "ctv-dot-ok" : "ctv-dot-bad"}`}>{check.ok ? "OK" : "X"}</span>
              <span class="ctv-check-name">{check.name}</span>
              <span class="ctv-check-detail">{check.detail}</span>
            </div>
          )}
        </For>
      </Show>
    </div>
  );
}

function Operations() {
  return (
    <div class="ctv-card">
      <div class="ctv-card-title">Operations</div>
      <div class="ctv-steps">
        <For each={STEPS}>
          {(step) => <div class={`ctv-step ctv-step-${stepState(step.stage)}`}>{step.label}</div>}
        </For>
      </div>
      <div class="ctv-headline" data-testid="ctv-headline">{flow().headline}</div>

      <Show when={flow().primary}>
        {(primary) => (
          <ActionButton spec={primary()} tone={primary().command === "abort_land" ? "caution" : "primary"} big testId="ctv-primary" />
        )}
      </Show>
      <Show when={flow().secondary.length > 0}>
        <div class="ctv-secondary-row">
          <For each={flow().secondary}>
            {(spec) => <ActionButton spec={spec} tone="secondary" testId={`ctv-${spec.command}`} />}
          </For>
        </div>
      </Show>

      <CommandStatus />

      <div class="ctv-terminate">
        <ActionButton spec={flow().terminate} tone="danger" testId="ctv-terminate" />
        <div class="ctv-muted">Safes the engine controller immediately and latches it until safing is cleared.</div>
      </div>
    </div>
  );
}

function Metric(props: { label: string, value: string, unit?: string, testId?: string }) {
  return (
    <div class="ctv-metric">
      <div class="ctv-metric-label">{props.label}</div>
      <div class="ctv-metric-value" data-testid={props.testId}>
        {props.value}
        <Show when={props.unit}>
          <span class="ctv-metric-unit"> {props.unit}</span>
        </Show>
      </div>
    </div>
  );
}

function StatusPanel() {
  const missedRecently = () => {
    const at = lastMissedIncrease();
    return at !== null && now() - at < MISSED_WARN_MS;
  };
  const missedSince = () => ctv().missed_ticks - (missedBaseline() ?? ctv().missed_ticks);
  const showSchedule = () => Math.abs(ctv().schedule_time_s - ctv().mission_time_s) > 0.05;
  const missionFraction = () => progressFraction(ctv().mission_time_s, ctv().schedule_end_s);
  const scheduleFraction = () => progressFraction(ctv().schedule_time_s, ctv().schedule_end_s);

  return (
    <div class="ctv-card">
      <div class="ctv-card-title">Status</div>

      <div class="ctv-status-top">
        <div
          class="ctv-mode"
          style={{ "background-color": flowContext().fresh ? MODE_COLORS[ctv().mode] : MODE_COLORS.Off }}
          data-testid="ctv-mode"
        >
          {flowContext().fresh ? ctv().mode : "NO DATA"}
        </div>
        <TelemetryBadge />
      </div>

      <div class={flowContext().fresh ? "" : "ctv-dim"}>
        <div class="ctv-mission">
          <div class="ctv-mission-text" data-testid="ctv-mission-time">
            T+ {formatNumber(ctv().mission_time_s, 1)} s
            <span class="ctv-muted"> of {formatNumber(ctv().schedule_end_s, 1)} s</span>
            <Show when={showSchedule()}>
              <span class="ctv-schedule-note"> (schedule at {formatNumber(ctv().schedule_time_s, 1)} s)</span>
            </Show>
          </div>
          <div class="ctv-progress">
            <div class="ctv-progress-fill" style={{ width: `${missionFraction() * 100}%` }} />
            <Show when={showSchedule()}>
              <div class="ctv-progress-marker" style={{ left: `${scheduleFraction() * 100}%` }} title="Position in the gain schedule" />
            </Show>
          </div>
        </div>

        <div class="ctv-metrics">
          <Metric label="Altitude" value={formatNumber(ctv().altitude_m, 2)} unit="m" testId="ctv-altitude" />
          <Metric label="Tilt" value={formatNumber(ctv().tilt_deg, 1)} unit="deg" testId="ctv-tilt" />
          <Metric label="Speed" value={formatNumber(ctv().speed_mps, 2)} unit="m/s" testId="ctv-speed" />
        </div>

        <div class="ctv-rows">
          <div class="ctv-row">
            <div class="ctv-row-label">Engine controller</div>
            <div class="ctv-row-value" data-testid="ctv-ec">
              <span class={`ctv-pill ${ctv().ec_connected ? "ctv-pill-ok" : "ctv-pill-bad"}`}>
                {ctv().ec_connected ? "connected" : "not connected"}
              </span>{" "}
              <span class={ecIs(ctv(), "safing") ? "ctv-bad-text" : ""}>{ctv().ec_state || "unknown"}</span>
              <Show when={ctv().ec_safing_reason}>
                <span class="ctv-bad-text"> ({ctv().ec_safing_reason})</span>
              </Show>
            </div>
          </div>
          <div class="ctv-row">
            <div class="ctv-row-label">Gains</div>
            <div class="ctv-row-value" data-testid="ctv-gains">{ctv().gains || "--"}</div>
          </div>
          <div class="ctv-row">
            <div class="ctv-row-label">Missed control periods</div>
            <div class={`ctv-row-value ${missedRecently() ? "ctv-warn-text" : ""}`} data-testid="ctv-missed">
              {ctv().missed_ticks}
              <Show when={missedSince() > 0}>
                <span> (+{missedSince()} since this window opened)</span>
              </Show>
            </div>
          </div>
          <div class="ctv-row">
            <div class="ctv-row-label">Last event</div>
            <div class="ctv-row-value" data-testid="ctv-last-event">{ctv().last_event || "--"}</div>
          </div>
        </div>
      </div>
    </div>
  );
}

function TelemetryBadge() {
  return (
    <div class={`ctv-telemetry ctv-telemetry-${health().kind}`} data-testid="ctv-telemetry">
      {(() => {
        const h = health();
        if (!isConnected()) return "Servo: disconnected";
        if (h.kind === "live") {
          return `Telemetry live${h.rateHz ? ` ${h.rateHz.toFixed(1)} Hz` : ""}`;
        }
        if (h.kind === "stale") return `Telemetry STALE: ${(h.ageMs / 1000).toFixed(1)} s old`;
        return "No telemetry";
      })()}
    </div>
  );
}

function Banners() {
  return (
    <>
      <Show when={haveStatus() && ctv().fault}>
        <div class="ctv-banner ctv-banner-fault" data-testid="ctv-fault">
          FAULT LATCHED: {ctv().fault}
          <Show when={!flowContext().fresh}>
            <span> (last known)</span>
          </Show>
        </div>
      </Show>
      <Show when={isConnected() && !sourceCarriesCtv()}>
        <div class="ctv-banner ctv-banner-warn">
          Radio telemetry carries no CTV status, so this page has nothing to show.
          <button class="ctv-button ctv-button-secondary ctv-inline-button" onClick={() => selectTelemetrySource("umbilical")}>
            Switch to Umbilical
          </button>
        </div>
      </Show>
      <Show when={isConnected() && sourceCarriesCtv() && health().kind !== "live"}>
        <div class="ctv-banner ctv-banner-warn">
          CTV telemetry is {health().kind === "stale" ? "stale" : "not arriving"}. Values shown are not current.
          Abort and Terminate are still sent to Servo.
        </div>
      </Show>
    </>
  );
}

function EventLog() {
  const clock = (at: Date) => at.toLocaleTimeString([], { hour12: false });

  return (
    <div class="ctv-card ctv-log-card">
      <div class="ctv-card-title">Event log</div>
      <div class="ctv-log" data-testid="ctv-log">
        <Show when={log().length > 0} fallback={<div class="ctv-muted">Waiting for events.</div>}>
          <For each={log()}>
            {(entry) => (
              <div class={`ctv-log-row ctv-log-${entry.kind}`}>
                <span class="ctv-log-time">{clock(entry.at)}</span>
                <span class="ctv-log-mission">{entry.mission === null ? "" : `T+${entry.mission.toFixed(1)}`}</span>
                <span class="ctv-log-text">{entry.text}</span>
              </div>
            )}
          </For>
        </Show>
      </div>
    </div>
  );
}

function Ctv() {
  const refreshStats = async () => {
    if (!isConnected() || !serverIp()) {
      setStats(null);
      return;
    }

    const response = await getTelemetryStats(serverIp() as string);
    if (response != null && typeof response === "object" && "umbilical" in response && "tel" in response) {
      const all = response as TelemetryStatsResponse;
      setStats(currentDataSource() === "umbilical" ? all.umbilical : all.tel);
    } else {
      setStats(null);
    }
  };

  refreshStats();
  const pollHandle = setInterval(refreshStats, 500);
  onCleanup(() => clearInterval(pollHandle));

  return (
    <div class="window-template">
      <div style="height: 60px">
        <GeneralTitleBar name="CTV" />
      </div>
      <div class="ctv-view">
        <Banners />
        <div class="ctv-columns">
          <div class="ctv-column">
            <Operations />
            <Checklist />
          </div>
          <div class="ctv-column">
            <StatusPanel />
            <EventLog />
          </div>
        </div>
        <PlotsCard view={plotView} status={ctv} onClear={clearPlots} />
      </div>
      <div>
        <Footer />
      </div>
    </div>
  );
}

export default Ctv;
