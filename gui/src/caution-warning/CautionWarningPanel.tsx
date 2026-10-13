import { createMemo, createSignal, For, Show } from "solid-js";
import { activeMessages, MessageRule, Severity, StatusContext } from "./rules";
import "./caution-warning.css";
import StatusTable from "./StatusTable";

interface Props {
  context: StatusContext;
  rules: MessageRule[];
  errors: string[];
  events: { time: string; agent: string; message: string }[];
}

/** Presentation only: the live connection is supplied by CautionWarning. */
export default function CautionWarningPanel(props: Props) {
  const [filter, setFilter] = createSignal<"all" | "warning" | "caution">("all");
  const timeZone = Intl.DateTimeFormat().resolvedOptions().timeZone;
  const active = createMemo(() => activeMessages(props.rules, props.context));
  // Keep the first-seen time across telemetry updates and filter changes.
  // Clearing and reactivating a condition starts a new timestamp in this view.
  const activationTimes = createMemo<Map<string, Date>>(previous => new Map(
    active().map(message => [message.id, previous.get(message.id) ?? new Date()]),
  ), new Map());
  const visible = createMemo(() => active().filter(message => message.severity !== "info" &&
    (filter() === "all" || message.severity === filter())));
  const information = createMemo(() => active().filter(message => message.severity === "info"));
  const summary = createMemo(() => {
    const count = (severity: Severity) => active().filter(message => message.severity === severity).length;
    const warnings = count("warning");
    const cautions = count("caution");
    return `${warnings} warning${warnings === 1 ? "" : "s"} · ${cautions} caution${cautions === 1 ? "" : "s"} · ${count("info")} informational`;
  });

  return <section class="cw-panel" aria-label="Caution & Warning">
    <header class="cw-header">
      <h2>Caution &amp; Warning</h2>
      <p role="status" aria-live="polite">{summary()}</p>
      <p class="cw-timezone">Time zone: {timeZone}</p>
    </header>
    <div class="cw-displays">
      <section class="cw-display" aria-label="Cautions and warnings">
        <header class="cw-display-header">
          <h3>Cautions &amp; warnings</h3>
          <label class="cw-filter">Show
            <select value={filter()} onChange={event => setFilter(event.currentTarget.value as "all" | "warning" | "caution")}>
              <option value="all">Cautions &amp; warnings</option>
              <option value="warning">Warnings</option>
              <option value="caution">Cautions</option>
            </select>
          </label>
        </header>
        <div class="cw-content" role="region" aria-label="Scrollable cautions and warnings" tabIndex={0}>
          <Show when={props.errors.length > 0}>
            <div class="cw-config-error" role="alert">
              <strong>Message configuration needs attention</strong>
              <p>Some messages could not be loaded.</p>
              <ul><For each={props.errors}>{error => <li>{error}</li>}</For></ul>
            </div>
          </Show>
          <StatusTable label="Caution and warning messages" messages={visible()} activationTimes={activationTimes()} timeZone={timeZone} />
          <Show when={visible().length === 0}>
            <p class="cw-empty">{filter() !== "all" ? "No active messages match this filter." : "No active cautions or warnings. This does not confirm system health."}</p>
          </Show>
        </div>
      </section>
      <section class="cw-display" aria-label="Information">
        <header class="cw-display-header"><h3>Information</h3></header>
        <div class="cw-content" role="region" aria-label="Scrollable information" tabIndex={0}>
          <StatusTable label="Information messages" messages={information()} activationTimes={activationTimes()} timeZone={timeZone} />
          <Show when={information().length === 0}><p class="cw-empty">No active information messages.</p></Show>
        </div>
      </section>
    </div>
    <details class="cw-events">
      <summary>Event log ({props.events.length})</summary>
      <Show when={props.events.length > 0} fallback={<p>No events recorded.</p>}>
        <ul><For each={props.events}>{event => <li><span>{event.time} · {event.agent}</span><p>{event.message}</p></li>}</For></ul>
      </Show>
    </details>
  </section>;
}
