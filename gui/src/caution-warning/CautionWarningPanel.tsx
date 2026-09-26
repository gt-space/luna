import { createMemo, createSignal, For, Show } from "solid-js";
import { activeMessages, MessageRule, Severity, StatusContext } from "./rules";
import "./caution-warning.css";

interface Props {
  context: StatusContext;
  rules: MessageRule[];
  errors: string[];
  events: { time: string; agent: string; message: string }[];
}

/** Presentation only: the live connection is supplied by CautionWarning. */
export default function CautionWarningPanel(props: Props) {
  const [filter, setFilter] = createSignal<"all" | Severity>("all");
  const active = createMemo(() => activeMessages(props.rules, props.context));
  const visible = createMemo(() => active().filter(message => filter() === "all" || message.severity === filter()));
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
      <label class="cw-filter">Show
        <select value={filter()} onChange={event => setFilter(event.currentTarget.value as "all" | Severity)}>
          <option value="all">All messages</option>
          <option value="warning">Warnings</option>
          <option value="caution">Cautions</option>
          <option value="info">Information</option>
        </select>
      </label>
    </header>
    <div class="cw-content">
      <Show when={props.errors.length > 0}>
        <div class="cw-config-error" role="alert">
          <strong>Message configuration needs attention</strong>
          <p>Some messages could not be loaded.</p>
          <ul><For each={props.errors}>{error => <li>{error}</li>}</For></ul>
        </div>
      </Show>
      <ul class="cw-messages" aria-label="Active status messages">
        <For each={visible()}>{message =>
          <li class={`cw-message cw-${message.severity}`}>
            <div class="cw-meta"><span class="cw-badge">{message.severity === "info" ? "Information" : message.severity}</span><span>{message.subsystem}</span></div>
            <h3>{message.title}</h3>
            <p>{message.message}</p>
          </li>
        }</For>
      </ul>
      <Show when={visible().length === 0}>
        <p class="cw-empty">{active().length > 0 ? "No active messages match this filter." : "No configured messages are active. This does not confirm system health."}</p>
      </Show>
      <details class="cw-events">
        <summary>Event log ({props.events.length})</summary>
        <Show when={props.events.length > 0} fallback={<p>No events recorded.</p>}>
          <ul><For each={props.events}>{event => <li><span>{event.time} · {event.agent}</span><p>{event.message}</p></li>}</For></ul>
        </Show>
      </details>
    </div>
  </section>;
}
