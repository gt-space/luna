import { createMemo, For } from "solid-js";
import { MessageRule } from "./rules";

interface Props {
  label: string;
  messages: MessageRule[];
  activationTimes: Map<string, Date>;
  timeZone: string;
}

/** Shared columns and timestamp formatting for both status displays. */
export default function StatusTable(props: Props) {
  const timeFormat = createMemo(() => new Intl.DateTimeFormat(undefined, {
    timeZone: props.timeZone, hour: "2-digit", minute: "2-digit", second: "2-digit",
    hourCycle: "h23", timeZoneName: "short",
  }));
  return <table class="cw-table" aria-label={props.label}>
    <thead><tr>
      <th scope="col">Error type</th>
      <th scope="col">Subsystem</th>
      <th scope="col">Time Stamp</th>
      <th scope="col">Description</th>
    </tr></thead>
    <tbody><For each={props.messages}>{message => {
      const activatedAt = () => props.activationTimes.get(message.id);
      return <tr class={`cw-message cw-${message.severity}`}>
        <td><span class="cw-badge">{message.severity === "info" ? "Information" : message.severity}</span></td>
        <td>{message.subsystem}</td>
        <td class="cw-timestamp">
          <time dateTime={activatedAt()?.toISOString()}
            title={activatedAt()?.toLocaleString(undefined, { timeZone: props.timeZone, timeZoneName: "long" })}>
            {activatedAt() && timeFormat().format(activatedAt())}
          </time>
        </td>
        <td><p>{message.message}</p></td>
      </tr>;
    }}</For></tbody>
  </table>;
}
