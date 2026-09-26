import { listen, UnlistenFn } from "@tauri-apps/api/event";
import { createEffect, createMemo, createSignal, onCleanup, onMount } from "solid-js";
import { ACTIVITY_WARN_THRESH } from "../appdata";
import { Alert, alerts, currentDataSource, isConnected, StreamState } from "../comm";
import CautionWarningPanel from "./CautionWarningPanel";
import { loadRules, StatusContext } from "./rules";

// Add another .json file in config/ to register more messages automatically.
const configuration = loadRules(import.meta.glob("./config/*.json", { eager: true, as: "raw" }));

export default function CautionWarning() {
  const [telemetry, setTelemetry] = createSignal<StreamState>();
  const [receivedAt, setReceivedAt] = createSignal<number>();
  const [now, setNow] = createSignal(performance.now());
  const [connectionError, setConnectionError] = createSignal("");

  createEffect(() => {
    // A reconnect or source change must not reuse the previous stream's data.
    currentDataSource();
    isConnected();
    setTelemetry(undefined);
    setReceivedAt(undefined);
  });

  onMount(() => {
    let disposed = false;
    let unlisten: UnlistenFn | undefined;
    const timer = window.setInterval(() => setNow(performance.now()), 100);
    listen<StreamState>("device_update", event => {
      if (!isConnected()) return;
      const time = performance.now();
      setTelemetry(event.payload);
      setReceivedAt(time);
      setNow(time);
    }).then(stop => {
      if (disposed) stop();
      else unlisten = stop;
    }).catch(error => setConnectionError(`Could not listen for telemetry: ${String(error)}`));
    onCleanup(() => {
      disposed = true;
      window.clearInterval(timer);
      unlisten?.();
    });
  });

  const context = createMemo<StatusContext>(() => {
    const received = receivedAt();
    const age = received === undefined ? null : Math.max(0, now() - received);
    return {
      connected: isConnected(),
      dataSource: currentDataSource(),
      hasTelemetry: received !== undefined,
      telemetryFresh: isConnected() && age !== null && age < ACTIVITY_WARN_THRESH,
      telemetryAgeMs: age,
      telemetry: telemetry(),
    };
  });

  return <CautionWarningPanel context={context()} rules={configuration.rules}
    errors={connectionError() ? [...configuration.errors, connectionError()] : configuration.errors}
    events={(alerts() as Alert[] | undefined) ?? []} />;
}
