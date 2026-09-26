import { listen, UnlistenFn } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { createSignal, onCleanup, onMount, Show } from "solid-js";
import { setAlerts, setCurrentDataSource, setIsConnected, State } from "../comm";
import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import CautionWarning from "./CautionWarning";

export default function CautionWarningWindow() {
  const [error, setError] = createSignal("");

  onMount(() => {
    let disposed = false;
    let unlisten: UnlistenFn | undefined;
    // Subscribe before requesting state so an already-connected session is loaded.
    listen<State>("state", event => {
      setAlerts(event.payload.alerts);
      setIsConnected(event.payload.isConnected);
      setCurrentDataSource(event.payload.currentDataSource);
    }).then(async stop => {
      if (disposed) {
        stop();
        return;
      }
      unlisten = stop;
      await invoke("initialize_state", { window: appWindow });
    }).catch(reason => {
      if (!disposed) setError(`Could not load system status: ${String(reason)}`);
    });
    onCleanup(() => {
      disposed = true;
      unlisten?.();
    });
  });

  return <div class="window-template">
    <GeneralTitleBar name="Caution & Warning" />
    <main class="cw-window-content">
      <Show when={error()}><p role="alert" class="cw-config-error">{error()}</p></Show>
      <CautionWarning />
    </main>
    <Footer />
  </div>;
}
