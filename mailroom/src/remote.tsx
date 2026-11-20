import { createSignal, For, onCleanup, onMount, Show, type JSX } from "solid-js";
import { invoke } from "@tauri-apps/api/core";
import { open } from "@tauri-apps/plugin-dialog";
import type { ConnectStatus, Endpoint, Listing } from "./api";

const defaults: Endpoint = { host: "flight.local", username: "ubuntu", port: 22, folder: "/home/ubuntu/flight_logs" };

export function FlightComputer(props: {
  onClose: () => void;
  download: (folder: string, names: string[], destination: string) => Promise<void>;
  busy: boolean;
  status: () => JSX.Element;
}) {
  let dialog!: HTMLDialogElement;
  const [endpoint, setEndpoint] = createSignal(defaults);
  const [stage, setStage] = createSignal<"connect" | "verify" | "password" | "browse">("connect");
  const [fingerprint, setFingerprint] = createSignal("");
  const [folder, setFolder] = createSignal(defaults.folder);
  const [listing, setListing] = createSignal<Listing>();
  const [selected, setSelected] = createSignal<Set<string>>(new Set());
  const [pending, setPending] = createSignal(false);
  const [error, setError] = createSignal("");
  const blocked = () => pending() || props.busy;

  onMount(() => {
    dialog.showModal();
    void task(async () => {
      const current = await invoke<Endpoint>("fc_settings");
      setEndpoint(current);
      setFolder(current.folder);
    });
  });
  onCleanup(() => { void invoke("disconnect_fc").catch(() => {}); });

  async function task(action: () => Promise<void>) {
    if (blocked()) return;
    setError("");
    setPending(true);
    try { await action(); }
    catch (reason) { setError(String(reason)); }
    finally { setPending(false); }
  }

  function acceptStatus(status: ConnectStatus) {
    if (status.status === "verify") {
      setFingerprint(status.fingerprint);
      setStage("verify");
    } else {
      setStage("password");
    }
  }

  function connect(event: SubmitEvent) {
    event.preventDefault();
    void task(async () => acceptStatus(await invoke<ConnectStatus>("connect_fc", { endpoint: endpoint() })));
  }

  function password(event: SubmitEvent & { currentTarget: HTMLFormElement }) {
    event.preventDefault();
    const fields = new FormData(event.currentTarget);
    event.currentTarget.reset();
    void task(async () => {
      const path = await invoke<string>("authenticate_fc", { password: String(fields.get("password") ?? "") });
      setStage("browse");
      await list(path);
    });
  }

  async function list(path: string) {
    setFolder(path);
    setSelected(new Set<string>());
    setListing(undefined);
    const result = await invoke<Listing>("list_remote", { path });
    setListing(result);
    setFolder(result.path);
  }

  function toggle(name: string, checked: boolean) {
    const next = new Set(selected());
    if (checked) next.add(name);
    else next.delete(name);
    setSelected(next);
  }

  function disconnect() {
    void task(async () => {
      await invoke("disconnect_fc");
      setStage("connect");
      setListing(undefined);
      setSelected(new Set<string>());
    });
  }

  function download() {
    const path = listing()?.path;
    if (!path) return;
    void task(async () => {
      const destination = await open({ directory: true, title: "Choose a folder for the downloaded logs" });
      if (destination) await props.download(path, [...selected()], destination);
    });
  }

  function goUp() {
    const path = listing()?.path ?? folder();
    void task(() => list(path.slice(0, path.lastIndexOf("/")) || "/"));
  }

  return <dialog ref={dialog} class="fc-dialog" aria-labelledby="fc-title" onCancel={(event) => {
    event.preventDefault();
    if (!blocked()) props.onClose();
  }}>
    <div class="dialog-heading">
      <h2 id="fc-title">Flight computer logs</h2>
      <button class="subtle" disabled={blocked()} onClick={props.onClose}>Close</button>
    </div>
    <Show when={error()}><p class="inline-error" role="alert">{error()}</p></Show>
    <Show when={stage() === "connect"}>
      <form class="settings-form" onSubmit={connect}>
        <label>Hostname <input required value={endpoint().host} disabled={blocked()} onInput={(event) => setEndpoint({ ...endpoint(), host: event.currentTarget.value })} /></label>
        <label>Username <input required value={endpoint().username} disabled={blocked()} onInput={(event) => setEndpoint({ ...endpoint(), username: event.currentTarget.value })} /></label>
        <label>SSH port <input required type="number" min="1" max="65535" value={endpoint().port} disabled={blocked()} onInput={(event) => setEndpoint({ ...endpoint(), port: event.currentTarget.valueAsNumber })} /></label>
        <label class="full-width">Starting folder <input required value={endpoint().folder} disabled={blocked()} onInput={(event) => setEndpoint({ ...endpoint(), folder: event.currentTarget.value })} /></label>
        <div class="dialog-actions full-width"><button class="primary" disabled={blocked()}>{pending() ? "Connecting…" : "Connect"}</button></div>
      </form>
    </Show>
    <Show when={stage() === "verify"}>
      <p>Verify the SSH fingerprint for <strong>{endpoint().host}</strong> with your FC operator before proceeding.</p>
      <p class="fingerprint"><code>{fingerprint()}</code></p>
      <div class="dialog-actions">
        <button disabled={blocked()} onClick={disconnect}>Back</button>
        <button class="primary" disabled={blocked()} onClick={() => void task(async () => acceptStatus(await invoke<ConnectStatus>("confirm_fc")))}>Trust this fingerprint</button>
      </div>
    </Show>
    <Show when={stage() === "password"}>
      <form class="password-form" onSubmit={password}>
        <label>Password for {endpoint().username}@{endpoint().host} <input type="password" name="password" autocomplete="current-password" required disabled={blocked()} /></label>
        <div class="dialog-actions">
          <button type="button" disabled={blocked()} onClick={disconnect}>Back</button>
          <button class="primary" disabled={blocked()}>{pending() ? "Signing in…" : "Sign in"}</button>
        </div>
      </form>
    </Show>
    <Show when={stage() === "browse"}>
      <form class="remote-path" onSubmit={(event) => { event.preventDefault(); void task(() => list(folder())); }}>
        <button type="button" disabled={blocked() || (listing()?.path ?? folder()) === "/"} onClick={goUp}>Up</button>
        <input aria-label="Remote folder" value={folder()} disabled={blocked()} onInput={(event) => setFolder(event.currentTarget.value)} />
        <button disabled={blocked()}>Open folder</button>
      </form>
      <div class="remote-list" aria-label="Remote files">
        <For each={listing()?.entries} fallback={<p class="muted">{pending() ? "Loading folder…" : listing() ? "This folder has no postcard files or subfolders." : "Open a folder to browse its logs."}</p>}>
          {(entry) => <Show when={entry.directory} fallback={
            <label class="remote-entry">
              <input type="checkbox" checked={selected().has(entry.name)} disabled={blocked()} onChange={(event) => toggle(entry.name, event.currentTarget.checked)} />
              <span>{entry.name}</span>
              <span class="muted">{(entry.bytes / 1048576).toFixed(1)} MiB</span>
            </label>
          }>
            <button class="subtle remote-entry" disabled={blocked()} onClick={() => void task(() => list(`${(listing()?.path ?? "/").replace(/\/$/, "")}/${entry.name}`))}>
              <span>{entry.name}/</span><span class="muted">Folder</span>
            </button>
          </Show>}
        </For>
      </div>
      <Show when={props.busy}>{props.status()}</Show>
      <div class="dialog-actions">
        <span class="muted">{selected().size} selected</span>
        <button disabled={blocked()} onClick={disconnect}>Disconnect</button>
        <button class="primary" disabled={blocked() || !selected().size} onClick={download}>Download &amp; open</button>
      </div>
    </Show>
  </dialog>;
}
