import { batch, createMemo, createResource, createSignal, For, onCleanup, onMount, Show } from "solid-js";
import { render } from "solid-js/web";
import { Channel, invoke } from "@tauri-apps/api/core";
import { getCurrentWebview } from "@tauri-apps/api/webview";
import { open, save } from "@tauri-apps/plugin-dialog";
import { PAGE_SIZE, utcTime, type Dataset, type DownloadReport, type Progress, type Source } from "./api";
import { FlightComputer } from "./remote";
import "./style.css";

const postcardFilter = { name: "Postcard logs", extensions: ["postcard"] };

function App() {
  const [dataset, setDataset] = createSignal<Dataset | null>(null);
  const [visible, setVisible] = createSignal<Set<string>>(new Set());
  const [page, setPage] = createSignal(0);
  const [jump, setJump] = createSignal("");
  const [filter, setFilter] = createSignal("");
  const [operation, setOperation] = createSignal("");
  const [progress, setProgress] = createSignal<Progress>();
  const [canceling, setCanceling] = createSignal(false);
  const [messages, setMessages] = createSignal<string[]>([]);
  const [error, setError] = createSignal("");
  const [dragging, setDragging] = createSignal(false);
  const [fcOpen, setFcOpen] = createSignal(false);

  const selected = createMemo(() => (dataset()?.columns ?? []).filter((column) => visible().has(column)));
  const matching = createMemo(() => (dataset()?.columns ?? []).filter((column) => column.toLowerCase().includes(filter().trim().toLowerCase())));
  const pageCount = () => Math.max(1, Math.ceil((dataset()?.rows ?? 0) / PAGE_SIZE));

  const [result] = createResource(() => {
    const id = dataset()?.id;
    return id && selected().length ? { id, start: page() * PAGE_SIZE, columns: selected() } : false;
  }, async (args) => {
    try {
      return { rows: await invoke<string[][]>("get_page", args), error: "" };
    } catch (reason) {
      return { rows: [], error: String(reason) };
    }
  });

  function useDataset(next: Dataset | null, preserveSelection: boolean) {
    const previous = dataset();
    const keepSubset = preserveSelection && previous && visible().size < previous.columns.length;
    batch(() => {
      setVisible(new Set(next?.columns.filter((column) => !keepSubset || visible().has(column)) ?? []));
      setDataset(next);
      setPage(0);
      setJump("");
      if (!preserveSelection) setFilter("");
    });
  }

  async function run<T>(label: string, action: (channel: Channel<Progress>) => Promise<T>): Promise<T> {
    if (operation()) throw new Error("An operation is already running.");
    const channel = new Channel<Progress>();
    channel.onmessage = setProgress;
    try {
      batch(() => {
        setError("");
        setMessages([]);
        setProgress(undefined);
        setCanceling(false);
        setOperation(label);
      });
      return await action(channel);
    } finally {
      channel.onmessage = () => {};
      setOperation("");
    }
  }

  async function importLogs(paths: string[], append: boolean) {
    try {
      const next = await run("Reading logs", (channel) => invoke<Dataset>("load_logs", { paths, append, progress: channel }));
      useDataset(next, append);
      setMessages(next.notices);
    } catch (reason) {
      setError(String(reason));
    }
  }

  onMount(() => {
    let disposed = false;
    let stop: (() => void) | undefined;
    void getCurrentWebview().onDragDropEvent(({ payload }) => {
      if (payload.type === "enter" || payload.type === "over") setDragging(!operation() && !fcOpen());
      if (payload.type === "leave" || payload.type === "drop") setDragging(false);
      if (payload.type === "drop" && !operation() && !fcOpen()) void importLogs(payload.paths, !!dataset());
    }).then((unlisten) => {
      if (disposed) unlisten();
      else stop = unlisten;
    }).catch((reason) => setError(`Drag and drop could not start. ${String(reason)}`));
    onCleanup(() => { disposed = true; stop?.(); });
  });

  async function chooseLogs(append: boolean) {
    try {
      const paths = await open({ title: append ? "Add postcard chunks" : "Open postcard logs", multiple: true, filters: [postcardFilter] });
      if (paths?.length) await importLogs(paths, append);
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function remove(source: Source) {
    const id = dataset()?.id;
    if (!id) return;
    try {
      const next = await run("Updating sources", () => invoke<Dataset | null>("remove_source", { id, path: source.path }));
      useDataset(next, true);
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function exportCsv(all: boolean) {
    const current = dataset();
    if (!current) return;
    try {
      const name = current.sources[0].name.replace(/\.postcard$/i, "");
      const chosen = await save({ title: "Save CSV", defaultPath: `${name}${current.sources.length > 1 ? "_stitched" : ""}.csv`, filters: [{ name: "CSV", extensions: ["csv"] }] });
      if (!chosen) return;
      const confirmedPath = /\.csv$/i.test(chosen);
      const path = confirmedPath ? chosen : `${chosen}.csv`;
      await run("Exporting CSV", (channel) => invoke("export_csv", {
        id: current.id, path, overwrite: confirmedPath, columns: all ? null : selected(), progress: channel,
      }));
      setMessages([`CSV saved to ${path}`]);
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function download(folder: string, names: string[], destination: string) {
    const report = await run("Downloading logs", (channel) => invoke<DownloadReport>("download_logs", { folder, names, destination, progress: channel }));
    if (report.files.length) {
      setFcOpen(false);
      await importLogs(report.files, false);
      setMessages((previous) => [`Saved ${report.files.length} ${report.files.length === 1 ? "file" : "files"} to ${destination}`, ...previous, ...report.notices]);
    } else if (report.notices.length) {
      throw new Error(report.notices.join("\n"));
    }
  }

  function cancel() {
    setCanceling(true);
    void invoke("cancel_job").catch((reason) => setError(String(reason)));
  }

  function toggle(column: string, checked: boolean) {
    const next = new Set(visible());
    if (checked) next.add(column);
    else next.delete(column);
    setVisible(next);
  }

  const status = () => <ProgressStatus label={operation()} progress={progress()} cancel={cancel} canceling={canceling()} />;

  return (
    <div class="app">
      <header class="toolbar">
        <h1>Mailroom</h1>
        <div class="toolbar-actions">
          <button disabled={!!operation()} onClick={() => void chooseLogs(false)}>{dataset() ? "Open new run…" : "Open files…"}</button>
          <Show when={dataset()}><button disabled={!!operation()} onClick={() => void chooseLogs(true)}>Add chunks…</button></Show>
          <button class="primary" disabled={!!operation()} onClick={() => setFcOpen(true)}>Connect to FC</button>
        </div>
      </header>

      <Show when={error() || messages().length > 0}>
        <div classList={{ banner: true, error: !!error() }} role={error() ? "alert" : "status"}>
          <div>{[error(), ...messages()].filter(Boolean).join("\n")}</div>
          <button class="subtle" aria-label="Dismiss message" onClick={() => { setError(""); setMessages([]); }}>Dismiss</button>
        </div>
      </Show>
      <Show when={operation() && !fcOpen()}>{status()}</Show>

      <Show when={dataset()} fallback={
        <main class="empty">
          <div class="drop-zone">
            <img src={new URL("../src-tauri/icons/icon.png", import.meta.url).href} alt="" width="72" height="72" />
            <h2>Drop postcard logs here</h2>
            <p>Open one file or drop all the chunks from a run together.</p>
            <button disabled={!!operation()} onClick={() => void chooseLogs(false)}>Browse files…</button>
          </div>
        </main>
      }>
        {(current) => <main class="workspace">
          <section class="run" aria-label="Run summary">
            <div>
              <strong>{current().kind === "Imu" ? "IMU logs" : "Flight logs"}</strong>
              <span class="muted">{current().sources.length} {current().sources.length === 1 ? "chunk" : "chunks"} · {current().rows.toLocaleString()} rows</span>
              <div class="run-times">{utcTime(current().firstTimestamp)} → {utcTime(current().lastTimestamp)}</div>
            </div>
            <div class="export-actions">
              <button disabled={!!operation()} onClick={() => void exportCsv(true)}>Export all columns</button>
              <button class="primary" disabled={!!operation() || !selected().length} onClick={() => void exportCsv(false)}>Export visible columns</button>
            </div>
          </section>

          <div class="workspace-body">
            <aside class="sidebar">
              <section class="panel columns" aria-labelledby="columns-heading">
                <div class="panel-header"><h2 id="columns-heading">Columns</h2><span class="muted">{selected().length} of {current().columns.length}</span></div>
                <input type="search" placeholder="Find columns" aria-label="Find columns" value={filter()} onInput={(event) => setFilter(event.currentTarget.value)} />
                <div class="column-actions">
                  <button class="subtle" onClick={() => setVisible(new Set(current().columns))}>Select all</button>
                  <button class="subtle" onClick={() => setVisible(new Set<string>())}>Clear selection</button>
                </div>
                <Show when={filter().trim() && matching().length}>
                  <button class="subtle matches" onClick={() => setVisible(new Set(matching()))}>Show matches only</button>
                </Show>
                <div class="column-list">
                  <For each={matching()} fallback={<p class="muted">No matching columns.</p>}>
                    {(column) => <label class="column-option" title={column}>
                      <input type="checkbox" checked={visible().has(column)} onChange={(event) => toggle(column, event.currentTarget.checked)} />
                      <span>{column}</span>
                    </label>}
                  </For>
                </div>
              </section>
              <details class="panel sources" open>
                <summary>Source files</summary>
                <div class="source-list">
                  <For each={current().sources}>
                    {(source) => <article class="source">
                      <div class="source-header">
                        <span title={source.path}>{source.name}</span>
                        <button class="subtle" aria-label={`Remove ${source.name}`} disabled={!!operation()} onClick={() => void remove(source)}>Remove</button>
                      </div>
                      <small class="muted">{source.rows.toLocaleString()} rows · {utcTime(source.firstTimestamp).slice(11)}</small>
                      <Show when={source.gapSeconds !== null}>
                        <small classList={{ warning: source.gapSeconds! <= 0, muted: source.gapSeconds! > 0 }}>
                          {source.gapSeconds! <= 0 ? `Timestamp overlap (${Math.abs(source.gapSeconds!).toFixed(3)} s)` : `${source.gapSeconds!.toFixed(3)} s after previous coverage`}
                        </small>
                      </Show>
                      <Show when={source.warning}><small class="warning">{source.warning}</small></Show>
                    </article>}
                  </For>
                </div>
              </details>
            </aside>

            <section class="panel preview" aria-label="Log preview">
              <div class="table-scroll" role="region" tabindex="0" aria-label="Log table">
                <Show when={selected().length} fallback={<p class="table-note">Select columns to display their values.</p>}>
                  <Show when={!result.loading} fallback={<p class="table-note" role="status">Loading rows…</p>}>
                    <Show when={!result()?.error} fallback={<p class="table-note warning" role="alert">{result()?.error}</p>}>
                      <table>
                        <thead><tr><For each={selected()}>{(column) => <th classList={{ timestamp: column === "timestamp" }} scope="col">{column}</th>}</For></tr></thead>
                        <tbody>
                          <For each={result()?.rows}>{(row) => <tr>
                            <For each={row}>{(cell, index) => (
                              <td classList={{ timestamp: selected()[index()] === "timestamp" }} title={cell}>{cell}</td>
                            )}</For>
                          </tr>}</For>
                        </tbody>
                      </table>
                    </Show>
                  </Show>
                </Show>
              </div>
              <footer class="pager">
                <span class="muted">Rows {(page() * PAGE_SIZE + 1).toLocaleString()}–{Math.min((page() + 1) * PAGE_SIZE, current().rows).toLocaleString()}</span>
                <form onSubmit={(event) => {
                  event.preventDefault();
                  const requested = Number(jump());
                  if (jump() && Number.isInteger(requested)) setPage(Math.min(pageCount(), Math.max(1, requested)) - 1);
                  setJump("");
                }}>
                  <button type="button" disabled={page() === 0} onClick={() => setPage(page() - 1)}>Previous</button>
                  <input type="number" aria-label="Go to page" min="1" max={pageCount()} placeholder={String(page() + 1)} value={jump()} onInput={(event) => setJump(event.currentTarget.value)} />
                  <span class="muted">of {pageCount().toLocaleString()}</span>
                  <button type="button" disabled={page() + 1 === pageCount()} onClick={() => setPage(page() + 1)}>Next</button>
                </form>
              </footer>
            </section>
          </div>
        </main>}
      </Show>
      <Show when={dragging()}><div class="drop-overlay" aria-hidden="true">{dataset() ? "Drop to add chunks" : "Drop to open logs"}</div></Show>
      <Show when={fcOpen()}>
        <FlightComputer onClose={() => setFcOpen(false)} download={download} busy={!!operation()} status={status} />
      </Show>
    </div>
  );
}

function ProgressStatus(props: { label: string; progress: Progress | undefined; cancel: () => void; canceling: boolean }) {
  const fraction = () => props.progress && props.progress.total > 0 ? Math.min(1, props.progress.done / props.progress.total) : undefined;
  return <div class="progress" role="status">
    <span><strong>{props.label}</strong><Show when={props.progress?.label}> · {props.progress?.label}</Show></span>
    <Show when={fraction() !== undefined} fallback={<progress max="1" aria-label={props.label} />}>
      <progress max="1" value={fraction() ?? 0} aria-label={props.label} />
    </Show>
    <button disabled={props.canceling} onClick={props.cancel}>{props.canceling ? "Canceling…" : "Cancel"}</button>
  </div>;
}

render(() => <App />, document.getElementById("root")!);
