import { For, createSignal, onCleanup } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import {
  TelemetrySource,
  TelemetrySourceStats,
  TelemetryStatsResponse,
  currentDataSource,
  getTelemetryStats,
  isConnected,
  selectTelemetrySource,
  serverIp,
} from "../../comm";

const emptyStats: TelemetrySourceStats = {
  time_since_update_ms: null,
  update_rate_hz: null,
  packet_size_bytes: null,
};

function formatStatValue(value: number | null, digits: number = 3) {
  return value == null ? "Waiting..." : value.toFixed(digits);
}

function sourceLabel(source: TelemetrySource) {
  return source === 'umbilical' ? 'Umbilical' : 'Radio';
}

function sourceStats(
  source: TelemetrySource,
  stats: TelemetryStatsResponse | null,
) {
  if (stats == null) {
    return emptyStats;
  }

  return source === 'umbilical' ? stats.umbilical : stats.tel;
}

invoke('initialize_state', {window: appWindow});

function TEL() {
  const [telemetryStats, setTelemetryStats] = createSignal<TelemetryStatsResponse | null>(null);

  const refreshTelemetryStats = async () => {
    if (!isConnected() || !serverIp()) {
      setTelemetryStats(null);
      return;
    }

    const response = await getTelemetryStats(serverIp() as string);
    if (
      response != null &&
      typeof response === "object" &&
      "umbilical" in response &&
      "tel" in response
    ) {
      setTelemetryStats(response as TelemetryStatsResponse);
    }
  };

  refreshTelemetryStats();
  const pollHandle = setInterval(refreshTelemetryStats, 500);
  onCleanup(() => clearInterval(pollHandle));

  const renderStats = (source: TelemetrySource) => {
    const stats = () => sourceStats(source, telemetryStats());
    const selected = () => currentDataSource() === source;

    return (
      <div class={`tel-source-panel ${selected() ? "tel-source-panel-active" : "tel-source-panel-inactive"}`}>
        <button
          class={`tel-source-button ${selected() ? "tel-source-button-active" : "tel-source-button-inactive"}`}
          onClick={() => selectTelemetrySource(source)}
        >
          {sourceLabel(source)}
        </button>
        <div class="tel-source-stat-list">
          <div class="tel-source-stat-row">
            <div class="tel-source-stat-name">Update rate</div>
            <div class="tel-source-stat-value">{formatStatValue(stats().update_rate_hz)} Hz</div>
          </div>
          <div class="tel-source-stat-row">
            <div class="tel-source-stat-name">Time since update</div>
            <div class="tel-source-stat-value">{formatStatValue(stats().time_since_update_ms)} ms</div>
          </div>
          <div class="tel-source-stat-row">
            <div class="tel-source-stat-name">Latest packet size</div>
            <div class="tel-source-stat-value">
              {stats().packet_size_bytes == null ? "Waiting..." : `${stats().packet_size_bytes} B`}
            </div>
          </div>
        </div>
      </div>
    );
  };

  return <div class="window-template">
  <div style="height: 60px">
    <GeneralTitleBar name="TEL"/>
  </div>
  <div class="tel-view">
    <div class="tel-data-container-button">
      <div class="tel-page-subtitle">Telemetry Source</div>
    </div>
    <div class="tel-horizontal-container">
      <For each={(['umbilical', 'tel'] as TelemetrySource[])}>{(source) =>
        renderStats(source)
      }</For>
    </div>
    {!isConnected() && (
      <div class="tel-disconnected-note">
        Connect to Servo to begin receiving telemetry source statistics.
      </div>
    )}
  </div>
  <div>
    <Footer/>
  </div>
</div>
}

export default TEL;
