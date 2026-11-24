import { Accessor, For, Show, createEffect, createSignal, onCleanup, onMount } from "solid-js";
import { Chart, ChartDataset, registerables } from "chart.js";
import type { CtvStatus } from "../comm";
import {
  GPS_GATE_NIS,
  NAV_PICK,
  PICK,
  Point,
  View,
  constant,
  navSeries,
  sensorBadges,
  series,
  xRange,
} from "./ctvTrace";

Chart.register(...registerables);

/** What a line of a chart needs besides the view. */
interface Context {
  tvcLimitDeg: number;
}

interface Line {
  label: string;
  color: string;
  points: (view: View, context: Context) => Point[];
  dashed?: boolean;
  /** drawn as dots, for readings that arrive as events */
  dots?: boolean;
}

interface ChartSpec {
  id: string;
  title: string;
  unit: string;
  lines: Line[];
  yMin?: number;
  /** A range the axis shows at least, so a flat line near zero is not blown up into noise. */
  suggestedMin?: number;
  suggestedMax?: number;
  hint?: string;
}

const BLUE = "#36A2EB";
const GREEN = "#1DB55A";
const ORANGE = "#E6A23C";
const RED = "#C53434";
const GREY = "#9a9a9a";
const PURPLE = "#B18CE0";

const NAV_CHARTS: ChartSpec[] = [
  {
    id: "position",
    title: "Estimated position",
    unit: "m",
    hint: "Navigation frame: north, east, up from the filter's origin (the IMU on the pad).",
    lines: [
      { label: "north", color: BLUE, points: (v) => series(v, PICK.estNorth) },
      { label: "east", color: GREEN, points: (v) => series(v, PICK.estEast) },
      { label: "up", color: ORANGE, points: (v) => series(v, PICK.estUp) },
    ],
  },
  {
    id: "velocity",
    title: "Estimated velocity",
    unit: "m/s",
    lines: [
      { label: "north", color: BLUE, points: (v) => series(v, PICK.velNorth) },
      { label: "east", color: GREEN, points: (v) => series(v, PICK.velEast) },
      { label: "up", color: ORANGE, points: (v) => series(v, PICK.velUp) },
    ],
  },
  {
    id: "tilt",
    title: "Tilt from vertical",
    unit: "deg",
    yMin: 0,
    suggestedMax: 2,
    lines: [{ label: "tilt", color: ORANGE, points: (v) => series(v, PICK.tilt) }],
  },
  {
    id: "rates",
    title: "Body rate",
    unit: "deg/s",
    suggestedMin: -2,
    suggestedMax: 2,
    hint: "Gyro minus bias. Body x is up on the pad: p is the roll rate.",
    lines: [
      { label: "p (roll)", color: BLUE, points: (v) => series(v, PICK.rateP) },
      { label: "q (pitch)", color: GREEN, points: (v) => series(v, PICK.rateQ) },
      { label: "r (yaw)", color: ORANGE, points: (v) => series(v, PICK.rateR) },
    ],
  },
  {
    id: "sigma",
    title: "Uncertainty (1 sigma): position and velocity",
    unit: "m, m/s",
    yMin: 0,
    hint: "How sure the filter is. It grows without GPS, and shrinks as readings are taken.",
    lines: [
      { label: "position, horizontal (m)", color: BLUE, points: (v) => navSeries(v, NAV_PICK.positionHorizontal) },
      { label: "position, vertical (m)", color: ORANGE, points: (v) => navSeries(v, NAV_PICK.positionVertical) },
      { label: "velocity, horizontal (m/s)", color: GREEN, dashed: true, points: (v) => navSeries(v, NAV_PICK.velocityHorizontal) },
      { label: "velocity, vertical (m/s)", color: PURPLE, dashed: true, points: (v) => navSeries(v, NAV_PICK.velocityVertical) },
    ],
  },
  {
    id: "attitude-sigma",
    title: "Uncertainty (1 sigma): attitude",
    unit: "deg",
    yMin: 0,
    lines: [
      { label: "tilt", color: ORANGE, points: (v) => navSeries(v, NAV_PICK.tiltStd) },
      { label: "heading", color: BLUE, points: (v) => navSeries(v, NAV_PICK.headingStd) },
    ],
  },
  {
    id: "gps-nis",
    title: "GPS innovation",
    unit: "NIS",
    yMin: 0,
    hint: "Normalized innovation squared of the latest GPS update. Above the gate the reading is rejected; many in a row make the filter unhealthy.",
    lines: [
      { label: "GPS NIS", color: BLUE, dots: true, points: (v) => navSeries(v, NAV_PICK.gpsNis) },
      {
        label: "gate",
        color: RED,
        dashed: true,
        points: (v) => constant(v, GPS_GATE_NIS),
      },
    ],
  },
];

const CONTROLLER_CHARTS: ChartSpec[] = [
  {
    id: "altitude",
    title: "Altitude: controller input and schedule",
    unit: "m",
    hint: "Pad frame. The controller is given the estimate; the schedule is what the gain table is built around.",
    lines: [
      { label: "estimate", color: BLUE, points: (v) => series(v, PICK.altitude) },
      { label: "schedule", color: GREY, dashed: true, points: (v) => series(v, PICK.referenceAltitude) },
    ],
  },
  {
    id: "position-error",
    title: "Position error (estimate minus schedule)",
    unit: "m",
    suggestedMin: -0.1,
    suggestedMax: 0.1,
    lines: [
      { label: "north", color: BLUE, points: (v) => series(v, PICK.northError) },
      { label: "east", color: GREEN, points: (v) => series(v, PICK.eastError) },
      { label: "up", color: ORANGE, points: (v) => series(v, PICK.upError) },
    ],
  },
  {
    id: "velocity-error",
    title: "Velocity error (estimate minus schedule)",
    unit: "m/s",
    suggestedMin: -0.1,
    suggestedMax: 0.1,
    lines: [
      { label: "north", color: BLUE, points: (v) => series(v, PICK.velNorthError) },
      { label: "east", color: GREEN, points: (v) => series(v, PICK.velEastError) },
      { label: "up", color: ORANGE, points: (v) => series(v, PICK.velUpError) },
    ],
  },
  {
    id: "thrust",
    title: "Commanded thrust",
    unit: "N",
    lines: [{ label: "thrust", color: BLUE, points: (v) => series(v, PICK.thrust) }],
  },
  {
    id: "tvc",
    title: "Commanded TVC angle",
    unit: "deg",
    hint: "The gimbal is held to the limit; a command that sits on it has run out of authority.",
    lines: [
      { label: "pitch", color: BLUE, points: (v) => series(v, PICK.tvcPitch) },
      { label: "yaw", color: GREEN, points: (v) => series(v, PICK.tvcYaw) },
      { label: "limit", color: RED, dashed: true, points: (v, c) => constant(v, c.tvcLimitDeg) },
      { label: "limit ", color: RED, dashed: true, points: (v, c) => constant(v, -c.tvcLimitDeg) },
    ],
  },
  {
    id: "roll-torque",
    title: "Commanded roll torque",
    unit: "N m",
    hint: "The flight computer drives no roll fans yet: this is what the controller asks for.",
    lines: [{ label: "roll torque", color: ORANGE, points: (v) => series(v, PICK.rollTorque) }],
  },
];

function PlotChart(props: {
  spec: ChartSpec;
  view: Accessor<View>;
  context: Accessor<Context>;
  scheduleEnd: Accessor<number>;
}) {
  let canvas: HTMLCanvasElement | undefined;
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  let chart: Chart<"line", any[]> | undefined;

  onMount(() => {
    // the points are {x, y} with a null y for a gap, which the chart takes as it is
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const datasets: ChartDataset<"line", any[]>[] = props.spec.lines.map((line) => ({
      label: line.label,
      data: [],
      borderColor: line.color,
      backgroundColor: line.color,
      borderWidth: line.dots ? 0 : 1.5,
      borderDash: line.dashed ? [6, 4] : [],
      pointRadius: line.dots ? 2.5 : 0,
      pointHoverRadius: 3,
      showLine: !line.dots,
      spanGaps: false,
      tension: 0,
    }));

    chart = new Chart<"line", any[]>(canvas as HTMLCanvasElement, {
      type: "line",
      data: { datasets },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        parsing: false,
        normalized: true,
        interaction: { mode: "nearest", intersect: false },
        scales: {
          x: {
            type: "linear",
            title: { display: true, text: "time since start, s", color: "#aaa", font: { size: 10 } },
            ticks: { color: "#aaa", font: { size: 10 } },
            grid: { color: "rgba(255,255,255,0.08)" },
          },
          y: {
            title: { display: true, text: props.spec.unit, color: "#aaa", font: { size: 10 } },
            ticks: { color: "#aaa", font: { size: 10 } },
            grid: { color: "rgba(255,255,255,0.08)" },
            min: props.spec.yMin,
            suggestedMin: props.spec.suggestedMin,
            suggestedMax: props.spec.suggestedMax,
          },
        },
        plugins: {
          legend: { labels: { color: "#ddd", boxWidth: 10, font: { size: 10 } } },
          tooltip: { enabled: true },
        },
      },
    });
  });

  createEffect(() => {
    const view = props.view();
    const context = props.context();

    if (!chart) return;

    props.spec.lines.forEach((line, i) => {
      chart!.data.datasets[i].data = line.points(view, context);
    });

    const range = xRange(view, props.scheduleEnd());
    const x = chart.options.scales!.x as { min?: number; max?: number; title?: { text?: string } };

    x.min = range.min;
    x.max = range.max;
    if (x.title) {
      x.title.text = view.inFlight ? "time since start, s" : "seconds ago";
    }

    chart.update("none");
  });

  onCleanup(() => chart?.destroy());

  return (
    <div class="ctv-plot" data-testid={`ctv-plot-${props.spec.id}`} title={props.spec.hint}>
      <div class="ctv-plot-title">{props.spec.title}</div>
      <div class="ctv-plot-canvas">
        <canvas ref={canvas} />
      </div>
    </div>
  );
}

function Sensors(props: { status: Accessor<CtvStatus> }) {
  const badges = () => sensorBadges(props.status().nav);

  return (
    <div class="ctv-sensors" data-testid="ctv-sensors">
      <span class="ctv-muted">Latest result per sensor:</span>
      <For each={badges()}>
        {(badge) => (
          <span class={`ctv-sensor ctv-sensor-${badge.result}`} title={`${badge.name}: ${badge.result === "none" ? "no reading yet" : badge.result}`}>
            {badge.name}
          </span>
        )}
      </For>
    </div>
  );
}

export function PlotsCard(props: {
  view: Accessor<View>;
  status: Accessor<CtvStatus>;
  onClear: () => void;
}) {
  const [tab, setTab] = createSignal<"nav" | "controller">("nav");
  const charts = () => (tab() === "nav" ? NAV_CHARTS : CONTROLLER_CHARTS);
  const context = () => ({ tvcLimitDeg: props.status().tvc_limit_deg ?? 8 });
  const scheduleEnd = () => props.status().schedule_end_s;
  const health = () => props.status().nav?.health ?? "";
  const healthy = () => health() === "Healthy";

  return (
    <div class="ctv-card ctv-plots-card" data-testid="ctv-plots">
      <div class="ctv-card-title">Plots</div>

      <div class="ctv-plots-bar">
        <div class="ctv-tabs">
          <button
            class={`ctv-tab ${tab() === "nav" ? "ctv-tab-active" : ""}`}
            onClick={() => setTab("nav")}
            data-testid="ctv-tab-nav"
          >
            Navigation filter
          </button>
          <button
            class={`ctv-tab ${tab() === "controller" ? "ctv-tab-active" : ""}`}
            onClick={() => setTab("controller")}
            data-testid="ctv-tab-controller"
          >
            Controller
          </button>
        </div>
        <div class="ctv-plots-info">
          <span class="ctv-muted">
            {props.view().inFlight ? "The latest flight, from its start" : "The last minute"}
          </span>
          <Show when={health()}>
            <span class={`ctv-pill ${healthy() ? "ctv-pill-ok" : "ctv-pill-bad"}`} data-testid="ctv-nav-health">
              filter: {health()}
            </span>
          </Show>
          <button class="ctv-button ctv-button-secondary ctv-inline-button" onClick={props.onClear}>
            Clear
          </button>
        </div>
      </div>

      <Show when={tab() === "nav"}>
        <Sensors status={props.status} />
      </Show>

      <Show
        when={props.view().samples.length > 0}
        fallback={<div class="ctv-muted ctv-plots-empty">Waiting for samples from the flight computer.</div>}
      >
        <div class="ctv-plot-grid">
          <For each={charts()}>
            {(spec) => <PlotChart spec={spec} view={props.view} context={context} scheduleEnd={scheduleEnd} />}
          </For>
        </div>
      </Show>
    </div>
  );
}
