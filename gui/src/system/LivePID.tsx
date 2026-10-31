/*import { Component, For, Show, createSignal } from "solid-js";
import hetsPressurant from "../assets/pid/hets/pressurant.svg?raw";
import hetsPropellant from "../assets/pid/hets/propellant.svg?raw";
import hetsPurge from "../assets/pid/hets/purgeandpneumatics.svg?raw";
import hetsFire from "../assets/pid/hets/firepreventionandsuppression.svg?raw";


const hetsPages = [
  { name: "Pressurant", svg: hetsPressurant },
  { name: "Propellant", svg: hetsPropellant },
  { name: "Purge & Pneumatics", svg: hetsPurge },
  { name: "Fire Protection", svg: hetsFire },
];

const [activeTab, setActiveTab] = createSignal("HETS");
const [activeHetsPage, setActiveHetsPage] = createSignal(0);

const LivePidPage: Component = () => {
  return <div class="system-live-pid-page">
    <div style="text-align: center; font-size: 14px">LIVE P&ID</div>
    <div class="live-pid-tabs">
      <button class={activeTab() === "HETS" ? "live-pid-tab active" : "live-pid-tab"}
              onClick={() => setActiveTab("HETS")}>HETS</button>
    </div>
    <div class="live-pid-view">
      <Show when={activeTab() === "HETS"}>
        <div class="live-pid-tabs">
          <For each={hetsPages}>{(page, i) =>
            <button class={activeHetsPage() === i() ? "live-pid-tab active" : "live-pid-tab"}
                    onClick={() => setActiveHetsPage(i())}>{page.name}</button>
          }</For>
        </div>
        <div innerHTML={hetsPages[activeHetsPage()].svg} />
      </Show>
    </div>
  </div>
}

export default LivePidPage;*/

import { Component, For, Show, createEffect, createSignal } from "solid-js";
import { listen } from "@tauri-apps/api/event";
import { StreamState } from "../comm";
import hetsPressurant from "../assets/pid/hets/pressurant.svg?raw";
import hetsPropellant from "../assets/pid/hets/propellant.svg?raw";
import hetsPurge from "../assets/pid/hets/purgeandpneumatics.svg?raw";
import hetsFire from "../assets/pid/hets/firepreventionandsuppression.svg?raw";

const hetsPages = [
  { name: "Pressurant", svg: hetsPressurant },
  { name: "Propellant", svg: hetsPropellant },
  { name: "Purge & Pneumatics", svg: hetsPurge },
  { name: "Fire Protection", svg: hetsFire },
];

//P&ID Fluid Colors
const fluidColors = {
  GN2: "#00B050",   // nitrogen bottles (pressurant and purge)
  FUEL: "#C00000",  // fuel tank
  LOX: "#0070C0",   // LOX tank
  LN2: "#789440",   // LN2 dewar
  CO2: "#A0A0A0",   // CO2 for fire suppression
};
type Fluid = keyof typeof fluidColors;

//Config valve name from notion connectsto the valve P&ID name and the fluid from connected tank.
const actuatedValves: Record<string, { id: string; fluid: Fluid }> = {
  //Pressurant
  "PR-ISO2":  { id: "PR-ISO-2", fluid: "GN2" },
  "FBANG":    { id: "F-BNG",    fluid: "GN2" },
  "OBANG":    { id: "O-BNG",    fluid: "GN2" },
  //Propellant
  "FVNT":     { id: "F-VNT",    fluid: "FUEL" },
  "OVNT":     { id: "O-VNT",    fluid: "LOX" },
  "FPV":      { id: "F-PV",     fluid: "FUEL" },
  "OPV":      { id: "O-PV",     fluid: "LOX" },
  "FMV":      { id: "F-MV",     fluid: "FUEL" },
  "OMV":      { id: "O-MV",     fluid: "LOX" },
  "OBV":      { id: "O-BV",     fluid: "LOX" },
  "LN2-FILL": { id: "LN2-FILL", fluid: "LN2" },
  "LOX-FILL": { id: "O-FILL",   fluid: "LOX" },
  //Purge & Pneumatics
  "PU-F":     { id: "PU-F",     fluid: "GN2" },
  "PU-O":     { id: "PU-O",     fluid: "GN2" },
  "PV-ISO2":  { id: "PU-ISO-2", fluid: "GN2" },
  //Fire Prevention & Suppression
  //"FS-ISO2":  { id: "FS-ISO-2", fluid: "CO2" },
};

//valve name -> state ("open", "closed", ...), kept up to date from live data
const [valveStates, setValveStates] = createSignal<Record<string, string>>({});

listen("device_update", (event) => {
  const valves = (event.payload as StreamState).valve_states as Record<string, any>;
  const next: Record<string, string> = {};
  for (const name in valves) next[name] = valves[name].actual;
  setValveStates(next);
});

//Fake Tester
if (import.meta.env.DEV) {
  (window as any).fakeValves = (states: Record<string, string>) =>
    setValveStates({ ...valveStates(), ...states });
}

//Fluid's color if open, orange if fault, white otherwise
function valveColor(state: string, fluid: Fluid) {
  if (state === "open") return fluidColors[fluid];
  if (state === "fault") return "#CC9A13";
  return "#FFFFFF";
}

//Find Visio valve tag
function findSymbol(container: HTMLElement, tag: string): Element | null {
  const byId = container.querySelector(`#${CSS.escape(tag)}`);
  if (byId) return byId;
  const title = Array.from(container.querySelectorAll("title"))
    .find(t => t.textContent?.trim() === tag);
  return title?.parentElement ?? null;
}

const [activeTab, setActiveTab] = createSignal("HETS");
const [activeHetsPage, setActiveHetsPage] = createSignal(0);

const LivePidPage: Component = () => {
  let svgContainer: HTMLDivElement | undefined;

  //Re-color the valves whenever their states/page change
  createEffect(() => {
    activeHetsPage();
    const states = valveStates();
    if (!svgContainer) return;
    for (const name in states) {
      const valve = actuatedValves[name];
      if (!valve) continue; // not an actuated valve on the P&ID
      const symbol = findSymbol(svgContainer, valve.id);
      if (!symbol) continue; // valve isn't on this page
      const color = valveColor(states[name], valve.fluid);
      // fill the valve's shapes, but leave its text label alone
      const shapes = symbol.matches("path, polygon, rect, ellipse, circle")
        ? [symbol]
        : Array.from(symbol.querySelectorAll("path, polygon, rect, ellipse, circle"));
      shapes.forEach(el => (el as SVGElement).style.fill = color);
    }
  });

  return <div class="system-live-pid-page">
    <div style="text-align: center; font-size: 14px">LIVE P&ID</div>
    <div class="live-pid-tabs">
      <button class={activeTab() === "HETS" ? "live-pid-tab active" : "live-pid-tab"}
              onClick={() => setActiveTab("HETS")}>HETS</button>
    </div>
    <div class="live-pid-view">
      <Show when={activeTab() === "HETS"}>
        <div class="live-pid-tabs">
          <For each={hetsPages}>{(page, i) =>
            <button class={activeHetsPage() === i() ? "live-pid-tab active" : "live-pid-tab"}
                    onClick={() => setActiveHetsPage(i())}>{page.name}</button>
          }</For>
        </div>
        <div class="live-pid-drawing" ref={svgContainer} innerHTML={hetsPages[activeHetsPage()].svg} />
      </Show>
    </div>
  </div>
}

export default LivePidPage;