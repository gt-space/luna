import { listen } from "@tauri-apps/api/event";
import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import PlotterView from "./PlotterView";
import { createSignal } from "solid-js";
import { appWindow } from "@tauri-apps/api/window";
import { invoke } from "@tauri-apps/api/tauri";
import { Config, State } from "../comm";

export const [plotterDevices, setPlotterDevices] = createSignal();
const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();

invoke('initialize_state', {window: appWindow});

listen('state', (event) => {
  //console.log(event.windowLabel);
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
  //console.log(activeConfig());
  //console.log(configurations() as Config[]);
  var activeconfmappings = (configurations() as Config[]).filter((conf) => {return conf.id == activeConfig() as string})[0];
  var newPlotterDevices = new Array<{id: string, board_id: string, channel: number, value: number}>;
  activeconfmappings.mappings.forEach(element => {
    newPlotterDevices.push({
      id: element.text_id,
      board_id: element.board_id,
      channel: element.channel,
      value: NaN
    });
  });
  setPlotterDevices(newPlotterDevices);
});

function Plotter() {
    return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="Plotter"/>
    </div>
    <div style="display: flex; flex-direction: column; overflow: hidden">
      <PlotterView />
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default Plotter;