import { createEffect, createSignal } from "solid-js";
import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import SensorSectionView from "./SensorSectionView";
import { Device, GenericDevice } from "../devices";
import { listen } from "@tauri-apps/api/event";
import { Config, State } from "../comm";
import { appWindow } from "@tauri-apps/api/window";
import { invoke } from "@tauri-apps/api/tauri";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();

const [sensors, setSensors] = createSignal(new Array);
  
export const [view, setView] = createSignal('sorted');

invoke('initialize_state', {window: appWindow});

listen('device_update', (event) => {
  var devices = event.payload as Array<GenericDevice>
  devices.forEach((device) => {
    var index = sensors().findIndex(item => (item.board_id === device.board_id && item.channel === device.channel));
    var new_sensors = JSON.parse(JSON.stringify(sensors()));
    new_sensors[index].value = device.floatValue;
    setSensors(new_sensors);
  });
});

listen('state', (event) => {
  //console.log(event.windowLabel);
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
  //console.log(activeConfig());
  //console.log(configurations() as Config[]);
  var activeconfmappings = (configurations() as Config[]).filter((conf) => {return conf.id == activeConfig() as string})[0];
  var sens = new Array;
  //console.log(activeconfmappings);
  for (const mapping of activeconfmappings.mappings) {
    if (mapping.channel_type === 'tc' || mapping.channel_type === 'current_loop') {
      sens.push(
        {
          name: mapping.text_id,
          group: 'Fuel',
          board_id: mapping.board_id,
          channel_type: mapping.channel_type,
          channel: mapping.channel,
          unit: mapping.channel_type === 'tc'? 'K' : 'psi',
          value: 0,
        } as Device,
      )
    }
  }
  //console.log(sensors())
  setSensors(sens);
});

function toggleView() {
  if (view() == 'sorted') {
    setView('grouped');
  } else {
    setView('sorted');
  }
  console.log(view());
}

function Sensors() {
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="Sensors"/>
    </div>
    <div style="display: flex; flex-direction: column; overflow: hidden">
      <div style="display: flex; justify-content: center">
        <button class="toggle-view-button" onClick={toggleView}>Toggle View</button>
      </div>
      <div class="sensors-body">
        <SensorSectionView sensors={sensors()}/>
      </div>
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default Sensors;