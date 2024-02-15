import { createEffect, createSignal } from "solid-js";
import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import SensorSectionView from "./SensorSectionView";
import { Device} from "../devices";
import { listen } from "@tauri-apps/api/event";
import { Config, State, StreamSensor, StreamState, sendCalibrate} from "../comm";
import { appWindow } from "@tauri-apps/api/window";
import { invoke } from "@tauri-apps/api/tauri";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [serverIp, setServerIp] = createSignal();

const [sensors, setSensors] = createSignal(new Array);
const [sensCalibrations, setSensCalibrations] = createSignal(new Map);

export const [view, setView] = createSignal('sorted');

const sensorTypes = ['tc', 'pt', 'current_loop', 'differential_signal'];

invoke('initialize_state', {window: appWindow});

// listens to device updates and updates the values of sensors accordingly for display
listen('device_update', (event) => {
  // get sensor data
  const sensor_object = (event.payload as StreamState).sensor_readings;
  var devices = Object.keys(sensor_object).map((key) => [key, sensor_object[key as keyof typeof sensor_object] as StreamSensor]);
  // update data
  devices.forEach((device) => {
    var index = sensors().findIndex(item => (item.name === device[0] as string));
    var new_sensors = structuredClone(sensors());
    new_sensors[index].value = (device[1] as StreamSensor).value;
    new_sensors[index].unit = (device[1] as StreamSensor).unit;
    setSensors(new_sensors);
  });
});

listen('state', (event) => {
  //console.log(event.windowLabel);
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
  setServerIp((event.payload as State).serverIp);
  setSensCalibrations((event.payload as State).calibrations);
  //console.log(activeConfig());
  console.log(configurations() as Config[]);
  var activeconfmappings = (configurations() as Config[]).filter((conf) => {return conf.id == activeConfig() as string})[0];
  var sens = new Array;
  //console.log(activeconfmappings);
  for (const mapping of activeconfmappings.mappings) {
    if (sensorTypes.includes(mapping.channel_type)) {
      sens.push(
        {
          name: mapping.text_id,
          group: 'Fuel',
          board_id: mapping.board_id,
          channel_type: mapping.channel_type,
          channel: mapping.channel,
          unit: mapping.channel_type === 'tc'? 'K' : 'psi',
          value: 0,
          offset: NaN//sensCalibrations().get(mapping.text_id),
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

async function calibrate() {
  var calibrations = await sendCalibrate(serverIp() as string);
}

function Sensors() {
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="Sensors"/>
    </div>
    <div style="display: flex; flex-direction: column; overflow: hidden">
      <div style="display: flex; justify-content: center">
        <button class="toggle-view-button" onClick={calibrate}>Calibrate</button>
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