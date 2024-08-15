import { For, createEffect, createSignal } from "solid-js";
import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import SensorSectionView from "./SensorSectionView";
import { Device} from "../devices";
import { listen } from "@tauri-apps/api/event";
import { Config, Mapping, State, StreamSensor, StreamState, sendCalibrate} from "../comm";
import { appWindow } from "@tauri-apps/api/window";
import { invoke } from "@tauri-apps/api/tauri";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [serverIp, setServerIp] = createSignal();

const [deviceOptions, setDeviceOptions] = createSignal(new Array);
const [sensors, setSensors] = createSignal(new Array);
const [sensCalibrations, setSensCalibrations] = createSignal(new Map);

export const [view, setView] = createSignal('sorted');

const sensorTypes = ['tc', 'pt', 'flow_meter', 'load_cell'];

// listens to device updates and updates the values of sensors accordingly for display
listen('device_update', (event) => {
  // get sensor data
  const sensor_object = (event.payload as StreamState).sensor_readings;
  var devices = Object.keys(sensor_object).map((key) => [key, sensor_object[key as keyof typeof sensor_object] as StreamSensor]);
  // update data
  console.log(devices);
  devices.forEach((device) => {
    var index = sensors().findIndex(item => (item.name === device[0] as string));
    if (index === -1) {
      return;
    }
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
  var deviceOptions = new Array;
  //console.log(activeconfmappings);
  for (const mapping of activeconfmappings.mappings) {
    if (sensorTypes.includes(mapping.sensor_type)) {
      deviceOptions.push(mapping);
    }
  }
  //console.log(sensors())
  setDeviceOptions(deviceOptions);
});

invoke('initialize_state', {window: appWindow});

function toggleView() {
  if (view() == 'sorted') {
    setView('grouped');
  } else {
    setView('sorted');
  }
  console.log(view());
}

function openDropdown() {
  console.log("opening dropdown");
  var button = document.getElementById("sensbutton")!;
  var dropdownContent = document.getElementById("sensdropdown")!;
  dropdownContent.style.display = "flex";
}

function closeDropdown(evt:MouseEvent) {
  var button = document.getElementById("sensbutton")!;
  var dropdownContent = document.getElementById("sensdropdown")!;
  if (evt.target != button) {
      dropdownContent.style.display = "none";
  }
}

async function calibrate() {
  var calibrations = await sendCalibrate(serverIp() as string);
}

function addSensDevice(mapping: Mapping) {
  var newSensors = [...sensors() as Device[]]
  var indexToRemove = -1;
  for (var i = 0; i < sensors().length; i++) {
      if (sensors()[i].name === mapping.text_id) {
          indexToRemove = i;
          break;
      }
  }
  if (indexToRemove != -1) {
      console.log('deleting...');
      newSensors.splice(indexToRemove, 1);
      setSensors(newSensors);
      return;
  }
  newSensors.push({
    name: mapping.text_id,
    group: 'Fuel',
    board_id: mapping.board_id,
    sensor_type: mapping.sensor_type,
    channel: mapping.channel,
    unit: '?',
    value: 0,
    offset: NaN
  });
  setSensors(newSensors);
}

document.addEventListener("click", (evt) => closeDropdown(evt));

function Sensors() {
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="Sensors"/>
    </div>
    <div style="display: flex; flex-direction: column; overflow: hidden">
      <div style="display: flex; justify-content: center; gap: 20px">
        <div id='sensbutton' class='addsensbutton' onClick={openDropdown}>Add/Remove Sensors</div>
        <div id="sensdropdown" class="sensdropdowncontent">
                {deviceOptions().length != 0? <For each={deviceOptions() as Mapping[]}>{(mapping, i) =>
                    <div class="sensdropdownitem" onClick={() => addSensDevice(mapping)}>{mapping.text_id}</div>
                }</For>:<div class="sensdropdownitem">no sensors or active config rip</div>
                }
        </div>
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