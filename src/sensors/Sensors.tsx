import { createEffect, createSignal } from "solid-js";
import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import SensorSectionView from "./SensorSectionView";
import { Device, GenericDevice } from "../devices";
import { listen } from "@tauri-apps/api/event";

const [sensors, setSensors] = createSignal(
  [
    {
      name: 'TC1',
      group: 'Fuel',
      board_id: 1,
      channel_type: 'TC',
      channel: 0,
      unit: 'K',
      value: 200,
    } as Device,
    {
      name: 'TC2',
      group: 'Oxygen',
      board_id: 1,
      channel_type: 'TC',
      channel: 3,
      unit: 'K',
      value: 236,
    } as Device,
    {
      name: 'PT1',
      group: 'Fuel',
      board_id: 2,
      channel_type: 'PT',
      channel: 3,
      unit: 'psi',
      value: 80,
    } as Device,
    {
      name: 'PT2',
      group: 'Pressurant',
      board_id: 2,
      channel_type: 'PT',
      channel: 5,
      unit: 'psi',
      value: 100,
    } as Device,
    {
      name: 'TC3',
      group: 'Fuel',
      board_id: 1,
      channel_type: 'TC',
      channel: 0,
      unit: 'K',
      value: 200,
    } as Device,
    {
      name: 'TC4',
      group: 'Fuel',
      board_id: 1,
      channel_type: 'TC',
      channel: 0,
      unit: 'K',
      value: 200,
    } as Device,
    {
      name: 'TC5',
      group: 'Oxygen',
      board_id: 1,
      channel_type: 'TC',
      channel: 0,
      unit: 'K',
      value: 200,
    } as Device,
  ]
);
export const [view, setView] = createSignal('sorted');

listen('device_update', (event) => {
  var devices = event.payload as Array<GenericDevice>
  devices.forEach((device) => {
    var index = sensors().findIndex(item => (item.board_id === device.board_id && item.channel === device.channel));
    var new_sensors = JSON.parse(JSON.stringify(sensors()));
    new_sensors[index].value = device.floatValue;
    setSensors(new_sensors);
  });
})

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