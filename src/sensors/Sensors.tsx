import { createSignal } from "solid-js";
import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import SensorSectionView from "./SensorSectionView";
import { Sensor } from "../devices";

const [sensors, setSensors] = createSignal();
export const [view, setView] = createSignal('sorted');

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
        <SensorSectionView sensors={[
          {
            name: 'TC1',
            group: 'Fuel',
            board_id: 1,
            channel_id: 'TC',
            node_id: 0,
            unit: 'K',
            value: 200,
          } as Sensor,
          {
            name: 'TC2',
            group: 'Oxygen',
            board_id: 1,
            channel_id: 'TC',
            node_id: 3,
            unit: 'K',
            value: 236,
          } as Sensor,
          {
            name: 'PT1',
            group: 'Fuel',
            board_id: 2,
            channel_id: 'PT',
            node_id: 3,
            unit: 'psi',
            value: 80,
          } as Sensor,
          {
            name: 'PT2',
            group: 'Pressurant',
            board_id: 2,
            channel_id: 'PT',
            node_id: 5,
            unit: 'psi',
            value: 100,
          } as Sensor,
          {
            name: 'TC3',
            group: 'Fuel',
            board_id: 1,
            channel_id: 'TC',
            node_id: 0,
            unit: 'K',
            value: 200,
          } as Sensor,
          {
            name: 'TC4',
            group: 'Fuel',
            board_id: 1,
            channel_id: 'TC',
            node_id: 0,
            unit: 'K',
            value: 200,
          } as Sensor,
          {
            name: 'TC5',
            group: 'Oxygen',
            board_id: 1,
            channel_id: 'TC',
            node_id: 0,
            unit: 'K',
            value: 200,
          } as Sensor,
        ]}/>
      </div>
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default Sensors;