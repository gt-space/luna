import { Device } from "../devices";
import SortableSensorView from "./SortableSensorView";
import GroupedSensorView from "./GroupedSensorView";
import { Component } from "solid-js";
import { view } from "./Sensors";

function displayView(view: string, sensors: Device[]) {
  switch(view) {
    case 'sorted': return <SortableSensorView sensors={sensors}/>
    case 'grouped': {
      let fuel: Device[] = [];
      let oxygen: Device[] = [];
      let pressurant: Device[] = [];
      for (var sensor in sensors) {
        switch(sensors[sensor].group.toLowerCase()) {
          case 'fuel': fuel.push(sensors[sensor]); break;
          case 'oxygen': oxygen.push(sensors[sensor]); break;
          case 'pressurant': pressurant.push(sensors[sensor]); break;
        }
      }
      return <div>
        <GroupedSensorView type="Fuel" color="#F14D1E" sensors={fuel}/>
        <div style="height: 20px"></div>
        <GroupedSensorView type="Oxygen" color="#4FB8FE" sensors={oxygen}/>
        <div style="height: 20px"></div>
        <GroupedSensorView type="Pressurant" color="#E0AA2E" sensors={pressurant}/>
      </div>
    }
  } 
}

const SensorSectionView: Component<{sensors: Device[]}> = (props) => {
  return <div style="display: flex; flex-direction: column; flex:1">{displayView(view(), props.sensors)}</div>
} 

export default SensorSectionView;