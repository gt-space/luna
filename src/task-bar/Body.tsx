import { Component, createSignal, For} from "solid-js";
import Scrollbars from 'solid-custom-scrollbars'

const [alerts, setAlerts] = createSignal([
  {"time" : "[Time 1] ", "agent": "[GUI]: ", "message": "Alert 1\n"}, 
  {"time" : "[Time 2] ", "agent": "[servo]: ","message": "Alert 2"},
  {"time" : "[Time 2] ", "agent": "[servo]: ","message": "Alert 2"}, 
  {"time" : "[Time 2] ", "agent": "[servo]: ","message": "Alert 2"}, 
  {"time" : "[Time 2] ", "agent": "[servo]: ","message": "Alert 2"}, 
  {"time" : "[Time 2] ", "agent": "[servo]: ","message": "Alert 2"}, 
  {"time" : "[Time 2] ", "agent": "[servo]: ","message": "Alert 2"}, 
  {"time" : "[Time 2] ", "agent": "[servo]: ","message": "Alert 2"}, 
  {"time" : "[Time 2] ", "agent": "[servo]: ","message": "Alert 2"}, 
  {"time" : "[Time 2] ", "agent": "[servo]: ","message": "Alert 2"},  
  {"time" : "[Time 3] ", "agent": "[FC]: ","message": "Alert 3"}]);

const Body: Component = (props) => {
  return <div class="taskbar-body">
    <div class="taskbar-body-item">
      System Overview
    </div>
    <div class="taskbar-body-item">
      Alerts
    </div>
    <div class="taskbar-body-item">
      <div class="scrollable-container">
      </div>
    </div>
    <div class="taskbar-body-item">
      <div class="scrollable-container">
        <Scrollbars>
          <For each={alerts()}>{(alert, i) =>
            <div>
              {alert.time + alert.agent + alert.message}
              {() => {if (i() == 0) return <div style={"height: 5px"}></div>}}            
            </div>
          }</For>
        </Scrollbars> 
      </div>
    </div>
  </div>
}

export default Body;