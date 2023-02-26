import { Component, createSignal, For} from "solid-js";
import Scrollbars from 'solid-custom-scrollbars'
import { Alert, alerts } from "../comm";

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
          <For each={alerts() as Alert[]}>{(alert, i) =>
            <div>
              {`[${alert.time}] [${alert.agent}]: ${alert.message}`}
              {() => {if (i() == 0) return <div style={"height: 5px"}></div>}}            
            </div>
          }</For>
        </Scrollbars> 
      </div>
    </div>
  </div>
}

export default Body;