import { Component, createSignal} from 'solid-js';
import logo from '../assets/yjsplogo.png';
import {appWindow, WebviewWindow } from '@tauri-apps/api/window';
import {Icon} from '@iconify-icon/solid';

const activity = "120";

const MenuBar: Component = (props) => {
  return <div class="menu-bar">
  <div class="logo">
    <img
      src={logo}
      width="100"
      height="70" 
    />
  </div>
  <div class="vertical-line"></div>
  <div class="grid-item" onClick={() => console.log("system")}>
    System
  </div>
  <div class="vertical-line"></div>
  <div class="grid-item" onClick={() => console.log("views")}>
    Views
  </div>
  <div class="vertical-line"></div>
  <div class="activity-status">
    <div>
      <div class="activity-status-labels">
        Activity:   
      </div>
      <div class="activity-status-labels">
        Status:   
      </div>
    </div>
    <div>
      <div class="activity" id="activity">
        {activity} ms
      </div>
      <div class="status" id="status">
        DISCONNECTED
      </div>
    </div>
  </div>
</div>
}

export default MenuBar;