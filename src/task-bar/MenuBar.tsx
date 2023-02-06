import { Component, createSignal} from 'solid-js';
import { Select } from "@thisbeyond/solid-select";
import logo from '../assets/yjsplogo.png';
import {appWindow, WebviewWindow } from '@tauri-apps/api/window';
import {Icon} from '@iconify-icon/solid';

const activity = "120";
const [dropdownOpen, setDropdownOpen] = createSignal(false);

function createConfigWindow() {
  const webview = new WebviewWindow('configuration', {
    url: 'index.html',
    fullscreen: false,
    title: 'System',
    decorations: false,
  })
}


var dropdownContent = document.getElementById("dropdowncontent")!;
var button = document.getElementById("viewbutton")!;

function openDropdown() {
  var button = document.getElementById("viewbutton")!;
  var dropdownContent = document.getElementById("dropdowncontent")!;
  dropdownContent.style.display = "flex"
  button.style.backgroundColor = "#3C3F41";
  setDropdownOpen(true);
}

function closeDropdown(evt:MouseEvent) {
  var button = document.getElementById("viewbutton")!;
  var dropdownContent = document.getElementById("dropdowncontent")!;
  dropdownContent.style.display = "none"
  if (evt.target != button){
    button.style.backgroundColor = "#333333";
    setDropdownOpen(false);
  }
}

document.addEventListener("click", (evt) => closeDropdown(evt));

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
  <div class="menu-item" onClick={() => {console.log("system"); createConfigWindow();}}>
    System
  </div>
  <div class="vertical-line"></div>
  <div id="viewbutton" class="menu-item" onClick={() => {console.log("views"); openDropdown()}}>
      <div>
        Views
      </div>
      <div class="dropdown">
        <div id="dropdowncontent" class="dropdown-content">
          <div class="dropdown-item">
            Sensors
          </div>
          <div class="dropdown-item">
            Valves
          </div>
          <div class="dropdown-item">
            AVI
          </div>
          <div class="dropdown-item">
            Feedsystem
          </div>
          <div class="dropdown-item">
            Plotter
          </div>
          <div class="dropdown-item">
            Logs
          </div>
        </div>
      </div>
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