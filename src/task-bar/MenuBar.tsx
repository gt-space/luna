import { Component } from 'solid-js';
import logo from '../assets/yjsplogo.png';
import { WebviewWindow } from '@tauri-apps/api/window';
import { isConnected, activity } from '../comm';

// function to create and open the system window
function createSystemWindow() {
  const webview = new WebviewWindow('configuration', {
    url: 'system.html',
    fullscreen: false,
    title: 'System',
    decorations: false,
  })
}

// function to open the dropdown for views
function openDropdown() {
  var button = document.getElementById("viewbutton")!;
  var dropdownContent = document.getElementById("dropdowncontent")!;
  dropdownContent.style.display = "flex"
  button.style.backgroundColor = "#3C3F41";
}

// function to close the dropdown for views
function closeDropdown(evt:MouseEvent) {
  var button = document.getElementById("viewbutton")!;
  var dropdownContent = document.getElementById("dropdowncontent")!;
  dropdownContent.style.display = "none"
  if (evt.target != button){
    button.style.backgroundColor = "#333333";
  }
}

// a listener to close the dropdown when a user clicks away from it
document.addEventListener("click", (evt) => closeDropdown(evt));

const MenuBar: Component = (props) => {
  return <div class="menu-bar">
  <div data-tauri-drag-region class="logo">
    <img style="user-select: none"
      src={logo}
      width="100"
      height="70" 
    />
  </div>
  <div class="vertical-line"></div>
  <div class="menu-item" onClick={() => {console.log("system"); createSystemWindow();}}>
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
        {activity() as number} ms
      </div>
      <div class="status" id="status">
        {isConnected()? 'CONNECTED':'DISCONNECTED'}
      </div>
    </div>
  </div>
</div>
}

export default MenuBar;