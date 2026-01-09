import {Component, createSignal} from 'solid-js';
import {appWindow, WebviewWindow } from '@tauri-apps/api/window';
import logo from '../assets/yjsplogo.png';
import { exit } from '@tauri-apps/api/process';
import MinimizeIcon from "../assets/window-minimize.svg";
import MaximizeIcon from "../assets/window-maximize.svg";
import CloseIcon from "../assets/window-close.svg";


function maximize() {
    if (maximized()) {
      appWindow.unmaximize();
      setMaximized(false)
    } else {
      appWindow.maximize();
      setMaximized(true)
    }
}

function minimize() {
  appWindow.minimize();
}

function close() {
  appWindow.close();
}

async function close_app() {
  await exit();
}

function createConfigWindow() {
  const webview = new WebviewWindow('configuration', {
    url: 'index.html',
  })
}

const [maximized, setMaximized] = createSignal(false);

const SimpleTitleBar: Component = (props) => {

  return <div data-tauri-drag-region class="titlebar">
    <div class="titlebar-buttons">
      <div class="titlebar-button" onClick={minimize}>
        <img src={MinimizeIcon} class="titlebar-icon" alt="minimize" />
      </div>
      <div class="titlebar-button" onClick={maximize}>
        <img src={MaximizeIcon} class="titlebar-icon" alt="maximize" />
      </div>
      <div class="titlebar-button" onclick={close}>
        <img src={CloseIcon} class="titlebar-icon" alt="close" />
      </div>
    </div>
  </div>
}



const GeneralTitleBar: Component<{name: string}> = (props) => {
  return <div data-tauri-drag-region class="general-titlebar">
    <div class="logo" style="margin-top: 10px; flex: 0 0 70px">
      <img
        src={logo}
        width="70"
        height="49" 
      />
    </div>
    <div data-tauri-drag-region class="page-name">
      {(props.name).toUpperCase()}
    </div>
    <div class="titlebar-buttons">
      <div class="titlebar-button" onClick={minimize}>
        <img src={MinimizeIcon} class="titlebar-icon" alt="minimize" />
      </div>
      <div class="titlebar-button" onClick={maximize}>
        <img src={MaximizeIcon} class="titlebar-icon" alt="maximize" />
      </div>
      <div class="titlebar-button" onclick={close}>
        <img src={CloseIcon} class="titlebar-icon" alt="close" />
      </div>
    </div>
</div>
}

export {SimpleTitleBar, GeneralTitleBar};