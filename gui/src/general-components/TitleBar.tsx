import {Component, createSignal} from 'solid-js';
import {appWindow, WebviewWindow } from '@tauri-apps/api/window';
import {Icon} from '@iconify-icon/solid';
import logo from '../assets/yjsplogo.png';
import { exit } from '@tauri-apps/api/process';


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
    <div class="titlebar-button">
      <Icon icon="mdi:window-minimize" onClick={() => minimize()}/>
    </div>
    <div class="titlebar-button">
      <Icon icon="mdi:window-maximize" onClick={() => maximize()}/>
    </div>
    <div class="titlebar-button">
      <Icon icon="mdi:window-close" onClick={() => close_app()}/>
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
      <div class="titlebar-button">
        <Icon icon="mdi:window-minimize" onClick={() => minimize()}/>
      </div>
      <div class="titlebar-button">
        <Icon icon="mdi:window-maximize" onClick={() => maximize()}/>
      </div>
      <div class="titlebar-button">
        <Icon icon="mdi:window-close" onClick={() => close()}/>
      </div>
    </div>
</div>
}

export {SimpleTitleBar, GeneralTitleBar};