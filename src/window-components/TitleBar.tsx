import {Component, createSignal} from 'solid-js';
import {appWindow, WebviewWindow } from '@tauri-apps/api/window';
import {Icon} from '@iconify-icon/solid';


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
      <Icon icon="mdi:window-close" onClick={() => close()}/>
    </div>
</div>
}

const GeneralTitleBar: Component = (props) => {

  return <div data-tauri-drag-region class="titlebar">
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
}

export {SimpleTitleBar, GeneralTitleBar};