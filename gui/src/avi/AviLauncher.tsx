import { For, createEffect, createSignal } from "solid-js";
import Footer from "../general-components/Footer";
import { GeneralTitleBar } from "../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { Config, Sequence, State, runSequence, serverIp, StreamState } from "../comm";
import { WebviewWindow } from '@tauri-apps/api/window';
import { emit } from '@tauri-apps/api/event';

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [activeBoards, setActiveBoards] = createSignal<string[]>([]);

// listen('state', (event) => {
//   console.log(event.windowLabel);
//   setConfigurations((event.payload as State).configs);
//   setActiveConfig((event.payload as State).activeConfig);
// });

listen('state', (event) => {
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
  const mappings = (configurations() as Config[]).filter((conf) => {return conf.id == activeConfig() as string})[0].mappings;
  const board_ids = mappings.map(mappings => mappings.board_id);
  const activeBoardsUnsorted = board_ids.filter(function(item, pos){
    return board_ids.indexOf(item)== pos; 
  });
  const activeBoards = activeBoardsUnsorted.sort(function(a, b) { return parseInt(a.substring(5)) - parseInt(b.substring(5)); });
  for (var i = 0; i < activeBoards.length; i++) {
    // activeBoards[i] = activeBoards[i].replace(/-/g, ' ');
    activeBoards[i] = activeBoards[i].toUpperCase();
  }
  setActiveBoards(activeBoards);
});

invoke('initialize_state', {window: appWindow});

async function createSAMWindow(board_name: string) {
  console.log(board_name);
  const webview = new WebviewWindow(board_name, {
    url: 'sam.html',
    fullscreen: false,
    title: board_name,
    decorations: false,
    height: 400,
    width: 1400,
  })
}

async function createBMSWindow() {
  const webview = new WebviewWindow('BMS', {
    url: 'bms.html',
    fullscreen: false,
    title: 'BMS',
    decorations: false,
    height: 700,
    width: 1400,
  })
}

async function createAHRSWindow() {
  const webview = new WebviewWindow('AHRS', {
    url: 'ahrs.html',
    fullscreen: false,
    title: 'AHRS',
    decorations: false,
    height: 700,
    width: 1000,
  })
}

function AVILauncher() {
    return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="AVI"/>
    </div>
    <div class="avilauncher-view">
        {activeBoards().map((boardName, i) => (
          <div style={{ width: "100%", display: "flex", "justify-content": "center" }}>
            <button class="sam-button" onClick={() => createSAMWindow(boardName)}>
              {boardName}
            </button>
          </div>
        ))}
      {/* <For each={(activeBoards() as Array<string>)}>{(boardName, i) => 
        <div style={{width: "100%", display: "flex", "justify-content": "center"}}>
          <button class="sam-button" onClick={() => createSAMWindow(boardName)}>{boardName}</button>
        </div>
      }</For> */}
      <div style={{width: "100%", display: "flex", "justify-content": "center"} }>
        <button class="sam-button" onClick={() => createBMSWindow()}> BMS </button></div>
      <div style={{width: "100%", display: "flex", "justify-content": "center"} }>
        <button class="sam-button" onClick={() => createAHRSWindow()}> AHRS </button></div>
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default AVILauncher;