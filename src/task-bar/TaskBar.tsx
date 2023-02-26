import "../App.css";
import { SimpleTitleBar } from "../window-components/TitleBar";
import MenuBar from "./MenuBar";
import Body from "./Body";
import Footer from "../window-components/Footer";
import { invoke } from '@tauri-apps/api/tauri'
import { emit, listen } from "@tauri-apps/api/event";
import { activity, Agent, Alert, isConnected, setActivity, setAlerts, setIsConnected, State } from "../comm";
import { appWindow } from '@tauri-apps/api/window';

// listener to update state for the taskbar window
listen('state', (event) => {
  setIsConnected((event.payload as State).isConnected);
  //setActivity((event.payload as State).activity);
  setAlerts((event.payload as State).alerts);
  if (isConnected()) {
    document.getElementById('status')!.style.color = '#1DB55A';
  } else {
    document.getElementById('status')!.style.color = '#C53434';
  }
});

// listen for updates on the activity
listen('activity', (event) => {
  setActivity(event.payload as number);
});

listen('requestActivity', (event) => {
  emit('updateActivity', activity());
});

function Taskbar() {
  // initialize state upon loading
  invoke('initialize_state', {window: appWindow});
  return <div class="taskbar">
    <div>
      <SimpleTitleBar/>
    </div>
    <div>
      <MenuBar/>
    </div>
    <Body/>
    <div>
      <Footer/>
    </div>
  </div>
}

export default Taskbar;
