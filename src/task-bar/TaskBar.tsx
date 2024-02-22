import "../App.css";
import { SimpleTitleBar } from "../general-components/TitleBar";
import MenuBar from "./MenuBar";
import Body from "./Body";
import Footer from "../general-components/Footer";
import { invoke } from '@tauri-apps/api/tauri'
import { emit, listen } from "@tauri-apps/api/event";
import { activity, Agent, Alert, isConnected, openStream, setActivity, setAlerts, setIsConnected, State } from "../comm";
import { appWindow } from '@tauri-apps/api/window';
import { DISCONNECT_ACTIVITY_THRESH } from "../appdata";

// listener to update state for the taskbar window
listen('state', (event) => {
  setAlerts((event.payload as State).alerts);
  if (isConnected() != (event.payload as State).isConnected) {
    setIsConnected((event.payload as State).isConnected);
    if (isConnected()) {
      document.getElementById('status')!.style.color = '#1DB55A';
    } else {
      document.getElementById('status')!.style.color = '#C53434';
      // invoke('add_alert', {window: appWindow, 
      //   value: {time: (new Date()).toLocaleTimeString(), agent: Agent.GUI.toString(), message: "Disconnected from Servo"} as Alert 
      // })
    }
  }
});

// listen for updates on the activity
listen('activity', (event) => {
  setActivity(event.payload as number);
});

listen('requestActivity', (event) => {
  emit('updateActivity', activity());
});

listen('open_stream', (event) => {
  openStream(event.payload as string);
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
