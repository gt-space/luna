import { listen } from "@tauri-apps/api/event";
import { isConnected, sessionId, setActivity, setForwardingId, setIsConnected, setSelfIp, setSelfPort, setServerIp, setSessionId, State } from "../comm";
import Footer from "../window-components/Footer";
import { GeneralTitleBar } from "../window-components/TitleBar";
import SideNavBar from "./SideNavBar";
import SystemMainView from "./SystemMainView";

// listener to update state for the system window
listen('state', (event) => {
  setIsConnected((event.payload as State).isConnected);
  //setActivity((event.payload as State).activity);
  setSelfIp((event.payload as State).selfIp);
  setSelfPort((event.payload as State).selfPort);
  setServerIp((event.payload as State).serverIp);
  setSessionId((event.payload as State).sessionId);
  setForwardingId((event.payload as State).forwardingId);
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

function System() {
  return <div class="system">
    <div style="height: 60px">
      <GeneralTitleBar name="System"/>
    </div>
    <div class="system-body">
      <SideNavBar/>
      <div style="display: grid; grid-template-rows: 20px 1fr 10px; height: 100%">
        <div></div>
        <div class="vertical-line-2"></div>
        <div></div>
      </div>
      <SystemMainView/>
    </div>
    <div>
      <Footer/>
    </div>
</div>
}

export default System;
