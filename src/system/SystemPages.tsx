import { Component, createSignal, Show } from "solid-js";
import { invoke } from '@tauri-apps/api/tauri'
import { setServerIp, connect, isConnected, setIsConnected, setActivity, serverIp, activity, selfIp, selfPort, sessionId, forwardingId } from "../comm";
import { turnOnLED, turnOffLED } from "../commands";
import { emit, listen } from '@tauri-apps/api/event'
import { appWindow } from "@tauri-apps/api/window";
import { DISCONNECT_ACTIVITY_THRESH } from "../appdata";

// states of error message and connect button
const [connectDisplay, setConnectDisplay] = createSignal("Connect");
const [connectionMessage, setConnectionMessage] = createSignal('');
const [showSessionId, setShowSessionId] = createSignal(false);
const [showForwardingId, setShowForwardingId] = createSignal(false);

// function to connect to the server + input validation
async function connectToServer() {
  setConnectDisplay("Connecting...");
  setConnectionMessage('');

  // getting the ip, username, and password from the relevant textfields
  var ip = (document.getElementsByName('server-ip')[0] as HTMLInputElement).value.trim();
  var username = (document.getElementsByName('username')[0] as HTMLInputElement).value.trim();
  var password = (document.getElementsByName('password')[0] as HTMLInputElement).value;
  var result = '';

  // presence check on username and password
  if (username != '' && password != '') {
    result = await connect(ip, username, password);
  } else {
    result = 'Please enter a username and password';
  }

  setConnectionMessage(result);
  setConnectDisplay("Connect");
}

// get the activity from the taskbar page
emit('requestActivity');
listen('updateActivity', (event) => {
  setActivity(event.payload as number);
  if (activity() < DISCONNECT_ACTIVITY_THRESH) {
    setIsConnected(true);
  }
});

// function to close the sessionId info
function closeSessionId(evt:MouseEvent) {
  try {
    var popup = document.getElementById("session-id")!;
    if ((evt.target as HTMLElement).id != popup.id && (evt.target as HTMLElement)!.className != 'id-display'){
      setShowSessionId(false);
    }
  } catch (e) {

  }
}
// function to close the forwardingId info
function closeForwardingId(evt:MouseEvent) {
  try{
    var popup = document.getElementById("forwarding-id")!;
    if ((evt.target as HTMLElement).id != popup.id && (evt.target as HTMLElement)!.className != 'id-display'){
      setShowForwardingId(false);
    }
  } catch (e) {
    
  }
}

document.addEventListener("click", (evt) => closeSessionId(evt));
document.addEventListener("click", (evt) => closeForwardingId(evt));

const Connect: Component = (props) => {
  return <div style="height: 100%; display: flex; flex-direction: column">
    <div style="text-align: center; font-size: 14px">CONNECT</div>
    <div class="system-connect-page">
      <div class="system-connect-section">
        <input class="connect-textfield"
          type="text"
          name="server-ip"
          placeholder="Server IP"
        />
        <input class="connect-textfield"
          type="text"
          name="username"
          placeholder="Username"
        />
        <input class="connect-textfield"
          type="password"
          name="password"
          placeholder="Password"
        />
        <div id="connect-message" style="font-size: 12px">
          {connectionMessage()}
        </div>
        <button class="connect-button" onClick={() => connectToServer()}>
          {connectDisplay()}
        </button>
        <div style="height: 20px"></div>
        <button style="padding: 5px" onClick={() => turnOnLED()}>
          LED test button (on)
        </button>
        <div style="height: 10px"></div>
        <button style="padding: 5px" onClick={() => turnOffLED()}>
          LED test button (off)
        </button>
      </div>
      <div class="system-connect-section">
        <div style="display: grid; grid-template-columns: 1fr 1fr">
          <div style="display: flex; flex-direction: column; margin-right: 20px">
            <div style="text-align: right">Activity:</div>
            <div style="text-align: right">Status:</div>
            <div style="text-align: right">IP:</div>
            <div style="text-align: right">Port:</div>
            <div style="text-align: right">Server IP:</div>
            <div style="text-align: right">Session ID:</div>
            <div style="text-align: right">Forwarding ID:</div>
          </div>
          <div style="display: flex; flex-direction: column; margin-left: 0px">
            <div style="text-align: center" id="activity">{activity()} ms</div>
            <div style="text-align: center" id="status">{isConnected()? "CONNECTED":"DISCONNECTED"}</div>
            <div style="text-align: center">{selfIp() as string}</div>
            <div style="text-align: center">{selfPort() as string}</div>
            <div style="text-align: center">{serverIp() as string}</div>
            <div id="session-id" style="text-align: center">{sessionId() == 'None'? sessionId() as string : 
              <Show 
                when={showSessionId()}
                fallback={<button id="session-id" class="connect-info-button" 
                onClick={() => {setShowSessionId(true); console.log(showSessionId())}}>Click to view</button>}
              >
                <div class='id-display'>{sessionId() as string}</div>
              </Show>}
            </div>
            <div style="text-align: center" id="forwarding-id">{forwardingId() == 'None'? forwardingId() as string : 
              <Show 
                when={showForwardingId()}
                fallback={<button id="forwarding-id" class="connect-info-button" 
                onClick={() => {setShowForwardingId(true); console.log(showForwardingId())}}>Click to view</button>}
              >
                <div class='id-display'>{forwardingId() as string}</div>
              </Show>}
            </div>
          </div>
        </div>
      </div>
    </div>
</div>
}

const Feedsystem: Component = (props) => {
  return <div style="height: 100%; display: flex; flex-direction: column">
    <div style="text-align: center; font-size: 14px">FEEDSYSTEM</div>
    <div class='select-feedsystem-body'>
      <div style={{'width': '200px','padding': '20px'}}> 
        <div style={{"margin-bottom": '10px'}}>Select feedsystem:</div>
        <div style={{'margin-left': '20px', 'display': 'flex', "flex-direction": 'column', 'align-items': 'flex-start'}}>
          <div style={{'display': 'flex', "flex-direction": 'row', "align-items": "center", 'padding-top': '5px'}}>
              <input class='radiobutton' style={{'margin': '10px'}} type="radio" name="feedsystem-select"></input>
              <div>
                Feedsystem 1
              </div>
          </div>
          <div style={{'display': 'flex', "flex-direction": 'row', "align-items": "center", 'padding-top': '5px'}}>
              <input class='radiobutton' style={{'margin': '10px'}} type="radio" name="feedsystem-select"></input>
              <div>
                Feedsystem 2
              </div>
          </div>
          <div style={{'display': 'flex', "flex-direction": 'row', "align-items": "center", 'padding-top': '5px'}}>
              <input class='radiobutton' style={{'margin': '10px'}} type="radio" name="feedsystem-select"></input>
              <div>
                Feedsystem 3
              </div>
          </div>
          <div style={{'display': 'flex', "flex-direction": 'row', "align-items": "center", 'padding-top': '5px'}}>
              <input class='radiobutton' style={{'margin': '10px'}} type="radio" name="feedsystem-select"></input>
              <div>
                Feedsystem 4
              </div>
          </div>
        </div>
      </div>
      <div style={{'flex': 2, 'padding-top': '20px'}}>
        <div style={{"margin-bottom": '10px'}}>Set configuration:</div>
        <div style={{'display': 'flex', "flex-direction": 'column', 'align-items': 'flex-start'}}>
          <div>
            <select
              class="feedsystem-config-dropdown"
              onChange={(e) => {
                console.log(e?.target.className);
              }}
            >
            <option class="seq-dropdown-item" value="seq1">Config 1</option>
            <option class="seq-dropdown-item" value="seq2">Config 2</option>
            <option class="seq-dropdown-item" value="seq3">Config 3</option>
            <option class="seq-dropdown-item" value="seq4">Config 4</option>
            <option class="seq-dropdown-item" value="seq5">Config 5</option>
            <option class="seq-dropdown-item" value="seq6">Config 6</option>
          </select>
          </div>
          <div>
            <select
              class="feedsystem-config-dropdown"
              onChange={(e) => {
                console.log(e?.target.className);
              }}
            >
            <option class="seq-dropdown-item" value="seq1">Config 1</option>
            <option class="seq-dropdown-item" value="seq2">Config 2</option>
            <option class="seq-dropdown-item" value="seq3">Config 3</option>
            <option class="seq-dropdown-item" value="seq4">Config 4</option>
            <option class="seq-dropdown-item" value="seq5">Config 5</option>
            <option class="seq-dropdown-item" value="seq6">Config 6</option>
          </select>
          </div>
          <div>
            <select
              class="feedsystem-config-dropdown"
              onChange={(e) => {
                console.log(e?.target.className);
              }}
            >
            <option class="seq-dropdown-item" value="seq1">Config 1</option>
            <option class="seq-dropdown-item" value="seq2">Config 2</option>
            <option class="seq-dropdown-item" value="seq3">Config 3</option>
            <option class="seq-dropdown-item" value="seq4">Config 4</option>
            <option class="seq-dropdown-item" value="seq5">Config 5</option>
            <option class="seq-dropdown-item" value="seq6">Config 6</option>
          </select>
          </div>
          <div>
            <select
              class="feedsystem-config-dropdown"
              onChange={(e) => {
                console.log(e?.target.className);
              }}
            >
            <option class="seq-dropdown-item" value="seq1">Config 1</option>
            <option class="seq-dropdown-item" value="seq2">Config 2</option>
            <option class="seq-dropdown-item" value="seq3">Config 3</option>
            <option class="seq-dropdown-item" value="seq4">Config 4</option>
            <option class="seq-dropdown-item" value="seq5">Config 5</option>
            <option class="seq-dropdown-item" value="seq6">Config 6</option>
          </select>
          </div>
          
        </div>
      </div>
    </div>
    <div class="system-feedsystem-page">
      
    </div>
</div>
}

function displayAddConfig() {
  const addConfigSection = document.querySelector(".add-config-connect-section");
  console.log("I'M DISPLAYING")
  addConfigSection.style.display = "flex";
}

function removeAddConfig() {
  const addConfigSection = document.querySelector(".add-config-connect-section");

  addConfigSection.style.display = "none";
}

function removeEditSection() {
  const editConfigSection = document.querySelector(".edit-section");
  console.log("I'M REMOVING")
  editConfigSection.style.display = "none";
}

function addEditSection() {
  const editConfigSection = document.querySelector(".edit-section");

  editConfigSection.style.display = "flex";
}

function displayEditBtns() {
  const editBtns = (document.querySelectorAll(".existing-config-edit-btns"));

  console.log(editBtns);

  editBtns.forEach((btn) => {
    console.log(btn);
    btn.style.display = "block";
  });
}

function addConfig() {
  console.log("I'M ADDING CONFIG");
  const node = document.querySelector(".editing-data");
  const child = document.querySelector(".edit-config-configurations");
  const childClone = child.cloneNode(true);

  // node.append("some text");
  node.append(childClone);

  // node.appendChild(child);

  // console.log("NODE " + node);
  // console.log("CHILD " + child);
}


const Config: Component = (props) => {
  return <div style="height: 100%">
    <div style="text-align: center; font-size: 14px">CONFIGURATION</div>
    <div class="system-config-page">
      <div class="system-connect-section">
        <div style="text-align: center; font-size: 14px; font-family: 'Rubik'">Existing Configurations</div>
        <div class="horizontal-line"></div>
        <div class="existing-configs-sections">
          {/* <div class="name-section">
            <div>Name</div>
            <div class="name-section-configs">Config 1</div>
          </div>
          <div class="date-section">
            <div>Date Created</div>
            <div class="date-section-configs">10/8/23</div>
          </div> */}
          <div class="row">
            <div>Name</div>
            <div>Date</div>
            {/* <div>Edit Button</div> */}
          </div>
          {/* <div id="row1" class="row" onClick={() => displayEditBtns()}> */}
          {/* <div id="row1" class="row" onClick={function(event){ displayEditBtns(); removeAddConfig(); addEditSection()}}> */}
          <div id="row1" class="row" onClick={function(event){ removeAddConfig(); addEditSection()}}>
          {/* <div id="row1" class="row"> */}
            <div class="row-subheadings">Name</div>
            <div class="row-subheadings">Date</div>
            <button class="existing-config-edit-btns">Edit</button>
          </div>
          <div class="row" onClick={function(event){ removeAddConfig(); addEditSection()}}>
            <div class="row-subheadings">Name</div>
            <div class="row-subheadings">Date</div>
            <button class="existing-config-edit-btns">Edit</button>
          </div>
        </div>
      </div>
      <div class="system-connect-section add-config-connect-section">
        <div class="add-config-section">
          <div class="add-config-setup">
            <p>Add new config:</p>
            <input class="add-config-input" type="text" placeholder="Name"/>
          </div>
          <div class="add-config-btns">
            <button class="add-config-cancel-btn" onClick={function(event){ removeEditSection(); displayAddConfig()}}>Cancel</button>
            <button class="add-config-save-btn" onClick={function(event){ removeEditSection(); displayAddConfig()}}>Save</button>
          </div>
        </div>
        <div class="horizontal-line"></div>
        <div class="add-config-configurations">
          <input type="text" placeholder="Name" class="add-config-styling"/>
          <input type="text" name="" id="" placeholder="Board ID" class="add-config-styling"/>
          <select name="" id="" class="add-config-styling">
            <option class="seq-dropdown-item">Channel Type</option>
            <option class="seq-dropdown-item">GPIO</option>
            <option class="seq-dropdown-item">LED</option>
            <option class="seq-dropdown-item">RAIL 3V3</option>
            <option class="seq-dropdown-item">RAIL 5V</option>
            <option class="seq-dropdown-item">RAIL 5V5</option>
            <option class="seq-dropdown-item">RAIL 24V</option>
            <option class="seq-dropdown-item">CURRENT LOOP</option>
            <option class="seq-dropdown-item">DIFFERENTIAL SIGNAL</option>
            <option class="seq-dropdown-item">TC</option>
            <option class="seq-dropdown-item">RTD</option>
            <option class="seq-dropdown-item">VALVE</option>
            <option class="seq-dropdown-item">VALVE CURRENT</option>
            <option class="seq-dropdown-item">VALVE VOLTAGE</option>
          </select>
          <input type="text" name="" id="" placeholder="Channel" class="add-config-styling"/>
          <select name="" id="" class="add-config-styling">
            <option class="seq-dropdown-item">Computer</option>
            <option class="seq-dropdown-item">Flight</option>
            <option class="seq-dropdown-item">Ground</option>
          </select>
        </div>
      </div>
      <div class="edit-section">
        <div class="system-connect-section editing-configs-section">
            <div class="existing-data">
              <h4>Config Name: </h4>
              <h4>Name: </h4>
              <h4>Board ID: </h4>
              <h4>Channel Type: </h4>
              <h4>Channel: </h4>
              <h4>Computer: </h4>
            </div>
        </div>

        <div class="editing-vertical-line"></div>

        <div class="system-connect-section">

          <div class="editing-data">
            <div class="add-config-section">
              <div class="add-config-setup">
                <p>Edit new config:</p>
                <input class="add-config-input" type="text" placeholder="Name"/>
              </div>
              <div class="add-config-btns">
                <button class="add-config-add-btn" onClick={() => addConfig()}>Add Config</button>
                <button class="add-config-cancel-btn" onClick={function(event){ removeEditSection(); displayAddConfig()}}>Cancel</button>
                <button class="add-config-save-btn" onClick={function(event){ removeEditSection(); displayAddConfig()}}>Save</button>
              </div>
            </div>
            <div class="horizontal-line"></div>
            <div class="add-config-configurations edit-config-configurations">
              <input type="text" placeholder="Name" class="add-config-styling"/>
              <input type="text" name="" id="" placeholder="Board ID" class="add-config-styling"/>
              <select name="" id="" class="add-config-styling">
                <option class="seq-dropdown-item">Channel Type</option>
                <option class="seq-dropdown-item">GPIO</option>
                <option class="seq-dropdown-item">LED</option>
                <option class="seq-dropdown-item">RAIL 3V3</option>
                <option class="seq-dropdown-item">RAIL 5V</option>
                <option class="seq-dropdown-item">RAIL 5V5</option>
                <option class="seq-dropdown-item">RAIL 24V</option>
                <option class="seq-dropdown-item">CURRENT LOOP</option>
                <option class="seq-dropdown-item">DIFFERENTIAL SIGNAL</option>
                <option class="seq-dropdown-item">TC</option>
                <option class="seq-dropdown-item">RTD</option>
                <option class="seq-dropdown-item">VALVE</option>
                <option class="seq-dropdown-item">VALVE CURRENT</option>
                <option class="seq-dropdown-item">VALVE VOLTAGE</option>
              </select>
              <input type="text" name="" id="" placeholder="Channel" class="add-config-styling"/>
              <select name="" id="" class="add-config-styling">
                <option class="seq-dropdown-item">Computer</option>
                <option class="seq-dropdown-item">Flight</option>
                <option class="seq-dropdown-item">Ground</option>
              </select>
            </div>
          </div>
        </div>
      </div>
    </div>
</div>
}

const Sequences: Component = (props) => {
  return <div style="height: 100%">
    <div style="text-align: center; font-size: 14px">SEQUENCES</div>
    <div class="system-sequences-page">
      
    </div>
</div>
}

export {Connect, Feedsystem, Config, Sequences};