import { Component, createSignal, For, Show } from "solid-js";
import { invoke } from '@tauri-apps/api/tauri'
import { setServerIp, connect, isConnected, setIsConnected, setActivity, serverIp, activity, selfIp, selfPort, sessionId, forwardingId, State, Config, sendActiveConfig, setSessionId, setForwardingId, setSelfIp, setSelfPort } from "../comm";
import { turnOnLED, turnOffLED } from "../commands";
import { emit, listen } from '@tauri-apps/api/event'
import { appWindow } from "@tauri-apps/api/window";
import { DISCONNECT_ACTIVITY_THRESH } from "../appdata";

// states of error message and connect button
const [connectDisplay, setConnectDisplay] = createSignal("Connect");
const [connectionMessage, setConnectionMessage] = createSignal('');
const [showSessionId, setShowSessionId] = createSignal(false);
const [showForwardingId, setShowForwardingId] = createSignal(false);
const [feedsystem, setFeedsystem] = createSignal('Feedsystem_1');
const [activeConfig, setActiveConfig] = createSignal('Config_1');
const [configurations, setConfigurations] = createSignal();
//configurations()

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
  // if (username != '' && password != '') {
  //   result = await connect(ip, username, password);
  // } else {
  //   result = 'Please enter a username and password';
  // }

  result = await connect(ip, username, password);

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

invoke('initialize_state', {window: appWindow});
listen('state', (event) => {
  setServerIp((event.payload as State).serverIp);
  setIsConnected((event.payload as State).isConnected);
  setSessionId((event.payload as State).sessionId);
  setForwardingId((event.payload as State).forwardingId);
  setSelfIp((event.payload as State).selfIp);
  setSelfPort((event.payload as State).selfPort);
  setConfigurations((event.payload as State).configs);
  setFeedsystem((event.payload as State).feedsystem);
  setActiveConfig((event.payload as State).activeConfig);
  console.log(configurations());
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

async function setFeedsystemAndActiveConfig() {
  var feedsystem = (document.querySelector('input[name="feedsystem-select"]:checked')! as HTMLInputElement);
  console.log(feedsystem.value);
  var dropdown = (document.getElementById("feed-config-drop-1"))! as HTMLSelectElement;
  console.log(dropdown.value);
  await invoke('update_feedsystem', {window: appWindow, value: feedsystem.value});
  await invoke('update_active_config', {window: appWindow, value: dropdown.value});
  sendActiveConfig(serverIp() as string, dropdown.value);

}
//get state updates
// invoke('initialize_state', {window: appWindow});
// listen('state', (event) => {
//   console.log((event.payload as State).feedsystem);
//   setFeedsystem((event.payload as State).feedsystem);
//   setActiveConfig((event.payload as State).activeConfig);
  
// });

async function setFeedsystemData() {
  await new Promise(r => setTimeout(r, 100));
  var feedsystemToSet = document.querySelectorAll('input[value="'+(feedsystem() as string)+'"]')[0];
  var dropdown = (document.getElementById("feed-config-drop-1"))! as HTMLSelectElement;
  console.log(feedsystemToSet);
  (feedsystemToSet as HTMLInputElement)!.checked = true
  console.log(activeConfig());
  console.log(dropdown);
  dropdown.value = activeConfig();
}  

const Feedsystem: Component = (props) => {
  listen('state', (event) => {
    setFeedsystem((event.payload as State).feedsystem);
    setActiveConfig((event.payload as State).activeConfig);
  });
  setFeedsystemData();
  return <div style="height: 100%; display: flex; flex-direction: column">
    <div style="text-align: center; font-size: 14px">FEEDSYSTEM</div>
    <div class='select-feedsystem-body'>
      <div style={{'display': 'flex', 'flex-direction': 'row'}}>
      <div style={{'width': '200px','padding': '20px'}}> 
        <div style={{"margin-bottom": '10px'}}>Select feedsystem:</div>
        <div style={{'margin-left': '20px', 'display': 'flex', "flex-direction": 'column', 'align-items': 'flex-start'}}>
          <div style={{'display': 'flex', "flex-direction": 'row', "align-items": "center", 'padding-top': '5px'}}>
              <input class='radiobutton' style={{'margin': '10px'}} type="radio" name="feedsystem-select" value="Feedsystem_1" id="Feedystem-id-1" checked></input>
              <div>
                Feedsystem 1
              </div>
          </div>
          <div style={{'display': 'flex', "flex-direction": 'row', "align-items": "center", 'padding-top': '5px'}}>
              <input class='radiobutton' style={{'margin': '10px'}} type="radio" name="feedsystem-select" value="Feedsystem_2" id="Feedystem-id-2"></input>
              <div>
                Feedsystem 2
              </div>
          </div>
          <div style={{'display': 'flex', "flex-direction": 'row', "align-items": "center", 'padding-top': '5px'}}>
              <input class='radiobutton' style={{'margin': '10px'}} type="radio" name="feedsystem-select" value="Feedsystem_3" id="Feedystem-id-3"></input>
              <div>
                Feedsystem 3
              </div>
          </div>
          <div style={{'display': 'flex', "flex-direction": 'row', "align-items": "center", 'padding-top': '5px'}}>
              <input class='radiobutton' style={{'margin': '10px'}} type="radio" name="feedsystem-select" value="Feedsystem_4" id="Feedystem-id-4"></input>
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
              id="feed-config-drop-1"
              class="feedsystem-config-dropdown"
              onChange={(e) => {
                console.log(e?.target.className);
              }}
            >
            <For each={configurations() as Config[]}>{(config, i) =>
              <option class="conf-dropdown-item" value={config.id}>{config.id}</option>
            }</For>
            {/* <option class="seq-dropdown-item" value="seq1">Config 1</option>
            <option class="seq-dropdown-item" value="seq2">Config 2</option>
            <option class="seq-dropdown-item" value="seq3">Config 3</option>
            <option class="seq-dropdown-item" value="seq4">Config 4</option>
            <option class="seq-dropdown-item" value="seq5">Config 5</option>
            <option class="seq-dropdown-item" value="seq6">Config 6</option> */}
          </select>
          </div>          
        </div>
      </div>
      </div>
      <div style={{'margin-left': '10px', 'margin-top': '10px','padding-left': '170px'}}>
        <button class='submit-feedsystem-button' onClick={setFeedsystemAndActiveConfig}> Submit </button>
      </div>
    </div>
</div>
}

// document.addEventListener("onload", retrieveData());
// document.addEventListener("load", retrieveData());

// document.addEventListener("DOMContentLoaded", retrieveData());
// document.addEventListener("click", (evt) => closeSessionId(evt));

var displayingExistingData = false;

async function retrieveData() {
  await new Promise(r => setTimeout(r, 100));
  console.log("HELLO I AM RETRIEVEING DATA");
  console.log("CONFIGURATIONS AGAIN " + (configurations() as Config[]));

  //For every element in configurations(), which is an array, ...
  //Update config row with name (access dictionary/object with "id")
  // (configurations() as Config[]).forEach((config) => {
  //   const configName = config["id"];
  //   const existingConfigsNode = document.querySelector(".existing-configs-sections");
    
  // });

  console.log("LENGHT " + (configurations() as Config[]).length);

  for (let i = 0; i < (configurations() as Config[]).length; i++) {
    // const configName = configurations()[i]["id"];
    const configName = (configurations() as Config[])[i].id;
    console.log("NAME " + configName);
    const existingConfigsNode = document.querySelector(".existing-configs-sections");

    if (i == 0) {
      const parentDiv = document.querySelector("#row0");
      const div = document.querySelector("#row-name-0");
      (div! as HTMLElement).innerHTML = configName;

      (parentDiv! as HTMLElement).addEventListener("click", (event) => {
        addExistingDataSection(event);
      });

    } else {
      console.log("INSIDE HERE");
      
      console.log("EXISTINCOFNIGNODE " + existingConfigsNode);
      var lastChild = (existingConfigsNode! as HTMLElement).lastChild;

      //ADDING CLONE CONFIG
      const childClone = (lastChild! as HTMLElement).cloneNode(true);
      (existingConfigsNode! as HTMLElement).append(childClone);
      // existingConfigsNode?.append(childClone);

      // const newLastChild = node.lastChild;
      const newLastChild = (existingConfigsNode! as HTMLElement).lastChild;
      console.log("NEW LAST CHILD " + newLastChild);
      console.log("i " + i);
      
      const rowId = (newLastChild! as HTMLElement).querySelector("#row-name-" + (i - 1));

      // console.log("NAMEID " + nameId);
      (newLastChild! as HTMLElement).id = "row" + i;
      (rowId! as HTMLElement).id = "row-name-" + i;

      const div = document.querySelector("#row-name-" + i);
      (div! as HTMLElement).innerHTML = configName;

      // (newLastChild! as HTMLElement).addEventListener("click", addExistingDataSection);
      (newLastChild! as HTMLElement).addEventListener("click", (event) => {
        addExistingDataSection(event);
      });
    }
  }

  //Access dictionary/object with "Mappings"
  //Update existing data (access, dictionary/object with "text_id", "board_id", "channel_type", "channel", "computer")
}

function displayAddConfig() {
  const addConfigSection = document.querySelector(".add-config-connect-section");
  console.log("I'M DISPLAYING");
  (addConfigSection as HTMLElement)!.style.display = "flex";
}

function addAddConfig() {
  const addConfigSection = document.querySelector(".add-config-connect-section");

  // addConfigSection.style.display = "flex";
  (addConfigSection! as HTMLElement).style.display = "flex";
}

function removeAddConfig() {
  const addConfigSection = document.querySelector(".add-config-connect-section");

  // addConfigSection.style.display = "none";
  (addConfigSection! as HTMLElement).style.display = "none";
}

function removeEditSection() {
  const editConfigSection = document.querySelector(".edit-section");
  console.log("I'M REMOVING");
  // editConfigSection.style.display = "none";
  (editConfigSection! as HTMLElement).style.display = "none";
}

// function addExistingDataSection() {
//   console.log("I'm displaying data");
//   displayingExistingData = !displayingExistingData;

//   console.log(displayingExistingData);

//   if(displayingExistingData) {
//     const existingDataSection = document.querySelector(".existing-data");

//     // existingDataSection.style.display = "flex";
//     (existingDataSection! as HTMLElement).style.display = "flex";

//     removeAddConfig();

//     const name = document.querySelector(".existing-data-name");
//     const divNode = document.querySelector(".data");
    // console.log(event);
    // const configNum = this.id.charAt(this.id.length - 1);
    // name.innerHTML = configurations()[configNum]["id"];

    // const mappings = configurations()[configNum]["mappings"];

    // for (let i = 0; i < mappings.length; i++) {
    //   const header = document.createElement("h4");
    //   for (let j = 0; j < 5; j++) {
    //     // header = document.createElement("h4");

    //     if (i == 0) {
    //       header.innerHTML = "Name" + mappings[i]["text_id"];
    //     } else if (i == 1) {
    //       header.innerHTML = "Board ID" + mappings[i]["board"];
    //     } else if (i == 2) {
    //       header.innerHTML = "Channel Type" + mappings[i]["channel_type"];
    //     } else if (i == 3) {
    //       header.innerHTML = "Channel" + mappings[i]["channel"];
    //     } else {
    //       header.innerHTML = "Computer" + mappings[i]["computer"];
    //     }
    //   }
       
    //   divNode?.append(header);
    //   mappings[i]["text_id"]

      {/* <h4>Name: </h4>
          <h4>Board ID: </h4>
          <h4>Channel Type: </h4>
          <h4>Channel: </h4>
          <h4>Computer: </h4> */}
    //}

  // } else {
  //   const existingDataSection = document.querySelector(".existing-data");

  //   // existingDataSection.style.display = "none";
  //   (existingDataSection! as HTMLElement).style.display = "none";

  //   addAddConfig();
  // }
//}

// const addExistingDataSection = e => {
//   console.log("Hello");
//   console.log(e.target.id);
// }

function addExistingDataSection(event) {
  console.log("Hello");
  console.log(event);
  console.log(event.target);
  console.log(event.target.id);

  console.log("BOOLEAN " + displayingExistingData);

  displayingExistingData = !displayingExistingData;

  if(displayingExistingData) {

    const existingDataSection = document.querySelector(".existing-data");
    
    // existingDataSection.style.display = "flex";
    (existingDataSection! as HTMLElement).style.display = "flex";
    
    removeAddConfig();

    const name = document.querySelector(".existing-data-name");
    const configNum = event.target.id.charAt(event.target.id.length - 1);
    (name! as HTMLElement).innerHTML = (configurations() as Config[])[configNum].id;

    const divNode = document.querySelector(".data");
    const mappings = (configurations() as Config[])[configNum].mappings;
    // const mappings = (((configurations() as Config[])[configNum].mappings) as Mapping[]);

    console.log("MAPPINGS " + mappings);
    console.log("TYPE " + (typeof mappings));

    // clearData();

    for (let i = 0; i < mappings.length; i++) {
      var header;
      // var header = document.createElement("h4");
      // var span = document.createElement("SPAN");
      // var span;

      for (let j = 0; j < 5; j++) {
        header = document.createElement("h4");
        // span = document.createElement("SPAN");

        if (j == 0) {
          header.innerHTML = "Name: " + mappings[i].text_id;
        } else if (j == 1) {
          header.innerHTML = "Board ID: " + mappings[i].board_id;
          console.log("BOARD ID FOR MAPPING 1 " + mappings[i].board_id);
        } else if (j == 2) {
          header.innerHTML = "Channel Type: " + mappings[i].channel_type;
        } else if (j == 3) {
          header.innerHTML = "Channel: " + mappings[i].channel;
        } else {
          header.innerHTML = "Computer: " + mappings[i].computer;
        }

        // if (j == 0) {
        //   header.innerHTML = "Name: ";
        //   span.innerHTML = mappings[i].text_id;
        // } else if (j == 1) {
        //   header.innerHTML = "Board ID: ";
        //   span.innerHTML = mappings[i].board_id + "";
        //   console.log("BOARD ID FOR MAPPING 1 " + mappings[i].board_id);
        // } else if (j == 2) {
        //   header.innerHTML = "Channel Type: ";
        //   span.innerHTML = mappings[i].channel_type;
        // } else if (j == 3) {
        //   header.innerHTML = "Channel: ";
        //   span.innerHTML = mappings[i].channel + "";
        // } else {
        //   header.innerHTML = "Computer: ";
        //   span.innerHTML = mappings[i].computer;
        // }

        (divNode! as HTMLElement).append(header);
        // (divNode! as HTMLElement).append(span);

        header.classList.add("data-child");
      }
    }

    

  } else {

    // clearData();

    const existingDataSection = document.querySelector(".existing-data");

    // existingDataSection.style.display = "none";
    (existingDataSection! as HTMLElement).style.display = "none";

    addAddConfig();
  }
}

function clearData() {
  const divNode = document.querySelector(".data");
  var lastChild = (divNode! as HTMLElement).lastChild;

  console.log(lastChild);

  // (lastChild! as HTMLElement).style.display = "none";

  while (lastChild != null) {
    (lastChild! as HTMLElement).style.display = "none";
    lastChild = (divNode! as HTMLElement).lastChild;
  }
}

function removeExistingDataSection() {
  const existingDataSection = document.querySelector(".existing-data");
  // existingDataSection.style.display = "none";
  (existingDataSection! as HTMLElement).style.display = "none";
}

function addEditSection() {
  // const editConfigSection = document.querySelector(".edit-section");
  const editSection = document.querySelector(".edit-section");

  // editConfigSection.style.display = "flex";
  // editSection.style.display = "flex";
  (editSection! as HTMLElement).style.display = "flex";
}

// function displayEditBtns() {
//   const editBtns = (document.querySelectorAll(".existing-config-edit-btns"));

//   console.log(editBtns);

//   editBtns.forEach((btn) => {
//     console.log(btn);
//     btn.style.display = "block";
//   });
// }

function addConfig() {
  //FINDING THE NUMBER OF CONFIGS SO FAR
  const node = document.querySelector(".editing-data");
  // var lastChild = node.lastChild;
  var lastChild = (node! as HTMLElement).lastChild;
  // lastChild.style.border = "2px solid red";
  // var numConfigs = node.lastChild.id.charAt(node.lastChild.id.length - 1);
  var numConfigs = (lastChild! as HTMLElement).id.charAt((lastChild! as HTMLElement).id.length - 1);

  console.log("NUM CONFIGS " + numConfigs);
  console.log("I'M ADDING CONFIG");

  //ADDING CLONE CONFIG
  // const childClone = lastChild.cloneNode(true);
  const childClone = (lastChild! as HTMLElement).cloneNode(true);
  // node.append(childClone);
  (node! as HTMLElement).append(childClone);

  console.log("NUM CONFIGS AFTER APPENDING " + numConfigs);
  //CHANGING ID'S OF NEW CONFIG
  // const newLastChild = node.lastChild;
  const newLastChild = (node! as HTMLElement).lastChild;

  console.log("NEW LAST CHILD " + newLastChild);
  // console.log("NEW LAST CHILD " + newLastChild.querySelector(".name2"));

  // const nameId = newLastChild.querySelector("#name" + numConfigs);
  const nameId = (newLastChild! as HTMLElement).querySelector("#name" + numConfigs);
  // const boardId = newLastChild.querySelector("#id" + numConfigs);
  const boardId = (newLastChild! as HTMLElement).querySelector("#id" + numConfigs);
  // const channelTypeId = newLastChild.querySelector("#channelType" + numConfigs);
  const channelTypeId = (newLastChild! as HTMLElement).querySelector("#channelType" + numConfigs);
  // const channelId = newLastChild.querySelector("#channel" + numConfigs);
  const channelId = (newLastChild! as HTMLElement).querySelector("#channel" + numConfigs);
  // const computerId = newLastChild.querySelector("#computer" + numConfigs);
  const computerId = (newLastChild! as HTMLElement).querySelector("#computer" + numConfigs);

  // numConfigs++;
  (numConfigs as unknown as number)++;
  // numConfigs = 2;

  console.log("I'M EDITING ID " + nameId);
  // newLastChild.id = "config" + numConfigs;
  (newLastChild! as HTMLElement).id = "config" + numConfigs;
  // nameId.id = "name" + numConfigs;
  (nameId! as HTMLElement).id = "name" + numConfigs;
  // boardId.id = "id" + numConfigs;
  (boardId! as HTMLElement).id = "id" + numConfigs;
  // channelTypeId.id = "channelType" + numConfigs;
  (channelTypeId! as HTMLElement).id = "channelType" + numConfigs;
  // channelId.id = "channel" + numConfigs;
  (channelId! as HTMLElement).id = "channel" + numConfigs;
  // computerId.id = "computer" + numConfigs;
  (computerId! as HTMLElement).id = "computer" + numConfigs;

  // nameId.id = "name2";
  // console.log("NEW NAME ID " + nameId.id);
  // console.log("NEW BOARD ID " + boardId.id);
  // console.log("NEW CHANNEL TYPE ID " + channelTypeId.id);
  // console.log("NEW CHANNEL ID " + channelId.id);
  // console.log("NEW COMPUTER ID " + computerId.id);


  // const node = document.querySelector(".editing-data");
  // const child = document.querySelector(".edit-config-configurations");
  // const childClone = child.cloneNode(true);

  // node.append("some text");
  // node.append(childClone);

  // const lastChild = node.lastChild;

  // const nameId = lastChild.querySelector("#name1");

  // nameId.id = "name2";

  // node.appendChild(child);

  // console.log("NODE " + node);
  // console.log("CHILD " + child);
}

// function checkNull(elem) {
//   return elem === null;
// }

function removeConfig() {
  console.log("I'm here");
  //FINDING LAST CONFIG
  const node = document.querySelector(".editing-data");
  var lastChild = node?.lastChild;

  //REMOVING LAST CONFIG
  console.log(lastChild);
  // lastChild.remove();
  (lastChild! as HTMLElement).remove();
}

function saveNewConfig() {
  // console.log(configurations()[0][1]);
  console.log("CONFIGURATIONS ");
  const node = document.querySelector(".existing-configs-sections");
  const child = node?.lastChild;
  const childClone = child?.cloneNode(true);

  childClone?.addEventListener("click", addExistingDataSection);

  // node?.append(childClone);
  (node! as HTMLElement).append((childClone! as HTMLElement));

  console.log("I'm saving");
  console.log(node);
  console.log(child);
  console.log(childClone);
}

// function checkNull(elem) {
//   return elem === null;
// }


const ConfigView: Component = (props) => {
  retrieveData();
  
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
          {/* <div id="row1" class="row" onClick={function(event){ removeAddConfig(); addEditSection()}}> */}
          {/* <div id="row1" class="row" onClick={function(event){ removeAddConfig(); addExistingDataSection()}}> */}
          {/* <div id="row0" class="row" onClick={() => addExistingDataSection()}> */}
          <div id="row0" class="row">
          {/* <div id="row1" class="row"> */}
            <div class="row-subheadings" id="row-name-0">Name</div>
            <div class="row-subheadings">Date</div>
            <button class="existing-config-edit-btns">Edit</button>
          </div>
          {/* <div class="row" onClick={function(event){ removeAddConfig(); addExistingDataSection()}}> */}
          {/* <div class="row" onClick={() => addExistingDataSection()}>
            <div class="row-subheadings">Name</div>
            <div class="row-subheadings">Date</div>
            <button class="existing-config-edit-btns">Edit</button>
          </div> */}
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
            <button class="add-config-save-btn" onClick={function(event){ saveNewConfig()}}>Save</button>
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
      <div class="existing-data system-connect-section">
        <div>
          <button class="add-config-add-btn" onClick={function(event){ removeExistingDataSection(); addEditSection()}}>Edit</button>
        </div>
        <div>
          <h4 class="existing-data-name">Config Name</h4>
        </div>
        <div class="data">
          {/* <h4>Name: </h4>
          <h4>Board ID: </h4>
          <h4>Channel Type: </h4>
          <h4>Channel: </h4>
          <h4>Computer: </h4> */}
          {/* <h4>Name: </h4>
          <h4>Board ID: </h4>
          <h4>Channel Type: </h4>
          <h4>Channel: </h4>
          <h4>Computer: </h4>
          <h4>Name: </h4>
          <h4>Board ID: </h4>
          <h4>Channel Type: </h4>
          <h4>Channel: </h4>
          <h4>Computer: </h4> */}
        </div>
      </div>

      <div class="system-connect-section edit-section">

          <div class="editing-data">
            <div class="add-config-section">
              <div class="add-config-setup">
                <p>Edit new config:</p>
                <input class="add-config-input" type="text" placeholder="Name"/>
              </div>
              <div class="add-config-btns">
                <button class="add-config-add-btn" onClick={() => addConfig()}>Add Config</button>
                <button class="add-config-remove-btn" onClick={() => removeConfig()}>Remove Config</button>
                <button class="add-config-cancel-btn" onClick={function(event){ removeEditSection(); displayAddConfig()}}>Cancel</button>
                <button class="add-config-save-btn" onClick={function(event){ removeEditSection(); displayAddConfig()}}>Save</button>
              </div>
            </div>
            <div class="horizontal-line"></div>
            <div class="add-config-configurations edit-config-configurations" id="config1">
              <input type="text" placeholder="Name" class="add-config-styling" id="name1"/>
              <input type="text" name="" id="id1" placeholder="Board ID" class="add-config-styling"/>
              <select name="" id="channelType1" class="add-config-styling">
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
              <input type="text" name="" id="channel1" placeholder="Channel" class="add-config-styling"/>
              <select name="" id="computer1" class="add-config-styling">
                <option class="seq-dropdown-item">Computer</option>
                <option class="seq-dropdown-item">Flight</option>
                <option class="seq-dropdown-item">Ground</option>
              </select>
            </div>
          </div>
        </div>




      {/* <div class="edit-section">
        <div class="system-connect-section editing-configs-section">
            <div class="existing-data">
              <h4>Config Name: </h4>
              <h4>Name: </h4>
              <h4>Board ID: </h4>
              <h4>Channel Type: </h4>
              <h4>Channel: </h4>
              <h4>Computer: </h4>
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
            <div class="add-config-configurations edit-config-configurations" id="config1">
              <input type="text" placeholder="Name" class="add-config-styling" id="name1"/>
              <input type="text" name="" id="id1" placeholder="Board ID" class="add-config-styling"/>
              <select name="" id="channelType1" class="add-config-styling">
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
              <input type="text" name="" id="channel1" placeholder="Channel" class="add-config-styling"/>
              <select name="" id="computer1" class="add-config-styling">
                <option class="seq-dropdown-item">Computer</option>
                <option class="seq-dropdown-item">Flight</option>
                <option class="seq-dropdown-item">Ground</option>
              </select>
            </div>
          </div>
        </div>
      </div> */}
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

export {Connect, Feedsystem, ConfigView, Sequences};