import { Component, createSignal, For, Show } from "solid-js";
import { invoke } from '@tauri-apps/api/tauri'
import { setServerIp, connect, isConnected, setIsConnected, setActivity, serverIp, activity, selfIp, selfPort, sessionId, forwardingId, State, Config, sendActiveConfig, setSessionId, setForwardingId, setSelfIp, setSelfPort, Mapping, sendSequence, Sequence, getConfigs, sendConfig, deleteConfig, getSequences, getAbortStages, sendActiveAbortStage, deleteAbortStage, sendAbortStage, AbortStage, AbortStageMapping } from "../comm";
import { turnOnLED, turnOffLED } from "../commands";
import { emit, listen } from '@tauri-apps/api/event'
import { appWindow } from "@tauri-apps/api/window";
import { DISCONNECT_ACTIVITY_THRESH } from "../appdata";
import { CodeMirror } from "@solid-codemirror/codemirror";
import { oneDark } from "@codemirror/theme-one-dark";
import { python } from "@codemirror/lang-python";
import { faTrash } from '@fortawesome/free-solid-svg-icons';
import Fa from 'solid-fa';
import { save } from '@tauri-apps/api/dialog';
import { writeTextFile } from '@tauri-apps/api/fs';

// states of error message and connect button
const [windowHeight, setWindowHeight] = createSignal(window.innerHeight);
const [connectDisplay, setConnectDisplay] = createSignal("Connect");
const [connectionMessage, setConnectionMessage] = createSignal('');
const [showSessionId, setShowSessionId] = createSignal(false);
const [showForwardingId, setShowForwardingId] = createSignal(false);
const [feedsystem, setFeedsystem] = createSignal('Feedsystem_1');
const [activeConfig, setActiveConfig] = createSignal('placeholderconfig');
const [configurations, setConfigurations] = createSignal();
const [currentSequnceText, setCurrentSequenceText] = createSignal('');
const [currentSequnceName, setCurrentSequenceName] = createSignal('');
const [sequences, setSequences] = createSignal();
const [refreshDisplay, setRefreshDisplay] = createSignal("Refresh");
const [saveConfigDisplay, setSaveConfigDisplay] = createSignal("Save");
const [confirmDelete, setConfirmDelete] = createSignal(false);
const [saveSequenceDisplay, setSaveSequenceDisplay] = createSignal("Submit");
const [currentConfigurationError, setCurrentConfigurationError] = createSignal('');
const [currentConfigurationErrorCode, setCurrentConfigurationErrorCode] = createSignal('');
const [currentAbortStageError, setCurrentAbortStageError] = createSignal('');
const [currentAbortStageErrorCode, setCurrentAbortStageErrorCode] = createSignal('');
const [activeAbortStage, setActiveAbortStage] = createSignal('placeholderconfig');
const [abortStages, setAbortStages] = createSignal();
const [refreshAbortStageDisplay, setRefreshAbortStageDisplay] = createSignal("Refresh");
const [saveAbortStageDisplay, setSaveAbortStageDisplay] = createSignal("Save");
const [confirmAbortStageDelete, setConfirmAbortStageDelete] = createSignal(false);
const default_entry = {
  text_id: '',
  board_id: '',
  sensor_type: 'PT',
  channel: NaN,
  computer: 'FLIGHT',
  min: NaN,
  max: NaN,
  powered_threshold: NaN,
  normally_closed: null
} as Mapping
const [channelTypes, setChannelTypes] = createSignal(["PT", "VALVE", "FLOW METER", "RAIL VOLTAGE", "RAIL CURRENT", "LOAD CELL", "RTD", "TC"]);
const [editableEntries, setEditableEntries] = createSignal([structuredClone(default_entry)]);
const [configFocusIndex, setConfigFocusIndex] = createSignal(0);
const [subConfigDisplay, setSubConfigDisplay] = createSignal('add');

const default_abort_stage_entry = {
  valve_name: '',
  abort_stage: null,
  timer_to_abort: NaN
} as AbortStageMapping
const [editableAbortStageEntries, setEditableAbortStageEntries] = createSignal([structuredClone(default_abort_stage_entry)]);
const [abortStageFocusIndex, setAbortStageFocusIndex] = createSignal(0);
const [subAbortStageDisplay, setSubAbortStageDisplay] = createSignal('add');

appWindow.onResized(({ payload: size }) => {
  setWindowHeight(window.innerHeight);
});


// function to connect to the server + input validation
async function connectToServer() {
  setConnectDisplay("Connecting...");
  setConnectionMessage('');

  // getting the ip from the relevant textfields
  var ip = (document.getElementsByName('server-ip')[0] as HTMLInputElement).value.trim();
  var result = '';

  result = await connect(ip) as string;

  setConnectionMessage(result);
  setConnectDisplay("Connect");
}

// get the activity from the taskbar page
emit('requestActivity');
listen('updateActivity', (event) => {
  setActivity(event.payload as number);
});

// listener for state updates
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
  setSequences((event.payload as State).sequences);
  setAbortStages((event.payload as State).abortStages);
  setActiveAbortStage((event.payload as State).activeAbortStage);
  console.log('from listener: ', configurations());
  console.log('sequences from listener:', sequences());
  console.log('abortStages from listener:', abortStages());
});
invoke('initialize_state', {window: appWindow});

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
          placeholder="Server IP/ hostname"
        />
        <div id="connect-message" style="font-size: 12px">
          {connectionMessage()}
        </div>
        <button class="connect-button" onClick={() => connectToServer()}>
          {connectDisplay()}
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
  setActiveConfig(dropdown.value);

}

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

async function refreshConfigs() {
  setRefreshDisplay("Refreshing...");
clear_configuration_error();
  var ip = serverIp() as string;
  await getConfigs(ip);
  var configs = await getConfigs(ip);
  console.log(configs);
  if (configs instanceof Error) {
    setRefreshDisplay('Error!');
    await new Promise(r => setTimeout(r, 1000));
    setRefreshDisplay('Refresh');
    return;
  }
  var configMap = new Map(Object.entries(configs));
  var configArray = Array.from(configMap, ([name, value]) => ({'id': name, 'mappings': value }));
  await invoke('update_configs', {window: appWindow, value: configArray});
  setConfigurations(configArray);
  setRefreshDisplay('Refreshed!');
  await new Promise(r => setTimeout(r, 1000));
  setRefreshDisplay('Refresh');
  console.log(configurations());
}

const Feedsystem: Component = (props) => {
  listen('state', (event) => {
    setFeedsystem((event.payload as State).feedsystem);
    setActiveConfig((event.payload as State).activeConfig);
  });
  setFeedsystemData();
  return <div style="height: 100%; display: flex; flex-direction: column">
    <div style="text-align: center; font-size: 14px">SETUP</div>
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
          </select>
          </div>
          <button style={{"margin": '5px'}} class='refresh-button' onClick={refreshConfigs}>{refreshDisplay()}</button>        
        </div>
      </div>
      </div>
      <div style={{'margin-left': '10px', 'margin-top': '10px','padding-left': '170px'}}>
        <button class='submit-feedsystem-button' onClick={setFeedsystemAndActiveConfig}> Submit </button>
      </div>
    </div>
</div>
}

function addNewConfigEntry() {
  var entries = [...editableEntries()];
  entries.push(structuredClone(default_entry));
  setEditableEntries(entries);
  console.log(editableEntries());
}

function deleteConfigEntry(entry: Mapping) {
  if (editableEntries().length === 1) {
    setEditableEntries([structuredClone(default_entry)]);
      return;
  }
  var entries = [...editableEntries()];
  var mappingnames = document.querySelectorAll("[id=addmappingname]") as unknown as Array<HTMLInputElement>;
  var mappingboardids = document.querySelectorAll("[id=addmappingboardid]") as unknown as Array<HTMLInputElement>;
  var mappingchanneltypes = document.querySelectorAll("[id=addmappingchanneltype]") as unknown as Array<HTMLSelectElement>;
  var mappingchannels = document.querySelectorAll("[id=addmappingchannel]") as unknown as Array<HTMLInputElement>;
  var mappingcomputers = document.querySelectorAll("[id=addmappingcomputer]") as unknown as Array<HTMLSelectElement>;
  var mappingmins = document.querySelectorAll("[id=addmappingmin]") as unknown as Array<HTMLSelectElement>;
  var mappingmaxs = document.querySelectorAll("[id=addmappingmax]") as unknown as Array<HTMLSelectElement>;
  var mappingvalveconnecteds = document.querySelectorAll("[id=addmappingvalveconnected]") as unknown as Array<HTMLSelectElement>;
  var mappingvalvepowereds = document.querySelectorAll("[id=addmappingvalvepowered]") as unknown as Array<HTMLSelectElement>;
  var mappingvalvenormcloseds = document.querySelectorAll("[id=addmappingvalvenormclosed]") as unknown as Array<HTMLSelectElement>;
  for (var i = 0; i < entries.length; i++) {
    entries[i].text_id = mappingnames[i].value;
    entries[i].board_id = mappingboardids[i].value;
    entries[i].sensor_type = mappingchanneltypes[i].value.replace(' ', '_').toLowerCase();
    entries[i].channel = mappingchannels[i].value as unknown as number;
    entries[i].computer = mappingcomputers[i].value.toLowerCase();
    entries[i].min = mappingmins[i].value === ""? NaN: mappingmins[i].value as unknown as number;
    entries[i].max = mappingmaxs[i].value === ""? NaN: mappingmaxs[i].value as unknown as number;
    entries[i].powered_threshold = mappingvalvepowereds[i].value === ""? 
      NaN: mappingvalvepowereds[i].value as unknown as number;
    entries[i].normally_closed = mappingvalvenormcloseds[i].value === "N/A"? 
      null : JSON.parse(mappingvalvenormcloseds[i].value.toLowerCase())
  }
  console.log(entry);
  entries.splice(entries.indexOf(entry), 1);
  setEditableEntries(entries);
  console.log('deleted somthing!');
  console.log(editableEntries());
}


function clear_configuration_error() {
  setCurrentConfigurationErrorCode('');
  setCurrentConfigurationError('');
}

// Returns true on success, false on failure
async function submitConfig(edited: boolean) {
  var newConfigNameInput = (document.getElementById('newconfigname') as HTMLInputElement)!;
  var configName;
  clear_configuration_error();

  if (edited) {
    configName = (configurations() as Config[])[configFocusIndex()].id;
  } else {
    configName = newConfigNameInput.value;
    if (configName === "") {
      setSaveConfigDisplay("Error!");
      newConfigNameInput.value = 'Enter a name here!';
      await new Promise(r => setTimeout(r, 1000));
      setSaveConfigDisplay("Save");
      newConfigNameInput.value = '';
      return false;
    }
  }

  setSaveConfigDisplay("Saving...");
  var entries = [...editableEntries()];
  var mappingnames = document.querySelectorAll("[id=addmappingname]") as unknown as Array<HTMLInputElement>;
  var mappingboardids = document.querySelectorAll("[id=addmappingboardid]") as unknown as Array<HTMLInputElement>;
  var mappingchanneltypes = document.querySelectorAll("[id=addmappingchanneltype]") as unknown as Array<HTMLSelectElement>;
  var mappingchannels = document.querySelectorAll("[id=addmappingchannel]") as unknown as Array<HTMLInputElement>;
  var mappingcomputers = document.querySelectorAll("[id=addmappingcomputer]") as unknown as Array<HTMLSelectElement>;
  var mappingmins = document.querySelectorAll("[id=addmappingmin]") as unknown as Array<HTMLSelectElement>;
  var mappingmaxs = document.querySelectorAll("[id=addmappingmax]") as unknown as Array<HTMLSelectElement>;
  var mappingvalveconnecteds = document.querySelectorAll("[id=addmappingvalveconnected]") as unknown as Array<HTMLSelectElement>;
  var mappingvalvepowereds = document.querySelectorAll("[id=addmappingvalvepowered]") as unknown as Array<HTMLSelectElement>;
  var mappingvalvenormcloseds = document.querySelectorAll("[id=addmappingvalvenormclosed]") as unknown as Array<HTMLSelectElement>;
  for (var i = 0; i < entries.length; i++) {
    entries[i].text_id = mappingnames[i].value;
    entries[i].board_id = mappingboardids[i].value;
    entries[i].sensor_type = mappingchanneltypes[i].value.replace(' ', '_').toLowerCase();
    entries[i].channel = mappingchannels[i].value as unknown as number;
    entries[i].computer = mappingcomputers[i].value.toLowerCase();
    entries[i].min = mappingmins[i].value === ""? NaN: mappingmins[i].value as unknown as number;
    entries[i].max = mappingmaxs[i].value === ""? NaN: mappingmaxs[i].value as unknown as number;
    entries[i].powered_threshold = mappingvalvepowereds[i].value === ""? 
      NaN: mappingvalvepowereds[i].value as unknown as number;
    entries[i].normally_closed = mappingvalvenormcloseds[i].value === "N/A"? 
      null : JSON.parse(mappingvalvenormcloseds[i].value.toLowerCase())
  }
  console.log(entries);

  const response = await sendConfig(serverIp() as string, {id: configName, mappings: entries} as Config);
  const statusCode = response.status;
  if (statusCode != 200) {
    refreshConfigs();
    if (statusCode == 400) {
      setCurrentConfigurationErrorCode('ERROR : BAD REQUEST');
    } else if (statusCode == 418) {
      setCurrentConfigurationErrorCode("ERROR : I'M A TEAPOT");
    } else {
      setCurrentConfigurationErrorCode("ERROR CODE " + statusCode);
    }
    setSaveConfigDisplay("Error!");
    const ErrorMessage = await response.text();
    setCurrentConfigurationError(ErrorMessage);
    alert(currentConfigurationErrorCode() + "\n" + ErrorMessage);
    await new Promise(r => setTimeout(r, 1000));
    setSaveConfigDisplay("Save");
    return false;
  }

  setSaveConfigDisplay("Saved!");
  refreshConfigs();

  await new Promise(r => setTimeout(r, 1000));

  setSaveConfigDisplay("Save");

  return true;
}

async function removeConfig(configId: string) {
  const success = await deleteConfig(serverIp() as string, configId) as object;
  const statusCode = success['status' as keyof typeof success];
  if (statusCode != 200) {
    refreshConfigs();
    setSaveConfigDisplay("Error!");
    await new Promise(r => setTimeout(r, 1000));
    setSaveConfigDisplay("Save");
    return;
  }
  setSubConfigDisplay('add');
  setSaveConfigDisplay("Deleted!");
  refreshConfigs();
  await new Promise(r => setTimeout(r, 1000));
  setSaveConfigDisplay("Save");
}

function readFile(e: any) {
  const file = e.target.files[0];
  const fr = new FileReader();

  fr.addEventListener("load", async e => {
    const json = JSON.parse(fr.result as string);
    console.log(json);
    console.log("In here")

    const newConfig: Config = {
      id: json.configuration_id,
      mappings: json.mappings
    };

    const success = await sendConfig(serverIp() as string, newConfig as Config) as object;
    const statusCode = success['status' as keyof typeof success];
    if (statusCode != 200) {
      // Add a notification informing the upload failed
      refreshConfigs();
      return;
    }
    refreshConfigs();
  });

  fr.readAsText(file);
}

async function exportToJsonFile(data: any, fileName: string) {
  console.log("Exporting json:");
  console.log("fileName:", fileName);
  console.log("data:", data);

  const transformedData = {
    configuration_id: data.id,
    mappings: data.mappings.map((m: any) => ({
      text_id: m.text_id,
      board_id: m.board_id,
      sensor_type: m.sensor_type,
      channel: m.channel,
      computer: m.computer,
      min: m.min,
      max: m.max,
      powered_threshold: m.powered_threshold,
      normally_closed: m.normally_closed
    })),
  };

  const jsonString = JSON.stringify(transformedData, null, 2); 
  const path = await save({
    defaultPath: `${fileName}.json`,
    filters: [{ name: "JSON Files", extensions: ["json"] }],
  });

  if (path) {
    await writeTextFile(path, jsonString);
    console.log(`Saved file to: ${path}`);
  }
}


const AddConfigView: Component = (props) => {
  return <div style={{width: '100%'}}>
    <div class="add-config-section">
      <div class="add-config-setup">
        <p>Add New Config:</p>
        <input id='newconfigname' class="add-config-input" type="text" placeholder="Name"/>
      </div>
      <div class="add-config-btns">
        <label for="file-upload" class="import-config">Import Configuration</label>
        <input id="file-upload" type="file" onChange={(e) => {readFile(e);}}/>
        <button class="add-config-btn" onClick={addNewConfigEntry}>Insert Mapping</button>
        <button style={{"background-color": '#C53434'}} class="add-config-btn" onClick={function(event){
          setEditableEntries([structuredClone(default_entry)]);
          clear_configuration_error();
        }}>Cancel</button>
        <button style={{"background-color": '#015878'}} class="add-config-btn" onClick={() => {submitConfig(false);}}>{saveConfigDisplay()}</button>
      </div>
    </div>
    <div class="horizontal-line"></div>
    <div style={{"margin-top": '5px', "margin-right": '20px'}} class="add-config-configurations">
      <div style={{width: '11%', "text-align": 'center'}}>Name</div>
      <div style={{width: '11%', "text-align": 'center'}}>Board ID</div>
      <div style={{width: '11%', "text-align": 'center'}}> Channel Type</div>
      <div style={{width: '11%', "text-align": 'center'}}>Channel</div>
      <div style={{width: '11%', "text-align": 'center'}}>Computer</div>
      <div style={{width: '11%', "text-align": 'center'}}>Min</div>
      <div style={{width: '11%', "text-align": 'center'}}>Max</div>
      <div style={{width: '11%', "text-align": 'center'}}>Valve Pow Thresh</div>
      <div style={{width: '11%', "text-align": 'center'}}>Valve Norm Closed</div>
    </div>
    <div style={{"max-height": '100%', "overflow-y": "auto"}}>
      <For each={editableEntries()}>{(entry, i) =>
          <div class="add-config-configurations">
            <input id={"addmappingname"} type="text" value={entry.text_id} placeholder="Name" class="add-config-styling"/>
            <input type="text" name="" id={"addmappingboardid"} value={entry.board_id} placeholder="Board ID" class="add-config-styling"/>
            <select name="" id={"addmappingchanneltype"} value={entry.sensor_type.toUpperCase()} class="add-config-styling">
              <For each={channelTypes()}>{(channel, i) => 
                <option class="seq-dropdown-item">{channel}</option>}                
              </For>
            </select>
            <input type="text" name="" id={"addmappingchannel"} value={Number.isNaN(entry.channel)? "": entry.channel} placeholder="Channel" class="add-config-styling"/>
            <select name="" id={"addmappingcomputer"} value={entry.computer as string} class="add-config-styling">
              <option class="seq-dropdown-item">FLIGHT</option>
              <option class="seq-dropdown-item">GROUND</option>
            </select>
            <input type="text" name="" id={"addmappingmin"} value={Number.isNaN(entry.min)? "": entry.min} placeholder="Min" class="add-config-styling"/>
            <input type="text" name="" id={"addmappingmax"} value={Number.isNaN(entry.max)? "": entry.max} placeholder="Max" class="add-config-styling"/>
            <input type="text" name="" id={"addmappingvalvepowered"} value={Number.isNaN(entry.powered_threshold)? "": entry.powered_threshold} placeholder="ValvePowThresh" class="add-config-styling"/>
            <select name="" id={"addmappingvalvenormclosed"} value={entry.normally_closed === null? 'N/A': (entry.normally_closed? "TRUE": "FALSE")} class="add-config-styling">
              <option class="seq-dropdown-item">N/A</option>
              <option class="seq-dropdown-item">TRUE</option>
              <option class="seq-dropdown-item">FALSE</option>
            </select>
            <div onClick={() => deleteConfigEntry(entry)}><Fa icon={faTrash} color='#C53434'/></div>
          </div>
        }
      </For>
    </div>
  </div>
}

function loadConfigEntries(index: number) {
  var entries: Mapping[] = [];
  (configurations() as Config[])[index].mappings.forEach( (value) => {
    entries.push({
      text_id: value.text_id,
      board_id: value.board_id,
      sensor_type: value.sensor_type.replace('_', ' ').toUpperCase(),
      channel: value.channel,
      computer: value.computer.toUpperCase(),
      min: value.min,
      max: value.max,
      powered_threshold: value.powered_threshold,
      normally_closed: value.normally_closed
    });
  });
  setEditableEntries(entries);
}

const EditConfigView: Component<{index: number}> = (props) => {
  var index = props.index;
  loadConfigEntries(index);
  console.log(editableEntries);
  return <div style={{width: '100%'}}>
    <div class="add-config-section">
      <div class="add-config-setup">
        <p>Editing config:</p>
        <div style={{"font-weight": "bold"}}>{(configurations() as Config[])[index].id}</div>
      </div>
      <div class="add-config-btns">
        <button class="add-config-btn" onClick={addNewConfigEntry}>Insert Mapping</button>
        <button style={{"background-color": '#C53434'}} class="add-config-btn" onClick={function(event){
          setEditableEntries([structuredClone(default_entry)]);
          setSubConfigDisplay('view');
          clear_configuration_error();
        }}>Cancel</button>
        <button style={{"background-color": '#015878'}} class="add-config-btn" onClick={async () => {
          if (await submitConfig(true)) { 
            setSubConfigDisplay('view'); 
          }
        }}>{saveConfigDisplay()}</button>
      </div>
    </div>
    <div class="horizontal-line"></div>
    <div style={{"margin-top": '5px', "margin-right": '20px'}} class="add-config-configurations">
      <div style={{width: '11%', "text-align": 'center'}}>Name</div>
      <div style={{width: '11%', "text-align": 'center'}}>Board ID</div>
      <div style={{width: '11%', "text-align": 'center'}}> Channel Type</div>
      <div style={{width: '11%', "text-align": 'center'}}>Channel</div>
      <div style={{width: '11%', "text-align": 'center'}}>Computer</div>
      <div style={{width: '11%', "text-align": 'center'}}>Min</div>
      <div style={{width: '11%', "text-align": 'center'}}>Max</div>
      <div style={{width: '11%', "text-align": 'center'}}>Valve Pow Thresh</div>
      <div style={{width: '11%', "text-align": 'center'}}>Valve Norm Closed</div>
    </div>
    <div style={{"max-height": '100%', "overflow-y": "auto"}}>
      <For each={editableEntries()}>{(entry, i) =>
          <div class="add-config-configurations">
            <input id={"addmappingname"} type="text" value={entry.text_id} placeholder="Name" class="add-config-styling"/>
            <input type="text" name="" id={"addmappingboardid"} value={entry.board_id} placeholder="Board ID" class="add-config-styling"/>
            <select name="" id={"addmappingchanneltype"} value={entry.sensor_type.toUpperCase()} class="add-config-styling">
              <For each={channelTypes()}>{(channel, i) => 
                <option class="seq-dropdown-item">{channel}</option>}                
              </For>
            </select>
            <input type="text" name="" id={"addmappingchannel"} value={Number.isNaN(entry.channel)? "": entry.channel} placeholder="Channel" class="add-config-styling"/>
            <select name="" id={"addmappingcomputer"} value={entry.computer as string} class="add-config-styling">
              <option class="seq-dropdown-item">FLIGHT</option>
              <option class="seq-dropdown-item">GROUND</option>
            </select>
            <input type="text" name="" id={"addmappingmin"} value={Number.isNaN(entry.min)? "": entry.min} placeholder="Min" class="add-config-styling"/>
            <input type="text" name="" id={"addmappingmax"} value={Number.isNaN(entry.max)? "": entry.max} placeholder="Max" class="add-config-styling"/>
            <input type="text" name="" id={"addmappingvalvepowered"} value={Number.isNaN(entry.powered_threshold)? "": entry.powered_threshold} placeholder="ValvePowThresh" class="add-config-styling"/>
            <select name="" id={"addmappingvalvenormclosed"} value={entry.normally_closed === null? 'N/A': (entry.normally_closed? "TRUE": "FALSE")} class="add-config-styling">
              <option class="seq-dropdown-item">N/A</option>
              <option class="seq-dropdown-item">TRUE</option>
              <option class="seq-dropdown-item">FALSE</option>
            </select>
            <div onClick={() => deleteConfigEntry(entry)}><Fa icon={faTrash} color='#C53434'/></div>
          </div>
        }
      </For>
    </div>
  </div>
}

const DisplayConfigView: Component<{index: number}> = (props) => {
  var index = props.index;
  refreshConfigs();

  const handleClickOutside = (e: MouseEvent) => {
    const target = e.target as HTMLElement | null;
    if (target && !target.closest('.del-config-btn')) {
      setConfirmDelete(false);
      document.removeEventListener('click', handleClickOutside);
    }
  }

  return <div style={{width: '100%'}}>
    <div class="add-config-section">
      <div class="add-config-setup">
        <p>Viewing Config:</p>
        <div style={{"font-weight": "bold"}}>{(configurations() as Config[])[index].id}</div>
      </div>
      <div class="add-config-btns">
      <button class="del-config-btn" onClick={async (e)=>{
        e.stopPropagation();
        if (confirmDelete()) {
          await removeConfig((configurations() as Config[])[index].id);
          console.log((configurations() as Config[]).length);
          setConfirmDelete(false);
          setConfigFocusIndex(prevIndex => prevIndex - 1 > 0 ? prevIndex - 1 : 0);
          document.removeEventListener('click', handleClickOutside);
        } else {
          setConfirmDelete(true);
          document.addEventListener('click', handleClickOutside);
        }
      }}>{confirmDelete() ? 'Confirm' : 'Delete'}</button>
      <button class="add-config-btn" onClick={()=>{exportToJsonFile((configurations() as Config[])[index], (configurations() as Config[])[index].id);}}>Export Configuration</button>
      <button class="add-config-btn" onClick={()=>{setSubConfigDisplay('edit'); refreshConfigs();}}>Edit</button>
      <button class="add-config-btn" onClick={()=>{
        setSubConfigDisplay('add');
        clear_configuration_error();
      }}>Exit</button>
      </div>
    </div>
    <div class="horizontal-line"></div>
    <div style={{"margin-top": '5px'}} class="add-config-configurations">
      <div style={{width: '11%', "text-align": 'center'}}>Name</div>
      <div style={{width: '11%', "text-align": 'center'}}>Board ID</div>
      <div style={{width: '11%', "text-align": 'center'}}> Channel Type</div>
      <div style={{width: '11%', "text-align": 'center'}}>Channel</div>
      <div style={{width: '11%', "text-align": 'center'}}>Computer</div>
      <div style={{width: '11%', "text-align": 'center'}}>Min</div>
      <div style={{width: '11%', "text-align": 'center'}}>Max</div>
      <div style={{width: '11%', "text-align": 'center'}}>Valve Pow Thresh</div>
      <div style={{width: '11%', "text-align": 'center'}}>Valve Norm Closed</div>
    </div>
    <div style={{"max-height": '100%', "overflow-y": "auto"}}>
      <For each={(configurations() as Config[])[index].mappings}>{(entry, i) =>
        <div class="add-config-configurations">
          <div style={{width: '11%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.text_id}</div>
          <div style={{width: '11%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.board_id}</div>
          <div style={{width: '11%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.sensor_type.replace('_', ' ').toUpperCase()}</div>
          <div style={{width: '11%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.channel}</div>
          <div style={{width: '11%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.computer.toUpperCase()}</div>
          <div style={{width: '11%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.min}</div>
          <div style={{width: '11%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.max}</div>
          <div style={{width: '11%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.powered_threshold}</div>
          <div style={{width: '11%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.normally_closed === null? 'N/A': (entry.normally_closed? "TRUE": "FALSE")}</div>
        </div>
        }
      </For>
    </div>
  </div>
}

const ConfigView: Component = (props) => {
  setEditableEntries([structuredClone(default_entry)]);
  return <div class="config-view">
    <div style="text-align: center; font-size: 14px">CONFIGURATION</div>
    {/* <div class="system-config-page"> */}
      <div class="system-config-above-section">
        <div style={{display: "grid", "grid-template-columns": "100px 1fr 100px", width: '100%', "margin-bottom": '5px'}}>
          <div></div>
          <div style="text-align: center; font-size: 14px; font-family: 'Rubik'">Available Configurations</div>
          <button style={{"justify-content": "end"}} class="refresh-button" onClick={refreshConfigs}>{refreshDisplay()}</button>
        </div>
        
        <div class="horizontal-line"></div>
        <div class="existing-configs-sections">
          <div style={{height: "5px"}}></div>
          <div style={{"overflow-y": "auto", "max-height": '100px'}}>
            <For each={configurations() as Config[]}>{(config, i) =>
                <div class="existing-config-row" onClick={()=>{if (subConfigDisplay() != 'view') {setSubConfigDisplay('view'); setConfigFocusIndex(i as unknown as number);}}}>
                  <span class="config-id">{config.id}</span>
                </div>
              }
            </For>
          </div>
        </div>
      </div>
      <div class="new-config-section" style={{height: (windowHeight()-390) as any as string + "px"}}>
        <div innerText={(() => {
          return currentConfigurationErrorCode()
        })()} style={{color: '#bf5744', 'text-align' : 'center'}}>
        </div>

        {(() => {
          console.log('some display set');
          console.log(configFocusIndex());
          if (subConfigDisplay() == 'add') {
            return <AddConfigView />;
          } else if (subConfigDisplay() == 'view') {
            return <DisplayConfigView index={configFocusIndex()} />;
          } else if (subConfigDisplay() == 'edit') {
            return <EditConfigView index={configFocusIndex()} />;
          } else {
            return <div>How did we get here??</div>;
          }
        })()}
      </div>
    {/* </div> */}
</div>
}

function displaySequence(index: number) {
  refreshSequences();
  setCurrentSequenceName((sequences() as Array<Sequence>)[index].name);
  setCurrentSequenceText((sequences() as Array<Sequence>)[index].script);
  var configDropdown = (document.getElementById("addassociatedconfig"))! as HTMLSelectElement;
  configDropdown.value = (sequences() as Array<Sequence>)[index].configuration_id;
}

function resetSequenceEditor() {
  setCurrentSequenceName('');
  setCurrentSequenceText('');
}

async function refreshSequences() {
  setRefreshDisplay("Refreshing...");
  var ip = serverIp() as string;
  var seq = await getSequences(ip);
  console.log(seq);
  if (seq instanceof Error) {
    setRefreshDisplay('Error!');
    await new Promise(r => setTimeout(r, 1000));
    setRefreshDisplay('Refresh');
    return;
  }
  const sequenceMap = seq as object;
  const sequenceArray = sequenceMap['sequences' as keyof typeof sequenceMap];
  await invoke('update_sequences', {window: appWindow, value: sequenceArray});
  setSequences(sequenceArray);
  setRefreshDisplay('Refreshed!');
  await new Promise(r => setTimeout(r, 1000));
  setRefreshDisplay('Refresh');
  console.log(sequences());
}

async function sendSequenceIntermediate() {
  const configDropdown = (document.getElementById("addassociatedconfig"))! as HTMLSelectElement
  if (currentSequnceName().length === 0) {
    setCurrentSequenceName('Enter a sequence name!');
    await new Promise(r => setTimeout(r, 1000));
    setCurrentSequenceName('');
    return;
  }
  if (currentSequnceText().trim().length === 0) {
    setCurrentSequenceText('Enter sequence code!');
    await new Promise(r => setTimeout(r, 1000));
    setCurrentSequenceText('');
    return;
  }
  if (configDropdown.value === "") {
    setSaveSequenceDisplay("No associated config!");
    await new Promise(r => setTimeout(r, 1000));
    setSaveSequenceDisplay("Submit");
    return;
  }
  setSaveSequenceDisplay("Submitting...");
  const success = await sendSequence(serverIp() as string, currentSequnceName(), btoa(currentSequnceText()), configDropdown.value) as object;
  const statusCode = success['status' as keyof typeof success];
  if (statusCode != 200) {
    refreshSequences();
    setSaveSequenceDisplay("Error!");
    await new Promise(r => setTimeout(r, 1000));
    setSaveSequenceDisplay("Submit");
    return;
  }
  setSaveSequenceDisplay("Submitted!");
  refreshSequences();
  await new Promise(r => setTimeout(r, 1000));
  setSaveSequenceDisplay("Submit");
}

const Sequences: Component = (props) => {
  return <div class="system-sequences-page">
    <div style="text-align: center; font-size: 14px">SEQUENCES</div>
      <div class="sequences-list-view">
        <div style={{display: "grid", "grid-template-columns": "100px 1fr 100px", width: '100%', "margin-bottom": '5px'}}>
          <div></div>
          <div style="text-align: center; font-size: 14px; font-family: 'Rubik'">Available Sequences</div>
          <button style={{"justify-content": "end"}} class="refresh-button" onClick={refreshSequences}>{refreshDisplay()}</button>
        </div>
        <div class="horizontal-line"></div>
        <div style={{"overflow-y": "auto", "max-height": '150px'}}>
          <For each={sequences() as Sequence[]}>{(seq, i) =>
              <div class="sequence-display-item" onClick={() => displaySequence(i())}>
                {seq.name}
              </div>
            }
          </For>
        </div>
      </div>
      <div class="sequences-editor">
        <div style={{display: "grid", "grid-template-columns": "240px 200px 10px 50px 1fr", height: '50px'}}>
          <input class="connect-textfield"
            type="text"
            name="sequence-name"
            placeholder="Sequence Name"
            value={currentSequnceName()}
            onInput={(event) => setCurrentSequenceName(event.currentTarget.value)}
          style={{width: '200px'}}/>
          <div style={{display: "flex", "flex-direction": 'row'}}>
            <div style={{"margin-right": "5px", "text-align": "right", width: "80px"}}>Associated Config:</div>
            <select name="" id={"addassociatedconfig"} class="sequence-config-dropdown">
            <For each={configurations() as Config[]}>{(config, i) =>
                <option class="conf-dropdown-item" value={config.id}>{config.id}</option>
              }</For>
            </select>
          </div>
          <div></div>
          <div><button class="add-config-btn" onClick={resetSequenceEditor}>New</button></div>
          <div style={{width: '100%'}}><button style={{float: "right"}} class="submit-sequence-button" onClick={() => sendSequenceIntermediate()}>{saveSequenceDisplay()}</button></div>
        </div>
        <div class="code-editor" style={{height: (windowHeight()-425) as any as string + "px"}}>
          <CodeMirror value={currentSequnceText()} onValueChange={(value) => {setCurrentSequenceText(value);}} extensions={[python()]} theme={oneDark}/>
        </div>
    </div>
</div>
}

function addNewAbortStageEntry() {
  var entries = [...editableAbortStageEntries()];
  entries.push(structuredClone(default_abort_stage_entry));
  setEditableAbortStageEntries(entries);
  console.log(editableAbortStageEntries());
}

function deleteAbortStageEntry(entry: AbortStageMapping) {
  if (editableAbortStageEntries().length === 1) {
    setEditableAbortStageEntries([structuredClone(default_abort_stage_entry)]);
      return;
  }
  var entries = [...editableAbortStageEntries()];
  var mappingnames = document.querySelectorAll("[id=addabortstagename]") as unknown as Array<HTMLInputElement>;
  var mappingabortstages = document.querySelectorAll("[id=addabortstage]") as unknown as Array<HTMLSelectElement>;
  var mappingtimers = document.querySelectorAll("[id=addabortstagetimer]") as unknown as Array<HTMLInputElement>;
  for (var i = 0; i < entries.length; i++) {
    entries[i].valve_name = mappingnames[i].value;
    entries[i].abort_stage = mappingabortstages[i].value === "N/A"? 
      null : mappingabortstages[i].value.toLowerCase()
    entries[i].timer_to_abort = mappingtimers[i].value === ""? NaN: mappingtimers[i].value as unknown as number;
  }
  console.log(entry);
  entries.splice(entries.indexOf(entry), 1);
  setEditableAbortStageEntries(entries);
  console.log('deleted somthing!');
  console.log(editableAbortStageEntries());
}


function clear_abort_stage_error() {
  setCurrentAbortStageErrorCode('');
  setCurrentAbortStageError('');
}

// Returns true on success, false on failure
async function refreshAbortStages() {
  setRefreshAbortStageDisplay("Refreshing...");
  clear_abort_stage_error();
  var ip = serverIp() as string;
  var abortStageResponse = await getAbortStages(ip);
  console.log(abortStageResponse);
  if (abortStageResponse instanceof Error) {
    setRefreshAbortStageDisplay('Error!');
    await new Promise(r => setTimeout(r, 1000));
    setRefreshAbortStageDisplay('Refresh');
    return;
  }
  
  const stages = (abortStageResponse as { stages: Array<{ stage_name: string, abort_condition: string, valve_safe_states: Record<string, { desired_state: string, safing_timer: number }> }> }).stages;
  
  const abortStageArray = stages.map(stage => {
    // convert valve_safe_states HashMap back to mappings array
    const mappings: AbortStageMapping[] = Object.entries(stage.valve_safe_states).map(([valve_name, valveState]) => ({
      valve_name: valve_name,
      abort_stage: valveState.desired_state, // "open" or "closed"
      timer_to_abort: valveState.safing_timer
    }));
    
    return {
      id: stage.stage_name,
      abort_condition: stage.abort_condition,
      mappings: mappings
    } as AbortStage;
  });
  
  await invoke('update_abort_stages', {window: appWindow, value: abortStageArray});
  setAbortStages(abortStageArray);
  setRefreshAbortStageDisplay('Refreshed!');
  await new Promise(r => setTimeout(r, 1000));
  setRefreshAbortStageDisplay('Refresh');
  console.log(abortStages());
}

async function submitAbortStage(edited: boolean) {
  var newAbortStageNameInput = (document.getElementById('newabortstagename') as HTMLInputElement)!;
  var abortStageName;
  clear_abort_stage_error();

  if (edited) {
    abortStageName = (abortStages() as AbortStage[])[abortStageFocusIndex()].id;
  } else {
    abortStageName = newAbortStageNameInput.value;
    if (abortStageName === "") {
      setSaveAbortStageDisplay("Error!");
      newAbortStageNameInput.value = 'Enter a name here!';
      await new Promise(r => setTimeout(r, 1000));
      setSaveAbortStageDisplay("Save");
      newAbortStageNameInput.value = '';
      return false;
    }
  }

  var abortCondition = (document.getElementById('newabortstagecondition') as HTMLInputElement)!.value;

  setSaveAbortStageDisplay("Saving...");
  var entries = [...editableAbortStageEntries()];
  var mappingnames = document.querySelectorAll("[id=addabortstagename]") as unknown as Array<HTMLInputElement>;
  var mappingabortstages = document.querySelectorAll("[id=addabortstage]") as unknown as Array<HTMLSelectElement>;
  var mappingtimersmin = document.querySelectorAll("[id=addabortstagetimermin]") as unknown as Array<HTMLInputElement>;
  var mappingtimerssec = document.querySelectorAll("[id=addabortstagetimersec]") as unknown as Array<HTMLInputElement>;
  var mappingtimersmil = document.querySelectorAll("[id=addabortstagetimermil]") as unknown as Array<HTMLInputElement>;
  for (var i = 0; i < entries.length; i++) {
    entries[i].valve_name = mappingnames[i].value;
    entries[i].abort_stage = mappingabortstages[i].value === "N/A"? 
      null : mappingabortstages[i].value.toLowerCase()

    const minVal = mappingtimersmin[i].value;
    const secVal = mappingtimerssec[i].value;
    const milVal = mappingtimersmil[i].value;

    if (isNaN(Number(minVal)) || isNaN(Number(secVal)) || isNaN(Number(milVal))) {
      refreshAbortStages();
      var message = '';
      if (minVal && isNaN(Number(minVal))) message = `Invalid minutes at entry ${i}`;
      if (secVal && isNaN(Number(secVal))) message = `Invalid seconds at entry ${i}`;
      if (milVal && isNaN(Number(milVal))) message = `Invalid milliseconds at entry ${i}`;
      setSaveAbortStageDisplay("Error!");
      setCurrentAbortStageError(message);
      alert(message);
      await new Promise(r => setTimeout(r, 1000));
      setSaveAbortStageDisplay("Save");
      return false;
    }

    entries[i].timer_to_abort = ((Number(minVal) || 0) * 1000 * 60)
      + ((Number(secVal) || 0) * 1000)
      + (Number(milVal) || 0);
  }
  console.log(entries);

  const response = await sendAbortStage(serverIp() as string, {id: abortStageName, abort_condition: abortCondition, mappings: entries} as AbortStage);
  const statusCode = response.status;
  if (statusCode != 200) {
    refreshAbortStages();
    if (statusCode == 400) {
      setCurrentAbortStageErrorCode('ERROR : BAD REQUEST');
    } else if (statusCode == 418) {
      setCurrentAbortStageErrorCode("ERROR : I'M A TEAPOT");
    } else {
      setCurrentAbortStageErrorCode("ERROR CODE " + statusCode);
    }
    setSaveAbortStageDisplay("Error!");
    const ErrorMessage = await response.text();
    setCurrentAbortStageError(ErrorMessage);
    alert(currentAbortStageErrorCode() + "\n" + ErrorMessage);
    await new Promise(r => setTimeout(r, 1000));
    setSaveAbortStageDisplay("Save");
    return false;
  }

  setSaveAbortStageDisplay("Saved!");
  refreshAbortStages();

  await new Promise(r => setTimeout(r, 1000));

  setSaveAbortStageDisplay("Save");

  return true;
}

async function removeAbortStage(abortStageId: string) {
  const response = await deleteAbortStage(serverIp() as string, abortStageId);
  const statusCode = response.status;
  console.log('statusCode:', statusCode);
  if (statusCode != 200) {
    refreshAbortStages();
    setSaveAbortStageDisplay("Error!");
    await new Promise(r => setTimeout(r, 1000));
    setSaveAbortStageDisplay("Save");
    return;
  }
  setSubAbortStageDisplay('add');
  setSaveAbortStageDisplay("Deleted!");
  refreshAbortStages();
  await new Promise(r => setTimeout(r, 1000));
  setSaveAbortStageDisplay("Save");
}

function readAbortStageFile(e: any) {
  const file = e.target.files[0];
  const fr = new FileReader();

  fr.addEventListener("load", async e => {
    const json = JSON.parse(fr.result as string);
    console.log(json);
    console.log("In here")

    const newAbortStage: AbortStage = {
      id: json.stage_name,
      abort_condition: json.abort_condition,
      mappings: json.mappings
    };

    const success = await sendAbortStage(serverIp() as string, newAbortStage as AbortStage) as object;
    const statusCode = success['status' as keyof typeof success];
    if (statusCode != 200) {
      // Add a notification informing the upload failed
      refreshAbortStages();
      return;
    }
    refreshAbortStages();
  });

  fr.readAsText(file);
}

async function exportAbortStageToJsonFile(data: any, fileName: string) {
  console.log("Exporting json:");
  console.log("fileName:", fileName);
  console.log("data:", data);

  const transformedData = {
    stage_name: data.id,
    abort_condition: data.abort_condition,
    mappings: data.mappings.map((m: any) => ({
      valve_name: m.valve_name,
      abort_stage: m.abort_stage,
      timer_to_abort: m.timer_to_abort
    })),
  };

  const jsonString = JSON.stringify(transformedData, null, 2); 
  const path = await save({
    defaultPath: `${fileName}.json`,
    filters: [{ name: "JSON Files", extensions: ["json"] }],
  });

  if (path) {
    await writeTextFile(path, jsonString);
    console.log(`Saved file to: ${path}`);
  }
}

const AddAbortStageView: Component = (props) => {
  return <div style={{width: '100%'}}>
    <div class="add-config-section">
      <div class="add-config-setup">
        <p>Add New Abort Stage:</p>
        <input id='newabortstagename' class="add-config-input" type="text" placeholder="Name"/>
      </div>
      <div class="add-config-btns">
        <label for="file-upload" class="import-config">Import Abort Stage</label>
        <input id="file-upload" type="file" onChange={(e) => {readAbortStageFile(e);}}/>
        <button class="add-config-btn" onClick={addNewAbortStageEntry}>Insert Mapping</button>
        <button style={{"background-color": '#C53434'}} class="add-config-btn" onClick={function(event){
          setEditableAbortStageEntries([structuredClone(default_abort_stage_entry)]);
          clear_abort_stage_error();
        }}>Cancel</button>
        <button style={{"background-color": '#015878'}} class="add-config-btn" onClick={() => {submitAbortStage(false);}}>{saveAbortStageDisplay()}</button>
      </div>
    </div>
    <div class="add-config-section">
      <div class="add-config-setup">
        <p>Add Abort Condition:</p>
        <input id='newabortstagecondition' class="add-abort-input" type="text" placeholder="Abort Condition"/>
      </div>
    </div>
    <div class="horizontal-line"></div>
    <div style={{"margin-top": '5px', "margin-right": '20px'}} class="add-config-configurations">
      <div style={{width: '20%', "text-align": 'center'}}>Valve Name</div>
      <div style={{width: '20%', "text-align": 'center'}}>Abort Stage</div>
      <div style={{width: '65%', "text-align": 'center', color: "#e3bf47ff"}}>Timer to Abort</div>
    </div>
    <div style={{"max-height": '100%', "overflow-y": "auto"}}>
      <For each={editableAbortStageEntries()}>{(entry, i) =>
          <div class="add-abort-mappings">
            <input id={"addabortstagename"} type="text" value={entry.valve_name} placeholder="Valve Name" class="add-config-styling"/>
            <select name="" id={"addabortstage"} value={entry.abort_stage === null ? 'N/A' : entry.abort_stage.toUpperCase()} class="add-config-styling">
              <option class="seq-dropdown-item">N/A</option>
              <option class="seq-dropdown-item">OPEN</option>
              <option class="seq-dropdown-item">CLOSED</option>
            </select>
            <input type="text" name="" id={"addabortstagetimermin"} value={Number.isNaN(entry.timer_to_abort)? "": (entry.timer_to_abort / (1000 * 60)).toFixed(0)} placeholder="Minutes" class="add-abort-styling"/>
            <input type="text" name="" id={"addabortstagetimersec"} value={Number.isNaN(entry.timer_to_abort)? "": ((entry.timer_to_abort / 1000) % 60).toFixed(0)} placeholder="Seconds" class="add-abort-styling"/>
            <input type="text" name="" id={"addabortstagetimermil"} value={Number.isNaN(entry.timer_to_abort)? "": entry.timer_to_abort % 1000} placeholder="Milliseconds" class="add-abort-styling"/>
            <div onClick={() => deleteAbortStageEntry(entry)}><Fa icon={faTrash} color='#C53434'/></div>
          </div>
        }
      </For>
    </div>
  </div>
}

function loadAbortStageEntries(index: number) {
  var entries: AbortStageMapping[] = [];
  (abortStages() as AbortStage[])[index].mappings.forEach( (value) => {
    entries.push({
      valve_name: value.valve_name,
      abort_stage: value.abort_stage,
      timer_to_abort: value.timer_to_abort
    });
  });
  setEditableAbortStageEntries(entries);
}

const EditAbortStageView: Component<{index: number}> = (props) => {
  var index = props.index;
  loadAbortStageEntries(index);
  console.log(editableAbortStageEntries);
  return <div style={{width: '100%'}}>
    <div class="add-config-section">
      <div class="add-config-setup">
        <p>Editing abort stage:</p>
        <div style={{"font-weight": "bold"}}>{(abortStages() as AbortStage[])[index].id}</div>
      </div>
      <div class="add-config-btns">
        <button class="add-config-btn" onClick={addNewAbortStageEntry}>Insert Mapping</button>
        <button style={{"background-color": '#C53434'}} class="add-config-btn" onClick={function(event){
          setEditableAbortStageEntries([structuredClone(default_abort_stage_entry)]);
          setSubAbortStageDisplay('view');
          clear_abort_stage_error();
        }}>Cancel</button>
        <button style={{"background-color": '#015878'}} class="add-config-btn" onClick={async () => {
          if (await submitAbortStage(true)) { 
            setSubAbortStageDisplay('view'); 
          }
        }}>{saveAbortStageDisplay()}</button>
      </div>
    </div>
    <div class="add-config-section">
      <div class="add-config-setup">
        <p>Edit Abort Condition:</p>
        <input id='newabortstagecondition' class="add-abort-input" type="text" placeholder="Abort Condition" value={(abortStages() as AbortStage[])[index].abort_condition}/>
      </div>
    </div>
    <div class="horizontal-line"></div>
    <div style={{"margin-top": '5px', "margin-right": '20px'}} class="add-config-configurations">
      <div style={{width: '20%', "text-align": 'center'}}>Valve Name</div>
      <div style={{width: '20%', "text-align": 'center'}}>Abort Stage</div>
      <div style={{width: '65%', "text-align": 'center', color: "#e3bf47ff"}}>Timer to Abort</div>
    </div>
    <div style={{"max-height": '100%', "overflow-y": "auto"}}>
      <For each={editableAbortStageEntries()}>{(entry, i) =>
          <div class="add-abort-mappings">
            <input id={"addabortstagename"} type="text" value={entry.valve_name} placeholder="Valve Name" class="add-config-styling"/>
            <select name="" id={"addabortstage"} value={entry.abort_stage === null ? 'N/A' : entry.abort_stage.toUpperCase()} class="add-config-styling">
              <option class="seq-dropdown-item">N/A</option>
              <option class="seq-dropdown-item">OPEN</option>
              <option class="seq-dropdown-item">CLOSED</option>
            </select>
            <input type="text" name="" id={"addabortstagetimermin"} value={Number.isNaN(entry.timer_to_abort)? "": (entry.timer_to_abort / (1000 * 60)).toFixed(0)} placeholder="Minutes" class="add-abort-styling"/>
            <input type="text" name="" id={"addabortstagetimersec"} value={Number.isNaN(entry.timer_to_abort)? "": ((entry.timer_to_abort / 1000) % 60).toFixed(0)} placeholder="Seconds" class="add-abort-styling"/>
            <input type="text" name="" id={"addabortstagetimermil"} value={Number.isNaN(entry.timer_to_abort)? "": entry.timer_to_abort % 1000} placeholder="Milliseconds" class="add-abort-styling"/>
            <div onClick={() => deleteAbortStageEntry(entry)}><Fa icon={faTrash} color='#C53434'/></div>
          </div>
        }
      </For>
    </div>
  </div>
}

const DisplayAbortStageView: Component<{index: number}> = (props) => {
  var index = props.index;
  refreshAbortStages();

  const handleClickOutside = (e: MouseEvent) => {
    const target = e.target as HTMLElement | null;
    if (target && !target.closest('.del-config-btn')) {
      setConfirmAbortStageDelete(false);
      document.removeEventListener('click', handleClickOutside);
    }
  }

  return <div style={{width: '100%'}}>
    <div class="add-config-section">
      <div class="add-config-setup">
        <p>Viewing Abort Stage:</p>
        <div style={{"font-weight": "bold"}}>{(abortStages() as AbortStage[])[index].id}</div>
      </div>
      <div class="add-config-btns">
      <button class="del-config-btn" onClick={async (e)=>{
        e.stopPropagation();
        if (confirmAbortStageDelete()) {
          await removeAbortStage((abortStages() as AbortStage[])[index].id);
          console.log((abortStages() as AbortStage[]).length);
          setConfirmAbortStageDelete(false);
          setAbortStageFocusIndex(prevIndex => prevIndex - 1 > 0 ? prevIndex - 1 : 0);
          document.removeEventListener('click', handleClickOutside);
        } else {
          setConfirmAbortStageDelete(true);
          document.addEventListener('click', handleClickOutside);
        }
      }}>{confirmAbortStageDelete() ? 'Confirm' : 'Delete'}</button>
      <button class="add-config-btn" onClick={()=>{exportAbortStageToJsonFile((abortStages() as AbortStage[])[index], (abortStages() as AbortStage[])[index].id);}}>Export Abort Stage</button>
      <button class="add-config-btn" onClick={()=>{setSubAbortStageDisplay('edit'); refreshAbortStages();}}>Edit</button>
      <button class="add-config-btn" onClick={()=>{
        setSubAbortStageDisplay('add');
        clear_abort_stage_error();
      }}>Exit</button>
      </div>
    </div>
    <div class="add-config-section">
      <div class="add-config-setup">
        <p>Viewing Abort Condition:</p>
        <div style={{"font-weight": "bold"}}>{(abortStages() as AbortStage[])[index].abort_condition}</div>
      </div>
    </div>
    <div class="horizontal-line"></div>
    <div style={{"margin-top": '5px'}} class="add-config-configurations">
      <div style={{width: '20%', "text-align": 'center'}}>Valve Name</div>
      <div style={{width: '20%', "text-align": 'center'}}>Abort Stage</div>
      <div style={{width: '60%', "text-align": 'center', color: "#e3bf47ff"}}>Timer to Abort</div>
    </div>
    <div style={{"max-height": '100%', "overflow-y": "auto"}}>
      <For each={(abortStages() as AbortStage[])[index].mappings}>{(entry, i) =>
        <div class="add-config-configurations">
          <div style={{width: '20%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.valve_name}</div>
          <div style={{width: '20%', "text-align": 'center', "font-family": 'RubikLight'}}>{entry.abort_stage === null? 'N/A': entry.abort_stage}</div>
          <div style={{width: '20%', "text-align": 'center', "font-family": 'RubikLight', color: "#e3bf47ff"}}>Minutes: {(Number(entry.timer_to_abort) / (1000 * 60)).toFixed(0)}</div>
          <div style={{width: '20%', "text-align": 'center', "font-family": 'RubikLight', color: "#e3bf47ff"}}>Seconds: {((Number(entry.timer_to_abort) / 1000) % 60).toFixed(0)}</div>
          <div style={{width: '20%', "text-align": 'center', "font-family": 'RubikLight', color: "#e3bf47ff"}}>Milliseconds: {Number(entry.timer_to_abort) % 1000}</div>
        </div>
        }
      </For>
    </div>
  </div>
}

const AbortStageView: Component = (props) => {
  setEditableAbortStageEntries([structuredClone(default_abort_stage_entry)]);
  refreshAbortStages();
  return <div class="config-view">
    <div style="text-align: center; font-size: 14px">ABORT STAGE</div>
    {/* <div class="system-config-page"> */}
      <div class="system-config-above-section">
        <div style={{display: "grid", "grid-template-columns": "100px 1fr 100px", width: '100%', "margin-bottom": '5px'}}>
          <div></div>
          <div style="text-align: center; font-size: 14px; font-family: 'Rubik'">Available Abort Stages</div>
          <button style={{"justify-content": "end"}} class="refresh-button" onClick={refreshAbortStages}>{refreshAbortStageDisplay()}</button>
        </div>
        
        <div class="horizontal-line"></div>
        <div class="existing-configs-sections">
          <div style={{height: "5px"}}></div>
          <div style={{"overflow-y": "auto", "max-height": '100px'}}>
            <For each={abortStages() as AbortStage[]}>{(abortStage, i) =>
                <div class="existing-config-row" onClick={()=>{if (subAbortStageDisplay() != 'view') {setSubAbortStageDisplay('view'); setAbortStageFocusIndex(i as unknown as number);}}}>
                  <span class="config-id">{abortStage.id}</span>
                </div>
              }
            </For>
          </div>
        </div>
      </div>
      <div class="new-config-section" style={{height: (windowHeight()-390) as any as string + "px"}}>
        <div innerText={(() => {
          return currentAbortStageErrorCode()
        })()} style={{color: '#bf5744', 'text-align' : 'center'}}>
        </div>

        {(() => {
          console.log('some display set');
          console.log(abortStageFocusIndex());
          if (subAbortStageDisplay() == 'add') {
            return <AddAbortStageView />;
          } else if (subAbortStageDisplay() == 'view') {
            return <DisplayAbortStageView index={abortStageFocusIndex()} />;
          } else if (subAbortStageDisplay() == 'edit') {
            return <EditAbortStageView index={abortStageFocusIndex()} />;
          } else {
            return <div>How did we get here??</div>;
          }
        })()}
      </div>
    {/* </div> */}
</div>
}

export {Connect, Feedsystem, ConfigView, Sequences, AbortStageView};