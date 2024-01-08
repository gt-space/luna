import { Component, createSignal, For, Show, onMount } from "solid-js";
import { invoke } from '@tauri-apps/api/tauri'
import { setServerIp, connect, isConnected, setIsConnected, setActivity, serverIp, activity, selfIp, selfPort, sessionId, forwardingId, State, Config, sendActiveConfig, setSessionId, setForwardingId, setSelfIp, setSelfPort, Mapping, sendSequence, Sequence } from "../comm";
import { turnOnLED, turnOffLED } from "../commands";
import { emit, listen } from '@tauri-apps/api/event'
import { appWindow } from "@tauri-apps/api/window";
import { DISCONNECT_ACTIVITY_THRESH } from "../appdata";
import { CodeMirror } from "@solid-codemirror/codemirror";
import { oneDark } from "@codemirror/theme-one-dark";
import { python } from "@codemirror/lang-python";

const [connectDisplay, setConnectDisplay] = createSignal("Connect");
const [connectionMessage, setConnectionMessage] = createSignal('');
const [showSessionId, setShowSessionId] = createSignal(false);
const [showForwardingId, setShowForwardingId] = createSignal(false);
const [feedsystem, setFeedsystem] = createSignal('Feedsystem_1');
const [activeConfig, setActiveConfig] = createSignal('Config_1');
const [configurations, setConfigurations] = createSignal();
const [currentSequnceText, setCurrentSequenceText] = createSignal('');
const [currentSequnceName, setCurrentSequenceName] = createSignal('');
const [sequences, setSequences] = createSignal();
//configurations()

// function to connect to the server + input validation
async function connectToServer() {
    setConnectDisplay("Connecting...");
    setConnectionMessage('');
  
    // getting the ip, username, and password from the relevant textfields
    var ip = (document.getElementsByName('server-ip')[0] as HTMLInputElement).value.trim();
    // var username = (document.getElementsByName('username')[0] as HTMLInputElement).value.trim();
    // var password = (document.getElementsByName('password')[0] as HTMLInputElement).value;
    var result = '';
  
    // presence check on username and password
    // if (username != '' && password != '') {
    //   result = await connect(ip, username, password);
    // } else {
    //   result = 'Please enter a username and password';
    // }
  
    result = await connect(ip) as string;
  
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
  setSequences((event.payload as State).sequences);
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
          {/* <input class="connect-textfield"
            type="text"
            name="username"
            placeholder="Username"
          />
          <input class="connect-textfield"
            type="password"
            name="password"
            placeholder="Password"
          /> */}
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

  

  const ConfigView: Component = (props) => {
    const [addConfigDisplay, setAddConfigDisplay] = createSignal(true);
    const [existingDataDisplay, setExistingDataDisplay] = createSignal(false);
    const [editConfigDisplay, setEditConfigDisplay] = createSignal(false);
    const [configNum, setConfigNum] = createSignal(-1);
    const [numAddMappings, setNumAddMappings] = createSignal(1);
    const [configObj, setConfigObj] = createSignal((configurations() as Config[])[configNum()]);

    

    function addExistingDataSection(clickedId: any) {
        const prevConfigNum = configNum();
        setConfigNum(clickedId);

        setConfigObj((configurations() as Config[])[configNum()]);

        console.log("ADD EXISTING");
        console.log(configObj());

        if (prevConfigNum != configNum()) {
            setAddConfigDisplay(false);
            setEditConfigDisplay(false);
            setExistingDataDisplay(true);
        } else {
            setAddConfigDisplay(!addConfigDisplay());
            //Done so that add config and existing data are never displayed at the same time
            setExistingDataDisplay(!addConfigDisplay());
    
            setEditConfigDisplay(false);
        }
    }
    
    function addMapping() {
      let prevConfig = Object.create(configObj());
      console.log("IN ADD MAPPING");
      prevConfig.mappings.push({
        text_id: "Name",
        board_id: "Board ID",
        channel_type: "Channel Type",
        channel: "Channel",
        computer: "Computer"
      })
      setConfigObj(prevConfig);
      console.log(configObj());
    }

    function removeMapping() {
      let prevConfig = Object.create(configObj());
      prevConfig.mappings.pop();

      setConfigObj(prevConfig);

      console.log(configObj());
    }

    return <div style="height: 100%">
         <div style="text-align: center; font-size: 14px">CONFIGURATION</div>
         <div class="system-config-page">
            <div class="system-connect-section">
                <div style="text-align: center; font-size: 14px; font-family: 'Rubik'">Existing Configurations</div>
                <div class="horizontal-line"></div>
                <div class="existing-configs-sections">
                    <div class="row">
                        <div>Name</div>
                        <div>Date</div>
                    </div>
                    <For each={configurations() as Config[]}>{(config, i) =>
                        <div class="row" onClick={() => addExistingDataSection(i)}>
                            <div class="row-subheadings" id={`row-name-${i}`}>{config.id}</div>
                            <div class="row-subheadings">Date</div>
                        </div>
                    }</For>
                </div>
            </div>


            {/* This is what is messing up the styling */}
            <div class="system-connect-section add-config-connect-section" style={`display: ${addConfigDisplay() ? 'flex' : 'none'}`}>
                <div class="add-config-section">
                    <div class="add-config-setup">
                        <p>Add new config:</p>
                        <input class="add-config-input" type="text" placeholder="Name"/>
                    </div>
                    <div class="add-config-btns">
                        <button class="add-config-cancel-btn">Cancel</button>
                        <button class="add-config-save-btn">Save</button>
                        <button class="add-config-add-mapping-btn" onClick={() => setNumAddMappings(numAddMappings() + 1)}>Add Mapping</button>
                        <button class="add-config-remove-mapping-btn" onClick={() => {if (numAddMappings() > 1) setNumAddMappings(numAddMappings() - 1)}}>Remove Mapping</button>
                    </div>
                </div>
                <div class="horizontal-line"></div>
                <For each={Array(numAddMappings()).fill(0)}>{(_, i) =>
                    <div class="add-config-configurations">
                        <input type="text" placeholder="Name" class="add-config-styling"/>
                        <input type="text" name="" id="" placeholder="Board ID" class="add-config-styling"/>
                        <select name="" id="" class="add-config-styling">
                            <option class="seq-dropdown-item" selected disabled hidden>Channel Type</option>
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
                            <option class="seq-dropdown-item" selected disabled hidden>Computer</option>
                            <option class="seq-dropdown-item">Flight</option>
                            <option class="seq-dropdown-item">Ground</option>
                        </select>
                    </div>
                }</For>
            </div>


            <div class="existing-data system-connect-section" style={`display: ${existingDataDisplay() ? 'flex' : 'none'}`}>
                <div>
                    <button class="add-config-add-btn" onClick={() => {setEditConfigDisplay(true); setExistingDataDisplay(false)}}>Edit</button>
                </div>
                <Show when={configNum() >= 0}>
                <div>
                    <h4 class="existing-data-name">{(configurations() as Config[])[configNum()].id}</h4>
                </div>
              
                <div class="data">    
                        <For each={(configurations() as Config[])[configNum()!].mappings}>{(mapping, i) =>
                            <><h4 class="data-child">Name: {mapping.text_id}</h4><h4 class="data-child">Board ID: {mapping.board_id}</h4><h4 class="data-child">Channel Type: {mapping.channel_type}</h4><h4 class="data-child">Channel: {mapping.channel}</h4><h4 class="data-child">Computer: {mapping.computer}</h4></>
                        }</For>
                </div>
                </Show>
            </div>

            <div class="system-connect-section edit-section" style={`display: ${editConfigDisplay() ? 'flex' : 'none'}`}>
                <Show when={configNum() >= 0}>
                <div class="editing-data">
                    <div class="add-config-section">
                    <div class="add-config-setup">
                        <p>Edit new config:</p>
                        <input class="add-config-input edit-config-input" type="text" value={(configurations() as Config[])[configNum()].id} onChange={(e) => {configObj().id = e.target.value; console.log(configObj())}}/>
                    </div>
                    <div class="add-config-btns">
                        <button class="add-config-add-btn" onClick={() => addMapping()}>Add Mapping</button>
                        <button class="add-config-remove-btn" onClick={() => removeMapping()}>Remove Mapping</button>
                        <button class="add-config-cancel-btn" onClick={() => {setEditConfigDisplay(false); setExistingDataDisplay(true)}}>Cancel</button>
                        <button class="add-config-save-btn">Save</button>
                    </div>
                    </div>
                    <div class="horizontal-line"></div>
                    <div class="add-config-configurations edit-config-configurations" id="config0">
                        <div class="edit-config-configurations-labels">
                          <h4>Text ID</h4> 
                          <h4>Board ID</h4>
                          <h4>Channel Type</h4>
                          <h4>Channel</h4>
                          <h4>Computer</h4>
                        </div>
                        <For each={configObj()?.mappings}>{(mapping, i) =>
                            <div>
                            <input type="text" placeholder={mapping.text_id} class="add-config-styling" id="name0" value={mapping.text_id} onChange={(e) => {configObj().mappings[i()].text_id = e.target.value; console.log(configObj())}}/>
                            <input type="text" name="" id="id0" value={mapping.board_id.toString()} class="add-config-styling" onChange={(e) => {configObj().mappings[i()].board_id = e.target.value as unknown as number; console.log(configObj())}}/>
                            <select name="" id="channelType0" class="add-config-styling" onChange={(e) => {configObj().mappings[i()].channel_type = e.target.value; console.log(configObj())}}>
                                <option class="seq-dropdown-item">Channel Type</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "gpio"}>GPIO</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "led"}>LED</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "rail_3v3"}>RAIL 3V3</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "rail_5V"}>RAIL 5V</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "rail_5V5"}>RAIL 5V5</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "rail_24V"}>RAIL 24V</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "current_loop"}>CURRENT LOOP</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "differential_signal"}>DIFFERENTIAL SIGNAL</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "tc"}>TC</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "rtd"}>RTD</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "valve"}>VALVE</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "valve_current"}>VALVE CURRENT</option>
                                <option class="seq-dropdown-item" selected={mapping.channel_type == "valve_voltage"}>VALVE VOLTAGE</option>
                            </select>
                            <input type="text" name="" id="channel0" value={mapping.channel.toString()} class="add-config-styling" onChange={(e) => {configObj().mappings[i()].channel = e.target.value as unknown as number; console.log(configObj())}}/>
                            <select name="" id="computer0" class="add-config-styling" onChange={(e) => {configObj().mappings[i()].computer = e.target.value; console.log(configObj())}}>
                                <option class="seq-dropdown-item">Computer</option>
                                <option class="seq-dropdown-item" selected={mapping.computer == "flight"}>Flight</option>
                                <option class="seq-dropdown-item" selected={mapping.computer == "ground"}>Ground</option>
                            </select>
                            </div>
                        }</For>
                    </div>
                </div>
                </Show>
            </div>
         </div>


        
    </div>

  }

// const libs = [
//   import('prismjs/components/prism-markup'),
//   import('prismjs/components/prism-python'),
// ]

const Sequences: Component = (props) => {
  return <div style="height: 100%">
    <div style="text-align: center; font-size: 14px">SEQUENCES</div>
    <div class="system-sequences-page">
      <div class="sequences-list-view">
        Available Sequences:
        <div>
          <For each={sequences() as Sequence[]}>{(seq, i) =>
              <div class="sequence-display-item">
                {seq.name}
              </div>
            }
          </For>
        </div>
      </div>
      <div class="sequences-editor">
        <div style={{display: "grid", "grid-template-columns": "300px 1fr", height: '50px'}}>
          <input class="connect-textfield"
            type="text"
            name="sequence-name"
            placeholder="Sequence Name"
            value={currentSequnceName()}
            onInput={(event) => setCurrentSequenceName(event.currentTarget.value)}
          style={{width: '200px'}}/>
          <div style={{width: '100%'}}><button style={{float: "right"}} class="submit-feedsystem-button" onClick={() => sendSequence(serverIp() as string, currentSequnceName(), btoa(currentSequnceText()))}> Submit Sequence </button></div>
        </div>
        <div class="code-editor">
          <CodeMirror style={{height: '100%'}} value={currentSequnceText()} onValueChange={(value) => {setCurrentSequenceText(value);}} extensions={[python()]} theme={oneDark}/>
        </div>
      </div>
    </div>
</div>
}

export {Connect, Feedsystem, ConfigView, Sequences};