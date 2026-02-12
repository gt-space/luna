import { createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { RecoFlasher as RecoFlasher_struct } from "../../comm";

// Float32 range: approximately ±3.4 × 10^38
const MAX_FLOAT32 = 3.4028234663852886e+38;
const MIN_FLOAT32 = -3.4028234663852886e+38;
const [recoFlasherDataA, setRecoFlasherDataA] = createSignal({
  quaternion: [0.0, 0.0, 0.0, 0.0],
  lla_pos: [0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0],
  g_bias: [0.0, 0.0, 0.0],
  a_bias: [0.0, 0.0, 0.0],
  g_sf: [0.0, 0.0, 0.0],
  a_sf: [0.0, 0.0, 0.0],
  alt_off: 0.0,
  fil_off: 0.0,
} as RecoFlasher_struct);
const [recoFlasherDataB, setRecoFlasherDataB] = createSignal({
  quaternion: [0.0, 0.0, 0.0, 0.0],
  lla_pos: [0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0],
  g_bias: [0.0, 0.0, 0.0],
  a_bias: [0.0, 0.0, 0.0],
  g_sf: [0.0, 0.0, 0.0],
  a_sf: [0.0, 0.0, 0.0],
  alt_off: 0.0,
  fil_off: 0.0,
} as RecoFlasher_struct);
const [recoFlasherDataC, setRecoFlasherDataC] = createSignal({
  quaternion: [0.0, 0.0, 0.0, 0.0],
  lla_pos: [0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0],
  g_bias: [0.0, 0.0, 0.0],
  a_bias: [0.0, 0.0, 0.0],
  g_sf: [0.0, 0.0, 0.0],
  a_sf: [0.0, 0.0, 0.0],
  alt_off: 0.0,
  fil_off: 0.0,
} as RecoFlasher_struct);

function isFloat32(value: string): boolean {
  const num = parseFloat(value);
  
  if (isNaN(num)) return false;
  
  if (num > MAX_FLOAT32 || num < MIN_FLOAT32) {
    return false;
  }
  return true;
}

function validateFloat32(value: string, previousValue: string): string {
  if (value === "-" || value === "+") return value;

  // Count decimal points - only allow 1
  const decimalCount = (value.match(/\./g) || []).length;
  if (decimalCount > 1) {
    return previousValue;
  }
  
  // Check for + or - signs (only allowed as first character)
  for (let i = 1; i < value.length; i++) {
    if (value[i] === '+' || value[i] === '-') {
      return previousValue;
    }
  }
  
  // Only allow digits, +, -, and .
  const filtered = value.replace(/[^0-9+\-\.]/g, "");
  
  // If nothing changed after filtering, return previous value
  if (filtered === "") return previousValue;
  
  const num = parseFloat(filtered);
  
  if (isNaN(num)) return previousValue;
  
  if (num > MAX_FLOAT32 || num < MIN_FLOAT32) {
    return previousValue;
  }
  
  return filtered;
}

function flashData(mcuNum: number) {
  // flash data here
  console.log("MCU Flash Clicked: " + mcuNum.toString());
}

function RecoFlasher() {
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="RECO Flasher"/>
    </div>
    <div class="reco-view">
      <div class="reco-horizontal-container">
        {RecoFlasherDataContainer(0)}
        {RecoFlasherDataContainer(1)}
        {RecoFlasherDataContainer(2)}
      </div>
    </div>
    <div>
      <Footer />
    </div>
  </div>
}

function RecoFlasherDataContainer(mcuNum: number) {
  var letter = "A";
  var recoData = recoFlasherDataA() as RecoFlasher_struct;
  if (mcuNum == 1) {
    letter = "B";
    recoData = recoFlasherDataB() as RecoFlasher_struct;
  } else if (mcuNum == 2) {
    letter = "C";
    recoData = recoFlasherDataC() as RecoFlasher_struct;
  }

  return <div class="reco-data-container">
    <div class="section-title"> MCU {letter} </div>
    <div class="column-title-row"></div>
    <div class="reco-data-row-container">

      <div class="reco-flasher-data-row">
        <div class="reco-flasher-data-variable"> Attitude: </div>
        <div class="reco-flasher-input-row">
          <input id='reco-flasher-attitude-w' class="add-reco-flasher-input" type="text" placeholder="W" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-attitude-x' class="add-reco-flasher-input" type="text" placeholder="X" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-attitude-y' class="add-reco-flasher-input" type="text" placeholder="Y" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-attitude-z' class="add-reco-flasher-input" type="text" placeholder="Z" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
        </div>
      </div>

      <div class="reco-flasher-data-row">
        <div class="reco-flasher-data-variable"> Position: </div>
        <div class="reco-flasher-input-row">
          <input id='reco-flasher-position-lon' class="add-reco-flasher-input" type="text" placeholder="Lon" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-position-lat' class="add-reco-flasher-input" type="text" placeholder="Lat" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-position-alt' class="add-reco-flasher-input" type="text" placeholder="Alt" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
        </div>
      </div>

      <div class="reco-flasher-data-row">
        <div class="reco-flasher-data-variable"> Accel Bias: </div>
        <div class="reco-flasher-input-row">
          <input id='reco-flasher-accel-bias-x' class="add-reco-flasher-input" type="text" placeholder="X" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-accel-bias-y' class="add-reco-flasher-input" type="text" placeholder="Y" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-accel-bias-z' class="add-reco-flasher-input" type="text" placeholder="Z" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
        </div>
      </div>

      <div class="reco-flasher-data-row">
        <div class="reco-flasher-data-variable"> Gyro Bias: </div>
        <div class="reco-flasher-input-row">
          <input id='reco-flasher-gyro-bias-x' class="add-reco-flasher-input" type="text" placeholder="X" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-gyro-bias-y' class="add-reco-flasher-input" type="text" placeholder="Y" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-gyro-bias-z' class="add-reco-flasher-input" type="text" placeholder="Z" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
        </div>
      </div>

      <div class="reco-flasher-data-row">
        <div class="reco-flasher-data-variable"> Accel SF: </div>
        <div class="reco-flasher-input-row">
          <input id='reco-flasher-accel-sf-x' class="add-reco-flasher-input" type="text" placeholder="X" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-accel-sf-y' class="add-reco-flasher-input" type="text" placeholder="Y" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-accel-sf-z' class="add-reco-flasher-input" type="text" placeholder="Z" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
        </div>
      </div>

      <div class="reco-flasher-data-row">
        <div class="reco-flasher-data-variable"> Gyro SF: </div>
        <div class="reco-flasher-input-row">
          <input id='reco-flasher-gyro-sf-x' class="add-reco-flasher-input" type="text" placeholder="X" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-gyro-sf-y' class="add-reco-flasher-input" type="text" placeholder="Y" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
          <input id='reco-flasher-gyro-sf-z' class="add-reco-flasher-input" type="text" placeholder="Z" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
        </div>
      </div>

      <div class="reco-flasher-data-row">
        <div class="reco-flasher-data-variable"> Altimeter Offset: </div>
        <div class="reco-flasher-input-row">
          <input id='reco-flasher-altimeter-offset' class="add-reco-flasher-input" type="text" placeholder="Offset" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
        </div>
      </div>

      <div class="reco-flasher-data-row">
        <div class="reco-flasher-data-variable"> Filter Offset: </div>
        <div class="reco-flasher-input-row">
          <input id='reco-flasher-filter-offset' class="add-reco-flasher-input" type="text" placeholder="Offset" onInput={(e) => {const prev = e.currentTarget.value.slice(0, -1); e.currentTarget.value = validateFloat32(e.currentTarget.value, prev);}}/>
        </div>
      </div>

      <div class="flash-button-container">
        <button class="flash-button" onclick={() => flashData(mcuNum)}> FLASH </button>
      </div>
    </div>
  </div>;
}

export default RecoFlasher;
