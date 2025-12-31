import { createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { StreamState, RECO as RECO_struct, GPS as GPS_struct } from "../../comm";

const [recoDataA, setRecoDataA] = createSignal({
  quaternion: [1.0, 0.0, 0.0, 0.0],
  lla_pos: [0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0],
  g_bias: [0.0, 0.0, 0.0],
  a_bias: [0.0, 0.0, 0.0],
  g_sf: [1.0, 1.0, 1.0],
  a_sf: [1.0, 1.0, 1.0],
  lin_accel: [0.0, 0.0, 0.0],
  angular_rate: [0.0, 0.0, 0.0],
  mag_data: [0.0, 0.0, 0.0],
  temperature: 0.0,
  pressure: 0.0,
  stage1_enabled: false,
  stage2_enabled: false,
  vref_a_stage1: false,
  vref_a_stage2: false,
  vref_b_stage1: false,
  vref_b_stage2: false,
  vref_c_stage1: false,
  vref_c_stage2: false,
  vref_d_stage1: false,
  vref_d_stage2: false,
  vref_e_stage1_1: false,
  vref_e_stage1_2: false,
} as RECO_struct);
const [recoDataB, setRecoDataB] = createSignal({
  quaternion: [1.0, 0.0, 0.0, 0.0],
  lla_pos: [0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0],
  g_bias: [0.0, 0.0, 0.0],
  a_bias: [0.0, 0.0, 0.0],
  g_sf: [1.0, 1.0, 1.0],
  a_sf: [1.0, 1.0, 1.0],
  lin_accel: [0.0, 0.0, 0.0],
  angular_rate: [0.0, 0.0, 0.0],
  mag_data: [0.0, 0.0, 0.0],
  temperature: 0.0,
  pressure: 0.0,
  stage1_enabled: false,
  stage2_enabled: false,
  vref_a_stage1: false,
  vref_a_stage2: false,
  vref_b_stage1: false,
  vref_b_stage2: false,
  vref_c_stage1: false,
  vref_c_stage2: false,
  vref_d_stage1: false,
  vref_d_stage2: false,
  vref_e_stage1_1: false,
  vref_e_stage1_2: false,
} as RECO_struct);
const [recoDataC, setRecoDataC] = createSignal({
  quaternion: [1.0, 0.0, 0.0, 0.0],
  lla_pos: [0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0],
  g_bias: [0.0, 0.0, 0.0],
  a_bias: [0.0, 0.0, 0.0],
  g_sf: [1.0, 1.0, 1.0],
  a_sf: [1.0, 1.0, 1.0],
  lin_accel: [0.0, 0.0, 0.0],
  angular_rate: [0.0, 0.0, 0.0],
  mag_data: [0.0, 0.0, 0.0],
  temperature: 0.0,
  pressure: 0.0,
  stage1_enabled: false,
  stage2_enabled: false,
  vref_a_stage1: false,
  vref_a_stage2: false,
  vref_b_stage1: false,
  vref_b_stage2: false,
  vref_c_stage1: false,
  vref_c_stage2: false,
  vref_d_stage1: false,
  vref_d_stage2: false,
  vref_e_stage1_1: false,
  vref_e_stage1_2: false,
} as RECO_struct);
const [gpsData, setGpsData] = createSignal({
  latitude_deg: 0.0,
  longitude_deg: 0.0,
  altitude_m: 0.0,
  north_mps: 0.0,
  east_mps: 0.0,
  down_mps: 0.0,
  timestamp_unix_ms: 0.0,
  has_fix: true,
  num_satellites: 0,
} as GPS_struct);
// listens to device updates and updates the values of AHRS values accordingly for display
listen('device_update', (event) => {
  // get sensor data
  const reco_object = (event.payload as StreamState).reco;
  const gps_object = (event.payload as StreamState).gps;
  console.log(event.payload);
  console.log(reco_object);
  console.log(gps_object);
  if (reco_object[0] != undefined) {
    setRecoDataA(reco_object[0]);
  }
  if (reco_object[1] != undefined) {
    setRecoDataB(reco_object[1]);
  }
  if (reco_object[2] != undefined) {
    setRecoDataC(reco_object[2]);
  }
  if (gps_object != undefined) {
    setGpsData(gps_object);
  }
});

invoke('initialize_state', {window: appWindow});

function RECO() {
  return <div class="window-template">
  <div style="height: 60px">
    <GeneralTitleBar name="RECO"/>
  </div>
  <div class="reco-view">
    <div class="reco-top-container">
      <div class="reco-data-container-row">
        <div class="reco-gps-center">
          <div style={{ "font-size": '18px' }}> GPS </div>
        </div>
        <div class="row-title-column"></div>

        <div class="reco-gps-column">
          <div class="reco-gps-data-container">
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Latitude: </div>
              <div class="reco-gps-value"> {((gpsData() as GPS_struct).latitude_deg).toFixed(7)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Longitude: </div>
              <div class="reco-gps-value"> {((gpsData() as GPS_struct).longitude_deg).toFixed(7)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Altitude: </div>
              <div class="reco-gps-value"> {((gpsData() as GPS_struct).altitude_m).toFixed(4)} </div>
            </div>
          </div>

          <div class="reco-gps-data-container">
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Velocity North: </div>
              <div class="reco-gps-value"> {((gpsData() as GPS_struct).north_mps).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Velocity East: </div>
              <div class="reco-gps-value"> {((gpsData() as GPS_struct).east_mps).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Velocity Down: </div>
              <div class="reco-gps-value"> {((gpsData() as GPS_struct).down_mps).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Satellites Connected: </div>
              <div class="reco-gps-value"> {((gpsData() as GPS_struct).num_satellites).toFixed(0)} </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="reco-horizontal-container">
      {RecoDataContainer(0)}
      {RecoDataContainer(1)}
      {RecoDataContainer(2)}
    </div>
  </div>
  <div>
    <Footer/>
  </div>
</div>
}

function RecoDataContainer(mcuNum: number) {
  var letter = "A";
  var recoData = recoDataA() as RECO_struct;
  if (mcuNum == 1) {
    letter = "B";
    recoData = recoDataB() as RECO_struct;
  } else if (mcuNum == 2) {
    letter = "C";
    recoData = recoDataC() as RECO_struct;
  }

  return <div class="reco-data-container">
    <div class="section-title"> MCU {letter} </div>
    <div class="column-title-row"></div>
    <div class="reco-data-row-container">

      <div class="reco-data-row">
        <div class="reco-data-variable"> Vehicle Attitude: </div>
        <div class="reco-data-value"> [W: {(recoData.quaternion[0]).toFixed(4)}, X: {(recoData.quaternion[1]).toFixed(4)}, Y: {(recoData.quaternion[2]).toFixed(4)}, Z: {(recoData.quaternion[3]).toFixed(4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Position: </div>
        <div class="reco-data-value"> [LON: {(recoData.lla_pos[0]).toFixed(4)}, LAT: {(recoData.lla_pos[1]).toFixed(4)}, ALT: {(recoData.lla_pos[2]).toFixed(4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Velocity: </div>
        <div class="reco-data-value"> [N: {(recoData.velocity[0]).toFixed(4)}, E: {(recoData.velocity[1]).toFixed(4)}, D: {(recoData.velocity[2]).toFixed(4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Gyroscope Bias: </div>
        <div class="reco-data-value"> [X: {(recoData.g_bias[0]).toFixed(4)}, Y: {(recoData.g_bias[1]).toFixed(4)}, Z: {(recoData.g_bias[2]).toFixed(4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Accelerometer Bias: </div>
        <div class="reco-data-value"> [X: { (recoData.a_bias[0]).toFixed(4)}, Y: {(recoData.a_bias[1]).toFixed(4)}, Z: {(recoData.a_bias[2]).toFixed(4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Gyroscope Scale: </div>
        <div class="reco-data-value"> [X: { (recoData.g_sf[0]).toFixed(4)}, Y: {(recoData.g_sf[1]).toFixed(4)}, Z: {(recoData.g_sf[2]).toFixed(4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Acceleration Scale: </div>
        <div class="reco-data-value"> [X: { (recoData.a_sf[0]).toFixed(4)}, Y: {(recoData.a_sf[1]).toFixed(4)}, Z: {(recoData.a_sf[2]).toFixed(4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> IMU Accelerometer: </div>
        <div class="reco-data-value"> [X: { (recoData.lin_accel[0]).toFixed(4)}, Y: {(recoData.lin_accel[1]).toFixed(4)}, Z: {(recoData.lin_accel[2]).toFixed(4)}] </div>
      </div>
      <div class="reco-data-row">
        <div class="reco-data-variable"> IMU Gyroscope: </div>
        <div class="reco-data-value"> [X: { (recoData.angular_rate[0]).toFixed(4)}, Y: {(recoData.angular_rate[1]).toFixed(4)}, Z: {(recoData.angular_rate[2]).toFixed(4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Magnetometer: </div>
        <div class="reco-data-value"> [X: { (recoData.mag_data[0]).toFixed(4)}, Y: {(recoData.mag_data[1]).toFixed(4)}, Z: {(recoData.mag_data[2]).toFixed(4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Barometer Pressure: </div>
        <div class="reco-data-value"> {(recoData.pressure).toFixed(4)} </div>
      </div>
      <div class="reco-data-row">
        <div class="reco-data-variable"> Barometer Temperature: </div>
        <div class="reco-data-value"> {(recoData.temperature).toFixed(4)} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Stage 1 Enabled: </div>
        <div class="reco-data-value"> {(recoData.stage1_enabled).toString()} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Stage 2 Enabled: </div>
        <div class="reco-data-value"> {(recoData.stage2_enabled).toString()} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF A Stage 1: </div>
        <div class="reco-data-value"> {(recoData.vref_a_stage1).toString()} | Stage 2: </div>
        <div class="reco-data-value"> {(recoData.vref_a_stage2).toString()} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF B Stage 1: </div>
        <div class="reco-data-value"> {(recoData.vref_b_stage1).toString()} | Stage 2: </div>
        <div class="reco-data-value"> {(recoData.vref_b_stage2).toString()} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF C Stage 1: </div>
        <div class="reco-data-value"> {(recoData.vref_c_stage1).toString()} | Stage 2: </div>
        <div class="reco-data-value"> {(recoData.vref_c_stage2).toString()} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF D Stage 1: </div>
        <div class="reco-data-value"> {(recoData.vref_d_stage1).toString()} | Stage 2: </div>
        <div class="reco-data-value"> {(recoData.vref_d_stage2).toString()} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF E Stage 1-1: </div>
        <div class="reco-data-value"> {(recoData.vref_e_stage1_1).toString()} | Stage 1-2: </div>
        <div class="reco-data-value"> {(recoData.vref_e_stage1_2).toString()} </div>
      </div>
    </div>
  </div>;
}

export default RECO;
