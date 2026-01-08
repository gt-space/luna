import { createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { StreamState, RECO as RECO_struct, GPS as GPS_struct } from "../../comm";

function formatRecoNumber(value: unknown, decimals: number): string {
  if (value === null || value === undefined) {
    return "NaN";
  }
  const num = Number(value);
  if (!Number.isFinite(num)) {
    return "NaN";
  }
  return num.toFixed(decimals);
}

function renderBoolean(value: boolean) {
  return <span style={{ "color": value ? "#00FF00" : "inherit" }}>{value.toString()}</span>;
}

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

invoke('initialize_state', { window: appWindow });

function RECO() {
  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="RECO" />
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
      <Footer />
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
        <div class="reco-data-value"> [W: {formatRecoNumber(recoData.quaternion[0], 4)}, X: {formatRecoNumber(recoData.quaternion[1], 4)}, Y: {formatRecoNumber(recoData.quaternion[2], 4)}, Z: {formatRecoNumber(recoData.quaternion[3], 4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Position: </div>
        <div class="reco-data-value"> [LON: {formatRecoNumber(recoData.lla_pos[0], 4)}, LAT: {formatRecoNumber(recoData.lla_pos[1], 4)}, ALT: {formatRecoNumber(recoData.lla_pos[2], 4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Velocity: </div>
        <div class="reco-data-value"> [N: {formatRecoNumber(recoData.velocity[0], 4)}, E: {formatRecoNumber(recoData.velocity[1], 4)}, D: {formatRecoNumber(recoData.velocity[2], 4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Gyroscope Bias: </div>
        <div class="reco-data-value"> [X: {formatRecoNumber(recoData.g_bias[0], 4)}, Y: {formatRecoNumber(recoData.g_bias[1], 4)}, Z: {formatRecoNumber(recoData.g_bias[2], 4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Accelerometer Bias: </div>
        <div class="reco-data-value"> [X: {formatRecoNumber(recoData.a_bias[0], 4)}, Y: {formatRecoNumber(recoData.a_bias[1], 4)}, Z: {formatRecoNumber(recoData.a_bias[2], 4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Gyroscope Scale: </div>
        <div class="reco-data-value"> [X: {formatRecoNumber(recoData.g_sf[0], 4)}, Y: {formatRecoNumber(recoData.g_sf[1], 4)}, Z: {formatRecoNumber(recoData.g_sf[2], 4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Acceleration Scale: </div>
        <div class="reco-data-value"> [X: {formatRecoNumber(recoData.a_sf[0], 4)}, Y: {formatRecoNumber(recoData.a_sf[1], 4)}, Z: {formatRecoNumber(recoData.a_sf[2], 4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> IMU Accelerometer: </div>
        <div class="reco-data-value"> [X: {formatRecoNumber(recoData.lin_accel[0], 4)}, Y: {formatRecoNumber(recoData.lin_accel[1], 4)}, Z: {formatRecoNumber(recoData.lin_accel[2], 4)}] </div>
      </div>
      <div class="reco-data-row">
        <div class="reco-data-variable"> IMU Gyroscope: </div>
        <div class="reco-data-value"> [X: {formatRecoNumber(recoData.angular_rate[0], 4)}, Y: {formatRecoNumber(recoData.angular_rate[1], 4)}, Z: {formatRecoNumber(recoData.angular_rate[2], 4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Magnetometer: </div>
        <div class="reco-data-value"> [X: {formatRecoNumber(recoData.mag_data[0], 4)}, Y: {formatRecoNumber(recoData.mag_data[1], 4)}, Z: {formatRecoNumber(recoData.mag_data[2], 4)}] </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Barometer Pressure: </div>
        <div class="reco-data-value"> {formatRecoNumber(recoData.pressure, 4)} </div>
      </div>
      <div class="reco-data-row">
        <div class="reco-data-variable"> Barometer Temperature: </div>
        <div class="reco-data-value"> {formatRecoNumber(recoData.temperature, 4)} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Stage 1 Enabled: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.stage1_enabled)} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> Stage 2 Enabled: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.stage2_enabled)} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF A Stage 1: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_a_stage1)} | Stage 2: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_a_stage2)} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF B Stage 1: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_b_stage1)} | Stage 2: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_b_stage2)} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF C Stage 1: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_c_stage1)} | Stage 2: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_c_stage2)} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF D Stage 1: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_d_stage1)} | Stage 2: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_d_stage2)} </div>
      </div>

      <div class="reco-data-row">
        <div class="reco-data-variable"> VREF E Stage 1-1: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_e_stage1_1)} | Stage 1-2: </div>
        <div class="reco-data-value"> {renderBoolean(recoData.vref_e_stage1_2)} </div>
      </div>
    </div>
  </div>;
}

export default RECO;
