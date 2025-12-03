import { For, createEffect, createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { Config, Sequence, State, runSequence, serverIp, StreamState, Bus, RECO as RECO_struct, GPS as GPS_struct, Vector } from "../../comm";
import { enableCommand, disableCommand } from "../../commands";

const [configurations, setConfigurations] = createSignal();
const [activeConfig, setActiveConfig] = createSignal();
const [activeBoards, setActiveBoards] = createSignal();
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

listen('state', (event) => {
  console.log(event.windowLabel);
  setConfigurations((event.payload as State).configs);
  setActiveConfig((event.payload as State).activeConfig);
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
              <div class="reco-data-value"> {((gpsData() as GPS_struct).latitude_deg).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Longitude: </div>
              <div class="reco-data-value"> {((gpsData() as GPS_struct).longitude_deg).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Altitude: </div>
              <div class="reco-data-value"> {((gpsData() as GPS_struct).altitude_m).toFixed(4)} </div>
            </div>
          </div>

          <div class="reco-gps-data-container">
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Velocity North: </div>
              <div class="reco-data-value"> {((gpsData() as GPS_struct).north_mps).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Velocity East: </div>
              <div class="reco-data-value"> {((gpsData() as GPS_struct).east_mps).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-gps-variable"> Velocity Down: </div>
              <div class="reco-data-value"> {((gpsData() as GPS_struct).down_mps).toFixed(4)} </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="reco-horizontal-container">
      <div class="reco-data-container">
        <div class="section-title"> MCU A </div>
        <div class="column-title-row"></div>
        <div class="reco-data-row-container">
          <div class="section-title" style={{"text-decoration": 'underline'}}> IMU </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Accelerometer: x </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).lin_accel[0]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Accelerometer: y </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).lin_accel[1]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Accelerometer: z </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).lin_accel[2]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Gyroscope: x </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).angular_rate[0]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Gyroscope: y </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).angular_rate[1]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Gyroscope: z </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).angular_rate[2]).toFixed(4)} </div>
          </div>

          <div class="section-title" style={{"text-decoration": 'underline'}}> Barometer </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Pressure: </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).pressure).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Temperature: </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).temperature).toFixed(4)} </div>
          </div>

          <div class="section-title" style={{"text-decoration": 'underline'}}> Magnetometer </div>
            <div class="reco-data-row">
              <div class="reco-data-variable"> Magnetometer: x </div>
              <div class="reco-data-value"> {((recoDataA() as RECO_struct).mag_data[0]).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-data-variable"> Magnetometer: y </div>
              <div class="reco-data-value"> {((recoDataA() as RECO_struct).mag_data[1]).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-data-variable"> Magnetometer: z </div>
              <div class="reco-data-value"> {((recoDataA() as RECO_struct).mag_data[2]).toFixed(4)} </div>
            </div>
        </div>
      </div>

      <div class="reco-data-container">
        <div class="section-title"> MCU B </div>
        <div class="column-title-row"></div>
        <div class="reco-data-row-container">
          <div class="section-title" style={{"text-decoration": 'underline'}}> IMU </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Accelerometer: x </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).lin_accel[0]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Accelerometer: y </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).lin_accel[1]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Accelerometer: z </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).lin_accel[2]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Gyroscope: x </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).angular_rate[0]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Gyroscope: y </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).angular_rate[1]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Gyroscope: z </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).angular_rate[2]).toFixed(4)} </div>
          </div>

          <div class="section-title" style={{"text-decoration": 'underline'}}> Barometer </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Pressure: </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).pressure).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Temperature: </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).temperature).toFixed(4)} </div>
          </div>

          <div class="section-title" style={{"text-decoration": 'underline'}}> Magnetometer </div>
            <div class="reco-data-row">
              <div class="reco-data-variable"> Magnetometer: x </div>
              <div class="reco-data-value"> {((recoDataA() as RECO_struct).mag_data[0]).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-data-variable"> Magnetometer: y </div>
              <div class="reco-data-value"> {((recoDataA() as RECO_struct).mag_data[1]).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-data-variable"> Magnetometer: z </div>
              <div class="reco-data-value"> {((recoDataA() as RECO_struct).mag_data[2]).toFixed(4)} </div>
            </div>
        </div>
      </div>

      <div class="reco-data-container">
        <div class="section-title"> MCU C </div>
        <div class="column-title-row"></div>
        <div class="reco-data-row-container">
          <div class="section-title" style={{"text-decoration": 'underline'}}> IMU </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Accelerometer: x </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).lin_accel[0]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Accelerometer: y </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).lin_accel[1]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Accelerometer: z </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).lin_accel[2]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Gyroscope: x </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).angular_rate[0]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Gyroscope: y </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).angular_rate[1]).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Gyroscope: z </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).angular_rate[2]).toFixed(4)} </div>
          </div>

          <div class="section-title" style={{"text-decoration": 'underline'}}> Barometer </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Pressure: </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).pressure).toFixed(4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> Temperature: </div>
            <div class="reco-data-value"> {((recoDataA() as RECO_struct).temperature).toFixed(4)} </div>
          </div>

          <div class="section-title" style={{"text-decoration": 'underline'}}> Magnetometer </div>
            <div class="reco-data-row">
              <div class="reco-data-variable"> Magnetometer: x </div>
              <div class="reco-data-value"> {((recoDataA() as RECO_struct).mag_data[0]).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-data-variable"> Magnetometer: y </div>
              <div class="reco-data-value"> {((recoDataA() as RECO_struct).mag_data[1]).toFixed(4)} </div>
            </div>
            <div class="reco-data-row">
              <div class="reco-data-variable"> Magnetometer: z </div>
              <div class="reco-data-value"> {((recoDataA() as RECO_struct).mag_data[2]).toFixed(4)} </div>
            </div>
        </div>
      </div>
    </div>
  </div>
  <div>
    <Footer/>
  </div>
</div>
}

export default RECO;
