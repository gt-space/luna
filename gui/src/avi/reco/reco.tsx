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

function defaultRecoData(): RECO_struct {
  return {
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
    vref_ch1_dr1: 0.0,
    vref_ch1_dr2: 0.0,
    vref_ch2_dr1: 0.0,
    vref_ch2_dr2: 0.0,
    sns1_current: 0.0,
    sns2_current: 0.0,
    v_rail_24v: 0.0,
    v_rail_3v3: 0.0,
    stage1_enabled: false,
    stage2_enabled: false,
    reco_recvd_launch: false,
    reco_driver_faults: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ekf_blown_up: false,
    drouge_timer_enable: false,
    main_timer_enable: false,
  };
}

function formatVector(
  labels: string[],
  values: number[],
  decimals: number,
): string {
  return `[${labels.map((label, index) =>
    `${label}: ${formatRecoNumber(values[index], decimals)}`
  ).join(", ")}]`;
}

const [recoDataA, setRecoDataA] = createSignal(defaultRecoData());
const [recoDataB, setRecoDataB] = createSignal(defaultRecoData());
const [recoDataC, setRecoDataC] = createSignal(defaultRecoData());
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
// listens to device updates and updates the RECO values accordingly for display
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
      <div class="reco-bottom-container">
        {RecoSharedDataContainer()}
      </div>
    </div>
    <div>
      <Footer />
    </div>
  </div>
}

function RecoDataContainer(mcuNum: number) {
  let letter = "A";
  let recoData = recoDataA() as RECO_struct;
  if (mcuNum == 1) {
    letter = "B";
    recoData = recoDataB() as RECO_struct;
  } else if (mcuNum == 2) {
    letter = "C";
    recoData = recoDataC() as RECO_struct;
  }

  const rows = [
    { label: "Vehicle Attitude", value: formatVector(["W", "X", "Y", "Z"], recoData.quaternion, 4) },
    { label: "Position", value: formatVector(["LON", "LAT", "ALT"], recoData.lla_pos, 4) },
    { label: "Velocity", value: formatVector(["N", "E", "D"], recoData.velocity, 4) },
    { label: "Gyroscope Bias", value: formatVector(["X", "Y", "Z"], recoData.g_bias, 4) },
    { label: "Accelerometer Bias", value: formatVector(["X", "Y", "Z"], recoData.a_bias, 4) },
    { label: "Gyroscope Scale", value: formatVector(["X", "Y", "Z"], recoData.g_sf, 4) },
    { label: "Acceleration Scale", value: formatVector(["X", "Y", "Z"], recoData.a_sf, 4) },
    { label: "IMU Accelerometer", value: formatVector(["X", "Y", "Z"], recoData.lin_accel, 4) },
    { label: "IMU Gyroscope", value: formatVector(["X", "Y", "Z"], recoData.angular_rate, 4) },
    { label: "Magnetometer", value: formatVector(["X", "Y", "Z"], recoData.mag_data, 4) },
    { label: "Barometer Pressure", value: formatRecoNumber(recoData.pressure, 4) },
    { label: "Barometer Temperature", value: formatRecoNumber(recoData.temperature, 4) },
  ];

  const booleans = [
    { label: "Stage 1 Enabled", value: recoData.stage1_enabled },
    { label: "Stage 2 Enabled", value: recoData.stage2_enabled },
    { label: "RECO Recvd Launch", value: recoData.reco_recvd_launch },
    { label: "EKF Blown Up", value: recoData.ekf_blown_up },
    { label: "Drogue Timer Enable", value: recoData.drouge_timer_enable },
    { label: "Main Timer Enable", value: recoData.main_timer_enable },
  ];

  return <div class="reco-data-container">
    <div class="section-title"> MCU {letter} </div>
    <div class="column-title-row"></div>
    <div class="reco-data-row-container">
      {rows.map((row) => (
        <div class="reco-data-row">
          <div class="reco-data-variable"> {row.label}: </div>
          <div class="reco-data-value"> {row.value} </div>
        </div>
      ))}
      {booleans.map((row) => (
        <div class="reco-data-row">
          <div class="reco-data-variable"> {row.label}: </div>
          <div class="reco-data-value"> {renderBoolean(row.value)} </div>
        </div>
      ))}
    </div>
  </div>;
}

function RecoSharedDataContainer() {
  const recoData = recoDataA() as RECO_struct;
  const driverLabels = ["A", "B", "C", "D", "E"];
  const driverFaultBoxes = driverLabels.map((driver, index) => {
    const ch1 = recoData.reco_driver_faults[index * 2] ?? 0;
    const ch2 = recoData.reco_driver_faults[index * 2 + 1] ?? 0;
    return {
      title: `Driver ${driver} Faults`,
      value: `CH1: ${ch1}  CH2: ${ch2}`,
    };
  });

  return <div class="reco-shared-section">
    <div class="reco-shared-note">All bottom hardware readings are from MCU A.</div>
    <div class="reco-shared-grid">
      <div class="reco-shared-data-container">
        <div class="section-title"> Power </div>
        <div class="column-title-row"></div>
        <div class="reco-data-row-container">
          <div class="reco-data-row">
            <div class="reco-data-variable"> 24V Rail: </div>
            <div class="reco-data-value"> {formatRecoNumber(recoData.v_rail_24v, 4)} </div>
          </div>
          <div class="reco-data-row">
            <div class="reco-data-variable"> 3.3V Rail: </div>
            <div class="reco-data-value"> {formatRecoNumber(recoData.v_rail_3v3, 4)} </div>
          </div>
        </div>
      </div>
      {driverFaultBoxes.map((box) => (
        <div class="reco-shared-data-container">
          <div class="section-title"> {box.title} </div>
          <div class="column-title-row"></div>
          <div class="reco-data-row-container reco-shared-box-body">
            <div class="reco-shared-fault-value"> {box.value} </div>
          </div>
        </div>
      ))}
    </div>
  </div>;
}

export default RECO;
