import { For, Show, createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { StreamState, FCSensors as FcSensors_struct } from "../../comm";

const [sensorReadings, setSensorReadings] = createSignal<StreamState["sensor_readings"]>({});
const lidarStatus: Record<string, string> = {
  not_sampled: "Waiting for data",
  at_floor: "Low signal",
  saturated_high: "Overexposed",
  out_of_range: "Out of range",
  stale: "Stale",
};
const [fcSensorsData, setFcSensorsData] = createSignal({
  barometer: { pressure: 0, temperature: 0 },
  imu: {
    accelerometer: { x: 0, y: 0, z: 0 },
    gyroscope: { x: 0, y: 0, z: 0 }
  },
  adc: {
    rail_3v3: { voltage: 0, current: 0 },
    rail_5v: { voltage: 0, current: 0 },
    current_loop_pt: 0,
  },
  magnetometer: { x: 0, y: 0, z: 0 },
} as FcSensors_struct);
// listens to device updates and updates the values of FC Sensor values accordingly for display
listen('device_update', (event) => {
  const state = event.payload as StreamState;
  setFcSensorsData(state.fc_sensors);
  setSensorReadings(state.sensor_readings);
});

invoke('initialize_state', { window: appWindow });

function FcSensors() {
  const lidarIds = () => [1, 2, 3, 4].filter((id) => sensorReadings()[`fc_lidar_${id}_distance`]);

  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="FC Sensors" />
    </div>
    <div class="fc-sensors-view">
      <div class="fc-sensors-horizontal-container">
        <div class="fc-sensors-data-container">
          <div class="section-title" style={{ "text-decoration": 'underline' }}> IMU </div>
          <div class="column-title-row">
            <div class="column-title" style={{ "font-size": "14px" }}> Variables </div>
            <div class="column-title" style={{ "font-size": "14px" }}> Values </div>
          </div>
          <div class="fc-sensors-data-row-container">
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Accelerometer: x </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.accelerometer.x).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Accelerometer: y </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.accelerometer.y).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Accelerometer: z </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.accelerometer.z).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Gyroscope: x </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.gyroscope.x).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Gyroscope: y </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.gyroscope.y).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Gyroscope: z </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).imu.gyroscope.z).toFixed(4)} </div>
            </div>
          </div>
        </div>

        <div class="fc-sensors-data-container">
          <div class="section-title" style={{ "text-decoration": 'underline' }}> Barometer </div>
          <div class="column-title-row">
            <div class="column-title" style={{ "font-size": "14px" }}> Variables </div>
            <div class="column-title" style={{ "font-size": "14px" }}> Values </div>
          </div>
          <div class="fc-sensors-data-row-container">
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Barometer: Pressure </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).barometer.pressure).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Barometer: Temperature </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).barometer.temperature).toFixed(4)} </div>
            </div>
          </div>
        </div>
      </div>

      <div class="fc-sensors-horizontal-container">
        <div class="fc-sensors-data-container fc-sensors-data-container--title-gap">
          <div class="section-title" style={{ "text-decoration": 'underline' }}> Magnetometer </div>
          <div class="column-title-row">
            <div class="column-title" style={{ "font-size": "14px" }}> Variables </div>
            <div class="column-title" style={{ "font-size": "14px" }}> Values </div>
          </div>
          <div class="fc-sensors-data-row-container">
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Magnetometer: x </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).magnetometer.x).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Magnetometer: y </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).magnetometer.y).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> Magnetometer: z </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).magnetometer.z).toFixed(4)} </div>
            </div>
          </div>
        </div>

        <div class="fc-sensors-data-container fc-sensors-data-container--title-gap">
          <div class="section-title" style={{ "text-decoration": 'underline' }}> Volt Rails </div>
          <div class="column-title-row">
            <div class="column-title" style={{ "font-size": "14px" }}> Variables </div>
            <div class="column-title" style={{ "font-size": "14px" }}> Values </div>
          </div>
          <div class="fc-sensors-data-row-container">
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> 5V Rail Voltage </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).adc.rail_5v.voltage).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> 5V Rail Current </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).adc.rail_5v.current).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> 3.3V Rail Voltage </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).adc.rail_3v3.voltage).toFixed(4)} </div>
            </div>
            <div class="fc-sensors-data-row">
              <div class="fc-sensors-data-variable"> 3.3V Rail Current </div>
              <div class="fc-sensors-data-value"> {((fcSensorsData() as FcSensors_struct).adc.rail_3v3.current).toFixed(4)} </div>
            </div>
          </div>
        </div>
      </div>
      <div class="fc-sensors-horizontal-container" style={{ flex: "0 0 auto" }}>
        <section class="fc-sensors-data-container" aria-labelledby="lidar-heading">
          <div class="section-title" id="lidar-heading" style={{ "text-decoration": "underline" }}>LiDAR</div>
          <div class="fc-sensors-data-row-container">
            <Show when={lidarIds().length} fallback={<div class="fc-sensors-data-value">Waiting for Umbilical LiDAR telemetry</div>}>
              <For each={lidarIds()}>{(id) => {
                const prefix = `fc_lidar_${id}`;
                const distance = () => sensorReadings()[`${prefix}_distance`];
                const signal = () => sensorReadings()[`${prefix}_signal_strength`];
                const temperature = () => sensorReadings()[`${prefix}_temperature`];

                return (
                  <div class="fc-sensors-data-row">
                    <div class="fc-sensors-data-variable">LiDAR {id}</div>
                    <div class="fc-sensors-data-value">
                      {distance()?.quality === "ok" && distance().raw_count != null
                        ? `${(distance().value / 1000).toFixed(3)} m`
                        : (lidarStatus[distance()?.quality] ?? "Invalid")}
                      {signal()?.raw_count != null && ` · Signal ${signal().value}`}
                      {temperature()?.raw_count != null && ` · ${temperature().value.toFixed(1)} °C`}
                    </div>
                  </div>
                );
              }}</For>
            </Show>
          </div>
        </section>
      </div>
    </div>
    <div>
      <Footer />
    </div>
  </div>
}

export default FcSensors;
