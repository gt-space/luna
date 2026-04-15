import { createSignal } from "solid-js";
import Footer from "../../general-components/Footer";
import { GeneralTitleBar } from "../../general-components/TitleBar";
import { listen } from "@tauri-apps/api/event";
import { invoke } from "@tauri-apps/api/tauri";
import { appWindow } from "@tauri-apps/api/window";
import { BMS, RBFState, StreamState } from "../../comm";
import {
  formatBmsRbf,
  formatEstop,
  formatRecoRbf,
  formatSamRbf,
} from "../rbfDisplay";
import { RbfStatusValue } from "../RbfStatusValue";

const [rbfData, setRbfData] = createSignal({
  bms: 0,
  reco: [0, 0, 0],
  sam: {},
} as RBFState);
const [bmsData, setBmsData] = createSignal({
  battery_bus: { voltage: 0, current: 0 },
  umbilical_bus: { voltage: 0, current: 0 },
  sam_power_bus: { voltage: 0, current: 0 },
  ethernet_bus: { voltage: 0, current: 0 },
  tel_bus: { voltage: 0, current: 0 },
  fcb_bus: { voltage: 0, current: 0 },
  five_volt_rail: { voltage: 0, current: 0 },
  charger: 0,
  chassis: 0,
  e_stop: 0,
  rbf_tag: 0,
  reco_load_switch_1: 0,
  reco_load_switch_2: 0,
} as BMS);

listen("device_update", (event) => {
  const payload = event.payload as StreamState;
  setRbfData(payload.rbf);
  setBmsData(payload.bms);
});

invoke("initialize_state", { window: appWindow });

function RBF() {
  const samBoards = () =>
    Object.entries((rbfData().sam || {}) as Record<string, number>).sort(
      ([left], [right]) => left.localeCompare(right),
    );

  return <div class="window-template">
    <div style="height: 60px">
      <GeneralTitleBar name="RBF" />
    </div>
    <div style={{
      padding: "10px 12px",
      display: "flex",
      "justify-content": "center",
      "align-items": "flex-start",
      overflow: "auto",
      "font-size": "11px",
      "line-height": "1.35",
    }}>
      <div
        class="rbf-panel"
        style={{
        width: "100%",
        "max-width": "640px",
        background: "#333333",
        border: "1px solid #212121",
        "border-radius": "4px",
        padding: "10px 12px 12px 12px",
        "box-sizing": "border-box",
      }}
      >
        <div
          class="section-title"
          style={{
            "text-decoration": "underline",
            "text-align": "center",
            "margin-bottom": "10px",
            "font-size": "12px",
            padding: "0",
          }}
        >
          RBF Status
        </div>
        <div style={{
          display: "grid",
          "grid-template-columns": "repeat(auto-fit, minmax(160px, 1fr))",
          gap: "8px",
          "align-items": "start",
        }}>
          <div class="bms-data-group" style={{ margin: "0", width: "auto", "min-height": "0" }}>
            <div class="bms-data-group-title" style={{ "font-size": "11px" }}>BMS</div>
            <div class="adc-data-row">
              <div class="adc-data-variable">RBF tag</div>
              <RbfStatusValue text={formatBmsRbf(rbfData().bms)} />
            </div>
            <div class="adc-data-row">
              <div class="adc-data-variable">E-stop</div>
              <RbfStatusValue text={formatEstop(bmsData().e_stop)} />
            </div>
          </div>

          <div class="bms-data-group" style={{ margin: "0", width: "auto", "min-height": "0" }}>
            <div class="bms-data-group-title" style={{ "font-size": "11px" }}>RECO</div>
            <div class="adc-data-row">
              <div class="adc-data-variable">MCU A</div>
              <RbfStatusValue text={formatRecoRbf(rbfData().reco[0])} />
            </div>
            <div class="adc-data-row">
              <div class="adc-data-variable">MCU B</div>
              <RbfStatusValue text={formatRecoRbf(rbfData().reco[1])} />
            </div>
            <div class="adc-data-row">
              <div class="adc-data-variable">MCU C</div>
              <RbfStatusValue text={formatRecoRbf(rbfData().reco[2])} />
            </div>
          </div>

          <div class="bms-data-group" style={{ margin: "0", width: "auto", "min-height": "0" }}>
            <div class="bms-data-group-title" style={{ "font-size": "11px" }}>SAM</div>
            {samBoards().length > 0 ? samBoards().map(([boardId, value]) => (
              <div class="adc-data-row">
                <div class="adc-data-variable">{boardId.toUpperCase()}</div>
                <RbfStatusValue text={formatSamRbf(value)} />
              </div>
            )) : (
              <div class="adc-data-row">
                <div class="adc-data-variable">Boards</div>
                <div class="adc-data-value">N/A</div>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
    <div>
      <Footer />
    </div>
  </div>;
}

export default RBF;