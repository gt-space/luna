import { Component, For, Show, createMemo, createSignal } from "solid-js";
import { createStore } from "solid-js/store";
import {
  RecoAltimeterOffsets,
  RecoEkfStateVector,
  RecoGuiCommandRequest,
  RecoGuiTarget,
  RecoInitialCovarianceMatrix,
  RecoMeasurementNoiseMatrix,
  RecoProcessNoiseMatrix,
  RecoTimerValues,
  isConnected,
  sendRecoGuiCommand,
  serverIp,
} from "../comm";

type MessageType = RecoGuiCommandRequest["message_type"];

type MatrixFieldSpec = {
  kind: "matrix";
  key: string;
  label: string;
  placeholders: string[];
};

type VectorFieldSpec = {
  kind: "vector";
  key: string;
  label: string;
  placeholders: string[];
};

type ScalarFieldSpec = {
  kind: "scalar";
  key: string;
  label: string;
  placeholder: string;
};

type ToggleFieldSpec = {
  kind: "toggle";
  key: string;
  label: string;
};

type FieldSpec =
  | MatrixFieldSpec
  | VectorFieldSpec
  | ScalarFieldSpec
  | ToggleFieldSpec;

type SectionSpec = {
  title: string;
  fields: FieldSpec[];
};

type MessageLayout = {
  label: string;
  description: string;
  rowMajorNote?: boolean;
  sections: SectionSpec[];
};

type SubmitTargetOption = {
  value: RecoGuiTarget;
  label: string;
  actionLabel: string;
};

const MATRIX_3X3_PLACEHOLDERS = [
  "r1c1",
  "r1c2",
  "r1c3",
  "r2c1",
  "r2c2",
  "r2c3",
  "r3c1",
  "r3c2",
  "r3c3",
];

const MESSAGE_OPTIONS: { value: MessageType; label: string }[] = [
  { value: "process_noise_matrix", label: "Process Noise Matrix" },
  { value: "measurement_noise_matrix", label: "Measurement Noise Matrix" },
  { value: "ekf_state_vector", label: "Initial State Vector" },
  { value: "initial_covariance_matrix", label: "Initial Covariance Matrix" },
  { value: "timer_values", label: "Timer Values" },
  { value: "altimeter_offsets", label: "Altimeter Offsets" },
];

const SUBMIT_TARGET_OPTIONS: SubmitTargetOption[] = [
  { value: "all", label: "all MCUs", actionLabel: "All MCUs" },
  { value: "a", label: "MCU A", actionLabel: "MCU A" },
  { value: "b", label: "MCU B", actionLabel: "MCU B" },
  { value: "c", label: "MCU C", actionLabel: "MCU C" },
];

const MESSAGE_LAYOUTS: Record<MessageType, MessageLayout> = {
  process_noise_matrix: {
    label: "Process Noise Matrix",
    description: "Send four 3x3 covariance matrices to RECO.",
    rowMajorNote: true,
    sections: [
      {
        title: "Process Noise Matrices",
        fields: [
          {
            kind: "matrix",
            key: "nu_gv_mat",
            label: "Gyro Covariance",
            placeholders: MATRIX_3X3_PLACEHOLDERS,
          },
          {
            kind: "matrix",
            key: "nu_gu_mat",
            label: "Gyro Bias Covariance",
            placeholders: MATRIX_3X3_PLACEHOLDERS,
          },
          {
            kind: "matrix",
            key: "nu_av_mat",
            label: "Accelerometer Covariance",
            placeholders: MATRIX_3X3_PLACEHOLDERS,
          },
          {
            kind: "matrix",
            key: "nu_au_mat",
            label: "Accelerometer Bias Covariance",
            placeholders: MATRIX_3X3_PLACEHOLDERS,
          },
        ],
      },
    ],
  },
  measurement_noise_matrix: {
    label: "Measurement Noise Matrix",
    description: "Send the GPS noise matrix and barometer noise value.",
    rowMajorNote: true,
    sections: [
      {
        title: "Measurement Noise",
        fields: [
          {
            kind: "matrix",
            key: "gps_noise_matrix",
            label: "GPS Noise Matrix",
            placeholders: MATRIX_3X3_PLACEHOLDERS,
          },
          {
            kind: "scalar",
            key: "barometer_noise",
            label: "Barometer Noise",
            placeholder: "Noise",
          },
        ],
      },
    ],
  },
  ekf_state_vector: {
    label: "Initial State Vector",
    description: "Send the initial EKF state vector to RECO.",
    sections: [
      {
        title: "State Vector",
        fields: [
          {
            kind: "vector",
            key: "quaternion",
            label: "Quaternion",
            placeholders: ["W", "X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "lla_pos",
            label: "LLA Position",
            placeholders: ["Lon", "Lat", "Alt"],
          },
          {
            kind: "vector",
            key: "velocity",
            label: "Velocity",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "g_bias",
            label: "Gyroscope Bias",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "a_bias",
            label: "Accelerometer Bias",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "g_sf",
            label: "Gyroscope Scale Factor",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "a_sf",
            label: "Acceleration Scale Factor",
            placeholders: ["X", "Y", "Z"],
          },
        ],
      },
    ],
  },
  initial_covariance_matrix: {
    label: "Initial Covariance Matrix",
    description: "Send the seven 3-element initial covariance vectors.",
    sections: [
      {
        title: "Covariance Inputs",
        fields: [
          {
            kind: "vector",
            key: "att_unc0",
            label: "Attitude Uncertainty",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "pos_unc0",
            label: "Position Uncertainty",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "vel_unc0",
            label: "Velocity Uncertainty",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "gbias_unc0",
            label: "Gyro Bias Uncertainty",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "abias_unc0",
            label: "Accelerometer Bias Uncertainty",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "gsf_unc0",
            label: "Gyro Scale Factor Uncertainty",
            placeholders: ["X", "Y", "Z"],
          },
          {
            kind: "vector",
            key: "asf_unc0",
            label: "Acceleration Scale Factor Uncertainty",
            placeholders: ["X", "Y", "Z"],
          },
        ],
      },
    ],
  },
  timer_values: {
    label: "Timer Values",
    description: "Send drogue and main timer values plus enable flags.",
    sections: [
      {
        title: "Timer Configuration",
        fields: [
          {
            kind: "scalar",
            key: "drouge_timer",
            label: "Drogue Timer",
            placeholder: "Seconds",
          },
          {
            kind: "scalar",
            key: "main_timer",
            label: "Main Timer",
            placeholder: "Seconds",
          },
          {
            kind: "toggle",
            key: "drouge_timer_enable",
            label: "Use Timer For Drogue",
          },
          {
            kind: "toggle",
            key: "main_timer_enable",
            label: "Use Timer For Main",
          },
        ],
      },
    ],
  },
  altimeter_offsets: {
    label: "Altimeter Offsets",
    description: "Send the altimeter offsets, filter values, and FMF parameters used by RECO.",
    sections: [
      {
        title: "Altimeter Parameters",
        fields: [
          {
            kind: "scalar",
            key: "ekf_lockout_time",
            label: "EKF Lockout Time",
            placeholder: "Milliseconds",
          },
          {
            kind: "scalar",
            key: "h_offset_alt",
            label: "h_offset_alt",
            placeholder: "Float",
          },
          {
            kind: "scalar",
            key: "h_offset_filter",
            label: "h_offset_filter",
            placeholder: "Float",
          },
          {
            kind: "scalar",
            key: "flight_baro_fmf_parameter",
            label: "Flight Baro FMF Parameter",
            placeholder: "Float",
          },
          {
            kind: "scalar",
            key: "ground_baro_fmf_parameter",
            label: "Ground Baro FMF Parameter",
            placeholder: "Float",
          },
          {
            kind: "scalar",
            key: "flight_gps_fmf_parameter",
            label: "Flight GPS FMF Parameter",
            placeholder: "Float",
          },
          {
            kind: "scalar",
            key: "ground_gps_fmf_parameter",
            label: "Ground GPS FMF Parameter",
            placeholder: "Float",
          },
        ],
      },
    ],
  },
};

function blankArray(length: number): string[] {
  return Array.from({ length }, () => "");
}

function parseRequiredFloat(value: string, label: string): number {
  const trimmed = value.trim();
  if (trimmed.length === 0) {
    throw new Error(`${label} is required.`);
  }

  const parsed = Number(trimmed);
  if (!Number.isFinite(parsed)) {
    throw new Error(`${label} must be a valid number.`);
  }

  return parsed;
}

function parseRequiredInteger(value: string, label: string): number {
  const parsed = parseRequiredFloat(value, label);
  if (!Number.isInteger(parsed) || parsed < 0) {
    throw new Error(`${label} must be a non-negative integer.`);
  }

  return parsed;
}

function parseNumberArray(values: string[], label: string): number[] {
  return values.map((value, index) => parseRequiredFloat(value, `${label} ${index + 1}`));
}

const EkfFlasherPage: Component = () => {
  const [selectedMessageType, setSelectedMessageType] = createSignal<MessageType>("process_noise_matrix");
  const [activeSubmitTarget, setActiveSubmitTarget] = createSignal<RecoGuiTarget | null>(null);
  const [statusMessage, setStatusMessage] = createSignal("");
  const [statusTone, setStatusTone] = createSignal<"neutral" | "success" | "error">("neutral");
  const [forms, setForms] = createStore<Record<MessageType, Record<string, string[] | string | boolean>>>({
    process_noise_matrix: {
      nu_gv_mat: blankArray(9),
      nu_gu_mat: blankArray(9),
      nu_av_mat: blankArray(9),
      nu_au_mat: blankArray(9),
    },
    measurement_noise_matrix: {
      gps_noise_matrix: blankArray(9),
      barometer_noise: "",
    },
    ekf_state_vector: {
      quaternion: blankArray(4),
      lla_pos: blankArray(3),
      velocity: blankArray(3),
      g_bias: blankArray(3),
      a_bias: blankArray(3),
      g_sf: blankArray(3),
      a_sf: blankArray(3),
    },
    initial_covariance_matrix: {
      att_unc0: blankArray(3),
      pos_unc0: blankArray(3),
      vel_unc0: blankArray(3),
      gbias_unc0: blankArray(3),
      abias_unc0: blankArray(3),
      gsf_unc0: blankArray(3),
      asf_unc0: blankArray(3),
    },
    timer_values: {
      drouge_timer: "",
      main_timer: "",
      drouge_timer_enable: false,
      main_timer_enable: false,
    },
    altimeter_offsets: {
      ekf_lockout_time: "",
      h_offset_alt: "",
      h_offset_filter: "",
      flight_baro_fmf_parameter: "",
      ground_baro_fmf_parameter: "",
      flight_gps_fmf_parameter: "",
      ground_gps_fmf_parameter: "",
    },
  });

  const currentLayout = createMemo(() => MESSAGE_LAYOUTS[selectedMessageType()]);

  const submitButtonLabel = (target: RecoGuiTarget): string => {
    const option = SUBMIT_TARGET_OPTIONS.find((candidate) => candidate.value === target);
    const actionLabel = option?.actionLabel ?? "Selected Target";
    return activeSubmitTarget() === target ? `Submitting to ${actionLabel}...` : `Submit to ${actionLabel}`;
  };

  const updateArrayField = (messageType: MessageType, key: string, index: number, value: string) => {
    setForms(messageType, key, index, value);
  };

  const updateScalarField = (messageType: MessageType, key: string, value: string) => {
    setForms(messageType, key, value);
  };

  const updateToggleField = (messageType: MessageType, key: string, checked: boolean) => {
    setForms(messageType, key, checked);
  };

  const buildRequest = (target: RecoGuiTarget): RecoGuiCommandRequest => {
    switch (selectedMessageType()) {
      case "process_noise_matrix": {
        const payload = forms.process_noise_matrix;
        return {
          target,
          message_type: "process_noise_matrix",
          payload: {
            nu_gv_mat: parseNumberArray(payload.nu_gv_mat as string[], "Gyro Covariance"),
            nu_gu_mat: parseNumberArray(payload.nu_gu_mat as string[], "Gyro Bias Covariance"),
            nu_av_mat: parseNumberArray(payload.nu_av_mat as string[], "Accelerometer Covariance"),
            nu_au_mat: parseNumberArray(payload.nu_au_mat as string[], "Accelerometer Bias Covariance"),
          } as RecoProcessNoiseMatrix,
        };
      }
      case "measurement_noise_matrix": {
        const payload = forms.measurement_noise_matrix;
        return {
          target,
          message_type: "measurement_noise_matrix",
          payload: {
            gps_noise_matrix: parseNumberArray(payload.gps_noise_matrix as string[], "GPS Noise Matrix"),
            barometer_noise: parseRequiredFloat(payload.barometer_noise as string, "Barometer Noise"),
          } as RecoMeasurementNoiseMatrix,
        };
      }
      case "ekf_state_vector": {
        const payload = forms.ekf_state_vector;
        return {
          target,
          message_type: "ekf_state_vector",
          payload: {
            quaternion: parseNumberArray(payload.quaternion as string[], "Quaternion"),
            lla_pos: parseNumberArray(payload.lla_pos as string[], "LLA Position"),
            velocity: parseNumberArray(payload.velocity as string[], "Velocity"),
            g_bias: parseNumberArray(payload.g_bias as string[], "Gyroscope Bias"),
            a_bias: parseNumberArray(payload.a_bias as string[], "Accelerometer Bias"),
            g_sf: parseNumberArray(payload.g_sf as string[], "Gyroscope Scale Factor"),
            a_sf: parseNumberArray(payload.a_sf as string[], "Acceleration Scale Factor"),
          } as RecoEkfStateVector,
        };
      }
      case "initial_covariance_matrix": {
        const payload = forms.initial_covariance_matrix;
        return {
          target,
          message_type: "initial_covariance_matrix",
          payload: {
            att_unc0: parseNumberArray(payload.att_unc0 as string[], "Attitude Uncertainty"),
            pos_unc0: parseNumberArray(payload.pos_unc0 as string[], "Position Uncertainty"),
            vel_unc0: parseNumberArray(payload.vel_unc0 as string[], "Velocity Uncertainty"),
            gbias_unc0: parseNumberArray(payload.gbias_unc0 as string[], "Gyro Bias Uncertainty"),
            abias_unc0: parseNumberArray(payload.abias_unc0 as string[], "Accelerometer Bias Uncertainty"),
            gsf_unc0: parseNumberArray(payload.gsf_unc0 as string[], "Gyro Scale Factor Uncertainty"),
            asf_unc0: parseNumberArray(payload.asf_unc0 as string[], "Acceleration Scale Factor Uncertainty"),
          } as RecoInitialCovarianceMatrix,
        };
      }
      case "timer_values": {
        const payload = forms.timer_values;
        return {
          target,
          message_type: "timer_values",
          payload: {
            drouge_timer: parseRequiredFloat(payload.drouge_timer as string, "Drogue Timer"),
            main_timer: parseRequiredFloat(payload.main_timer as string, "Main Timer"),
            drouge_timer_enable: (payload.drouge_timer_enable as boolean) ? 1 : 0,
            main_timer_enable: (payload.main_timer_enable as boolean) ? 1 : 0,
          } as RecoTimerValues,
        };
      }
      case "altimeter_offsets": {
        const payload = forms.altimeter_offsets;
        return {
          target,
          message_type: "altimeter_offsets",
          payload: {
            ekf_lockout_time: parseRequiredInteger(payload.ekf_lockout_time as string, "EKF Lockout Time"),
            h_offset_alt: parseRequiredFloat(payload.h_offset_alt as string, "h_offset_alt"),
            h_offset_filter: parseRequiredFloat(payload.h_offset_filter as string, "h_offset_filter"),
            flight_baro_fmf_parameter: parseRequiredFloat(payload.flight_baro_fmf_parameter as string, "Flight Baro FMF Parameter"),
            ground_baro_fmf_parameter: parseRequiredFloat(payload.ground_baro_fmf_parameter as string, "Ground Baro FMF Parameter"),
            flight_gps_fmf_parameter: parseRequiredFloat(payload.flight_gps_fmf_parameter as string, "Flight GPS FMF Parameter"),
            ground_gps_fmf_parameter: parseRequiredFloat(payload.ground_gps_fmf_parameter as string, "Ground GPS FMF Parameter"),
          } as RecoAltimeterOffsets,
        };
      }
    }
  };

  const submitCurrentMessage = async (target: RecoGuiTarget) => {
    const ip = serverIp() as string | undefined;
    if (!isConnected() || !ip) {
      setStatusTone("error");
      setStatusMessage("Connect to Servo before sending a RECO parameter message.");
      return;
    }

    const targetLabel = SUBMIT_TARGET_OPTIONS.find((option) => option.value === target)?.label ?? "selected target";
    let request: RecoGuiCommandRequest;
    try {
      request = buildRequest(target);
    } catch (error) {
      setStatusTone("error");
      setStatusMessage((error as Error).message);
      return;
    }

    setActiveSubmitTarget(target);
    setStatusTone("neutral");
    setStatusMessage("");

    const response = await sendRecoGuiCommand(ip, request);
    if (response instanceof Error) {
      setStatusTone("error");
      setStatusMessage(`Request failed: ${response.message}`);
      setActiveSubmitTarget(null);
      return;
    }

    if (!response.ok) {
      const body = await response.text();
      setStatusTone("error");
      setStatusMessage(body.length > 0 ? `Submit failed: ${body}` : `Submit failed with HTTP ${response.status}.`);
      setActiveSubmitTarget(null);
      return;
    }

    setStatusTone("success");
    setStatusMessage(`${currentLayout().label} sent successfully to ${targetLabel}.`);
    setActiveSubmitTarget(null);
  };

  return <div class="config-view">
    <div style="text-align: center; font-size: 14px">EKF PARAMETER FLASHER</div>
    <div class="new-config-section ekf-flasher-page">
      <div class="ekf-flasher-toolbar">
        <div class="ekf-flasher-toolbar-row">
          <div class="ekf-flasher-toolbar-label">Message Type</div>
          <select
            class="feedsystem-config-dropdown ekf-flasher-select"
            value={selectedMessageType()}
            onChange={(event) => setSelectedMessageType(event.currentTarget.value as MessageType)}
          >
            <For each={MESSAGE_OPTIONS}>{(option) =>
              <option value={option.value}>{option.label}</option>
            }</For>
          </select>
        </div>
        <div class="ekf-flasher-description">{currentLayout().description}</div>
        <Show when={currentLayout().rowMajorNote}>
          <div class="ekf-flasher-note">
            Matrices are serialized in row-major order: [a11, a12, a13, a21, a22, a23, a31, a32, a33].
          </div>
        </Show>
      </div>

      <div class="ekf-flasher-sections">
        <For each={currentLayout().sections}>{(section) =>
          <div
            class={`ekf-flasher-section${section.fields.every((field) => field.kind === "matrix") ? " ekf-flasher-section-matrix-layout" : ""}`}
          >
            <div class="ekf-flasher-section-title">{section.title}</div>
            <For each={section.fields}>{(field) =>
              <>
                <Show when={field.kind === "matrix"}>
                  <div class="ekf-flasher-field">
                    <div class="ekf-flasher-field-label">{(field as MatrixFieldSpec).label}</div>
                    <div class="ekf-flasher-matrix-grid">
                      <For each={((forms[selectedMessageType()][field.key] as string[]) ?? [])}>{(value, index) =>
                        <input
                          class="add-config-input ekf-flasher-input"
                          type="text"
                          value={value}
                          placeholder={(field as MatrixFieldSpec).placeholders[index()]}
                          onInput={(event) => updateArrayField(selectedMessageType(), field.key, index(), event.currentTarget.value)}
                        />
                      }</For>
                    </div>
                  </div>
                </Show>
                <Show when={field.kind === "vector"}>
                  <div class="ekf-flasher-field ekf-flasher-vector-field">
                    <div class="ekf-flasher-field-label">{(field as VectorFieldSpec).label}</div>
                    <div class="ekf-flasher-inline-inputs">
                      <For each={((forms[selectedMessageType()][field.key] as string[]) ?? [])}>{(value, index) =>
                        <input
                          class="add-config-input ekf-flasher-input"
                          type="text"
                          value={value}
                          placeholder={(field as VectorFieldSpec).placeholders[index()]}
                          onInput={(event) => updateArrayField(selectedMessageType(), field.key, index(), event.currentTarget.value)}
                        />
                      }</For>
                    </div>
                  </div>
                </Show>
                <Show when={field.kind === "scalar"}>
                  <div class="ekf-flasher-field ekf-flasher-scalar-field">
                    <div class="ekf-flasher-field-label">{(field as ScalarFieldSpec).label}</div>
                    <input
                      class="add-config-input ekf-flasher-scalar-input"
                      type="text"
                      value={(forms[selectedMessageType()][field.key] as string) ?? ""}
                      placeholder={(field as ScalarFieldSpec).placeholder}
                      onInput={(event) => updateScalarField(selectedMessageType(), field.key, event.currentTarget.value)}
                    />
                  </div>
                </Show>
                <Show when={field.kind === "toggle"}>
                  <div class="ekf-flasher-field ekf-flasher-toggle-field">
                    <label class="ekf-flasher-toggle-label">
                      <input
                        type="checkbox"
                        checked={Boolean(forms[selectedMessageType()][field.key])}
                        onChange={(event) => updateToggleField(selectedMessageType(), field.key, event.currentTarget.checked)}
                      />
                      <span>{(field as ToggleFieldSpec).label}</span>
                    </label>
                  </div>
                </Show>
              </>
            }</For>
          </div>
        }</For>
      </div>

      <div class="ekf-flasher-submit-row">
        <div
          class="ekf-flasher-status"
          style={{
            color: statusTone() === "error" ? "#C53434" : statusTone() === "success" ? "#1DB55A" : "#f0f0f0",
          }}
        >
          {statusMessage()}
        </div>
        <div class="ekf-flasher-submit-buttons">
          <For each={SUBMIT_TARGET_OPTIONS}>{(option) =>
            <button
              class="submit-sequence-button ekf-flasher-submit-button"
              disabled={activeSubmitTarget() !== null}
              onClick={() => void submitCurrentMessage(option.value)}
            >
              {submitButtonLabel(option.value)}
            </button>
          }</For>
        </div>
      </div>
    </div>
  </div>;
};

export default EkfFlasherPage;
