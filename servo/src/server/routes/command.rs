use crate::server::{
  self,
  error::{bad_request, internal},
  Shared,
};
use axum::{extract::State, Json};
use common::comm::{bms, FlightControlMessage, reco, Sequence};
use serde::{Deserialize, Serialize};

/// Request struct containing all necessary information to execute a command.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct OperatorCommandRequest {
  command: String,
  target: Option<String>,
  state: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "message_type", content = "payload", rename_all = "snake_case")]
/// Payload body for RECO GUI parameter commands.
pub enum RecoGuiCommandPayload {
  /// Send process noise matrices to RECO.
  ProcessNoiseMatrix(reco::ProcessNoiseMatrix),
  /// Send measurement noise values to RECO.
  MeasurementNoiseMatrix(reco::MeasurementNoiseMatrix),
  /// Send an initial EKF state vector to RECO.
  EkfStateVector(reco::EkfStateVector),
  /// Send an initial covariance matrix to RECO.
  InitialCovarianceMatrix(reco::InitialCovarianceMatrix),
  /// Send timer values to RECO.
  TimerValues(reco::TimerValues),
  /// Send FMF parameters to RECO.
  AltimeterOffsets(reco::AltimeterOffsets),
}

/// Request body for RECO GUI parameter commands.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RecoGuiCommandRequest {
  /// The RECO MCU that should receive the command. We use 
  /// #[serde(default)] to make the target field optional, and if no target is 
  /// specified `target` defaults to the specified default value for this type, 
  /// which is `TargetMCU::All`.
  #[serde(default)]
  target: reco::TargetMCU,
  /// The command that we receive from the GUI. We use #[serde(flatten)] to 
  /// automatically deserialize the command payload into the appropriate variant 
  /// of the RecoGuiCommandPayload enum as the GUI sends it in a flattened manner.
  #[serde(flatten)]
  command: RecoGuiCommandPayload,
}

/// Route handler to dispatch a single manual operator command
pub async fn dispatch_operator_command(
  State(shared): State<Shared>,
  Json(request): Json<OperatorCommandRequest>,
) -> server::Result<()> {
  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    let command = match request.command.as_str() {
      "click_valve" => {
        let target = request
          .target
          .clone()
          .ok_or(bad_request("must supply target name"))?;

        let script = match request.state.as_deref() {
          Some("open") => format!("{target}.open()"),
          Some("closed") => format!("{target}.close()"),
          None => Err(bad_request("valve state is required"))?,
          _ => Err(bad_request("unrecognized state identifier"))?,
        };

        common::comm::FlightControlMessage::Sequence(Sequence {
          name: "command".to_owned(),
          script,
        })
      }
      // Currently does nothing until the flight side is finalized
      "bms" => {
        // Inefficient code but this doesnt need to be any better
        if request.target.as_deref().unwrap_or_default() == "estop" {
          FlightControlMessage::BmsCommand(bms::Command::ResetEstop)
        } else {
          let state = match request.state.as_deref() {
            Some("enabled") => true,
            Some("disabled") => false,
            None => {
              Err(bad_request("state is a required field for all but estop"))?
            }
            _ => Err(bad_request("unrecognized state identifier"))?,
          };

          FlightControlMessage::BmsCommand(match request.target.as_deref() {
            Some("battery_ls") => bms::Command::BatteryLoadSwitch(state),
            Some("sam_ls") => bms::Command::SamLoadSwitch(state),
            Some("charge") => bms::Command::Charge(state),
            Some("tel_ls") => bms::Command::TelLoadSwitch(state),
            None => Err(bad_request("must supply target name"))?,
            _ => Err(bad_request("unrecognized bms target"))?,
          })
        }
      }
      _ => return Err(bad_request("unrecognized command identifier")),
    };

    let serialized = postcard::to_allocvec(&command).map_err(internal)?;

    flight.send_bytes(&serialized).await.map_err(internal)?;
  } else {
    return Err(internal("flight computer not connected"));
  }

  Ok(())
}

/// Route handler to forward a typed RECO GUI command to the flight computer.
pub async fn send_reco_gui_command(
  State(shared): State<Shared>,
  Json(request): Json<RecoGuiCommandRequest>,
) -> server::Result<()> {
  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    let RecoGuiCommandRequest { target, command } = request;

    let gui_command = match command {
      RecoGuiCommandPayload::ProcessNoiseMatrix(matrix) => {
        reco::GuiCommand::ProcessNoiseMatrix(matrix)
      }
      RecoGuiCommandPayload::MeasurementNoiseMatrix(matrix) => {
        reco::GuiCommand::MeasurementNoiseMatrix(matrix)
      }
      RecoGuiCommandPayload::EkfStateVector(vector) => {
        reco::GuiCommand::EkfStateVector(vector)
      }
      RecoGuiCommandPayload::InitialCovarianceMatrix(matrix) => {
        reco::GuiCommand::InitialCovarianceMatrix(matrix)
      }
      RecoGuiCommandPayload::TimerValues(values) => {
        reco::GuiCommand::TimerValues(values)
      }
      RecoGuiCommandPayload::AltimeterOffsets(offsets) => {
        reco::GuiCommand::AltimeterOffsets(offsets)
      }
    };

    let message = FlightControlMessage::RecoCommand(reco::TargetedGuiCommand {
      target,
      command: gui_command,
    });
    let serialized = postcard::to_allocvec(&message).map_err(internal)?;

    flight.send_bytes(&serialized).await.map_err(internal)?;
  } else {
    return Err(internal("flight computer not connected"));
  }

  Ok(())
}

/// Obvious
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CameraEnable {
  enabled: bool
}

/// Route handler to tell the flight computer to change the enabled state of the
/// camera.
pub async fn enable_camera(
  State(shared): State<Shared>,
  Json(request): Json<CameraEnable>,
)-> server::Result<()> {
  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    let message = FlightControlMessage::CameraEnable(request.enabled);
    let serialized = postcard::to_allocvec(&message).map_err(internal)?;

    flight.send_bytes(&serialized).await.map_err(internal)?;
  }
  Ok(())
}

/// Request struct for changing the arming state of the launch lugs.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct LugArm {
  armed: bool
}

/// Route handler to tell the flight computer to change the arming state of the
/// launch lugs.
pub async fn arm_lugs(
  State(shared): State<Shared>,
  Json(request): Json<LugArm>,
)-> server::Result<()> {
  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    let message = FlightControlMessage::DetonatorArm(request.armed);
    let serialized = postcard::to_allocvec(&message).map_err(internal)?;

    flight.send_bytes(&serialized).await.map_err(internal)?;
  }
  Ok(())
}

/// Request struct for detonating the launch lugs.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct LugDetonate {
  enabled: bool
}

/// Route handler to tell the flight computer to detonate the launch lugs.
pub async fn detonate_lugs(
  State(shared): State<Shared>,
  Json(request): Json<LugDetonate>,
)-> server::Result<()> {
  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    let message = FlightControlMessage::DetonateEnable(request.enabled);
    let serialized = postcard::to_allocvec(&message).map_err(internal)?;

    flight.send_bytes(&serialized).await.map_err(internal)?;
  }
  Ok(())
}
