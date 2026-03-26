use crate::server::{
  self,
  error::{bad_request, internal},
  Shared,
};
use axum::{extract::State, Json};
use common::comm::{
  ahrs, 
  bms, 
  reco,
  FlightControlMessage, 
  Sequence,
};
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
/// Request body for RECO GUI parameter commands.
pub enum RecoGuiCommandRequest {
  /// Send process noise matrices to RECO.
  ProcessNoiseMatrix(reco::ProcessNoiseMatrix),
  /// Send measurement noise values to RECO.
  MeasurementNoiseMatrix(reco::MeasurementNoiseMatrix),
  /// Send an initial EKF state vector to RECO.
  EKFStateVector(reco::EkfStateVector),
  /// Send an initial covariance matrix to RECO.
  InitialCovarianceMatrix(reco::InitialCovarianceMatrix),
  /// Send timer values to RECO.
  TimerValues(reco::TimerValues),
  /// Send FMF parameters to RECO.
  AltimeterOffsets(reco::AltimeterOffsets),
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
            None => Err(bad_request("must supply target name"))?,
            _ => Err(bad_request("unrecognized bms target"))?,
          })
        }
      }
      "ahrs" => {
        let state = match request.state.as_deref() {
          Some("enabled") => true,
          Some("disabled") => false,
          None => Err(bad_request("state is a required field"))?,
          _ => Err(bad_request("unrecognized state identifier"))?,
        };

        FlightControlMessage::AhrsCommand(match request.target.as_deref() {
          Some("camera") => ahrs::Command::CameraEnable(state),
          None => Err(bad_request("must supply target name"))?,
          _ => Err(bad_request("unrecognized ahrs target"))?,
        })
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
    let gui_command = match request {
      RecoGuiCommandRequest::ProcessNoiseMatrix(matrix) => {
        reco::GuiCommand::ProcessNoiseMatrix(matrix)
      }
      RecoGuiCommandRequest::MeasurementNoiseMatrix(matrix) => {
        reco::GuiCommand::MeasurementNoiseMatrix(matrix)
      }
      RecoGuiCommandRequest::EKFStateVector(vector) => {
        reco::GuiCommand::EKFStateVector(vector)
      }
      RecoGuiCommandRequest::InitialCovarianceMatrix(matrix) => {
        reco::GuiCommand::InitialCovarianceMatrix(matrix)
      }
      RecoGuiCommandRequest::TimerValues(values) => {
        reco::GuiCommand::TimerValues(values)
      }
      RecoGuiCommandRequest::AltimeterOffsets(offsets) => {
        reco::GuiCommand::AltimeterOffsets(offsets)
      }
    };

    let message = FlightControlMessage::RecoCommand(gui_command);
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
  enabled : bool
}

/// Route handler to enable the camera.
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


/// Struct for the arm lugs request.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct LugArm {
  armed : bool
}

/// Route handler to arm the lugs.
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

/// Struct for the detonate lugs request.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct LugDetonate {
  enabled : bool
}

/// Route handler to detonate the lugs.
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