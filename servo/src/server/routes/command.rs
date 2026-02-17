use crate::server::{
  self,
  error::{bad_request, internal},
  Shared,
};
use axum::{extract::State, http::request, Json};
use common::comm::{
  ahrs, 
  bms, 
  FlightControlMessage, 
  Sequence,
  reco::EkfBiasParameters,
};
use serde::{Deserialize, Serialize};

/// These fields correspond to the EkfBiasParameters struct. 
/// This type is needed as the GUI will send some foelds as Option<f32> to indicate
/// that the field is not set, which we want to map to f32::NAN before forwarding
/// to the flight computer. 
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RecoEkfParamsRequest {
  /// Quaternion representing vehicle attitude [w, x, y, z] 
  pub quaternion: [Option<f32>; 4],
  /// Position [longitude, latitude, altitude] in degrees and meters
  pub lla_pos: [Option<f32>; 3],
  /// Gyroscope bias offset [x, y, z]
  pub g_bias: [Option<f32>; 3],
  /// Accelerometer bias offset [x, y, z]
  pub a_bias: [Option<f32>; 3],
  /// Gyro scale factor [x, y, z]
  pub g_sf: [Option<f32>; 3],
  /// Acceleration scale factor [x, y, z]
  pub a_sf: [Option<f32>; 3],
  /// Altimeter pressure offset
  pub alt_off: Option<f32>,
  /// Filter pressure offset
  pub fil_off: Option<f32>,
}

/// Request struct containing all necessary information to execute a command.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct OperatorCommandRequest {
  command: String,
  target: Option<String>,
  state: Option<String>,
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

/// Obvious
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CameraEnable {
  enabled : bool
}

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


/// Obvious
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct LugArm {
  armed : bool
}

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

/// Obvious
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct LugDetonate {
  enabled : bool
}

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

/// Accept EKF bias parameters from the GUI and forward them to the flight
/// computer as a `FlightControlMessage::SetEKFParameters`.
pub async fn set_reco_ekf_parameters(
  State(shared): State<Shared>,
  Json(request): Json<RecoEkfParamsRequest>,
) -> server::Result<()> {

  // Map the Option<f32> fields to f32::NAN if they are None
  let params = EkfBiasParameters {
    quaternion: [
      request.quaternion[0].unwrap_or(f32::NAN),
      request.quaternion[1].unwrap_or(f32::NAN),
      request.quaternion[2].unwrap_or(f32::NAN),
      request.quaternion[3].unwrap_or(f32::NAN),
    ],
    lla_pos: [
      request.lla_pos[0].unwrap_or(f32::NAN),
      request.lla_pos[1].unwrap_or(f32::NAN),
      request.lla_pos[2].unwrap_or(f32::NAN),
    ],
    a_bias: [
      request.a_bias[0].unwrap_or(f32::NAN),
      request.a_bias[1].unwrap_or(f32::NAN),
      request.a_bias[2].unwrap_or(f32::NAN),
    ],
    g_bias: [
      request.g_bias[0].unwrap_or(f32::NAN),
      request.g_bias[1].unwrap_or(f32::NAN),
      request.g_bias[2].unwrap_or(f32::NAN),
    ],
    a_sf: [
      request.a_sf[0].unwrap_or(f32::NAN),
      request.a_sf[1].unwrap_or(f32::NAN),
      request.a_sf[2].unwrap_or(f32::NAN),
    ],
    g_sf: [
      request.g_sf[0].unwrap_or(f32::NAN),
      request.g_sf[1].unwrap_or(f32::NAN),
      request.g_sf[2].unwrap_or(f32::NAN),
    ],
    alt_press_off: request.alt_off.unwrap_or(f32::NAN),
    filter_press_off: request.fil_off.unwrap_or(f32::NAN),
  };

  let message = FlightControlMessage::SetEKFParameters(params);
  let serialized = postcard::to_allocvec(&message).map_err(internal)?;

  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    flight.send_bytes(&serialized).await.map_err(internal)?;
  }

  Ok(())
}