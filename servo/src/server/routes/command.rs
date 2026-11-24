use axum::{extract::State, Json};
use common::comm::{bms, ctv::CtvOperatorCommand, reco, FlightControlMessage, Sequence};
use jeflog::pass;
use serde::{Deserialize, Serialize};

use crate::server::{
    self,
    error::{bad_request, internal},
    Shared,
};

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
    /// specified `target` defaults to the specified default value for this
    /// type, which is `TargetMCU::All`.
    #[serde(default)]
    target: reco::TargetMCU,
    /// The command that we receive from the GUI. We use #[serde(flatten)] to
    /// automatically deserialize the command payload into the appropriate
    /// variant of the RecoGuiCommandPayload enum as the GUI sends it in a
    /// flattened manner.
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
                        None => Err(bad_request("state is a required field for all but estop"))?,
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
            RecoGuiCommandPayload::TimerValues(values) => reco::GuiCommand::TimerValues(values),
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

/// Request body for CTV operations commands.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CtvCommandRequest {
    /// The snake_case name of a `CtvOperatorCommand`, for example
    /// `"start_flight"`.
    command: String,
}

/// Parses the name of a CTV operator command. Unknown names are a bad request.
fn parse_ctv_command(name: &str) -> server::Result<CtvOperatorCommand> {
    serde_json::from_value(serde_json::Value::String(name.to_owned()))
        .map_err(|_| bad_request(format!("unrecognized ctv command: {name}")))
}

/// Route handler to forward a CTV operations command (arm EC, start flight,
/// abort to land, ...) to the flight computer.
pub async fn send_ctv_command(
    State(shared): State<Shared>,
    Json(request): Json<CtvCommandRequest>,
) -> server::Result<()> {
    let command = parse_ctv_command(&request.command)?;

    shared
        .flight
        .0
        .lock()
        .await
        .as_mut()
        .ok_or(internal("flight computer not connected"))?
        .send_ctv_command(command)
        .await
        .map_err(internal)?;

    pass!("Operator sent CTV command {command:?} to the flight computer.");

    Ok(())
}

/// Obvious
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CameraEnable {
    enabled: bool,
}

/// Route handler to tell the flight computer to change the enabled state of the
/// camera.
pub async fn enable_camera(
    State(shared): State<Shared>,
    Json(request): Json<CameraEnable>,
) -> server::Result<()> {
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
    armed: bool,
}

/// Route handler to tell the flight computer to change the arming state of the
/// launch lugs.
pub async fn arm_lugs(
    State(shared): State<Shared>,
    Json(request): Json<LugArm>,
) -> server::Result<()> {
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
    enabled: bool,
}

/// Route handler to tell the flight computer to detonate the launch lugs.
pub async fn detonate_lugs(
    State(shared): State<Shared>,
    Json(request): Json<LugDetonate>,
) -> server::Result<()> {
    if let Some(flight) = shared.flight.0.lock().await.as_mut() {
        let message = FlightControlMessage::DetonateEnable(request.enabled);
        let serialized = postcard::to_allocvec(&message).map_err(internal)?;

        flight.send_bytes(&serialized).await.map_err(internal)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use axum::http::StatusCode;
    use tokio::{
        io::AsyncReadExt,
        net::{TcpListener, TcpStream},
    };

    use super::*;
    use crate::server::{FlightComputer, Server};

    const ALL_COMMANDS: [(&str, CtvOperatorCommand); 7] = [
        ("arm_ec", CtvOperatorCommand::ArmEc),
        ("disarm_ec", CtvOperatorCommand::DisarmEc),
        ("start_flight", CtvOperatorCommand::StartFlight),
        ("abort_land", CtvOperatorCommand::AbortLand),
        ("terminate", CtvOperatorCommand::Terminate),
        ("clear_safing", CtvOperatorCommand::ClearSafing),
        ("reinit_nav", CtvOperatorCommand::ReinitNav),
    ];

    fn status_of(error: server::Error) -> StatusCode {
        match error {
            server::Error::Raw(_, status) => status,
            server::Error::Sql(_) => StatusCode::INTERNAL_SERVER_ERROR,
        }
    }

    #[test]
    fn request_body_parses_as_the_gui_sends_it() {
        let request: CtvCommandRequest =
            serde_json::from_str(r#"{ "command": "start_flight" }"#).unwrap();
        assert_eq!(
            parse_ctv_command(&request.command).unwrap(),
            CtvOperatorCommand::StartFlight
        );
    }

    #[test]
    fn every_command_name_parses() {
        for (name, command) in ALL_COMMANDS {
            assert_eq!(parse_ctv_command(name).unwrap(), command, "{name}");
        }
    }

    #[test]
    fn unknown_commands_are_a_bad_request() {
        for name in ["", "StartFlight", "start-flight", "fly", "arm_ec "] {
            let error = parse_ctv_command(name).unwrap_err();
            assert_eq!(status_of(error), StatusCode::BAD_REQUEST, "{name:?}");
        }
    }

    #[tokio::test]
    async fn rejects_unknown_commands_before_checking_for_flight() {
        let server = Server::new(None).unwrap();
        let result = send_ctv_command(
            State(server.shared.clone()),
            Json(CtvCommandRequest {
                command: "self_destruct".to_owned(),
            }),
        )
        .await;

        assert_eq!(status_of(result.unwrap_err()), StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn errors_when_no_flight_computer_is_connected() {
        let server = Server::new(None).unwrap();
        let result = send_ctv_command(
            State(server.shared.clone()),
            Json(CtvCommandRequest {
                command: "terminate".to_owned(),
            }),
        )
        .await;

        assert_eq!(
            status_of(result.unwrap_err()),
            StatusCode::INTERNAL_SERVER_ERROR
        );
    }

    #[tokio::test]
    async fn forwards_each_command_to_the_flight_computer() {
        let server = Server::new(None).unwrap();
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();

        let (client, accepted) = tokio::join!(TcpStream::connect(address), listener.accept());
        let mut flight_end = accepted.unwrap().0;

        *server.shared.flight.0.lock().await = Some(FlightComputer::from_stream(
            server.shared.database.clone(),
            client.unwrap(),
        ));

        for (name, command) in ALL_COMMANDS {
            send_ctv_command(
                State(server.shared.clone()),
                Json(CtvCommandRequest {
                    command: name.to_owned(),
                }),
            )
            .await
            .unwrap();

            // Frames are a big-endian u16 length followed by a postcard message.
            let mut length = [0u8; 2];
            flight_end.read_exact(&mut length).await.unwrap();
            let mut body = vec![0u8; u16::from_be_bytes(length) as usize];
            flight_end.read_exact(&mut body).await.unwrap();

            match postcard::from_bytes::<FlightControlMessage>(&body).unwrap() {
                FlightControlMessage::CtvCommand(received) => assert_eq!(received, command),
                other => panic!("expected a CtvCommand, got {other:?}"),
            }
        }
    }
}
