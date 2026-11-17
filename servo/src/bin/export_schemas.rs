use schemars::{schema_for, JsonSchema};
use std::fs;

#[derive(JsonSchema)]
#[allow(dead_code)]
struct APITypes {
    execute_sql_request: routes::ExecuteSqlRequest,
    execute_sql_response: routes::ExecuteSqlResponse,

    operator_command_request: routes::OperatorCommandRequest,
    reco_gui_command_request: routes::RecoGuiCommandRequest,
    reco_gui_command_payload: routes::RecoGuiCommandPayload,
    camera_enable: routes::CameraEnable,
    lug_arm: routes::LugArm,
    lug_detonate: routes::LugDetonate,

    export_request: routes::ExportRequest,
    telemetry_source_query: routes::TelemetrySourceQuery,
    telemetry_source_stats: routes::TelemetrySourceStats,
    telemetry_stats_response: routes::TelemetryStatsResponse,

    set_mappings_request: routes::SetMappingsRequest,
    delete_mappings_request: routes::DeleteMappingsRequest,
    retrieve_abort_configs_response: routes::RetrieveAbortConfigsResponse,
    set_abort_config: routes::SetAbortConfig,
    delete_abort_config_request: routes::DeleteAbortConfigRequest,
    active_configuration: routes::ActiveConfiguration,

    sequence_with_configuration: routes::SequenceWithConfiguration,
    retrieve_sequence_response: routes::RetrieveSequenceResponse,
    save_sequence_request: routes::SaveSequenceRequest,
    delete_sequence_request: routes::DeleteSequenceRequest,
    run_sequence_request: routes::RunSequenceRequest,
    stop_sequence_request: routes::StopSequenceRequest,

    delete_trigger_request: routes::DeleteTriggerRequest,

    vehicle_state: common::comm::VehicleState,
    node_mapping: common::comm::NodeMapping,
    trigger: common::comm::Trigger,
    abort_stage_config: common::comm::AbortStageConfig,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let schema = schema_for!(APITypes);
    let json = serde_json::to_string_pretty(&schema)?;

    fs::create_dir_all("../../api/generated")?;
    fs::write("../../api/generated/types.json", json)?;

    println!("Exported API JSON schema objects to servo/api/generated/types.json");
    Ok(())
}
