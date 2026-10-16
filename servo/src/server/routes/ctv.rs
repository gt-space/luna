#![allow(missing_docs)]

use crate::server::Shared;
use axum::{extract::State, http::StatusCode, Json};
use common::ctv_config::{
    content_hash, hash_bytes, Bundle, FcReadout, Lifecycle, ValidationError, MAX_BUNDLE_BYTES,
};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::time::{Duration, Instant};

#[derive(Debug, Default)]
pub struct CtvState {
    last: Option<(FcReadout, Instant)>,
    last_attempt: Option<Instant>,
    connected: bool,
    error: Option<String>,
    request_id: u64,
    records: Vec<ConfigRecord>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ConfigRecord {
    pub request_id: String,
    pub bundle_id: String,
    pub source: String,
    pub state: Lifecycle,
    pub history: Vec<Lifecycle>,
    pub validation_scope: String,
    pub errors: Vec<ValidationError>,
    pub payload_hashes: Value,
    pub bundle: Option<Bundle>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Upload {
    pub request_id: String,
    pub bundle: Value,
}

impl CtvState {
    fn upload(&mut self, request: Upload) -> Result<ConfigRecord, (StatusCode, String)> {
        let fail = |code, reason: &str| (code, reason.to_owned());
        if request.request_id.is_empty() || request.request_id.len() > 128 {
            return Err(fail(
                StatusCode::BAD_REQUEST,
                "request_id must contain 1 to 128 bytes",
            ));
        }
        let bytes = serde_json::to_vec(&request.bundle)
            .map_err(|e| (StatusCode::BAD_REQUEST, e.to_string()))?;
        if bytes.len() > MAX_BUNDLE_BYTES {
            return Err(fail(StatusCode::PAYLOAD_TOO_LARGE, "bundle exceeds 1 MiB"));
        }
        let parsed = Bundle::parse(&bytes);
        let bundle_id = match &parsed {
            Ok(bundle) => {
                content_hash(bundle).map_err(|e| (StatusCode::BAD_REQUEST, e.to_string()))?
            }
            Err(_) => hash_bytes(&bytes),
        };
        if let Some(record) = self
            .records
            .iter()
            .find(|r| r.request_id == request.request_id)
        {
            return if record.bundle_id == bundle_id {
                Ok(record.clone())
            } else {
                Err(fail(
                    StatusCode::CONFLICT,
                    "request_id was already used for different content",
                ))
            };
        }
        if self.records.len() >= 16 {
            return Err(fail(
                StatusCode::SERVICE_UNAVAILABLE,
                "candidate store is full (16 records); no record was evicted",
            ));
        }
        let (bundle, errors) = match parsed {
            Ok(bundle) => (Some(bundle), vec![]),
            Err(errors) => (None, errors),
        };
        let mut state = Lifecycle::Candidate;
        let mut history = vec![state];
        for next in [
            Lifecycle::Uploaded,
            Lifecycle::Validation,
            if errors.is_empty() {
                Lifecycle::Validated
            } else {
                Lifecycle::Rejected
            },
        ] {
            state = state
                .transition(next, false, false)
                .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, e.to_owned()))?;
            history.push(state);
        }
        let payload_hashes = bundle.as_ref().map(|b| {
            let fc = json!({"schema_version": b.schema_version, "compatibility": b.compatibility, "controller": b.controller,
                "navigation": b.navigation, "fdir": b.fdir, "tvc": b.tvc, "startup": b.startup});
            json!({"fc": content_hash(&fc).ok(), "ec": content_hash(&b.ec).ok()})
        }).unwrap_or(Value::Null);
        let record = ConfigRecord {
            request_id: request.request_id,
            bundle_id,
            source: "servo_memory".into(),
            state,
            history,
            validation_scope:
                "schema_only; FC/EC compatibility and hardware calibration are unverified".into(),
            errors,
            payload_hashes,
            bundle,
        };
        self.records.push(record.clone());
        Ok(record)
    }

    fn status(&self) -> Value {
        let age = self
            .last
            .as_ref()
            .map(|(_, time)| time.elapsed().as_secs_f64() * 1000.0);
        let readout = self.last.as_ref().map(|(reply, _)| reply);
        let battery = readout.and_then(|r| r.battery.as_ref()).map(|sample| json!({
            "source": "bms_via_fc", "source_id": sample.source_id, "voltage_v": sample.voltage_v, "current_a": sample.current_a,
            "power_w": sample.voltage_v * sample.current_a, "soc": null,
            "sample_time_s": sample.sample_time_s, "observed_age_ms": sample.observed_age_ms + age.unwrap_or(0.0),
            "measurement_age_ms": null, "clock_mapping": "unavailable", "validity": "age_unverified",
            "connection": if self.connected { "connected" } else { "lost_connection_stale" }
        })).unwrap_or_else(|| json!({"source": "bms_via_fc", "voltage_v": null, "current_a": null, "power_w": null, "soc": null,
            "observed_age_ms": null, "measurement_age_ms": null, "validity": "unavailable", "reason": "no BMS measurement received"}));
        json!({
            "protocol_version": 1, "vehicle_id": null,
            "connection": if self.connected { "connected" } else if readout.is_some() { "lost_connection_stale" } else { "not_connected" },
            "last_error": self.error, "mode": null, "first_fault": null, "fault_response": null,
            "fc": {"source": "live_fc", "available": readout.is_some(), "age_ms": age,
                "boot_id": readout.map(|r| &r.boot_id), "uptime_ms_at_sample": readout.map(|r| r.uptime_ms),
                "firmware": readout.map(|r| &r.configuration["firmware"]),
                "legacy_abort": readout.map(|r| &r.legacy_abort)},
            "ec": {"source": "live_ec", "available": false, "boot_id": null, "age_ms": null,
                "arming_state": null, "output_authority": null, "rails": null, "driver_readiness": null,
                "reason": "EC status/acknowledgement producer is not integrated"},
            "battery": battery,
            "configuration": {"fc": readout.map(|r| &r.configuration), "ec": null, "age_ms": age},
            "readiness": {"available": false, "ready": null, "source": "unavailable",
                "conditions": [
                    {"condition": "fc_link", "met": self.connected},
                    {"condition": "ec_status_and_driver_readiness", "met": null},
                    {"condition": "compatible_active_bundles", "met": null},
                    {"condition": "bench_readiness_policy", "met": null}],
                "reason": "authoritative FC/EC readiness is not integrated; navigation is not assumed present"},
            "run": {"available": false, "run_id": null, "source": "unavailable", "reason": "FC run manager is not integrated"},
            "capabilities": {"config_upload": true, "config_activate": false, "config_rollback": false, "run_start": false, "output_commands": false}
        })
    }
}

pub async fn status(State(shared): State<Shared>) -> Json<Value> {
    let mut state = shared.ctv.lock().await;
    if state
        .last_attempt
        .is_none_or(|last| last.elapsed() >= Duration::from_millis(100))
    {
        state.last_attempt = Some(Instant::now());
        state.request_id = state.request_id.wrapping_add(1);
        match tokio::time::timeout(Duration::from_millis(300), shared.flight.0.lock()).await {
            Ok(mut flight) => {
                if let Some(fc) = flight.as_mut() {
                    match tokio::time::timeout(
                        Duration::from_millis(300),
                        fc.ctv_readout(state.request_id),
                    )
                    .await
                    {
                        Ok(Ok(reply)) => {
                            state.last = Some((reply, Instant::now()));
                            state.connected = true;
                            state.error = None;
                        }
                        result => {
                            state.error = Some(match result {
                                Ok(Err(e)) => e.to_string(),
                                _ => "FC readout timed out; last values remain stale".into(),
                            });
                            state.connected = false;
                            if fc.check_closed() {
                                *flight = None;
                            }
                        }
                    }
                } else {
                    state.connected = false;
                    state.error = Some("flight computer is not connected".into());
                }
            }
            Err(_) => {
                state.connected = false;
                state.error = Some("FC connection is busy".into());
            }
        }
    }
    Json(state.status())
}

pub async fn config(State(shared): State<Shared>) -> Json<Value> {
    let live = status(State(shared.clone())).await.0;
    let state = shared.ctv.lock().await;
    Json(
        json!({"source": "servo_memory", "durability": "until Servo restarts", "records": state.records,
        "staged": null, "active": null, "previous_known_good": null,
        "running": live["configuration"], "connection": live["connection"],
        "reason": "device configuration deployment/activation is not implemented"}),
    )
}

pub async fn readiness(state: State<Shared>) -> Json<Value> {
    Json(status(state).await.0["readiness"].clone())
}
pub async fn run(state: State<Shared>) -> Json<Value> {
    Json(status(state).await.0["run"].clone())
}

pub async fn upload(
    State(shared): State<Shared>,
    Json(request): Json<Upload>,
) -> (StatusCode, Json<Value>) {
    match shared.ctv.lock().await.upload(request) {
        Ok(record) => (
            if record.errors.is_empty() {
                StatusCode::OK
            } else {
                StatusCode::UNPROCESSABLE_ENTITY
            },
            Json(
                json!({"authority": "servo", "outcome": if record.errors.is_empty() { "validated" } else { "rejected" },
                "fc_validation": "not_requested", "ec_validation": "not_requested", "record": record}),
            ),
        ),
        Err((code, reason)) => (
            code,
            Json(json!({"authority": "servo", "outcome": "rejected", "reason": reason})),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn request(id: &str) -> Upload {
        Upload {
            request_id: id.into(),
            bundle: serde_json::from_str(include_str!(
                "../../../../common/examples/ctv-bench.json"
            ))
            .unwrap(),
        }
    }

    #[test]
    fn upload_is_idempotent_and_does_not_activate() {
        let mut state = CtvState::default();
        let first = state.upload(request("one")).unwrap();
        let second = state.upload(request("one")).unwrap();
        assert_eq!(first.bundle_id, second.bundle_id);
        assert_eq!(state.records.len(), 1);
        assert_eq!(first.state, Lifecycle::Validated);
        assert!(!state.status()["capabilities"]["config_activate"]
            .as_bool()
            .unwrap());
        let mut changed = request("one");
        changed.bundle["vehicle_id"] = "other".into();
        assert_eq!(state.upload(changed).unwrap_err().0, StatusCode::CONFLICT);
    }

    #[test]
    fn rejected_records_have_paths_and_store_is_bounded() {
        let mut state = CtvState::default();
        let mut invalid = request("invalid");
        invalid.bundle["schema_version"] = 99.into();
        let record = state.upload(invalid).unwrap();
        assert_eq!(record.state, Lifecycle::Rejected);
        assert_eq!(record.errors[0].path, "schema_version");
        for i in 0..15 {
            state.upload(request(&format!("{i}"))).unwrap();
        }
        assert_eq!(
            state.upload(request("full")).unwrap_err().0,
            StatusCode::SERVICE_UNAVAILABLE
        );
        let mut retry = request("invalid");
        retry.bundle["schema_version"] = 99.into();
        assert!(state.upload(retry).is_ok());
    }

    #[test]
    fn empty_status_does_not_invent_sensor_or_arm_state() {
        let status = CtvState::default().status();
        for field in ["voltage_v", "current_a", "soc"] {
            assert!(status["battery"][field].is_null());
        }
        assert!(status["readiness"]["ready"].is_null());
        assert!(status["ec"]["arming_state"].is_null());
        assert!(status["run"]["run_id"].is_null());
    }
}
