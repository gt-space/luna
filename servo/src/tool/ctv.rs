#![allow(missing_docs)]

use clap::{Arg, ArgAction, ArgMatches, Command};
use common::ctv_config::{content_hash, Bundle, MAX_BUNDLE_BYTES};
use serde_json::{json, Value};
use std::{
    fs::File,
    io::Read,
    thread,
    time::{Duration, Instant},
};

#[derive(Debug)]
struct Error {
    code: i32,
    reason: String,
    details: Value,
}
type Result<T> = std::result::Result<T, Error>;
fn error(code: i32, reason: impl ToString) -> Error {
    Error {
        code,
        reason: reason.to_string(),
        details: Value::Null,
    }
}

pub fn command() -> Command {
    let file = || Arg::new("file").required(true);
    let watch = || Arg::new("watch").long("watch").action(ArgAction::SetTrue);
    Command::new("ctv")
        .about("CTV operator readouts and configuration candidates")
        .subcommand_required(true)
        .arg(
            Arg::new("server")
                .long("server")
                .global(true)
                .default_value("localhost"),
        )
        .arg(
            Arg::new("json")
                .long("json")
                .global(true)
                .action(ArgAction::SetTrue),
        )
        .arg(
            Arg::new("timeout")
                .long("timeout")
                .global(true)
                .default_value("5")
                .value_parser(clap::value_parser!(f64)),
        )
        .arg(
            Arg::new("interval-ms")
                .long("interval-ms")
                .global(true)
                .default_value("100")
                .value_parser(clap::value_parser!(u64).range(10..=60000)),
        )
        .subcommand(Command::new("status").arg(watch()))
        .subcommand(Command::new("battery").arg(watch()))
        .subcommand(Command::new("health").arg(watch()))
        .subcommand(
            Command::new("config")
                .subcommand_required(true)
                .subcommand(Command::new("validate").arg(file()))
                .subcommand(
                    Command::new("show")
                        .arg(Arg::new("file").long("file"))
                        .arg(
                            Arg::new("bundle-id")
                                .long("bundle-id")
                                .conflicts_with("file"),
                        )
                        .arg(
                            Arg::new("state")
                                .long("state")
                                .value_parser([
                                    "uploaded",
                                    "validation",
                                    "validated",
                                    "staged",
                                    "active",
                                    "rejected",
                                    "previous_known_good",
                                ])
                                .conflicts_with("file"),
                        )
                        .arg(Arg::new("section").long("section").value_parser([
                            "controller",
                            "navigation",
                            "fdir",
                            "tvc",
                            "ec",
                            "startup",
                            "workers",
                            "logging",
                            "network",
                            "timing",
                            "safety",
                        ])),
                )
                .subcommand(
                    Command::new("diff")
                        .arg(file())
                        .arg(Arg::new("baseline").long("baseline"))
                        .arg(
                            Arg::new("bundle-id")
                                .long("bundle-id")
                                .conflicts_with("baseline"),
                        ),
                )
                .subcommand(Command::new("upload").arg(file()))
                .subcommand(Command::new("activate").arg(Arg::new("bundle-id").required(true)))
                .subcommand(Command::new("rollback")),
        )
        .subcommand(
            Command::new("run")
                .subcommand_required(true)
                .subcommand(Command::new("show"))
                .subcommand(
                    Command::new("start").arg(Arg::new("label").long("label").required(true)),
                ),
        )
}

mod client {
    use super::*;

    pub(super) struct Client {
        http: reqwest::blocking::Client,
        base: reqwest::Url,
    }

    impl Client {
        pub fn new(server: &str, timeout: f64) -> Result<Self> {
            if !timeout.is_finite() || timeout <= 0.0 || timeout > 3600.0 {
                return Err(error(2, "timeout must be between 0 and 3600 seconds"));
            }
            let explicit_scheme = server.contains("://");
            let mut base = reqwest::Url::parse(&if explicit_scheme {
                server.to_owned()
            } else {
                format!("http://{server}")
            })
            .map_err(|e| error(2, e))?;
            if !["http", "https"].contains(&base.scheme())
                || base.host_str().is_none()
                || !base.username().is_empty()
                || base.password().is_some()
                || base.query().is_some()
                || base.fragment().is_some()
                || base.path() != "/"
            {
                return Err(error(
                    2,
                    "server must be an HTTP(S) origin, optionally with a port",
                ));
            }
            if !explicit_scheme && base.port().is_none() {
                base.set_port(Some(7200))
                    .map_err(|_| error(2, "invalid server port"))?;
            }
            let http = reqwest::blocking::Client::builder()
                .timeout(Duration::from_secs_f64(timeout))
                .redirect(reqwest::redirect::Policy::none())
                .build()
                .map_err(|e| error(3, e))?;
            Ok(Self { http, base })
        }

        fn request(&self, route: &str, body: Option<&Value>) -> Result<Value> {
            let url = self.base.join(route).map_err(|e| error(2, e))?;
            let mutation = body.is_some();
            let request = if let Some(body) = body {
                self.http.post(url).json(body)
            } else {
                self.http.get(url)
            };
            let response = request.send().map_err(|e| {
                error(
                    if mutation {
                        6
                    } else if e.is_timeout() {
                        5
                    } else {
                        3
                    },
                    if mutation {
                        format!("request failed; outcome unknown; do not replay automatically: {e}")
                    } else {
                        e.to_string()
                    },
                )
            })?;
            let status = response.status();
            let mut bytes = Vec::new();
            response
                .take(20 * MAX_BUNDLE_BYTES as u64 + 1)
                .read_to_end(&mut bytes)
                .map_err(|e| error(if mutation { 6 } else { 3 }, e))?;
            if bytes.len() > 20 * MAX_BUNDLE_BYTES {
                return Err(error(
                    if mutation { 6 } else { 7 },
                    "response exceeds limit",
                ));
            }
            let value = serde_json::from_slice::<Value>(&bytes);
            if !status.is_success() {
                let code = match status.as_u16() {
                    404 | 501 => 7,
                    400 | 409 | 413 | 422 => 4,
                    _ => 3,
                };
                return Err(Error {
                    code,
                    reason: format!("HTTP {status}: {}", String::from_utf8_lossy(&bytes)),
                    details: value.unwrap_or(Value::Null),
                });
            }
            value.map_err(|e| {
                error(
                    if mutation { 6 } else { 7 },
                    format!(
                        "invalid response{}: {e}",
                        if mutation { "; outcome unknown" } else { "" }
                    ),
                )
            })
        }

        pub fn status(&self) -> Result<Value> {
            let mut status = self.request("/operator/ctv/status", None)?;
            if status["protocol_version"] != 1 {
                return Err(error(7, "unsupported operator status protocol"));
            }
            status["transport"] =
                json!({"source": "servo", "stats": self.request("/data/telemetry-stats", None)?});
            status["mapping_configuration"] = self.mapping_configuration()?;
            Ok(status)
        }

        fn mapping_configuration(&self) -> Result<Value> {
            Ok(match self.request("/operator/active-configuration", None) {
                Ok(value) => {
                    json!({"source": "servo_database", "configuration_id": value["configuration_id"]})
                }
                Err(e) if e.code == 7 => {
                    json!({"source": "servo_database", "configuration_id": null, "reason": "no active mapping configuration"})
                }
                Err(e) => return Err(e),
            })
        }

        pub fn config(&self) -> Result<Value> {
            let mut value = self.request("/operator/ctv/config", None)?;
            value["mapping_configuration"] = self.mapping_configuration()?;
            Ok(value)
        }
        pub fn run(&self) -> Result<Value> {
            self.request("/operator/ctv/run", None)
        }

        pub fn upload(&self, bundle: &Bundle) -> Result<Value> {
            let hash = content_hash(bundle).map_err(|e| error(4, e))?;
            let request_id = format!("upload-{hash}");
            let value = self.request(
                "/operator/ctv/config",
                Some(&json!({"request_id": request_id, "bundle": bundle})),
            )?;
            if value["authority"] != "servo"
                || value["outcome"] != "validated"
                || value["record"]["request_id"] != request_id
                || value["record"]["bundle_id"] != hash
                || value["record"]["state"] != "validated"
            {
                return Err(error(6, "upload outcome is unknown: response did not confirm the candidate identity and validation result"));
            }
            Ok(value)
        }

        pub fn mutation(&self, command: &str, target: Option<&str>) -> Result<Value> {
            let value = self.request(
                "/operator/command",
                Some(&json!({"command": command, "target": target})),
            )?;
            Err(Error { code: 6, reason: "outcome unknown: authoritative FC/EC transaction result is unavailable; do not replay automatically".into(), details: value })
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        fn response(status: &str, body: Value) -> (String, std::thread::JoinHandle<()>) {
            use std::io::{BufRead, BufReader, Write};
            let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            let address = listener.local_addr().unwrap().to_string();
            let status = status.to_owned();
            let task = std::thread::spawn(move || {
                let (mut socket, _) = listener.accept().unwrap();
                socket
                    .set_read_timeout(Some(Duration::from_secs(2)))
                    .unwrap();
                let mut reader = BufReader::new(&socket);
                let mut content_length = 0;
                loop {
                    let mut line = String::new();
                    reader.read_line(&mut line).unwrap();
                    if line == "\r\n" {
                        break;
                    }
                    if let Some(length) = line.to_lowercase().strip_prefix("content-length:") {
                        content_length = length.trim().parse().unwrap();
                    }
                }
                let mut request = vec![0; content_length];
                reader.read_exact(&mut request).unwrap();
                let body = body.to_string();
                write!(
                    socket,
                    "HTTP/1.1 {status}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
                    body.len()
                )
                .unwrap();
            });
            (address, task)
        }

        #[test]
        fn http_success_without_result_is_unknown() {
            let (address, task) = response("200 OK", json!({"outcome": "accepted"}));
            assert_eq!(
                Client::new(&address, 2.0)
                    .unwrap()
                    .mutation("ctv_run_start", Some("bench"))
                    .unwrap_err()
                    .code,
                6
            );
            task.join().unwrap();
        }

        #[test]
        fn upload_checks_authority_and_content_identity() {
            let bundle =
                Bundle::parse(include_bytes!("../../../common/examples/ctv-bench.json")).unwrap();
            let (address, task) = response(
                "200 OK",
                json!({"authority": "servo", "outcome": "validated", "record": {"state": "validated", "bundle_id": "wrong", "request_id": "wrong"}}),
            );
            assert_eq!(
                Client::new(&address, 2.0)
                    .unwrap()
                    .upload(&bundle)
                    .unwrap_err()
                    .code,
                6
            );
            task.join().unwrap();
        }

        #[test]
        fn unsupported_mutation_is_explicit() {
            let (address, task) = response(
                "501 Not Implemented",
                json!({"reason": "EC transactions unavailable"}),
            );
            assert_eq!(
                Client::new(&address, 2.0)
                    .unwrap()
                    .mutation("ctv_config_rollback", None)
                    .unwrap_err()
                    .code,
                7
            );
            task.join().unwrap();
        }

        #[test]
        fn server_origin_and_timeout_are_checked() {
            assert_eq!(
                Client::new("localhost", 1.0).unwrap().base.as_str(),
                "http://localhost:7200/"
            );
            assert_eq!(
                Client::new("https://example.com", 1.0)
                    .unwrap()
                    .base
                    .as_str(),
                "https://example.com/"
            );
            for server in [
                "ftp://example.com",
                "http://example.com/path",
                "http://user:pass@example.com",
                "http://example.com?x=1",
            ] {
                assert!(Client::new(server, 1.0).is_err());
            }
            assert!(Client::new("localhost", f64::NAN).is_err());
            assert!(Client::new("localhost", 0.0).is_err());
        }
    }
}

fn candidate(path: &str) -> Result<Bundle> {
    let mut bytes = Vec::new();
    File::open(path)
        .map_err(|e| error(4, e))?
        .take(MAX_BUNDLE_BYTES as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| error(4, e))?;
    Bundle::parse(&bytes).map_err(|errors| Error {
        code: 4,
        reason: "candidate validation failed".into(),
        details: json!(errors),
    })
}

fn candidate_view(path: &str, bundle: &Bundle) -> Result<Value> {
    Ok(
        json!({"source": "stored_file", "file": path, "state": "candidate", "bundle_id": content_hash(bundle).map_err(|e| error(4, e))?,
        "validation_scope": "schema_only; FC/EC compatibility and hardware calibration are unverified", "bundle": bundle}),
    )
}

fn diff(path: &str, before: &Value, after: &Value, changes: &mut Vec<Value>) {
    if before == after {
        return;
    }
    if let (Some(left), Some(right)) = (before.as_object(), after.as_object()) {
        let keys: std::collections::BTreeSet<_> = left.keys().chain(right.keys()).collect();
        for key in keys {
            diff(&format!("{path}.{key}"), &before[key], &after[key], changes);
        }
    } else if let (Some(left), Some(right)) = (before.as_array(), after.as_array()) {
        for i in 0..left.len().max(right.len()) {
            diff(
                &format!("{path}[{i}]"),
                left.get(i).unwrap_or(&Value::Null),
                right.get(i).unwrap_or(&Value::Null),
                changes,
            );
        }
    } else {
        changes.push(json!({"path": path, "before": before, "after": after}));
    }
}

fn section_view(data: &mut Value, section: &str) {
    if let Some(bundle) = data.get_mut("bundle").filter(|b| b.is_object()) {
        *bundle = json!({section: bundle[section]});
    }
    if let Some(records) = data.get_mut("records").and_then(Value::as_array_mut) {
        for record in records {
            section_view(record, section);
        }
    }
    if let Some(sections) = data.pointer_mut("/running/fc/sections") {
        *sections = json!({section: sections[section]});
    }
}

fn age_last_value(value: &mut Value, elapsed_ms: f64) {
    match value {
        Value::Object(map) => {
            for (key, value) in map {
                if matches!(
                    key.as_str(),
                    "age_ms" | "observed_age_ms" | "time_since_update_ms"
                ) {
                    if let Some(age) = value.as_f64() {
                        *value = json!(age + elapsed_ms);
                    }
                } else {
                    age_last_value(value, elapsed_ms);
                }
            }
        }
        Value::Array(values) => {
            for value in values {
                age_last_value(value, elapsed_ms);
            }
        }
        _ => {}
    }
}

fn stale(mut value: Value, since: Instant, reason: &str) -> Value {
    age_last_value(&mut value, since.elapsed().as_secs_f64() * 1000.0);
    value["connection"] = "lost_connection_stale".into();
    value["last_error"] = reason.into();
    if value["battery"].is_object() {
        value["battery"]["connection"] = "lost_connection_stale".into();
    }
    value
}

fn read_exit(operation: &str, value: &Value) -> i32 {
    if value["connection"] != "connected" {
        3
    } else if (operation == "battery" && value["battery"]["validity"] != "valid")
        || (operation == "health" && value["readiness"]["ready"] != true)
    {
        8
    } else {
        0
    }
}

fn render(operation: &str, value: &Value, json_mode: bool) -> Result<()> {
    let output = if operation == "battery" {
        json!({"connection": value["connection"], "battery": value["battery"], "ec_rails": value["ec"]["rails"], "transport": value["transport"], "last_error": value["last_error"]})
    } else if operation == "health" {
        json!({"connection": value["connection"], "fc": value["fc"], "ec": value["ec"], "readiness": value["readiness"], "first_fault": value["first_fault"], "fault_response": value["fault_response"], "last_error": value["last_error"]})
    } else {
        value.clone()
    };
    if json_mode {
        println!(
            "{}",
            serde_json::to_string(&output).map_err(|e| error(7, e))?
        );
        return Ok(());
    }
    if !["status", "battery", "health"].contains(&operation) {
        println!(
            "{}",
            serde_json::to_string_pretty(&output).map_err(|e| error(7, e))?
        );
        return Ok(());
    }
    let text = |v: &Value| {
        if v.is_null() {
            "unavailable".to_owned()
        } else if let Some(v) = v.as_str() {
            v.to_owned()
        } else {
            v.to_string()
        }
    };
    println!("Connection: {}", text(&value["connection"]));
    if !value["last_error"].is_null() {
        println!("Link detail: {}", text(&value["last_error"]));
    }
    if operation != "battery" {
        println!(
            "Vehicle: {} | Mode: {}",
            text(&value["vehicle_id"]),
            text(&value["mode"])
        );
        println!(
            "FC boot: {} | live FC age: {} ms",
            text(&value["fc"]["boot_id"]),
            text(&value["fc"]["age_ms"])
        );
        println!(
            "EC boot: {} | Arming: {} | Output authority: {}",
            text(&value["ec"]["boot_id"]),
            text(&value["ec"]["arming_state"]),
            text(&value["ec"]["output_authority"])
        );
        println!("EC: {}", text(&value["ec"]["reason"]));
        println!(
            "Mapping ID (Servo database): {}",
            text(&value["mapping_configuration"]["configuration_id"])
        );
        println!(
            "Active bundle (FC / EC): {} / {}",
            text(&value["configuration"]["fc"]["bundle_id"]),
            text(&value["configuration"]["ec"]["bundle_id"])
        );
        println!(
            "First fault: {} | Response: {}",
            text(&value["first_fault"]),
            text(&value["fault_response"])
        );
        println!("Readiness: {}", text(&value["readiness"]["reason"]));
        if let Some(conditions) = value["readiness"]["conditions"].as_array() {
            for condition in conditions {
                println!(
                    "  {}: {}",
                    text(&condition["condition"]),
                    text(&condition["met"])
                );
            }
        }
        println!("Run ID: {}", text(&value["run"]["run_id"]));
    }
    if operation != "health" {
        let battery = &value["battery"];
        println!(
            "Battery (BMS): {} V | {} A | {} W (calculated) | SoC: {}",
            text(&battery["voltage_v"]),
            text(&battery["current_a"]),
            text(&battery["power_w"]),
            text(&battery["soc"])
        );
        println!("  Observed age: {} ms | Measurement age: {} | Validity: {} | Clock mapping: unavailable", text(&battery["observed_age_ms"]), text(&battery["measurement_age_ms"]), text(&battery["validity"]));
        println!("EC rails: {}", text(&value["ec"]["rails"]));
        for source in ["umbilical", "tel"] {
            let stats = &value["transport"]["stats"][source];
            println!(
                "{source} transport: age {} ms | {} Hz | {} bytes",
                text(&stats["time_since_update_ms"]),
                text(&stats["update_rate_hz"]),
                text(&stats["packet_size_bytes"])
            );
        }
    }
    Ok(())
}

fn execute(args: &ArgMatches) -> Result<i32> {
    let json_mode = args.get_flag("json");
    let timeout = *args
        .get_one::<f64>("timeout")
        .ok_or_else(|| error(2, "timeout missing"))?;
    let server = args
        .get_one::<String>("server")
        .ok_or_else(|| error(2, "server missing"))?;
    let (operation, sub) = args
        .subcommand()
        .ok_or_else(|| error(2, "command required"))?;
    if ["status", "battery", "health"].contains(&operation) {
        let client = client::Client::new(server, timeout)?;
        let mut last: Option<(Value, Instant)> = None;
        loop {
            match client.status() {
                Ok(value) => {
                    let exit = read_exit(operation, &value);
                    render(operation, &value, json_mode)?;
                    last = Some((value, Instant::now()));
                    if !sub.get_flag("watch") {
                        return Ok(exit);
                    }
                }
                Err(e) => {
                    if !sub.get_flag("watch") {
                        return Err(e);
                    }
                    let value = match &last {
                        Some((value, since)) => stale(value.clone(), *since, &e.reason),
                        None => json!({"connection": "not_connected", "last_error": e.reason}),
                    };
                    render(operation, &value, json_mode)?;
                }
            }
            thread::sleep(Duration::from_millis(
                *args
                    .get_one::<u64>("interval-ms")
                    .ok_or_else(|| error(2, "interval missing"))?,
            ));
        }
    }
    if operation == "run" {
        let client = client::Client::new(server, timeout)?;
        let (action, options) = sub
            .subcommand()
            .ok_or_else(|| error(2, "run action required"))?;
        let value = if action == "show" {
            client.run()?
        } else {
            client.mutation(
                "ctv_run_start",
                options.get_one::<String>("label").map(String::as_str),
            )?
        };
        render("run", &value, json_mode)?;
        return Ok(if value["available"] == false { 7 } else { 0 });
    }
    let (action, options) = sub
        .subcommand()
        .ok_or_else(|| error(2, "config action required"))?;
    let value = match action {
        "validate" => {
            let path = options
                .get_one::<String>("file")
                .ok_or_else(|| error(2, "file required"))?;
            let bundle = candidate(path)?;
            json!({"source": "stored_file", "file": path, "state": "candidate", "bundle_id": content_hash(&bundle).map_err(|e| error(4, e))?,
                "schema_valid": true, "validation_scope": "schema_only; FC/EC compatibility and hardware calibration are unverified", "errors": []})
        }
        "show" => {
            let mut value = if let Some(path) = options.get_one::<String>("file") {
                candidate_view(path, &candidate(path)?)?
            } else {
                client::Client::new(server, timeout)?.config()?
            };
            if let Some(id) = options.get_one::<String>("bundle-id") {
                let records = value["records"]
                    .as_array_mut()
                    .ok_or_else(|| error(7, "configuration records unavailable"))?;
                records.retain(|record| record["bundle_id"] == *id);
                if records.is_empty() {
                    return Err(error(4, "bundle ID not found in Servo candidate store"));
                }
            }
            if let Some(state) = options.get_one::<String>("state") {
                if let Some(records) = value["records"].as_array_mut() {
                    records.retain(|record| record["state"] == *state);
                }
                value["requested_state"] = state.clone().into();
            }
            if let Some(section) = options.get_one::<String>("section") {
                section_view(&mut value, section);
            }
            value
        }
        "diff" => {
            let path = options
                .get_one::<String>("file")
                .ok_or_else(|| error(2, "file required"))?;
            let after = candidate_view(path, &candidate(path)?)?;
            let before = if let Some(path) = options.get_one::<String>("baseline") {
                candidate_view(path, &candidate(path)?)?
            } else {
                let configs = client::Client::new(server, timeout)?.config()?;
                if let Some(id) = options.get_one::<String>("bundle-id") {
                    configs["records"]
                        .as_array()
                        .and_then(|records| {
                            records.iter().find(|record| {
                                record["bundle_id"] == *id && record["state"] == "validated"
                            })
                        })
                        .cloned()
                        .ok_or_else(|| error(4, "validated baseline bundle was not found"))?
                } else {
                    return Err(error(7, "running FC/EC bundle readback is unavailable; specify --baseline <file> or --bundle-id <uploaded-hash>"));
                }
            };
            let mut changes = vec![];
            diff("$", &before["bundle"], &after["bundle"], &mut changes);
            json!({"baseline_source": before["source"], "baseline_state": before["state"], "baseline_hash": before["bundle_id"], "candidate_hash": after["bundle_id"], "changes": changes})
        }
        "upload" => {
            let path = options
                .get_one::<String>("file")
                .ok_or_else(|| error(2, "file required"))?;
            client::Client::new(server, timeout)?.upload(&candidate(path)?)?
        }
        "activate" => client::Client::new(server, timeout)?.mutation(
            "ctv_config_activate",
            options.get_one::<String>("bundle-id").map(String::as_str),
        )?,
        "rollback" => {
            client::Client::new(server, timeout)?.mutation("ctv_config_rollback", None)?
        }
        _ => return Err(error(2, "unknown config action")),
    };
    render("config", &value, json_mode)?;
    Ok(0)
}

pub fn run(args: &ArgMatches) -> i32 {
    match execute(args) {
        Ok(code) => code,
        Err(e) => {
            if args.get_flag("json") {
                println!(
                    "{}",
                    json!({"outcome": "error", "exit_code": e.code, "reason": e.reason, "details": e.details})
                );
            } else {
                eprintln!("{}", e.reason);
                if !e.details.is_null() {
                    eprintln!("{}", e.details);
                }
            }
            e.code
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cli_accepts_global_flags_after_subcommands() {
        let args = command()
            .try_get_matches_from([
                "ctv",
                "config",
                "validate",
                "file.json",
                "--json",
                "--timeout",
                "2",
            ])
            .unwrap();
        assert!(args.get_flag("json"));
        assert_eq!(args.get_one::<f64>("timeout"), Some(&2.0));
    }
    #[test]
    fn stale_values_age_without_refreshing_measurement_identity() {
        let value = json!({"fc": {"age_ms": 10.0}, "battery": {"sample_time_s": 42.0, "observed_age_ms": 5000.0, "measurement_age_ms": null}});
        let since = Instant::now() - Duration::from_secs(1);
        let stale = stale(value, since, "link lost");
        assert_eq!(stale["connection"], "lost_connection_stale");
        assert!(stale["battery"]["observed_age_ms"].as_f64().unwrap() >= 6000.0);
        assert_eq!(stale["battery"]["sample_time_s"], 42.0);
        assert!(stale["battery"]["measurement_age_ms"].is_null());
    }
    #[test]
    fn diff_reports_array_indices() {
        let mut changes = vec![];
        diff(
            "$",
            &json!({"a": [1, 2]}),
            &json!({"a": [1, 3]}),
            &mut changes,
        );
        assert_eq!(changes[0]["path"], "$.a[1]");
    }

    #[test]
    fn section_selection_preserves_candidate_identity() {
        let mut value = json!({"bundle_id": "hash", "source": "stored_file", "bundle": {"controller": {"gain": 1}, "navigation": {}}});
        section_view(&mut value, "controller");
        assert_eq!(value["bundle_id"], "hash");
        assert_eq!(value["bundle"], json!({"controller": {"gain": 1}}));
    }

    #[test]
    fn read_exit_codes_distinguish_link_and_measurement_availability() {
        let value = json!({"connection": "connected", "battery": {"validity": "age_unverified"}, "readiness": {"ready": null}});
        assert_eq!(read_exit("status", &value), 0);
        assert_eq!(read_exit("battery", &value), 8);
        assert_eq!(read_exit("health", &value), 8);
        assert_eq!(read_exit("status", &json!({"connection": "lost_connection_stale"})), 3);
    }
}
