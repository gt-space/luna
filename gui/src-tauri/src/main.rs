#![cfg_attr(
    all(not(debug_assertions), target_os = "windows"),
    windows_subsystem = "windows"
)]

use futures::lock::Mutex;
use local_ip_address::local_ip;
use state::{
    add_alert, get_feedsystem, update_abort_stages, update_active_abort_stage,
    update_active_config, update_calibrations, update_configs, update_current_data_source,
    update_feedsystem, update_forwarding_id, update_is_connected, update_self_ip, update_sequences,
    update_server_ip, update_session_id, AppState,
};
use std::{collections::HashMap, sync::Arc};
use tauri::{Emitter, Manager, State, Window};
use tokio::net::UdpSocket;

mod state;
mod utilities;

#[derive(serde::Deserialize)]
struct YorgExport {
    servo: String,
    folder_url: String,
    source: String,
    name: String,
    from: f64,
    to: f64,
    verifier: String,
    challenge: String,
    nonce: String,
}

#[derive(serde::Serialize)]
struct YorgExportResult {
    url: String,
    no_data: bool,
}

fn yorg_destination(value: &str) -> Result<(reqwest::Url, Option<i64>), String> {
    if value.trim().is_empty() {
        return Ok((reqwest::Url::parse("https://lucazerega.tail07089c.ts.net/").unwrap(), None));
    }
    let mut site = reqwest::Url::parse(value.trim()).map_err(|_| "Paste the Yorg folder's full URL.")?;
    let local = matches!(site.host_str(), Some("localhost" | "127.0.0.1" | "[::1]"));
    if !(site.scheme() == "https" || (site.scheme() == "http" && local))
        || site.host_str().is_none() || !site.username().is_empty() || site.password().is_some()
        || site.query().is_some() || site.fragment().is_some() {
        return Err("Use an HTTPS Yorg folder URL. HTTP is allowed only on localhost.".into());
    }
    let folder = site.path().trim_end_matches('/').strip_prefix("/yorg/folders/")
        .and_then(|id| id.parse::<i64>().ok()).filter(|id| *id > 0)
        .ok_or("Open the destination folder in Yorg and copy its URL (/yorg/folders/…).")?;
    site.set_path("/");
    Ok((site, Some(folder)))
}

fn yorg_redirect_response(url: &str) -> String {
    format!("HTTP/1.1 303 See Other\r\nLocation: {url}\r\nCache-Control: no-store\r\nReferrer-Policy: no-referrer\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
}

// Accept only the matching loopback callback; keep the issued token in native memory.
async fn yorg_sign_in(app: &tauri::AppHandle, client: &reqwest::Client, site: &reqwest::Url, input: &YorgExport) -> Result<(String, tokio::sync::oneshot::Sender<String>), String> {
    use tauri_plugin_shell::ShellExt;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    let safe = |value: &str| value.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_');
    if input.verifier.len() != 43 || input.challenge.len() != 43 || input.nonce.len() != 43
        || !safe(&input.verifier) || !safe(&input.challenge) || !safe(&input.nonce) {
        return Err("Couldn't start Yorg sign-in. Please try again.".into());
    }
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.map_err(|_| "Couldn't start Yorg sign-in.")?;
    let port = listener.local_addr().map_err(|_| "Couldn't start Yorg sign-in.")?.port();
    let mut login = site.join("auth/client").map_err(|_| "Invalid Yorg address.")?;
    login.query_pairs_mut().append_pair("port", &port.to_string()).append_pair("state", &input.nonce)
        .append_pair("challenge", &input.challenge).append_pair("name", "Luna export");
    #[allow(deprecated)]
    app.shell().open(login.as_str(), None).map_err(|_| "Couldn't open the browser for Yorg sign-in.")?;
    let (code, mut callback_stream) = tokio::time::timeout(std::time::Duration::from_secs(180), async {
        loop {
            let (mut stream, _) = listener.accept().await.map_err(|_| "Couldn't receive Yorg sign-in.")?;
            let request = tokio::time::timeout(std::time::Duration::from_secs(3), async {
                let mut bytes = Vec::new();
                loop {
                    let mut chunk = [0; 1024];
                    let count = stream.read(&mut chunk).await.ok()?;
                    if count == 0 { return None; }
                    bytes.extend_from_slice(&chunk[..count]);
                    if bytes.len() > 8192 { return None; }
                    if bytes.windows(4).any(|w| w == b"\r\n\r\n") { break; }
                }
                String::from_utf8(bytes).ok()
            }).await.ok().flatten();
            let callback = request.as_deref().and_then(|r| r.lines().next())
                .and_then(|line| line.strip_prefix("GET ")).and_then(|line| line.strip_suffix(" HTTP/1.1"))
                .filter(|path| path.starts_with("/callback?"))
                .and_then(|path| reqwest::Url::parse(&format!("http://127.0.0.1{path}")).ok());
            let matching = callback.as_ref().map(|url| url.query_pairs().any(|(k, v)| k == "state" && v == input.nonce)).unwrap_or(false);
            if !matching {
                let _ = tokio::time::timeout(std::time::Duration::from_secs(2), stream.write_all(b"HTTP/1.1 400 Bad Request\r\nConnection: close\r\n\r\nInvalid sign-in callback.")).await;
                continue;
            }
            let callback = callback.unwrap();
            let code = if callback.query_pairs().any(|(k, _)| k == "error") {
                Err("Yorg sign-in was cancelled.")
            } else {
                callback.query_pairs().find(|(k, _)| k == "code").map(|(_, v)| v.into_owned())
                    .filter(|v| !v.is_empty()).ok_or("Yorg didn't return a sign-in code.")
            };
            return Ok::<_, &str>((code, stream));
        }
    }).await.map_err(|_| "Yorg sign-in timed out. Please try again.")??;
    // Keep this browser tab waiting for the upload, then redirect it to the result.
    // Dropping the sender on any error still takes the user back to Yorg.
    let (_, folder) = yorg_destination(&input.folder_url)?;
    let path = folder.map(|id| format!("yorg/folders/{id}")).unwrap_or_else(|| "yorg".into());
    let fallback = site.join(&path).map_err(|_| "Invalid Yorg address.")?.to_string();
    let (destination, redirect) = tokio::sync::oneshot::channel::<String>();
    tokio::spawn(async move {
        let url = match tokio::time::timeout(std::time::Duration::from_secs(180), redirect).await {
            Ok(Ok(url)) => url,
            _ => fallback,
        };
        let reply = yorg_redirect_response(&url);
        let _ = tokio::time::timeout(std::time::Duration::from_secs(2), callback_stream.write_all(reply.as_bytes())).await;
    });
    let code = code?;
    let response = client.post(site.join("auth/client/token").map_err(|_| "Invalid Yorg address.")?)
        .json(&serde_json::json!({"code": code, "verifier": input.verifier}))
        .send().await.map_err(|_| "Couldn't complete Yorg sign-in.")?
        .error_for_status().map_err(|_| "Yorg sign-in expired or was refused. Please try again.")?;
    let result = response.json::<serde_json::Value>().await.map_err(|_| "Couldn't read Yorg sign-in response.")?;
    let token = result["token"].as_str().filter(|token| !token.is_empty()).map(str::to_owned)
        .ok_or("Yorg didn't return an access token.")?;
    Ok((token, destination))
}

// Sign in through the browser and upload directly; never persist or log credentials.
#[tauri::command]
async fn export_to_yorg(app: tauri::AppHandle, input: YorgExport) -> Result<YorgExportResult, String> {
    use reqwest::redirect::Policy;

    let (site, folder) = yorg_destination(&input.folder_url)?;
    if !input.from.is_finite() || !input.to.is_finite() || input.from >= input.to {
        return Err("Choose an end time after the start time.".into());
    }
    if !matches!(input.source.as_str(), "umbilical" | "tel") || input.name.trim().is_empty()
        || input.name.chars().count() > 200 || input.servo.contains(['/', '?', '#', '@', ':']) {
        return Err("Check the Servo hostname, source, and dataset name.".into());
    }
    let client = reqwest::Client::builder().redirect(Policy::none())
        .timeout(std::time::Duration::from_secs(180)).build().map_err(|_| "Couldn't connect.")?;
    let csv = client.post(format!("http://{}:7200/data/export?source={}", input.servo, input.source))
        .json(&serde_json::json!({"format": "csv", "from": input.from, "to": input.to}))
        .send().await.map_err(|_| "Couldn't reach Servo.")?
        .error_for_status().map_err(|_| "Servo couldn't export this time range.")?;
    if !csv.headers().get("content-type").and_then(|value| value.to_str().ok())
        .unwrap_or("").starts_with("text/csv") {
        return Err("Servo didn't return CSV data. Nothing was uploaded.".into());
    }
    let csv = csv.bytes().await.map_err(|_| "Couldn't read Servo's export.")?;
    let no_data = !csv.split(|byte| *byte == b'\n').skip(1).any(|row| row.iter().any(|byte| !byte.is_ascii_whitespace()));

    let (token, destination) = yorg_sign_in(&app, &client, &site, &input).await?;
    let mut upload = site.join("api/yorg/datasets").map_err(|_| "Invalid Yorg address.")?;
    upload.query_pairs_mut().append_pair("filename", "servo.csv").append_pair("name", input.name.trim());
    if let Some(folder) = folder {
        upload.query_pairs_mut().append_pair("folder", &folder.to_string());
    }
    // Never retry an upload automatically: Yorg may have stored it even if the response is lost.
    let response = client.post(upload).bearer_auth(token).header("content-type", "text/csv")
        .body(csv).send().await.map_err(|_| "Upload response lost. Check Yorg before retrying to avoid a duplicate.")?;
    if matches!(response.status().as_u16(), 401 | 403) {
        return Err("Yorg refused access. Sign in again and check that your account is approved.".into());
    }
    if !response.status().is_success() {
        return Err(format!("Yorg returned HTTP {}. Check Yorg before retrying.", response.status().as_u16()));
    }
    let dataset = response.json::<serde_json::Value>().await
        .map_err(|_| "Upload response unreadable. Check Yorg before retrying.")?;
    let id = dataset["id"].as_u64().ok_or("Check Yorg for the uploaded dataset before retrying.")?;
    let suffix = if dataset["status"].as_str() == Some("ready") { "" } else { "/about" };
    let url = format!("{}yorg/datasets/{id}{suffix}", site);
    let _ = destination.send(url.clone());
    Ok(YorgExportResult { url, no_data })
}

#[cfg(test)]
mod yorg_destination_tests {
    use super::{yorg_destination, yorg_redirect_response};

    #[test]
    fn callback_redirects_without_exposing_the_sign_in_code() {
        let response = yorg_redirect_response("https://example.org/yorg/datasets/42");
        assert!(response.starts_with("HTTP/1.1 303 See Other\r\n"));
        assert!(response.contains("Location: https://example.org/yorg/datasets/42\r\n"));
        assert!(response.contains("Referrer-Policy: no-referrer\r\n"));
        assert!(response.ends_with("Content-Length: 0\r\nConnection: close\r\n\r\n"));
    }


    #[test]
    fn blank_folder_uses_the_main_directory() {
        for value in ["", "   "] {
            let (site, folder) = yorg_destination(value).unwrap();
            assert_eq!(site.as_str(), "https://lucazerega.tail07089c.ts.net/");
            assert_eq!(folder, None);
        }
    }

    #[test]
    fn folder_link_supplies_the_site_and_destination() {
        let (site, folder) = yorg_destination(" https://example.org/yorg/folders/42/ ").unwrap();
        assert_eq!(site.as_str(), "https://example.org/");
        assert_eq!(folder, Some(42));
        let (site, folder) = yorg_destination("http://localhost:8000/yorg/folders/7").unwrap();
        assert_eq!(site.as_str(), "http://localhost:8000/");
        assert_eq!(folder, Some(7));
    }

    #[test]
    fn rejects_non_folder_links_and_unsafe_destinations() {
        for url in ["https://example.org", "https://example.org/yorg/datasets/42",
            "https://example.org/yorg/folders/0", "https://example.org/yorg/folders/abc",
            "http://example.org/yorg/folders/1", "https://user:password@example.org/yorg/folders/1",
            "https://example.org/yorg/folders/1?token=secret", "https://example.org/yorg/folders/1#other"] {
            assert!(yorg_destination(url).is_err(), "{url}");
        }
    }
}

#[tauri::command]
async fn initialize_state(
    window: Window,
    state: State<'_, Arc<Mutex<AppState>>>,
) -> Result<(), ()> {
    println!("initializing state!");
    let inner_state = Arc::clone(&state);
    let _ = window.emit("state", &*(inner_state.lock().await));
    return Ok(());
}

#[tokio::main]
async fn main() {
    let socket = UdpSocket::bind("0.0.0.0:0")
        .await
        .expect("Couldn't find a free port");
    let port = socket.local_addr().unwrap().port();

    // This is a workaround to prevent pages from turning black
    // The error occurs due to an issue with webkit rendering on linux
    #[cfg(target_os = "linux")]
    std::env::set_var("WEBKIT_DISABLE_COMPOSITING_MODE", "1");

    tauri::Builder::default()
        .plugin(tauri_plugin_fs::init())
        .plugin(tauri_plugin_clipboard_manager::init())
        .plugin(tauri_plugin_notification::init())
        .plugin(tauri_plugin_global_shortcut::Builder::new().build())
        .plugin(tauri_plugin_shell::init())
        .plugin(tauri_plugin_os::init())
        .plugin(tauri_plugin_http::init())
        .plugin(tauri_plugin_process::init())
        .plugin(tauri_plugin_dialog::init())
        .setup(move |app| {
            app.manage(Arc::new(Mutex::new(AppState {
                selfIp: match local_ip() {
                    Ok(ip) => ip.to_string(),
                    Err(_err) => "No network".into(),
                },
                selfPort: port,
                sessionId: "None".into(),
                forwardingId: "None".into(),
                currentDataSource: "umbilical".into(),
                serverIp: "-".into(),
                isConnected: false,
                alerts: Vec::new(),
                feedsystem: "Feedsystem1".into(),
                configs: Vec::new(),
                activeConfig: "".into(),
                sequences: Vec::new(),
                calibrations: HashMap::new(),
                abortStages: Vec::new(),
                activeAbortStage: "".into(),
            })));
            // let inner_state = Arc::clone(&app.state::<Arc<Mutex<AppState>>>());
            // let state = inner_state.try_lock();
            // app.manage(socket);
            Ok(())
        })
        .manage(socket)
        .invoke_handler(tauri::generate_handler![
            export_to_yorg,
            initialize_state,
            update_is_connected,
            update_server_ip,
            update_self_ip,
            update_session_id,
            update_forwarding_id,
            update_current_data_source,
            add_alert,
            update_feedsystem,
            get_feedsystem,
            update_configs,
            update_active_config,
            update_sequences,
            update_calibrations,
            update_abort_stages,
            update_active_abort_stage
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
