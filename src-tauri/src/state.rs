use std::sync::Arc;
use local_ip_address::local_ip;

use tauri::{Window, State, Manager};
use futures::lock::Mutex;
use crate::utilities::{Alert};

#[tauri::command]
pub async fn update_is_connected(window: Window, value: bool, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).isConnected = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn update_server_ip(window: Window, value: String, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).serverIp = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn update_self_ip(window: Window, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).selfIp = match local_ip() {Ok(ip) => ip.to_string(), Err(_) => "No network".into()};
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn update_session_id(window: Window, value: String, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).sessionId = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn update_forwarding_id(window: Window, value: String, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).forwardingId = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn add_alert(window: Window, value: Alert, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  let inner_state = Arc::clone(&state);
  if (*inner_state.lock().await).alerts.len() + 1 > 10 {
    (*inner_state.lock().await).alerts.pop();
  }
  (*inner_state.lock().await).alerts.insert(0, value);
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[derive(Clone, serde::Serialize)]
pub struct AppState {
  pub selfIp: String,
  pub selfPort: u16,
  pub sessionId: String,
  pub forwardingId: String,
  pub serverIp: String,
  pub isConnected: bool,
  //activity: u64,
  pub alerts: Vec<Alert>,
  pub feedsystem: String
}