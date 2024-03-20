use std::{sync::Arc, collections::HashMap};
use local_ip_address::local_ip;

use tauri::{Window, State, Manager, App};
use futures::{future::Map, lock::Mutex};
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

#[tauri::command]
pub async fn update_feedsystem(window: Window, value: String, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).feedsystem = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn get_feedsystem(window: Window, state: State<'_, Arc<Mutex<AppState>>>) -> Result<String, ()> {
  let inner_state = Arc::clone(&state);
  let value = &(*inner_state.lock().await).feedsystem;
  return Ok(value.into());
}

#[tauri::command]
pub async fn update_configs(window: Window, value: Vec<Config>, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  println!("updating configs!");
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).configs = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn update_active_config(window: Window, value: String, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  println!("updating active config to {}", value);
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).activeConfig = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn update_sequences(window: Window, value: Vec<Sequence>, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  println!("updating sequences!");
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).sequences = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn update_triggers(window: Window, value: Vec<Trigger>, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  println!("updating triggers!");
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).triggers = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tauri::command]
pub async fn update_calibrations(window: Window, value: HashMap<String, f64>, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  println!("updating calibrations!");
  let inner_state = Arc::clone(&state);
  (*inner_state.lock().await).calibrations = value;
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}



#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Mapping {
  pub text_id: String,
  pub board_id: String,
  pub sensor_type: String,
  pub channel: u64,
  pub computer: String,
  pub min: Option<f64>,
  pub max: Option<f64>,
  pub powered_threshold: Option<f64>,
  pub normally_closed: Option<bool>
}

#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Config {
  pub id: String,
  pub mappings: Vec<Mapping>
}

#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Sequence {
  pub name: String,
  pub configuration_id: String,
  pub script: String
}

#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Trigger {
  pub name: String,
  pub script: String,
  pub active: bool,
  pub condition: String
}


#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct AppState {
  pub selfIp: String,
  pub selfPort: u16,
  pub sessionId: String,
  pub forwardingId: String,
  pub serverIp: String,
  pub isConnected: bool,
  //activity: u64,
  pub alerts: Vec<Alert>,
  pub feedsystem: String,
  pub configs: Vec<Config>,
  pub activeConfig: String,
  pub sequences: Vec<Sequence>,
  pub calibrations: HashMap<String, f64>,
  pub triggers: Vec<Trigger>
}