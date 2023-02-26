#![cfg_attr(
  all(not(debug_assertions), target_os = "windows"),
  windows_subsystem = "windows"
)]

use comm::Connection;
use local_ip_address::local_ip;
use reqwest::Client;
use portpicker::pick_unused_port;
use std::sync::Arc;
use futures::lock::{Mutex, MutexGuard};
use tauri::{State, Manager, App, Window, AppHandle};
use state::{AppState, 
  update_is_connected, 
  update_server_ip, 
  update_self_ip, 
  update_session_id, 
  update_forwarding_id, 
  add_alert
};

mod auth;
mod comm;
mod utilities;
mod state;

#[tauri::command]
async fn initialize_state(window: Window, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  let inner_state = Arc::clone(&state);
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

fn main() {
  tauri::Builder::default()
  .manage(Arc::new(Mutex::new(AppState {
    selfIp: match local_ip() {
      Ok(ip) => ip.to_string(),
      Err(_err) => "No network".into()
    },
    selfPort: pick_unused_port().unwrap_or_else(||0),
    sessionId: "None".into(),
    forwardingId: "None".into(), 
    serverIp: "-".into(), 
    isConnected: false, 
    //activity: 0,
    alerts: Vec::new()
  })))
  .invoke_handler(tauri::generate_handler![
    initialize_state, 
    update_is_connected, 
    update_server_ip,
    update_self_ip,
    update_session_id,
    update_forwarding_id,
    add_alert,  
  ])
  .run(tauri::generate_context!())
  .expect("error while running tauri application");
  println!("HELLO!")
}
