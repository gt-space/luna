#![cfg_attr(
  all(not(debug_assertions), target_os = "windows"),
  windows_subsystem = "windows"
)]

use local_ip_address::local_ip;
use std::{sync::Arc, time::Duration};
use tokio::net::UdpSocket;
use futures::lock::Mutex;
use tauri::{State, Manager, Window, window, WindowBuilder};
use state::{AppState, 
  update_is_connected, 
  update_server_ip, 
  update_self_ip, 
  update_session_id, 
  update_forwarding_id, 
  add_alert
};
use comm::receive_data;

mod comm;
mod utilities;
mod state;

#[tauri::command]
async fn initialize_state(window: Window, state: State<'_, Arc<Mutex<AppState>>>) -> Result<(), ()> {
  let inner_state = Arc::clone(&state);
  window.emit_all("state", &*(inner_state.lock().await));
  return Ok(());
}

#[tokio::main]
async fn main() {
  let socket = UdpSocket::bind("0.0.0.0:0").await.expect("Couldn't find a free port");
  let port = socket.local_addr().unwrap().port();

  tauri::Builder::default()
  .setup( move |app| {
    app.manage(Arc::new(Mutex::new(AppState {
      selfIp: match local_ip() {
        Ok(ip) => ip.to_string(),
        Err(_err) => "No network".into()
      },
      selfPort: port,
      sessionId: "None".into(),
      forwardingId: "None".into(), 
      serverIp: "-".into(), 
      isConnected: false, 
      alerts: Vec::new(),
      feedsystem: "Feedsystem1".into()
    })));
    // let inner_state = Arc::clone(&app.state::<Arc<Mutex<AppState>>>());
    // let state = inner_state.try_lock();
    // app.manage(socket);
    Ok(())
  })
  .manage(socket)
  .invoke_handler(tauri::generate_handler![
    initialize_state, 
    update_is_connected, 
    update_server_ip,
    update_self_ip,
    update_session_id,
    update_forwarding_id,
    add_alert,
    receive_data
  ])
  .run(tauri::generate_context!())
  .expect("error while running tauri application");
}
