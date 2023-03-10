use std::{sync::Arc, thread};
use futures::lock::Mutex;
use tokio::net::UdpSocket;
use tauri::{Window, State};

pub struct Socket {
  pub socket: Option<UdpSocket>
}

#[tauri::command]
pub async fn receive_data(window: Window, socket: State<'_, UdpSocket>, mut buf: Vec<u8>) -> Result<Vec<u8>, String> {
  socket.recv_from(&mut buf).await;
  return Ok(buf);
}