use std::{sync::Arc, thread};
use futures::lock::Mutex;
use tokio::net::UdpSocket;
use tauri::{Window, State};
use quick_protobuf::deserialize_from_slice;
use fs_protobuf_rust::compiled::mcfs::core;
use fs_protobuf_rust::compiled::mcfs::device;
use fs_protobuf_rust::compiled::mcfs::command;
use fs_protobuf_rust::compiled::mcfs::data;

pub struct Socket {
  pub socket: Option<UdpSocket>
}

#[tauri::command]
pub async fn receive_data(window: Window, socket: State<'_, UdpSocket>, mut buf: Vec<u8>) -> Result<Vec<u8>, String> {
  let (amt, src) = socket.recv_from(&mut buf).await.expect("recv_from call failed");
  println!("Received {} bytes from {}", amt, src);
  let message: core::Message;
  match deserialize_from_slice::<core::Message>(&buf) {
    Ok(m) => {
      message = m;
      parse_data(message);
    },
    Err(e) => println!("Error: {}",e)
  }
  return Ok(buf);
}

fn parse_data(message: core::Message){
  match message.content {
    core::mod_Message::OneOfcontent::command(c) => {},
    core::mod_Message::OneOfcontent::data(mut d) => {
      for data_point in d.node_data.iter_mut() {
        if let Some(node) = &data_point.node {
          println!("{:#?}", node.channel);
        }
      }
    },
    core::mod_Message::OneOfcontent::status(s) => {},
    _ => {println!("Not a valid type")}
  };
}