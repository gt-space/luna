use std::{sync::Arc, thread};
use fs_protobuf_rust::compiled::mcfs::data::ChannelData;
use futures::lock::Mutex;
use tokio::net::UdpSocket;
use tauri::{Window, State};
use quick_protobuf::deserialize_from_slice;
use fs_protobuf_rust::compiled::mcfs::core;
use fs_protobuf_rust::compiled::mcfs::data;
use fs_protobuf_rust::compiled::mcfs::board;

pub struct Socket {
  pub socket: Option<UdpSocket>
}

#[tauri::command]
pub async fn receive_data(window: Window, socket: State<'_, UdpSocket>, mut buf: Vec<u8>) -> Result<Vec<SendingData>, String> {
  let (amt, src) = socket.recv_from(&mut buf).await.expect("recv_from call failed");
  println!("Received {} bytes from {}", amt, src);
  let message: core::Message;
  match deserialize_from_slice::<core::Message>(&buf) {
    Ok(m) => {
      message = m;
      let to_send = parse_data(message);
      return Ok(to_send);
      println!("{:#?}", to_send);
    },
    Err(e) => println!("Error: {}",e)
  }
  return Err("Failed retreive data".into());
}

fn parse_data(message: core::Message) -> Vec<SendingData>{
  let mut sending_vec = Vec::new();
  match message.content {
    core::mod_Message::OneOfcontent::command(_) => {},
    core::mod_Message::OneOfcontent::data(d) => {
      for data_point in d.channel_data {
        if let Some(ref channel) = data_point.channel {
          //println!("{:?}", channel.channel_type);
          match data_point.data_points {
            data::mod_ChannelData::OneOfdata_points::bool_array(ref a) => {
              //println!("{:?}", a.data);
              if let Some(value) = a.data.first() {
                if let Some(data) = get_sending_data(&data_point, 
                  0.0,  *value, channel.channel_type, channel.board_id, channel.channel) {
                  sending_vec.push(data);
                }
              }
            },
            data::mod_ChannelData::OneOfdata_points::i32_array(ref a) => {
              //println!("{:?}", a.data);
              if let Some(value) = a.data.first() {
                if let Some(data) = get_sending_data(&data_point, 
                  f64::from(*value),  false, channel.channel_type, channel.board_id, channel.channel) {
                  sending_vec.push(data);
                }
              }
            },
            data::mod_ChannelData::OneOfdata_points::u32_array(ref a) => {
              //println!("{:?}", a.data);
              if let Some(value) = a.data.first() {
                if let Some(data) = get_sending_data(&data_point, 
                  f64::from(*value),  false, channel.channel_type, channel.board_id, channel.channel) {
                  sending_vec.push(data);
                }
              }
            },
            data::mod_ChannelData::OneOfdata_points::i64_array(ref a) => {
              //println!("{:?}", a.data);
              if let Some(value) = a.data.first() {
                if let Some(data) = get_sending_data(&data_point, 
                  f64::from(*value),  false, channel.channel_type, channel.board_id, channel.channel) {
                  sending_vec.push(data);
                }
              }
            },
            data::mod_ChannelData::OneOfdata_points::u64_array(ref a) => {
              //println!("{:?}", a.data);
              if let Some(value) = a.data.first() {
                if let Some(data) = get_sending_data(&data_point, 
                  f64::from(*value),  false, channel.channel_type, channel.board_id, channel.channel) {
                  sending_vec.push(data);
                }
              }
            },
            data::mod_ChannelData::OneOfdata_points::f32_array(ref a) => {
              //println!("{:?}", a.data);
              if let Some(value) = a.data.first() {
                if let Some(data) = get_sending_data(&data_point, 
                  f64::from(*value),  false, channel.channel_type, channel.board_id, channel.channel) {
                  sending_vec.push(data);
                }
              }
            },
            data::mod_ChannelData::OneOfdata_points::f64_array(ref a) => {
              //println!("{:#?}", a.data);
              if let Some(value) = a.data.first() {
                if let Some(data) = get_sending_data(&data_point, 
                  f64::from(*value),  false, channel.channel_type, channel.board_id, channel.channel) {
                  sending_vec.push(data);
                }
              }
            },
            _ => {println!("Illegal datapoint type")}
          }
        }
      }
    },
    core::mod_Message::OneOfcontent::status(_) => {},
    _ => {println!("Not a valid type")}
  };
  return sending_vec;
}

fn get_sending_data(data_point: &ChannelData<'_>, 
  value: f64, bool_val: bool, channel_type: board::ChannelType, board_id: u32, channel: u32) -> Option<SendingData> {
  let timestamp = &data_point.timestamp;
  if let Some(t) = timestamp {
    if let Some(o) = data_point.micros_offsets.first() {
      let seconds = t.seconds;
      let nanos = t.nanos;
      return Some(SendingData{
        seconds, 
        nanos, 
        micros: *o, 
        floatValue: value, 
        boolValue: bool_val,
        kind: channel_type as u8,
        board_id,
        channel
      });
    }
  }
  return None;
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct SendingData{
  pub seconds: i64,
  pub nanos: i32,
  pub micros: u32,
  pub floatValue: f64,
  pub boolValue: bool,
  pub kind: u8,
  pub board_id: u32, 
  pub channel: u32
}