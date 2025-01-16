use crate::{
  handler,
  state::SharedState,
  COMMAND_MESSAGE_BUFFER_SIZE,
  SAM_PORT,
};

use common::comm::{ahrs, bms, flight::BoardId, sam::SamControlMessage};
use jeflog::{fail, pass};
use std::{
  collections::HashMap,
  net::{SocketAddr, UdpSocket},
  sync::{mpsc::Receiver, Arc, RwLock},
};

#[derive(serde::Serialize)]
pub enum Command {
  Sam(SamControlMessage),
  Bms(bms::Command),
  Ahrs(ahrs::Command),
}

/// "fast lane" for sending SamControlMessages. Only wakes up when there's a
/// command to be sent.
pub fn commander(
  shared: SharedState,
  commands: Receiver<(BoardId, Command)>,
  sender: UdpSocket,
  sockets: Arc<RwLock<HashMap<BoardId, SocketAddr>>>,
) -> impl FnOnce() {
  move || {
    let mut buffer = [0; COMMAND_MESSAGE_BUFFER_SIZE];

    for (board_id, command) in commands {
      let output = match command {
        Command::Sam(c) => {
          postcard::to_slice::<SamControlMessage>(&c, &mut buffer)
        }
        Command::Ahrs(c) => {
          postcard::to_slice::<ahrs::Command>(&c, &mut buffer)
        }
        Command::Bms(c) => postcard::to_slice::<bms::Command>(&c, &mut buffer),
      };

      let Ok(message) = output else {
        fail!("Failed to serialize control message.");
        handler::abort(&shared);
        continue;
      };

      let sockets = sockets.read().unwrap();
      if let Some(socket) = sockets.get(&board_id) {
        let socket = (socket.ip(), SAM_PORT);

        match sender.send_to(message, socket) {
          Ok(_) => pass!("Sent command!"),
          Err(e) => {
            fail!("Failed to send control message to board {board_id}: {e}")
          }
        };
      } else {
        fail!("Failed to locate socket with of board {board_id}.");
      }
    }

    fail!("The FC unexpectedly dropped the command channel. Aborting.");
    handler::abort(&shared);
  }
}
