use crate::{
  handler,
  state::SharedState,
  COMMAND_MESSAGE_BUFFER_SIZE,
  SAM_PORT,
};

use common::comm::{bms, sam::{BoardId, SamControlMessage}};
use jeflog::{fail, pass};
use std::{
  collections::HashMap,
  net::{SocketAddr, UdpSocket},
  sync::{mpsc::Receiver, Arc, RwLock},
};

pub enum Command {
  Sam(SamControlMessage),
  Bms(bms::Command)
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
      // send sam control message to SAM
      let Ok(message) = serialize(&mut buffer, command) else {
        handler::abort(&shared);
        return;
      };

      let sockets = sockets.read().unwrap();
      if let Some(socket) = sockets.get(&board_id) {
        let socket = (socket.ip(), SAM_PORT);

        match sender.send_to(message, socket) {
          Ok(_) => 
            pass!("Sent command!"),
          Err(e) =>
            fail!("Failed to send control message to board {board_id}: {e}"),
        };
      } else {
        fail!("Failed to locate socket with of board {board_id}.");
      }
    }

    fail!("The FC unexpectedly dropped the command channel. Aborting.");
    handler::abort(&shared);
  }
}
// rushed code
fn serialize<'a>(buffer: &'a mut [u8], command: Command) -> Result<&'a mut [u8], ()> {
  let package: &'a mut [u8] = match command {
    Command::Sam(command) => {
      match postcard::to_slice(&command, buffer) {
        Ok(package) => package,
        Err(error) => {
          fail!("Failed to serialize control message: {error}");
          return Err(());
        }
      }
    }
    Command::Bms(command) => {
      match postcard::to_slice(&command, buffer) {
        Ok(package) => package,
        Err(error) => {
          fail!("Failed to serialize control message: {error}");
          return Err(());
        }
      }
    }
  };

  Ok(package)
}