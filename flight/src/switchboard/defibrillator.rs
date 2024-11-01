use crate::{handler, state::SharedState, HEARTBEAT_PERIOD};

use common::comm::flight::{BoardId, DataMessage};
use jeflog::fail;
use std::{
  collections::{HashMap, HashSet},
  net::{SocketAddr, UdpSocket},
  sync::{Arc, Mutex, RwLock},
  thread,
};

/// Wakes every `HEARTBEAT_RATE` to send heartbeats to all the connected Sam
/// boards to ensure that the FC isn't disconnected.
pub fn defibrillator(
  shared: SharedState,
  sender: UdpSocket,
  sockets: Arc<RwLock<HashMap<BoardId, SocketAddr>>>,
  statuses: Arc<Mutex<HashSet<BoardId>>>,
) -> impl FnOnce() {
  move || {
    let mut buf = vec![0; crate::HEARTBEAT_BUFFER_SIZE];
    let heartbeat = postcard::to_slice(&DataMessage::FlightHeartbeat, &mut buf);

    let heartbeat = match heartbeat {
      Ok(package) => package,
      Err(error) => {
        fail!("Failed to serialize serialize heartbeat: {error}");
        handler::abort(&shared);
        return;
      }
    };

    loop {
      thread::sleep(HEARTBEAT_PERIOD);

      let sockets = sockets.read().unwrap();
      let statuses = statuses.lock().unwrap();
      let mut abort = false;
      for (board_id, address) in sockets.iter() {
        if !statuses.contains(board_id) {
          continue;
        }

        if let Err(e) = sender.send_to(heartbeat, address) {
          fail!("Couldn't send heartbeat to address {address:#?}: {e}");
          abort = true;
        }
      }

      if abort {
        fail!("Aborting...");
        handler::abort(&shared);
      }
    }
  }
}
