use crate::{handler, state::SharedState, REFRESH_COUNT, TIME_TIL_DEATH};
use common::comm::flight::BoardId;
use jeflog::fail;
use std::{
  collections::{HashMap, HashSet},
  sync::{
    mpsc::{Receiver, TryRecvError},
    Arc,
    Mutex,
  },
  time::Instant,
};

/// Tracks the state of each board, detected if boards lose communications.
pub fn lifetime(
  shared: SharedState,
  snooze: Receiver<BoardId>,
  statuses: Arc<Mutex<HashSet<BoardId>>>,
) -> impl FnOnce() {
  move || {
    let mut timers = HashMap::new();

    'main: loop {
      let mut statuses = statuses.lock().unwrap();

      for _ in 0..REFRESH_COUNT {
        // get board to configure
        let board_id = match snooze.try_recv() {
          Ok(board_id) => board_id,
          Err(TryRecvError::Disconnected) => {
            break 'main;
          }
          Err(TryRecvError::Empty) => {
            break;
          }
        };

        if !statuses.contains(&board_id) {
          statuses.insert(board_id.clone());
        }

        // refresh timer
        timers.insert(board_id, Instant::now());
      }

      let mut abort = false;
      for board_id in timers.keys() {
        if !statuses.contains(board_id) {
          continue;
        }

        if Instant::now() - *timers.get(board_id).unwrap() > TIME_TIL_DEATH {
          statuses.remove(board_id);
          abort = true;

          fail!("Detected loss of comms from {board_id}");
        }
      }

      drop(statuses);
      if abort {
        fail!("Aborting...");
        handler::abort(&shared);
      }
    }

    fail!("Switchboard unexpectedly dropped the snooze channel. Aborting.");
    handler::abort(&shared);
  }
}
