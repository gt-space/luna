pub mod commander;
mod defibrillator;
mod lifetime;
mod worker;

use crate::{handler, state::SharedState, CommandSender, FC_BOARD_ID};
use commander::commander;
use common::comm::flight::{BoardId, DataMessage};
use defibrillator::defibrillator;
use jeflog::{fail, pass, warn};
use lifetime::lifetime;
use std::{
  collections::{HashMap, HashSet},
  io,
  net::{SocketAddr, UdpSocket},
  sync::{
    mpsc::{self, Sender},
    Arc,
    Mutex,
    RwLock,
  },
  thread,
};
use worker::{worker, Gig};

// Concerns: might be a bit too abort happy?

/// one-shot function that starts the switchboard.
pub fn start(
  shared: SharedState,
  socket: UdpSocket,
) -> io::Result<CommandSender> {
  let reciever = socket.try_clone()?;
  let sender = socket.try_clone()?;
  let command_sender = socket.try_clone()?;

  let (snooze_tx, snooze_rx) = mpsc::channel();
  let (gig_tx, gig_rx) = mpsc::channel();
  let (command_tx, command_rx) = mpsc::channel();

  let statuses = Arc::new(Mutex::new(HashSet::new()));
  let sockets = Arc::new(RwLock::new(HashMap::new()));

  thread::spawn(switchboard(
    shared.clone(),
    snooze_tx,
    gig_tx,
    socket,
    reciever,
    sockets.clone(),
  ));
  thread::spawn(lifetime(shared.clone(), snooze_rx, statuses.clone()));
  thread::spawn(defibrillator(
    shared.clone(),
    sender,
    sockets.clone(),
    statuses.clone(),
  ));
  thread::spawn(worker(shared.clone(), gig_rx));
  thread::spawn(commander(
    shared.clone(),
    command_rx,
    command_sender,
    sockets.clone(),
  ));

  Ok(command_tx)
}

/// Wakes when there's something to be passed along. Think of it like a
/// telephone operator.
pub fn switchboard(
  shared: SharedState,
  snooze: Sender<BoardId>,
  gig: Sender<(BoardId, Gig)>,
  handshake_sender: UdpSocket,
  reciever: UdpSocket,
  sockets: Arc<RwLock<HashMap<BoardId, SocketAddr>>>,
) -> impl FnOnce() {
  move || {
    let mut buffer = [0; crate::DATA_MESSAGE_BUFFER_SIZE];

    loop {
      // Move the incoming UDP data into a buffer
      let (message_length, sender_address) =
        match reciever.recv_from(&mut buffer) {
          Ok(data) => data,
          Err(e) => {
            fail!("Failed to insert data into switchboard buffer: {e}");
            handler::abort(&shared);
            continue;
          }
        };

      let incoming_data = postcard::from_bytes(&buffer[..message_length]);

      // Interpret the data in the buffer
      let incoming_data = match incoming_data {
        Ok(data) => data,
        Err(error) => {
          fail!("Failed to interpret buffer data: {error}");
          continue;
        }
      };

      let board_id = match incoming_data {
        DataMessage::Identity(board_id) => {
          let mut sockets = sockets.write().unwrap();
          sockets.insert(board_id.clone(), sender_address);

          pass!("Recieved identity message from board {board_id}");

          let identity = DataMessage::Identity(String::from(FC_BOARD_ID));

          let handshake = match postcard::to_slice(&identity, &mut buffer) {
            Ok(identity) => identity,
            Err(error) => {
              warn!("Failed to deserialize identity message: {error}");
              continue;
            }
          };

          if let Err(e) = handshake_sender.send_to(handshake, sender_address) {
            fail!("Failed to send identity to {sender_address}: {e}");
          } else {
            pass!("Sent identity to {sender_address}.");
          }

          board_id
        }
        DataMessage::Sam(board_id, datapoints) => {
          if let Err(e) =
            gig.send((board_id.clone(), Gig::Sam(datapoints.to_vec())))
          {
            fail!("Worker dropped the receiving end of the gig channel ({e}).");
            handler::abort(&shared);
            break;
          }

          board_id
        }
        DataMessage::Bms(board_id, datapoint) => {
          if let Err(e) =
            gig.send((board_id.clone(), Gig::Bms(vec![datapoint.into_owned()])))
          {
            fail!("Worker dropped the receiving end of the gig channel ({e}).");
            handler::abort(&shared);
            break;
          }

          board_id
        }
        DataMessage::Ahrs(board_id, datapoints) => {
          if let Err(e) =
            gig.send((board_id.clone(), Gig::Ahrs(datapoints.to_vec())))
          {
            fail!("Worker dropped the receiving end of the gig channel ({e}).");
            handler::abort(&shared);
            break;
          }

          board_id
        }
        DataMessage::FlightHeartbeat => {
          warn!("Recieved a FlightHeartbeat from {sender_address}.");
          continue;
        }
      };

      if let Err(e) = snooze.send(board_id) {
        fail!("Lifetime dropped the receiver of the snooze channel ({e}).");
        handler::abort(&shared);
        break;
      }
    }
  }
}
