use crate::state::SharedState;
use jeflog::fail;
use std::{
  net::UdpSocket,
  thread,
  time::{Duration, Instant},
};

pub fn forward_vehicle_state(shared: &mut SharedState) -> impl Fn() {
  let server_address = shared.server_address.clone();
  let vehicle_state = shared.vehicle_state.clone();
  let last_update = shared.last_updates.clone();

  let socket =
    UdpSocket::bind("0.0.0.0:0").expect("failed to bind to UDP socket");

  move || {
    loop {
      if let Some(server_address) = *server_address.lock().unwrap() {
        let mut vehicle_state = vehicle_state.lock().unwrap();
        let last_update = last_update.lock().unwrap();

        // update stats for time since last update
        let now = Instant::now();
        for (name, stats) in &mut vehicle_state.rolling {
          if !last_update.contains_key(name.as_str()) {
            continue;
          }

          let last_update_time = *last_update
            .get(name.as_str())
            .expect("already checked if exists");

          stats.time_since_last_update =
            now.duration_since(last_update_time).as_secs_f64();
        }

        // TODO: Change to something that doesn't allocate every iteration
        match postcard::to_allocvec(&*vehicle_state) {
          Ok(serialized) => {
            let result = socket.send_to(&serialized, (server_address, 7201));

            if result.is_err() {
              fail!("Failed to send vehicle state to {server_address}:7201.");
            }
          }
          Err(error) => {
            fail!(
              "Failed to serialize vehicle state with Postcard: {}.",
              error.to_string()
            );
          }
        }
      }

      thread::sleep(Duration::from_millis(2));
    }
  }
}
