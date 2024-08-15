use crate::state::SharedState;
use jeflog::fail;
use std::{net::UdpSocket, thread, time::Duration};

pub fn forward_vehicle_state(shared: &SharedState) -> impl Fn() {
  let server_address = shared.server_address.clone();
  let vehicle_state = shared.vehicle_state.clone();

  let socket =
    UdpSocket::bind("0.0.0.0:0").expect("failed to bind to UDP socket");

  move || {
    loop {
      if let Some(server_address) = *server_address.lock().unwrap() {
        let vehicle_state = vehicle_state.lock().unwrap();

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

      thread::sleep(Duration::from_millis(10));
    }
  }
}
