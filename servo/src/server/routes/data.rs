use crate::server::{
  self,
  error::{bad_request, internal},
  Shared,
};
use axum::{
  extract::{ws, ConnectInfo, State, WebSocketUpgrade},
  http::header,
  response::{IntoResponse, Response},
  Json,
};
use common::comm::VehicleState;
use futures_util::{SinkExt, StreamExt};
use hdf5::DatasetBuilder;
use jeflog::warn;
use serde::{Deserialize, Serialize};
use std::{
  collections::HashSet,
  net::SocketAddr,
  path::Path,
  sync::atomic::{AtomicU32, Ordering},
  time::Duration,
};
use tokio::{fs, time::MissedTickBehavior};

/// Request struct for export requests.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ExportRequest {
  format: String,
  from: f64,
  to: f64,
}

// An integer used to create unique filenames for exports in case two exports
// overlap in time, which is atomic to be thread-safe.
static EXPORT_FILE_INDEX_ATOMIC: AtomicU32 = AtomicU32::new(0);

/// A function that creates an HDF5 file at a given path containing the
/// timestamps, sensor, and valve values as specified in sensor_names and
/// valve_names in each vehicle state.
pub fn make_hdf5_file(
  sensor_names: &[String],
  valve_names: &[String],
  vehicle_states: &[(f64, VehicleState)],
  path: &Path,
) -> hdf5::Result<()> {
  // Create the HDF5 file
  let file = hdf5::File::create(path)?;

  // Create the organizational groups
  let reading_metadata_group = file.create_group("metadata")?;
  let valve_state_ids_group =
    reading_metadata_group.create_group("valve_state_ids")?;

  let sensors_group = file.create_group("sensors")?;
  let valves_group = file.create_group("valves")?;

  // Initialize with the size of the vehicle state vector, since we'll have
  // equal count of them.
  let mut timestamps_vec = Vec::with_capacity(vehicle_states.len());

  // Turn timestamps into dataset
  for (timestamp, _) in vehicle_states {
    timestamps_vec.push(*timestamp);
  }

  DatasetBuilder::new(&reading_metadata_group)
    .with_data(&timestamps_vec)
    .create("timestamps")?;

  for name in sensor_names {
    let mut reading_vec = Vec::with_capacity(vehicle_states.len());
    let mut unit_vec = Vec::with_capacity(vehicle_states.len());

    // Yes I know iterating through the vehicle states for every sensor / valve
    // is dumb, but I'm avoiding storing the entirety of the vehicle state in
    // memory twice, so each sensor is grabbed seperately
    for (_, state) in vehicle_states {
      let value = state.sensor_readings.get(name);
      // Put in bad data if nothing is found
      match value {
        Some(x) => {
          reading_vec.push(x.value);

          // Should never panic unless absurd amounts of units are added
          let id = x.unit as i32;
          unit_vec.push(id);
        }
        // Immature but nobody will see this and not realize it's garbage data.
        // Might replace with an infinity or something
        None => {
          reading_vec.push(-6942069420.0);
          unit_vec.push(-69);
        }
      };
    }
    let curr_sensor_group = sensors_group.create_group(name.as_str())?;

    // Make datasets
    curr_sensor_group
      .new_dataset_builder()
      .deflate(9)
      .with_data(&reading_vec)
      .create("readings")?;

    curr_sensor_group
      .new_dataset_builder()
      .deflate(9)
      .with_data(&unit_vec)
      .create("units")?;
  }

  // A vector of all the possible ValveStates seen. Used to create the
  // attributes that indicate what each value of ValveState means. Likely more
  // efficient as a simple vector, since ValveState has few possible elements.
  // Will check later. I was originally going to make this a single attribute in
  // the metadata category, but you can't iterate through an enum, so I'll talk
  // to Jeff about making a possible ValveState iter to replace this.
  let mut seen_valve_states = HashSet::new();

  // Will make all values of valves metadata later
  for name in valve_names {
    // A vector of all the values of the valve in each timeframe
    let mut state_vec = Vec::with_capacity(vehicle_states.len());

    // Yes I know iterating through the vehicle states for every sensor / valve
    // is dumb, but I'm avoiding storing the entirety of the vehicle state in
    // memory twice, so each sensor is grabbed seperately
    for (_, state) in vehicle_states {
      let valve_state = state.valve_states.get(name);
      // Put in bad data if nothing is found
      match valve_state {
        Some(state) => {
          let commanded = state.commanded;

          if !seen_valve_states.contains(&commanded) {
            // Keep track of seen valve states
            seen_valve_states.insert(commanded);
          }

          state_vec.push(commanded as u8);

          // state_vec.push((*x as i8).try_into()?)
        }
        // Immature but nobody will see this and not realize it's garbage data.
        // Might replace with an infinity or something, will go over with Jeff.
        None => state_vec.push(69),
      };
    }

    // Make dataset
    valves_group
      .new_dataset_builder()
      .deflate(9)
      .with_data(&state_vec)
      .create(name.as_str())?;
  }

  // Put an attribute of what id each valve state is represented by into the
  // valve state id's metadata group
  // TLDR; it's an enum of attributes on a folder
  for state in seen_valve_states {
    let attr = valve_state_ids_group
      .new_attr::<i8>()
      .shape(1)
      .create(state.to_string().as_str())?;
    let id = state as u8;
    if let Err(error) = attr.write(&[id]) {
      warn!("Failed to write HDF5 attribute: {error}");
    }
  }

  // Close the file
  file.close()?;

  Ok(())
}

/// Route function which exports all vehicle data from the database into a
/// specified format.
pub async fn export(
  State(shared): State<Shared>,
  Json(request): Json<ExportRequest>,
) -> server::Result<impl IntoResponse> {
  let database = shared.database.connection.lock().await;

  let vehicle_states = database
    .prepare(
      "
      SELECT recorded_at, vehicle_state
      FROM VehicleSnapshots
      WHERE recorded_at >= ?1 AND recorded_at <= ?2
    ",
    )
    .map_err(internal)?
    .query_map([request.from, request.to], |row| {
      let bytes = row.get::<_, Vec<u8>>(1)?;

      let vehicle_state = postcard::from_bytes::<VehicleState>(&bytes)
        .map_err(|error| {
          rusqlite::Error::FromSqlConversionFailure(
            1,
            rusqlite::types::Type::Blob,
            Box::new(error),
          )
        })?;

      Ok((row.get::<_, f64>(0)?, vehicle_state))
    })
    .and_then(|iter| iter.collect::<Result<Vec<_>, rusqlite::Error>>())
    .map_err(internal)?;

  match request.format.as_str() {
    "csv" => {
      let mut sensor_names = HashSet::new();
      let mut valve_names = HashSet::new();

      for (_, state) in &vehicle_states {
        for name in state.sensor_readings.keys() {
          // yes, a HashSet will not allow duplicate items even with a plain
          // insert, but the .clone() incurs a notable performance penalty,
          // and if it was just .insert(name.clone()) here, then it would clone
          // name every time despite the fact that it will rarely actually
          // need to be inserted. the same applies for valve_states.
          if !sensor_names.contains(name) {
            sensor_names.insert(name.clone());
          }
        }

        for name in state.valve_states.keys() {
          if !valve_names.contains(name) {
            valve_names.insert(name.clone());
          }
        }
      }

      let sensor_names = sensor_names.into_iter().collect::<Vec<_>>();

      let valve_names = valve_names.into_iter().collect::<Vec<_>>();

      let header = sensor_names
        .iter()
        .chain(valve_names.iter())
        .fold("timestamp".to_owned(), |header, name| header + "," + name);

      let mut content = header + "\n";

      for (timestamp, state) in vehicle_states {
        // first column is the timestamp
        content += &timestamp.to_string();

        for name in &sensor_names {
          let reading = state.sensor_readings.get(name);
          content += ",";

          // currently, if there is no data here, the column is empty.
          // we may want to change this.
          if let Some(reading) = reading {
            content += &reading.to_string();
          }
        }

        for name in &valve_names {
          let valve_state = state.valve_states.get(name);
          content += ",";

          // see comment in sensor readings above.
          if let Some(valve_state) = valve_state {
            content += &valve_state.actual.to_string();
          }
        }

        content += "\n";
      }

      let headers = [(header::CONTENT_TYPE, "text/csv; charset=utf-8")];
      Ok((headers, content.into_response()))
    }
    "hdf5" => {
      // Generally a modified version of the csv export section

      // Get all sensor and valve reading names
      let mut sensor_names = HashSet::new();
      let mut valve_names = HashSet::new();

      for (_, state) in &vehicle_states {
        for name in state.sensor_readings.keys() {
          // yes, a HashSet will not allow duplicate items even with a plain
          // insert, but the .clone() incurs a notable performance penalty,
          // and if it was just .insert(name.clone()) here, then it would clone
          // name every time despite the fact that it will rarely actually
          // need to be inserted. the same applies for valve_states.
          if !sensor_names.contains(name) {
            sensor_names.insert(name.clone());
          }
        }

        for name in state.valve_states.keys() {
          if !valve_names.contains(name) {
            valve_names.insert(name.clone());
          }
        }
      }

      // Frontload iterating through the hashmap into two vectors for faster
      // access in the loop
      let sensor_names = sensor_names.into_iter().collect::<Vec<_>>();

      let valve_names = valve_names.into_iter().collect::<Vec<_>>();

      // Temporary until I make it pass
      #[cfg(target_family = "windows")]
      let temp = &std::env::var("USERPROFILE");

      #[cfg(target_family = "unix")]
      let temp = &std::env::var("HOME");

      let home_path = match temp {
        Ok(x) => Path::new(x),
        _ => return Err(internal("Could not get home path")),
      };

      let servo_dir = Path::new(home_path).join(".servo");

      // Get unique file index
      let file_index = EXPORT_FILE_INDEX_ATOMIC
        .fetch_add(1, Ordering::Relaxed)
        .to_string();

      // Uneccessary since main should already make it
      if !servo_dir.is_dir() {
        return Err(internal(String::from("Could not get .servo path")));
      }

      // Prob can convert to just being str code. Will check later.
      let path = servo_dir.join(
        (String::from("ExportFile") + &file_index + &String::from(".hdf5"))
          .as_str(),
      );

      make_hdf5_file(&sensor_names, &valve_names, &vehicle_states, &path)
        .map_err(internal)?;

      let content = fs::read(&path).await.map_err(internal)?;

      // remove file to free up space
      if let Err(error) = std::fs::remove_file(&path) {
        warn!("Failed to remove temporary HDF5 file at {path:?}: {error}");
      }

      let headers = [(header::CONTENT_TYPE, "application/x-hdf")];
      Ok((headers, content.into_response()))
    }
    _ => Err(bad_request("invalid export format")),
  }
}

/// Route function which accepts a WebSocket connection and begins forwarding
/// vehicle state data.
pub async fn forward_data(
  ws: WebSocketUpgrade,
  State(shared): State<Shared>,
  ConnectInfo(peer): ConnectInfo<SocketAddr>,
) -> Response {
  ws.on_upgrade(move |socket| async move {
    let vehicle = shared.vehicle.clone();
    let (mut writer, mut reader) = socket.split();

    // spawn separate task for forwarding while the "main" task waits
    // until it can abort this task when the user wants to close
    let forwarding_handle = tokio::spawn(async move {
      let (vehicle_state, _) = vehicle.as_ref();

      // setup forwarding agent to send vehicle state every 100ms (10Hz)
      let mut interval = tokio::time::interval(Duration::from_millis(100));
      interval.set_missed_tick_behavior(MissedTickBehavior::Delay);

      loop {
        let vehicle_state = vehicle_state.lock().await.clone();

        // Serialize vehicle state into JSON so it is easily digestible by the
        // GUI. Vehicle state comes in as postcard and gets reserialized here.
        // Overhead isn't bad.
        let json = match serde_json::to_string(&vehicle_state) {
          Ok(json) => json,
          Err(error) => {
            warn!("Failed to serialize vehicle state into JSON: {error}");
            continue;
          }
        };

        // drop vehicle state before sending to prevent holding lock
        drop(vehicle_state);

        // attempt to forward vehicle state and break if connection is severed.
        if let Err(_error) = writer.send(ws::Message::Text(json)).await {
          warn!(
            "Forwarding connection with peer \x1b[1m{}\x1b[0m severed.",
            peer
          );
          _ = writer.close().await;
          break;
        }

        // wait for 100ms to retransmit vehicle state
        interval.tick().await;
      }
    });

    // wait until reader from socket receives a ws::Message::Close or a None,
    // indicating that the stream is no longer readable
    while !matches!(reader.next().await, Some(Ok(ws::Message::Close(_))) | None)
    {
    }

    // cancel the forwarding stream upon receipt of a close message
    forwarding_handle.abort();
  })
}

#[cfg(test)]
mod tests {
  use super::*;
  use common::comm::{CompositeValveState, Measurement, Unit, ValveState};
  use rand::{Rng, RngCore};
  use std::collections::HashMap;

  #[test]
  fn test_hdf5_file_creation() {
    // Do the same test a few times just cause this does use RNG
    for _ in 0..8 {
      let path: &Path = Path::new("./CompilingTestsExportSample.hdf5");

      let count = 64;

      let mut vehicle_states = Vec::with_capacity(count);

      let mut rng = rand::thread_rng();
      let mut time: f64 = 0.0;

      let mut timestamps_vec: Vec<f64> = Vec::with_capacity(count);

      let sensor_units = [Unit::Amps, Unit::Psi, Unit::Volts, Unit::Kelvin];

      let valve_names = [
        String::from("V1"),
        String::from("V2"),
        String::from("V3"),
        String::from("V4"),
      ];
      let mut valve_state_vecs = [
        Vec::with_capacity(count),
        Vec::with_capacity(count),
        Vec::with_capacity(count),
        Vec::with_capacity(count),
      ];

      let mut seen_valve_states = Vec::with_capacity(10);

      let sensor_names = [
        String::from("S1"),
        String::from("S2"),
        String::from("S3"),
        String::from("S4"),
      ];
      let mut sensor_state_vecs = [
        Vec::with_capacity(count),
        Vec::with_capacity(count),
        Vec::with_capacity(count),
        Vec::with_capacity(count),
      ];
      let mut sensor_unit_vecs = [
        Vec::with_capacity(count),
        Vec::with_capacity(count),
        Vec::with_capacity(count),
        Vec::with_capacity(count),
      ];

      for _ in 0..count {
        let mut state = VehicleState {
          valve_states: HashMap::new(),
          sensor_readings: HashMap::new(),
        };

        for i in 0..4 {
          if rng.next_u32() % 10 > 0 {
            // have some "empty" timeframes for a bit of data
            let valve_state_temp = match rng.next_u32() % 5 {
              0 => ValveState::Disconnected,
              1 => ValveState::Open,
              2 => ValveState::Closed,
              3 => ValveState::Fault,
              4 => ValveState::Undetermined,
              _ => ValveState::Disconnected,
            };

            if !seen_valve_states.contains(&valve_state_temp) {
              seen_valve_states.push(valve_state_temp);
            }

            let composite = CompositeValveState {
              commanded: valve_state_temp,
              actual: ValveState::Undetermined,
            };

            state.valve_states.insert(valve_names[i].clone(), composite);
            valve_state_vecs[i].push(valve_state_temp as i8);
          } else {
            valve_state_vecs[i].push(69);
          }
        }

        for i in 0..4 {
          if rng.next_u32() % 10 > 0 {
            // have some "empty" timeframes for a bit of data
            let x: f64 = rng.gen::<f64>() * 5.0;
            sensor_state_vecs[i].push(x);
            sensor_unit_vecs[i].push(sensor_units[i] as i8);
            state.sensor_readings.insert(
              sensor_names[i].clone(),
              Measurement {
                value: x,
                unit: sensor_units[i],
              },
            );
          } else {
            sensor_state_vecs[i].push(-6942069420.0);
            sensor_unit_vecs[i].push(-69);
          }
        }
        vehicle_states.push((time, state));
        timestamps_vec.push(time);
        time += 0.1;
      }

      make_hdf5_file(&sensor_names, &valve_names, &vehicle_states, path)
        .expect("HDF5 should not error out when making this basic dataset");

      let file = hdf5::File::open(path)
        .expect("File should exist after make_hdf5_file runs"); //

      // You have to close groups to be able to close a file, so we simply do
      // all of the HDF5 operations inside of a namespace like this so they
      // automatically deconstruct and close.
      {
        // get metadata group / ensure it exists
        let metadata_group = file.group("metadata").expect(
          "HDF5 file for data exports should always have metadata group in it",
        );

        // ensure timestamps are accurate
        {
          let timestamps = metadata_group
            .dataset("timestamps")
            .expect("HDF5 file should have timestamps in the metadata group");

          assert_eq!(timestamps.shape(), vec![count]);
          assert_eq!(
            timestamps
              .read_raw::<f64>()
              .expect("timestamps should be readable."),
            timestamps_vec
          );
        }

        // ensure valve_state_id lookup attributes are accurate
        {
          let valve_state_ids = metadata_group.group("valve_state_ids").expect(
            "HDF5 file should have valve_state_ids group in the metadata group",
          );

          assert_eq!(
            valve_state_ids
              .attr_names()
              .expect("valve_state_ids should have attributes.")
              .len(),
            seen_valve_states.len()
          ); // make sure they have equal element counts

          for state in seen_valve_states {
            let attr_value = valve_state_ids.attr(&state.to_string())
							.expect("valve_state_ids should have all valve states that are seen during creation of a dataset in it's attributes")
							.read_raw::<i8>()
							.expect("valve_state_ids attributes should be readable as a signed byte");

            assert_eq!(attr_value.len(), 1); // This should be a single value
            assert_eq!(attr_value[0], state as i8);
          }
        }

        // ensure valve readings are accurate
        let valves_group = file.group("valves").expect(
          "HDF5 file for data exports should always have valve folder in it",
        );
        for i in 0..4 {
          let name = &valve_names[i];
          let valve_ds = valves_group
            .dataset(name)
            .expect("All valves specified should have a dataset");
          assert_eq!(valve_ds.shape(), vec![count]);
          assert_eq!(
            valve_ds
              .read_raw::<i8>()
              .expect("valve state dataset should be readable."),
            valve_state_vecs[i]
          );
        }

        // ensure sensor readings are accurate
        let sensors_group = file.group("sensors").expect(
          "HDF5 file for data exports should always have sensor folder in it",
        );
        for i in 0..4 {
          let name = &sensor_names[i];
          let this_sensor_group = sensors_group
            .group(name)
            .expect("All sensors specified should have a group");
          let sensor_ds = this_sensor_group
            .dataset("readings")
            .expect("All sensor groups should have a readings dataset");
          let unit_ds = this_sensor_group
            .dataset("units")
            .expect("All sensor groups should have a unit dataset");
          assert_eq!(sensor_ds.shape(), vec![count]);
          assert_eq!(unit_ds.shape(), vec![count]);
          assert_eq!(
            sensor_ds
              .read_raw::<f64>()
              .expect("sensor value dataset should be readable."),
            sensor_state_vecs[i]
          );
          assert_eq!(
            unit_ds
              .read_raw::<i8>()
              .expect("sensor unit dataset should be readable."),
            sensor_unit_vecs[i]
          );
        }
      }

      file
        .close()
        .expect("File should properly close after reading hdf5 values from it");

      std::fs::remove_file(path)
        .expect("You should be able to delete the HDF5 file after closing it ");
    }
  }
}
