use axum::{extract::State, Json};
use common::comm::NodeMapping;
use rusqlite::params;
use serde::{Deserialize, Serialize};
use serde_json::Value as JsonValue;
use std::collections::HashMap;

use crate::server::{
  self,
  error::{bad_request, internal, not_found},
  Shared,
};

/// Request struct for getting mappings.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GetMappingResponse {
  /// Array of all mappings in no specific order
  pub mappings: Vec<NodeMapping>,
}

/// A route function which retrieves the current stored mappings.
pub async fn get_mappings(
  State(shared): State<Shared>,
) -> server::Result<Json<JsonValue>> {
  let database = shared.database.connection.lock().await;

  let mappings = database
    .prepare(
      "
			SELECT
				configuration_id,
				text_id,
				board_id,
				sensor_type,
				channel,
				computer,
				max,
				min,
				calibrated_offset,
				powered_threshold,
				normally_closed
			FROM NodeMappings
		",
    )
    .map_err(internal)?
    .query_and_then([], |row| {
      let configuration_id = row.get(0)?;

      let mapping = NodeMapping {
        text_id: row.get(1)?,
        board_id: row.get(2)?,
        sensor_type: row.get(3)?,
        channel: row.get(4)?,
        computer: row.get(5)?,
        max: row.get(6)?,
        min: row.get(7)?,
        calibrated_offset: row.get(8)?,
        powered_threshold: row.get(9)?,
        normally_closed: row.get(10)?,
      };

      Ok((configuration_id, mapping))
    })
    .map_err(internal)?
    .collect::<rusqlite::Result<Vec<(String, NodeMapping)>>>()
    .map_err(internal)?;

  let mut configurations = HashMap::<String, Vec<NodeMapping>>::new();

  for (configuration_id, mapping) in mappings {
    if let Some(config) = configurations.get_mut(&configuration_id) {
      config.push(mapping);
    } else {
      configurations.insert(configuration_id, vec![mapping]);
    }
  }

  Ok(Json(serde_json::to_value(&configurations).unwrap()))
}

/// Request struct for setting a mapping.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SetMappingsRequest {
  /// An ID uniquely identifying the configuration being set or modified
  pub configuration_id: String,

  /// Array of all mappings in no specific order
  pub mappings: Vec<NodeMapping>,
}

/// A route function which deletes and replaces a previous configuration
pub async fn post_mappings(
  State(shared): State<Shared>,
  Json(request): Json<SetMappingsRequest>,
) -> server::Result<()> {
  let database = shared.database.connection.lock().await;

  database
    .execute(
      "DELETE FROM NodeMappings WHERE configuration_id = ?1",
      [&request.configuration_id],
    )
    .map_err(internal)?;

  for mapping in &request.mappings {
    database
      .execute(
        "
				INSERT INTO NodeMappings (
					configuration_id,
					text_id,
					board_id,
					sensor_type,
					channel,
					computer,
					max,
					min,
					calibrated_offset,
					powered_threshold,
					normally_closed,
					active
				) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, TRUE)
			",
        params![
          request.configuration_id,
          mapping.text_id,
          mapping.board_id,
          mapping.sensor_type,
          mapping.channel,
          mapping.computer,
          mapping.max,
          mapping.min,
          mapping.calibrated_offset,
          mapping.powered_threshold,
          mapping.normally_closed,
        ],
      )
      .map_err(internal)?;
  }

  drop(database);

  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    flight.send_mappings().await.map_err(internal)?;
  }

  Ok(())
}

/// A route function which inserts a new mapping or updates an existing one
pub async fn put_mappings(
  State(shared): State<Shared>,
  Json(request): Json<SetMappingsRequest>,
) -> server::Result<()> {
  let database = shared.database.connection.lock().await;

  for mapping in &request.mappings {
    database
      .execute(
        "
				INSERT INTO NodeMappings (
					configuration_id,
					text_id,
					board_id,
					sensor_types,
					channel,
					computer,
					max,
					min,
					calibrated_offset,
					powered_threshold,
					normally_closed,
					active
				) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, TRUE)
				ON CONFLICT (configuration_id, text_id) DO UPDATE SET
					board_id = excluded.board_id,
					channel = excluded.channel,
					sensor_types = excluded.sensor_types,
					computer = excluded.computer,
					scale = excluded.scale,
					offset = excluded.offset,
					powered_threshold = excluded.powered_threshold,
					normally_closed = excluded.normally_closed,
					active = excluded.active
			",
        params![
          request.configuration_id,
          mapping.text_id,
          mapping.board_id,
          mapping.sensor_type,
          mapping.channel,
          mapping.computer,
          mapping.max,
          mapping.min,
          mapping.calibrated_offset,
          mapping.powered_threshold,
          mapping.normally_closed,
        ],
      )
      .map_err(internal)?;
  }

  drop(database);

  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    flight.send_mappings().await.map_err(internal)?;
  }

  Ok(())
}

/// The request struct used with the route function to delete mappings.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct DeleteMappingsRequest {
  /// The configuration ID of the mappings being deleted.
  pub configuration_id: String,

  /// The mappings to be deleted. If this is `None`, then all mappings
  /// with the corresponding configuration ID will be deleted.
  pub mappings: Option<Vec<NodeMapping>>,
}

/// A route function which deletes the specified mappings.
pub async fn delete_mappings(
  State(shared): State<Shared>,
  Json(request): Json<DeleteMappingsRequest>,
) -> server::Result<()> {
  let database = shared.database.connection.lock().await;

  // if the mappings are specified, then only delete them
  // if not, then delete all mappings for that configuration
  // (thus deleting the config)
  if let Some(mappings) = &request.mappings {
    for mapping in mappings {
      database
        .execute(
          "DELETE FROM NodeMappings
          WHERE configuration_id = ?1
          AND text_id = ?2",
          params![request.configuration_id, mapping.text_id],
        )
        .map_err(internal)?;
    }
  } else {
    database
      .execute(
        "DELETE FROM NodeMappings WHERE configuration_id = ?1",
        params![request.configuration_id],
      )
      .map_err(internal)?;
  }

  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    flight.send_mappings().await.map_err(internal)?;
  }

  Ok(())
}

/// Request/response struct for getting and setting the active configuration.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ActiveConfiguration {
  configuration_id: String,
}

/// A route function which activates a particular configuration
pub async fn activate_configuration(
  State(shared): State<Shared>,
  Json(request): Json<ActiveConfiguration>,
) -> server::Result<()> {
  let database = shared.database.connection.lock().await;

  database
    .execute(
      "UPDATE NodeMappings SET active = FALSE WHERE active = TRUE",
      [],
    )
    .map_err(internal)?;

  let rows_updated = database
    .execute(
      "UPDATE NodeMappings SET active = TRUE WHERE configuration_id = ?1",
      [&request.configuration_id],
    )
    .map_err(internal)?;

  drop(database);

  if rows_updated > 0 {
    if let Some(flight) = shared.flight.0.lock().await.as_mut() {
      flight.send_mappings().await.map_err(internal)?;
    }
  } else {
    return Err(bad_request("configuration_id does not exist"));
  }

  Ok(())
}

/// A route function which returns the active configuration
pub async fn get_active_configuration(
  State(shared): State<Shared>,
) -> server::Result<Json<ActiveConfiguration>> {
  let configuration_id = shared
    .database
    .connection
    .lock()
    .await
    .query_row(
      "SELECT configuration_id FROM NodeMappings WHERE active = TRUE",
      [],
      |row| row.get::<_, String>(0),
    )
    .map_err(|_| not_found("no configurations active"))?;

  Ok(Json(ActiveConfiguration { configuration_id }))
}

/// Maps sensor names (stored in mappings) to calibrated offset floats.
pub type CalibratedOffsets = HashMap<String, f64>;

/// Route handler to calibrate all sensors in the current configuration.
pub async fn calibrate(
  State(shared): State<Shared>,
) -> server::Result<Json<CalibratedOffsets>> {
  let database = shared.database.connection.lock().await;

  let to_calibrate = database
    .prepare(
      "
			SELECT text_id
			FROM NodeMappings
			WHERE
				sensor_type IN ('pt', 'load_cell')
				AND active
		",
    )
    .map_err(internal)?
    .query_and_then([], |row| row.get(0))
    .map_err(internal)?
    .collect::<rusqlite::Result<Vec<String>>>()
    .map_err(internal)?;

  let vehicle_state = shared.vehicle.0.lock().await.clone();
  let mut updated = HashMap::new();

  for sensor in to_calibrate {
    if let Some(measurement) = vehicle_state.sensor_readings.get(&sensor) {
      database
        .execute(
          "
					UPDATE NodeMappings
					SET calibrated_offset = ?1
					WHERE text_id = ?2
				",
          params![sensor, measurement.value],
        )
        .map_err(internal)?;

      updated.insert(sensor.clone(), measurement.value);
    }
  }

  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    flight.send_mappings().await.map_err(internal)?;
  }

  Ok(Json(updated))
}
