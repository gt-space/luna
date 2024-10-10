use axum::{extract::State, Json};
use common::comm::NodeMapping;
use rusqlite::params;
use serde::{Deserialize, Serialize};
use serde_json::Value as JsonValue;
use std::collections::{HashMap, HashSet};

use crate::server::{
  self, error::{bad_request, internal, not_found, ServerResult}, Shared
};

/// Request struct for getting mappings.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GetMappingResponse {
  /// Array of all mappings in no specific order
  pub mappings: Vec<NodeMapping>,
}

const PYTHON_KEYWORDS : [&str; 70] = [
  "abs", "aiter", "all", "anext", "any", "ascii", "bin", "bool", "breakpoint", 
  "bytearray", "bytes", "callable", "chr", "classmethod", "compile", "complex", 
  "delattr", "dict", "dir", "divmod", "enumerate", "eval", "exec", "filter", 
  "float", "format", "frozenset", "getattr", "globals", "hasattr", "hash", 
  "help", "hex", "id", "input", "int", "isinstance", "issubclass", "iter", 
  "len", "list", "locals", "map", "max", "memoryview", "min", "next", "object", 
  "oct", "open", "ord", "pow", "print", "property", "range", "repr", "reversed",
  "round", "set", "setattr", "slice", "sorted", "staticmethod", "str", "sum", 
  "super", "tuple", "type", "vars", "zip"
];
    
/// Returns where an identifier is a preexisting python keyword
/// (Only checks base python functions)
fn is_python_keyword(identifier : &String) -> bool {
  PYTHON_KEYWORDS.contains(&identifier.as_str())
}

/// Validates the text_id of a mapping against python variable naming 
/// conventions
fn validate_mapping_identifier(mapping : &NodeMapping)-> ServerResult<()> {
  let text_id : &String = &mapping.text_id;

  // Mapping's text_id should not be a python keyword
  if is_python_keyword(text_id) {
    return Err(bad_request(
      format!("Mapping name \"{}\" is already a python keyword", text_id)
    ));
  }

  // Mapping's text_id should not be empty
  if text_id.is_empty() {
    return Err(bad_request(
      // As there is no name, identify the mapping through it's parameters
      // May change to sensor_type + board_id + channel if needed
      format!("A mapping had an empty name field!\n{:#?}", mapping)
    ));
  }

  
  // all characters must be alphanumber or '_'
  for character in text_id.chars() {
    // While this is checked covered in future code,
    // people testing constantly mess this up, so there is a specific
    // Error message when using '-' in mapping identifiers
    if character == '-' {
      return Err(bad_request(
        format!("mapping name \"{}\" {}", 
          text_id,
          "contains the symbol '-', which is NOT ALLOWED AS IT BREAKS SEQUENCES"
        )
      ));
    }
    
    if character == ' ' {
      return Err(bad_request(
        format!("mapping name \"{}\" {}", 
          text_id,
          "contains a space, which is NOT ALLOWED"
        )
      ));
    }
    // Actually check if alphanumeric or '_'
    if !(character.is_alphanumeric() || character == '_') {
      return Err(bad_request(
        format!("mapping name \"{}\" contains invalid character '{}'", 
          text_id, character,)
      ));
    }
  }

  // First character cannot be a digit
  let first_character : char = text_id.chars().next()
    .expect("text_id should not be empty, and thus this should never panic");

  if !(first_character.is_alphabetic() || first_character == '_') {
    return Err(bad_request(
      format!("mapping name \"{}\" cannot start with a digit", 
        text_id)
    ));
  }

  // Yay it passed
  Ok(())
}

/// Validates the text_id's of a list of mappings against python variable naming 
/// conventions
fn validate_mappings(mappings : &Vec<NodeMapping>) -> ServerResult<()> {
  // Anti-duplicate mapping prevention
  let mut used_identifiers : HashSet<String> = HashSet::new();

  // Check all mappings
  for mapping in mappings {
    // Prevent dupliate identifiers
    if used_identifiers.contains(&mapping.text_id) {
      return Err(bad_request(
        format!("mapping \"{}\" defined multiple times", mapping.text_id)
      ));
    }
    // Add mapping name to used list
    used_identifiers.insert(mapping.text_id.clone());

    // Validate name against python naming rules
    validate_mapping_identifier(mapping)?;
  }

  // Yay they all passed
  Ok(())
}


#[cfg(test)]
mod mapping_validation_tests {
  use super::*;

  const SAMPLE_VALID_MAPPING_NAMES : [&str; 17] = [
    "KBT_V", "tbh", "YJSP", "FT_PT", "Vlt12", "AND", "BAZINGA", "go_2_da_store", 
    "COGNITO", "ergo", "SUM", 
    "hor", "her", "or", "har",
    "_twelve_", "input_V",
  ];
  const SAMPLE_INVALID_MAPPING_NAMES : [&str; 11] = [
    "1KBT_V", "tbh idk", "1overhead2", "go 2 the store", 
    "input", "id", "sum", "", " ", "1", "-"
  ];

  #[test]
  fn valid_names_full() {
    let mut mapping : NodeMapping = NodeMapping {
      text_id : String::new(),
      board_id : String::from("sam01"),
      sensor_type : common::comm::SensorType::Pt,
      channel : 0,
      computer : common::comm::Computer::Flight,
      max : None,
      min : None,
      calibrated_offset : 0.0,
      powered_threshold : None,
      normally_closed : None
    };
    let mut mapping_vector : Vec<NodeMapping> = Vec::new();
    for name in SAMPLE_VALID_MAPPING_NAMES {
      mapping.text_id = String::from(name);
      assert!(
        validate_mapping_identifier(&mapping).is_ok(),
        "Mapping name {} should be valid", 
        mapping.text_id
      );
      mapping_vector.push(mapping.clone());
    }

    
    let result = validate_mappings(&mapping_vector);
    // We already checked if mappings are valid, this should pass
    assert!(
      result.is_ok(),
      "We already checked if mappings are valid, this should pass instead of 
      creating the error \"{:#?}\"", 
      result.expect_err("This is an error if it's not Ok")
    );
  }

  #[test]
  fn duplicate_name_catching() {
    let mut mapping : NodeMapping = NodeMapping {
      text_id : String::new(),
      board_id : String::from("sam01"),
      sensor_type : common::comm::SensorType::Pt,
      channel : 0,
      computer : common::comm::Computer::Flight,
      max : None,
      min : None,
      calibrated_offset : 0.0,
      powered_threshold : None,
      normally_closed : None
    };
    let base_mapping_vector : Vec<NodeMapping> = 
      SAMPLE_VALID_MAPPING_NAMES.iter()
        .map(|x| { mapping.text_id = String::from(*x); mapping.clone() })
        .collect();

    for name in SAMPLE_VALID_MAPPING_NAMES {
      let mut mapping_vector = base_mapping_vector.clone();
      mapping.text_id = String::from(name);
      mapping_vector.push(mapping.clone());

      let result = validate_mappings(&mapping_vector);
      // This has a duplicate, so it should fail
      assert!(
        result.is_err(),
        "Mapping validation should fail on duplicate mapping names (\"{}\")", 
        name
      );
    }
  }

  #[test]
  fn invalid_mappings_single() {
    let mut mapping : NodeMapping = NodeMapping {
      text_id : String::new(),
      board_id : String::from("sam01"),
      sensor_type : common::comm::SensorType::Pt,
      channel : 0,
      computer : common::comm::Computer::Flight,
      max : None,
      min : None,
      calibrated_offset : 0.0,
      powered_threshold : None,
      normally_closed : None
    };
    for name in SAMPLE_INVALID_MAPPING_NAMES {
      mapping.text_id = String::from(name);
      assert!(
        validate_mapping_identifier(&mapping).is_err(),
        "Mapping name {} should be invalid", 
        mapping.text_id
      );
    }
  }

  #[test]
  fn invalid_mappings_in_vector() {
    let mut mapping : NodeMapping = NodeMapping {
      text_id : String::new(),
      board_id : String::from("sam01"),
      sensor_type : common::comm::SensorType::Pt,
      channel : 0,
      computer : common::comm::Computer::Flight,
      max : None,
      min : None,
      calibrated_offset : 0.0,
      powered_threshold : None,
      normally_closed : None
    };
    let base_mapping_vector : Vec<NodeMapping> = 
      SAMPLE_VALID_MAPPING_NAMES.iter()
        .map(|x| { mapping.text_id = String::from(*x); mapping.clone() })
        .collect();

    for invalid_name in SAMPLE_INVALID_MAPPING_NAMES {
      mapping.text_id = String::from(invalid_name);
      for slot in 0..base_mapping_vector.len() {
        let mut mapping_vector = base_mapping_vector.clone();
        mapping_vector.insert(slot, mapping.clone());

        assert!(
          validate_mapping_identifier(&mapping).is_err(),
          "Mapping name {} should be invalid", 
          mapping.text_id)
      }
    }
  }
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
  // Make sure mappings are even valid
  validate_mappings(&request.mappings)?;

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
  // Make sure mappings are even valid
  validate_mappings(&request.mappings)?;

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
