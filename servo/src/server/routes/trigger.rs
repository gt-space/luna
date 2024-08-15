use axum::{extract::State, Json};
use common::comm::Trigger;
use rusqlite::params;
use serde::{Deserialize, Serialize};

use crate::server::{self, error::internal, Shared};

/// Route function which returns all existing triggers in the database.
pub async fn get_triggers(
  State(shared): State<Shared>,
) -> server::Result<Json<Vec<Trigger>>> {
  let database = shared.database.connection.lock().await;

  let triggers = database
    .prepare("SELECT name, condition, script, active FROM Triggers")
    .map_err(internal)?
    .query_and_then([], |row| {
      Ok(Trigger {
        name: row.get(0)?,
        condition: row.get(1)?,
        script: row.get(2)?,
        active: row.get(3)?,
      })
    })
    .map_err(internal)?
    .collect::<rusqlite::Result<Vec<Trigger>>>()
    .map_err(internal)?;

  Ok(Json(triggers))
}

/// Route function which creates or updates a trigger in the database and on the
/// flight computer.
pub async fn set_trigger(
  State(shared): State<Shared>,
  Json(request): Json<Trigger>,
) -> server::Result<()> {
  let database = shared.database.connection.lock().await;

  database
    .execute(
      "
			INSERT INTO Triggers (name, condition, script, active)
			VALUES (?1, ?2, ?3, ?4)
			ON CONFLICT (name) DO UPDATE SET
				condition = excluded.condition,
				script = excluded.script,
				active = excluded.active
		",
      params![
        request.name,
        request.condition,
        request.script,
        request.active
      ],
    )
    .map_err(internal)?;

  drop(database);

  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    flight.send_trigger(request).await.map_err(internal)?;
  }

  Ok(())
}

/// Request struct used to delete a trigger from the database and make it
/// inactive on the flight computer.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct DeleteTriggerRequest {
  /// The name of the trigger to be deleted.
  pub name: String,
}

/// Route function which deletes a trigger from the database and sets it
/// inactive on the flight computer.
pub async fn delete_trigger(
  State(shared): State<Shared>,
  Json(request): Json<DeleteTriggerRequest>,
) -> server::Result<()> {
  let database = shared.database.connection.lock().await;

  database
    .execute(
      "DELETE FROM Triggers WHERE name = ?1",
      params![request.name],
    )
    .map_err(internal)?;

  drop(database);

  if let Some(flight) = shared.flight.0.lock().await.as_mut() {
    flight
      .send_trigger(Trigger {
        name: request.name,
        condition: "False".to_owned(),
        script: "".to_owned(),
        active: false,
      })
      .await
      .map_err(internal)?;
  }

  Ok(())
}
