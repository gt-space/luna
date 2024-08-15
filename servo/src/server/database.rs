use anyhow::anyhow;
use include_dir::{include_dir, Dir};
use jeflog::warn;
use rusqlite::Connection as SqlConnection;
use std::{cmp::Ordering, future::Future, path::Path, sync::Arc};
use tokio::sync::Mutex;

use super::Shared;

// include_dir is a separate library which evidently accesses files relative to
// the project root, while include_str is a standard library macro which
// accesses relative to the current file. why the difference? who knows.
const MIGRATIONS: Dir = include_dir!("./servo/src/migrations");
const BOOTSTRAP_QUERY: &str = include_str!("../migrations/bootstrap.sql");

/// A convenience type representing a `rusqlite::Connection` that may be passed
/// to multiple async contexts at once.
#[derive(Clone, Debug)]
pub struct Database {
  /// The raw SQL connection, wrapped in an `Arc` and `Mutex` for thread
  /// safety.
  pub connection: Arc<Mutex<SqlConnection>>,
}

impl Database {
  /// Opens a new `Database` at the path, enclosing a raw SQL connection.
  pub fn open(path: &Path) -> rusqlite::Result<Self> {
    Ok(Database {
      connection: Arc::new(Mutex::new(SqlConnection::open(path)?)),
    })
  }

  /// Opens a new `Database` in memory, so if it is closed, it's not saved.
  pub fn volatile() -> rusqlite::Result<Self> {
    Ok(Database {
      connection: Arc::new(Mutex::new(SqlConnection::open_in_memory()?)),
    })
  }

  /// Migrates the database to the latest available migration version.
  pub fn migrate(&self) -> anyhow::Result<()> {
    let latest_migration = MIGRATIONS
      .dirs()
      .filter_map(|directory| {
        directory
          .path()
          .file_name()
          .and_then(|name| name.to_string_lossy().parse::<i32>().ok())
      })
      .max();

    if let Some(latest_migration) = latest_migration {
      self.migrate_to(latest_migration)
    } else {
      Ok(())
    }
  }

  /// Migrates the database to a specific migration index.
  pub fn migrate_to(&self, target_migration: i32) -> anyhow::Result<()> {
    let connection = self.connection.blocking_lock();

    // the bootstrap query ensures that migration is set up
    // and changes nothing if it is already set up
    connection.execute_batch(BOOTSTRAP_QUERY)?;

    let current_migration = connection.query_row(
      "SELECT MAX(migration_id) FROM Migrations",
      [],
      |row| row.get::<_, i32>(0),
    )?;

    match current_migration.cmp(&target_migration) {
      Ordering::Less => {
        for migration in current_migration + 1..=target_migration {
          let sql = MIGRATIONS
            .get_file(format!("{migration}/up.sql"))
            .ok_or(anyhow!(
              "up.sql script for migration {migration} not found"
            ))?
            .contents_utf8()
            .ok_or(anyhow!("up.sql for migration {migration} is not UTF-8"))?;

          connection.execute_batch(sql)?;
          connection.execute(
            "INSERT INTO Migrations (migration_id) VALUES (?1)",
            [migration],
          )?;
        }
      }
      Ordering::Greater => {
        for migration in (target_migration..=current_migration).rev() {
          let sql = MIGRATIONS
            .get_file(format!("{migration}/down.sql"))
            .ok_or(anyhow!(
              "down.sql script for migration {migration} not found"
            ))?
            .contents_utf8()
            .ok_or(anyhow!(
              "down.sql for migration {migration} is not UTF-8"
            ))?;

          connection.execute_batch(sql)?;
          connection.execute(
            "DELETE FROM Migrations WHERE migration_id = ?1",
            [migration],
          )?;
        }
      }
      Ordering::Equal => {}
    };

    Ok(())
  }

  /// Continuously logs the vehicle state each time a new one arrives into the
  /// database.
  pub fn log_vehicle_state(&self, shared: &Shared) -> impl Future<Output = ()> {
    let vehicle_state = shared.vehicle.clone();
    let connection = self.connection.clone();

    async move {
      let mut buffer = [0_u8; 10_000];

      loop {
        vehicle_state.1.notified().await;
        let vehicle_state = vehicle_state.0.lock().await.clone();

        match postcard::to_slice(&vehicle_state, &mut buffer) {
          Ok(serialized) => {
            let query_result = connection.lock().await.execute(
              "INSERT INTO VehicleSnapshots (vehicle_state) VALUES (?1)",
              [&*serialized],
            );

            if let Err(error) = query_result {
              warn!("Failed to insert vehicle state into database: {error}");
            }
          }
          Err(error) => {
            warn!("Failed to serialize vehicle state into Postcard: {error}");
          }
        };
      }
    }
  }
}
