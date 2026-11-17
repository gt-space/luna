use std::{cmp::Ordering, future::Future, path::Path, sync::Arc};

use anyhow::anyhow;
use common::comm::{NodeMapping, VehicleState};
use include_dir::{include_dir, Dir};
use jeflog::warn;
use rusqlite::Connection as SqlConnection;
use tokio::sync::Mutex;

use super::Shared;
use crate::server::telemetry::TelemetrySource;

// include_dir is a separate library which evidently accesses files relative to
// the project root, while include_str is a standard library macro which
// accesses relative to the current file. why the difference? who knows.
const MIGRATIONS: Dir = include_dir!("./servo/src/migrations");
const BOOTSTRAP_QUERY: &str = include_str!("../migrations/bootstrap.sql");

/// Maximum number of snapshots committed to the database in one transaction.
const SNAPSHOT_BATCH_LIMIT: usize = 500;

/// Inserts a batch of vehicle states in a single transaction.
///
/// States that fail to serialize are skipped with a warning; any SQL error
/// rolls back the whole batch.
fn insert_snapshot_batch(
    connection: &mut SqlConnection,
    query: &str,
    batch: &[VehicleState],
) -> rusqlite::Result<()> {
    let mut buffer = [0_u8; 10_000];
    let transaction = connection.transaction()?;

    {
        let mut statement = transaction.prepare_cached(query)?;

        for vehicle_state in batch {
            match postcard::to_slice(vehicle_state, &mut buffer) {
                Ok(serialized) => {
                    statement.execute([&*serialized])?;
                }
                Err(error) => {
                    warn!("Failed to serialize vehicle state into Postcard: {error}");
                }
            }
        }
    }

    transaction.commit()
}

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
        let connection = SqlConnection::open(path)?;

        // WAL mode makes each commit much cheaper than the default rollback
        // journal and lets readers (e.g. exports) run alongside the logger.
        // FULL sync keeps the same power-loss durability as before: a commit
        // is only reported once it is safely on disk.
        let journal_mode: String =
            connection.query_row("PRAGMA journal_mode = WAL", [], |row| row.get(0))?;
        if !journal_mode.eq_ignore_ascii_case("wal") {
            warn!("Failed to enable WAL mode; SQLite journal mode is {journal_mode}");
        }
        connection.pragma_update(None, "synchronous", "FULL")?;

        Ok(Database {
            connection: Arc::new(Mutex::new(connection)),
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

        let current_migration =
            connection.query_row("SELECT MAX(migration_id) FROM Migrations", [], |row| {
                row.get::<_, i32>(0)
            })?;

        match current_migration.cmp(&target_migration) {
            Ordering::Less => {
                for migration in current_migration + 1..=target_migration {
                    let sql = MIGRATIONS
                        .get_file(format!("{migration}/up.sql"))
                        .ok_or(anyhow!("up.sql script for migration {migration} not found"))?
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
                        .ok_or(anyhow!("down.sql for migration {migration} is not UTF-8"))?;

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
    pub fn log_vehicle_state(
        &self,
        shared: &Shared,
        source: TelemetrySource,
    ) -> impl Future<Output = ()> {
        let snapshots = shared
            .telemetry
            .get(source)
            .snapshot_receiver
            .lock()
            .unwrap()
            .take();
        let connection = self.connection.clone();

        async move {
            let Some(mut snapshots) = snapshots else {
                warn!("Snapshot logger already started for {source}");
                return;
            };
            let query = format!(
                "INSERT INTO {} (vehicle_state) VALUES (?1)",
                source.snapshot_table()
            );
            let mut batch = Vec::with_capacity(SNAPSHOT_BATCH_LIMIT);

            // Waits for at least one snapshot, then takes everything already
            // queued (up to the limit) and commits it as one transaction, so
            // the per-commit disk sync cost is paid once per batch instead of
            // once per snapshot.
            while snapshots.recv_many(&mut batch, SNAPSHOT_BATCH_LIMIT).await > 0 {
                let mut connection = connection.clone().lock_owned().await;
                let query = query.clone();

                // SQLite calls block (including the disk sync on commit), so
                // they run on Tokio's blocking pool rather than an async
                // worker thread.
                let result = tokio::task::spawn_blocking(move || {
                    let result = insert_snapshot_batch(&mut connection, &query, &batch);
                    batch.clear();
                    (batch, result)
                })
                .await;

                match result {
                    Ok((returned_batch, Ok(()))) => batch = returned_batch,
                    Ok((returned_batch, Err(error))) => {
                        warn!("Failed to insert vehicle state batch into database: {error}");
                        batch = returned_batch;
                    }
                    Err(error) => {
                        warn!("Snapshot logging batch task failed: {error}");
                        batch = Vec::with_capacity(SNAPSHOT_BATCH_LIMIT);
                    }
                }
            }
        }
    }

    /// Returns the currently active node mappings used for flight updates.
    pub async fn active_mappings(&self) -> rusqlite::Result<Vec<NodeMapping>> {
        let connection = self.connection.lock().await;
        let mut statement = connection.prepare(
            "
        SELECT
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
        WHERE active = TRUE
      ",
        )?;
        let rows = statement.query_and_then([], |row| {
            Ok::<_, rusqlite::Error>(NodeMapping {
                text_id: row.get(0)?,
                board_id: row.get(1)?,
                sensor_type: row.get(2)?,
                channel: row.get(3)?,
                computer: row.get(4)?,
                max: row.get(5)?,
                min: row.get(6)?,
                calibrated_offset: row.get(7)?,
                powered_threshold: row.get(8)?,
                normally_closed: row.get(9)?,
            })
        })?;
        let mappings = rows.collect::<rusqlite::Result<Vec<_>>>()?;
        Ok(mappings)
    }
}

#[cfg(test)]
mod tests {
    use std::{sync::atomic::Ordering, time::Duration};

    use common::comm::{sam::Unit, Measurement, VehicleState};
    use tokio::time::{sleep_until, timeout, Instant};

    use super::*;
    use crate::server::{telemetry::update_live_telemetry, Server};

    async fn persist_numbered_snapshots(stall_database: bool) {
        let server = Server::new(None).unwrap();
        let database = server.shared.database.clone();
        database
            .connection
            .lock()
            .await
            .execute_batch(
                "CREATE TABLE VehicleSnapshots (vehicle_state BLOB NOT NULL);
             CREATE TABLE RadioTelemetry (vehicle_state BLOB NOT NULL);",
            )
            .unwrap();
        let sources = [TelemetrySource::Umbilical, TelemetrySource::Radio];
        let loggers: Vec<_> = sources
            .iter()
            .map(|&source| tokio::spawn(database.log_vehicle_state(&server.shared, source)))
            .collect();
        let mut stalled = if stall_database {
            Some(database.connection.lock().await)
        } else {
            None
        };

        let start = Instant::now();
        for id in 1..=100 {
            sleep_until(start + Duration::from_millis(id * 10)).await;
            for (index, source) in sources.iter().enumerate() {
                let mut state = VehicleState::new();
                state.sensor_readings.insert(
                    "sequence_id".into(),
                    Measurement {
                        value: (id + index as u64 * 1000) as f64,
                        unit: Unit::Unknown,
                    },
                );
                update_live_telemetry(server.shared.telemetry.get(*source), state, 128).await;
            }
            if id == 20 {
                drop(stalled.take());
            }
        }

        for source in sources {
            assert_eq!(
                server
                    .shared
                    .telemetry
                    .get(source)
                    .logging_dropped_snapshots
                    .load(Ordering::Relaxed),
                0
            );
        }
        drop(server);
        timeout(Duration::from_secs(5), async {
            for logger in loggers {
                logger.await.unwrap();
            }
        })
        .await
        .expect("loggers must drain and stop when producers close");

        let connection = database.connection.lock().await;
        for (index, source) in sources.iter().enumerate() {
            let mut statement = connection
                .prepare(&format!(
                    "SELECT vehicle_state FROM {} ORDER BY rowid",
                    source.snapshot_table(),
                ))
                .unwrap();
            let ids: Vec<_> = statement
                .query_map([], |row| row.get::<_, Vec<u8>>(0))
                .unwrap()
                .map(|row| {
                    let state: VehicleState = postcard::from_bytes(&row.unwrap()).unwrap();
                    state.sensor_readings["sequence_id"].value as u64
                })
                .collect();
            let offset = index as u64 * 1000;
            assert_eq!(ids, (1..=100).map(|id| id + offset).collect::<Vec<_>>());
        }
    }

    #[tokio::test]
    async fn snapshot_logging_preserves_numbered_states_at_100_hz() {
        persist_numbered_snapshots(false).await;
    }

    #[tokio::test]
    async fn snapshot_logging_preserves_states_across_200_ms_database_stall() {
        persist_numbered_snapshots(true).await;
    }

    #[tokio::test]
    async fn cancelled_logger_closes_channel_and_live_updates_continue() {
        let server = Server::new(None).unwrap();
        let telemetry = server.shared.telemetry.get(TelemetrySource::Umbilical);
        let logger = tokio::spawn(
            server
                .shared
                .database
                .log_vehicle_state(&server.shared, TelemetrySource::Umbilical),
        );
        tokio::task::yield_now().await;
        logger.abort();
        assert!(logger.await.unwrap_err().is_cancelled());
        assert!(telemetry.snapshot_sender.is_closed());
        update_live_telemetry(telemetry, VehicleState::new(), 128).await;
        assert_eq!(
            telemetry.logging_dropped_snapshots.load(Ordering::Relaxed),
            1
        );
        assert_eq!(*telemetry.packet_size.0.lock().await, Some(128));
    }

    #[test]
    fn dropping_unpolled_logger_closes_channel() {
        let server = Server::new(None).unwrap();
        drop(
            server
                .shared
                .database
                .log_vehicle_state(&server.shared, TelemetrySource::Umbilical),
        );
        assert!(server
            .shared
            .telemetry
            .umbilical
            .snapshot_sender
            .is_closed());
        assert!(!server.shared.telemetry.radio.snapshot_sender.is_closed());
    }
}
