CREATE TABLE RadioTelemetry (
	snapshot_id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
	vehicle_state BLOB NOT NULL,
	recorded_at REAL NOT NULL DEFAULT(unixepoch('now', 'subsec')) CHECK(recorded_at > 0)
);
