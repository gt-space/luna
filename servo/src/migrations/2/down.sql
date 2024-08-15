DROP TABLE VehicleSnapshots;

CREATE TABLE DataLogs (
	log_id INTEGER NOT NULL PRIMARY KEY,
	raw_accumulated BLOB NOT NULL,
	frame_split_indices BLOB NOT NULL
);
