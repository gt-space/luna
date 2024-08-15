-- PRAGMAS --
PRAGMA foreign_keys = ON;

-- TABLES --
CREATE TABLE ForwardingTargets (
	target_id TEXT NOT NULL PRIMARY KEY,
	socket_address TEXT NOT NULL UNIQUE,
	expiration INTEGER NOT NULL CHECK(expiration > 0)
);

CREATE TABLE RequestLogs (
	log_id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
	endpoint TEXT NOT NULL,
	origin TEXT NOT NULL,
	hostname TEXT,
	status_code INTEGER DEFAULT NULL,
	timestamp REAL NOT NULL DEFAULT(unixepoch('now', 'subsec')) CHECK(timestamp > 0) 
);

CREATE TABLE DataLogs (
	log_id INTEGER NOT NULL PRIMARY KEY,
	raw_accumulated BLOB NOT NULL,
	frame_split_indices BLOB NOT NULL
);

CREATE TABLE NodeMappings (
	text_id TEXT NOT NULL,
	configuration_id TEXT NOT NULL,
	channel INTEGER NOT NULL,
	board_id INTEGER NOT NULL,
	sensor_type TEXT NOT NULL,
	computer TEXT NOT NULL CHECK(computer = 'flight' OR computer = 'ground'),
	active BOOLEAN NOT NULL DEFAULT FALSE,

	CONSTRAINT primary_key PRIMARY KEY (text_id, configuration_id),
	CHECK (sensor_type IN (
		'pt',
		"load_cell",
		'rail_voltage',
		'rail_current',
		'tc',
		'rtd',
		'valve'
	))
);

-- TRIGGERS --
CREATE TRIGGER update_forwarding
AFTER UPDATE ON ForwardingTargets
WHEN old.socket_address != new.socket_address
BEGIN
	SELECT forward_target(old.socket_address, 0);
	SELECT forward_target(new.socket_address, 1);
END;

CREATE TRIGGER add_forwarding
AFTER INSERT ON ForwardingTargets
BEGIN
	SELECT forward_target(new.socket_address, 1);
END;

CREATE TRIGGER remove_forwarding
AFTER DELETE ON ForwardingTargets
BEGIN
	SELECT forward_target(old.socket_address, 0);
END;

CREATE TRIGGER no_update_request_logs
BEFORE UPDATE ON RequestLogs
WHEN old.status_code IS NOT NULL
BEGIN
	SELECT RAISE(ABORT, 'Updating request logs is not permitted.');
END;

CREATE TRIGGER no_delete_request_logs
BEFORE DELETE ON RequestLogs
BEGIN
	SELECT RAISE(ABORT, 'Deleting request logs is not permitted.');
END;
