-- This query is used to bootstrap the database whenever Servo is started up.
-- It must assume nothing about the state of the database at this point, and if it
-- is already bootstrapped for migrations, then it must not make any changes.
-- This query simply sets up the migration table and lets migration 1 everything else.

CREATE TABLE IF NOT EXISTS Migrations (
	migration_id INTEGER NOT NULL PRIMARY KEY,
	completed_at INTEGER NOT NULL DEFAULT (unixepoch('now'))
);

INSERT INTO Migrations (migration_id) VALUES (0) ON CONFLICT DO NOTHING;
