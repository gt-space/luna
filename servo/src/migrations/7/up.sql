CREATE TABLE AbortConfigs (
	name TEXT NOT NULL PRIMARY KEY,
	condition TEXT NOT NULL,
	config BLOB NOT NULL
);
