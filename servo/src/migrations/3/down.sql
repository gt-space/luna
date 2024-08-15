CREATE TABLE ForwardingTargets (
	target_id TEXT NOT NULL PRIMARY KEY,
	socket_address TEXT NOT NULL UNIQUE,
	expiration INTEGER NOT NULL CHECK(expiration > 0)
);

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
