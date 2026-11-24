# Servo API

Servo serves HTTP and WebSocket on port 7200 with permissive CORS. This file
documents the CTV (hopper) additions; the other routes are listed in
`servo/src/server/mod.rs`.

## `POST /operator/ctv-command`

Forwards a CTV operations command to the flight computer
(`FlightControlMessage::CtvCommand`). Used by the GUI's CTV page.

Request body (`Content-Type: application/json`):

```json
{ "command": "start_flight" }
```

`command` is a `CtvOperatorCommand` in snake_case, one of:

| `command`      | Meaning                                                                    |
| -------------- | -------------------------------------------------------------------------- |
| `arm_ec`       | Give the engine controller output authority.                               |
| `disarm_ec`    | Take output authority away.                                                |
| `start_flight` | Arm the flight logic from the nav estimate and start the mission clock.    |
| `abort_land`   | Land: abort to land in flight, safe the EC on the ground.                  |
| `terminate`    | Safe the EC immediately and latch it.                                      |
| `clear_safing` | Clear the EC's safing latch.                                               |
| `reinit_nav`   | Throw the navigation filter away and level it again. Pad only: refused in flight. |

Responses:

* `200 OK`, empty body: Servo wrote the command to the flight computer's TCP
  connection. This is not an acknowledgement that the flight computer acted on
  it: watch `ctv` in the telemetry stream for the effect.
* `400 Bad Request`, body `unrecognized ctv command: <name>`: unknown command.
  Checked before anything is sent.
* `500 Internal Server Error`, body `flight computer not connected`: no flight
  computer connection (same behaviour as `/operator/abort`). Other 500s are I/O
  errors writing to the flight computer.
* A body that is not JSON with a string `command` is rejected by axum's JSON
  extractor (`415` / `422`), as for the other routes.

Every accepted command is logged on Servo's console.

## `GET /data/forward?source=umbilical|tel` (WebSocket)

Sends the whole `VehicleState` as JSON text every 100 ms (the last received
state is re-sent even if the flight computer has gone silent; use
`/data/telemetry-stats` to tell). `VehicleState` has a field `ctv`, the
`CtvStatus` of the CTV flight logic:

```json
{
  "ctv": {
    "mode": "Idle",
    "mission_time_s": 0.0,
    "schedule_time_s": 0.0,
    "schedule_end_s": 30.0,
    "altitude_m": 0.0,
    "tilt_deg": 0.0,
    "speed_mps": 0.0,
    "ec_connected": true,
    "ec_state": "Disarmed",
    "ec_safing_reason": "",
    "fault": "",
    "last_event": "",
    "gains": "lqr_ctv_tuning @ 38b9708",
    "missed_ticks": 0,
    "preflight": [{ "name": "Navigation estimate", "ok": true, "detail": "..." }],
    "can_start": false
  }
}
```

`mode` is one of `Off`, `Idle`, `Armed`, `Flight`, `AbortLand`, `Landed`,
`Safed`. `ec_state` is `Disarmed`, `Armed`, `Safing`, or empty if unknown.

`ctv` is only present with `source=umbilical`. It is excluded from the
compressed radio telemetry, so with `source=tel` it is always the default
(`mode: "Off"`). It is stored with every snapshot in the `VehicleSnapshots`
table (postcard), like the rest of the state; it is not in the CSV/HDF5
exports.

## `GET /data/telemetry-stats`

Transport stats per source: `time_since_update_ms`, `update_rate_hz`,
`packet_size_bytes` for `umbilical` and `tel`. The CTV page uses
`time_since_update_ms` to detect stale CTV telemetry.
