# Caution & Warning messages

Open **Views → Caution & Warning** to see the status table. Warnings are red, cautions amber, and information gray.

The top display contains only cautions and warnings; the bottom display contains only information. They share the available table area equally and scroll independently, with sticky column headers and keyboard-accessible scroll areas. The Show filter affects only the upper display. The event log remains available below both displays.

Timestamps use the device's local time zone. Its full name (for example, America/New_York) appears above the displays; each timestamp also includes the applicable abbreviation or UTC offset, including daylight-saving changes. Hovering shows the full date and time zone.

## Add a message

Open a `.json` file in `config/` and add an entry to its list:

```json
[
  {
    "severity": "caution",
    "system": "GPS",
    "description": "GPS has no position fix.",
    "conditions": { "telemetry.gps.has_fix": false }
  }
]
```

Only four fields are needed:

| Field | What to enter |
| --- | --- |
| `severity` | `warning`, `caution`, or `info` |
| `system` | Name shown in the Subsystem column |
| `description` | Text shown in Description |
| `conditions` | Field and value that trigger the message |

Put a comma between entries. Additional files in `config/` use the same list format. No IDs, titles, version, source, or enabled fields are needed. Delete an entry to remove a message. JSON does not allow comments or trailing commas.

## Conditions

A plain value means **equals**. Multiple fields in `conditions` must **all match**:

```json
"conditions": { "connected": true, "hasTelemetry": false }
```

For a numeric limit, use `>`, `>=`, `<`, or `<=`:

```json
"conditions": { "telemetry.bms.battery_bus.voltage": { "<": 24 } }
```

The number above is an example; choose the appropriate limit for your system.

Use `!=` for “not equal,” or `exists` to check whether a field is available:

```json
"conditions": { "telemetry.gps": { "exists": false } }
```

Use one comparison per field. Numbers must be numbers (`24`), not strings (`"24"`). Missing/null readings never satisfy ordinary comparisons, including `!=`; use `exists: false` for missing data. Conditions are not executable JavaScript.

## Available fields

| Field | Meaning |
| --- | --- |
| `connected` | Server connected: true/false |
| `hasTelemetry` | A frame arrived since connection/source changed |
| `telemetryFresh` | Connected with a recent frame |
| `telemetryAgeMs` | Milliseconds since the last frame, or null before the first |
| `dataSource` | `umbilical` or `tel` |
| `telemetry.…` | A field from the live telemetry payload |

Examples: `telemetry.gps.has_fix`, `telemetry.reco.0.ekf_blown_up` (RECO A), `telemetry.reco.1.ekf_blown_up` (B), and `telemetry.reco.2.ekf_blown_up` (C). See `StreamState` in `../comm.tsx` for available telemetry fields.

Telemetry conditions automatically stop displaying when disconnected or stale. Freshness measures GUI frame arrival, not individual sensor age. The threshold is `ACTIVITY_WARN_THRESH` in `../appdata.tsx`, checked every 100 ms.

Conditions clear when they stop matching. The timestamp records when a condition became active in that view, stays fixed during updates/filtering, and resets on reactivation or reopening the view.

Configuration problems appear in the panel; other valid messages keep working. The display does not send commands, persist alarms, or confirm overall system health.

## Apply changes

Restart the development server after adding a configuration file. Rebuild/restart packaged apps after changing JSON. Files are bundled with the GUI, not loaded from an external runtime folder.

Old files must be migrated: use a list instead of the `version`/`messages` wrapper, remove `id`, `title`, `source`, and `enabled`, and turn `all` entries into `conditions` fields. For example, `{ "path": "connected", "operator": "eq", "value": false }` becomes `"conditions": { "connected": false }`.

From `gui/`, run `npm run build` and `npx playwright test tests/caution-warning.spec.ts`. Browser checks are in `tests/caution-warning-ui.spec.ts` and require a running development server and Playwright Chromium.

For the previous four-field format, rename subsystem to system, message to description, and when to conditions. Condition values are unchanged.
