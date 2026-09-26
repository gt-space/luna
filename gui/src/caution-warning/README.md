# Caution & Warning

The main window shows active status messages in severity order: **Warning**, **Caution**, then **Information**. Each card includes a subsystem, a short title, and an explanation. Use the Show menu to filter messages; totals always count all active messages. The expandable Event log preserves the existing server/GUI alerts.

## Add a message

Edit a file in `config/`, or add another `.json` file there. Every file uses this structure:

```json
{
  "version": 1,
  "messages": [
    {
      "id": "gps-fix-acquired",
      "severity": "info",
      "subsystem": "GPS",
      "title": "GPS fix acquired",
      "message": "GPS reports that a position fix is available.",
      "source": "telemetry",
      "all": [
        { "path": "telemetry.gps.has_fix", "operator": "eq", "value": true }
      ]
    }
  ]
}
```

- `id`: unique across all configuration files.
- `severity`: `warning`, `caution`, or `info`. Use these for faults, conditions needing attention, and ordinary status messages respectively.
- `subsystem`, `title`, `message`: plain text displayed on the card.
- `source`: `telemetry` for rules inspecting stream values; `system` for connection/status rules.
- `all`: one or more conditions; every condition must match.
- `enabled`: optional boolean; set to `false` to disable a message.

Files are bundled by Vite. Restart the development server after adding a file; rebuild/restart the packaged app after configuration changes. They are not externally editable runtime files. JSON does not support comments or trailing commas. Invalid entries are skipped and reported visibly in the panel; valid entries continue to work.

## Conditions

Paths use dot notation. Array indexes are numeric segments, such as `telemetry.reco.0.ekf_blown_up` (RECO A), `.1.` (B), or `.2.` (C). Stream fields are defined by `StreamState` in `../comm.tsx`.

| Operator | Meaning | Value |
| --- | --- | --- |
| `eq`, `ne` | Strict equality / inequality | String, number, or boolean |
| `gt`, `gte`, `lt`, `lte` | Numeric comparison | Number |
| `exists`, `missing` | Value is present / absent | Omit |

No JavaScript expressions are executed. Missing, null, or non-finite values fail comparisons, including `ne`. To explicitly report an unavailable field, use `missing`. Numeric comparisons never convert strings to numbers.

Available roots:

| Path | Meaning |
| --- | --- |
| `connected` | GUI connection flag |
| `dataSource` | `umbilical` or `tel` |
| `hasTelemetry` | A frame arrived since connection/source changed |
| `telemetryAgeMs` | Milliseconds since that frame, or null before the first frame |
| `telemetryFresh` | Connected and frame age below `ACTIVITY_WARN_THRESH` (currently 500 ms) |
| `telemetry` | Latest `device_update` payload |

Telemetry rules are suspended when disconnected or stale. Reconnecting or selecting another source clears the cached frame. Freshness measures GUI frame arrival, not individual sensor sample age; if a device repeats old values inside fresh frames, add rules using its reported age/status fields. The panel polls age every 100 ms.

Messages clear automatically when conditions stop matching. They are not latched, acknowledged, or persisted, and do not issue commands or replace abort logic. No active messages means only that no configured conditions matched, not that the whole system is healthy. The initial rules cover connection/stream status, GPS no-fix, and the three RECO EKF fault flags; add validated limits and device-specific coverage as needed.

## Code layout

- `rules.ts`: types, configuration validation, and pure condition evaluation.
- `CautionWarning.tsx`: telemetry subscription and connection/freshness state.
- `CautionWarningPanel.tsx`: presentation and filtering, with no transport logic.
- `caution-warning.css`: scoped card styling and responsive main-window layout.

From `gui/`, run `npm run build` and `npx playwright test tests/caution-warning.spec.ts` to verify configuration and evaluator behavior.

For browser integration tests, install Chromium with `npx playwright install chromium`, start `npm run dev` in another terminal, then run `npx playwright test tests/caution-warning-ui.spec.ts`. These use mocked Tauri events to exercise the real main window, including stale data and source changes; they do not require connected hardware.
