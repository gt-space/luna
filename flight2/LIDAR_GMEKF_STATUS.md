# Update to the Rust GMEKF handoff: LiDAR

**Update date:** 2026-11-21  
**Project:** Luna flight computer, `flight2`  
**Companion document:** `gmekf-rust-handoff-2026-11-20.md`

This is an update to the original handoff, not a replacement for it. The
original describes the GMEKF port, shadow-navigation design, controller work,
and non-LiDAR hardware unknowns. Keep using that document for those topics.
This addendum records progress on the LiDAR work since the original handoff
was written and clarifies which LiDAR statements in it are now outdated.

## Summary

The LiDAR measurement model, four-beam MATLAB reference sequence, GMEKF
wrapper, and shadow-navigation software path are now implemented in the
working tree. The MATLAB-generated reference checks succeeded in the earlier
work. The Rust suite has **not** yet passed: a Windows test attempt stopped
while compiling the Unix-specific `rppal` GPIO dependency, before the flight
computer tests ran.

LiDAR fusion remains disabled by default. It must stay disabled until the team
confirms the model's COM-versus-IMU frame convention and supplies measured
mount geometry. The driver also currently knows only two adapter endpoints;
the real endpoints for LiDAR IDs 3 and 4 are still needed.

## Changes to the original handoff

The following statements in the 2026-11-20 handoff are superseded or need
updating:

- **§0 TL;DR:** the LiDAR model, wrapper, reference tests, and shadow-nav
  plumbing are no longer wholly undone. The remaining work is listed below.
- **§8 known issue 5:** LiDAR is no longer entirely unused in software; the
  implementation is present but opt-in and not validated on the vehicle.
- **§11 LiDAR steps:** steps 1-6 have implementation work completed (see
  below). Step 7, frame agreement and the physical sign test, is still open.
- **§12 item 7 and §14 question 4:** the vehicle is being treated as having
  four LiDARs, per the current project clarification. However, the checked-in
  driver endpoint list still configures only IDs 1 and 2. Confirm endpoint
  addresses and the COM/IMU convention with the team and David.
- **§13 test-day checklist:** add LiDAR telemetry and geometry checks before
  enabling LiDAR fusion; do not infer correct operation just because the
  general navigation estimate is healthy.

The original handoff was checked against an earlier repository snapshot. Its
branch status, controller merge status, non-LiDAR TODOs, and other hardware
facts should be rechecked separately before acting on them.

## LiDAR work completed

### Model and GMEKF wrapper

- Ported David's `lidar_update_model.m` to `lidar_model(...)` in
  `src/navigate/gmekf/math.rs`. It returns predicted-minus-measured range and
  the error-state Jacobian, and marks unusable geometry with `None`.
- Added four mount slots and `Measurement::Lidar` handling in
  `src/navigate/gmekf.rs`.
- Added the calibrated `lidar_covariance(range)` lookup in `math.rs`, using the
  nearest nominal-range band and the brick-histogram variance-plus-bias values.
  It accepts 0.2-4 m and returns `None` for invalid, non-finite, or out-of-band
  ranges.
- The wrapper rejects indices outside 0-3 and invalid covariance ranges, uses
  the returned variance directly, gates at 6.635, tracks last-used time per
  sensor, and reports LiDAR 1-4 through diagnostics slots 3-6 (zero-based).

### MATLAB reference and sequence

- Added LiDAR model cases and a four-beam simulated sequence in
  `src/navigate/gmekf/testdata/make_reference.m`.
- Regenerated `unit_cases.json` and `sequence.json`. The sequence includes a
  missing reading and a planted outlier.
- Added model and sequence assertions for all four sensors in
  `src/navigate/gmekf/tests.rs`.
- Added tests for every supplied calibration value, nearest-band lookup,
  accepted range endpoints and invalid inputs. Updated the sequence generator
  and Rust sequence comparison to use the calibrated covariance.
- Regenerated the MATLAB sequence with the new covariance: valid samples lie
  within 0.2-4.0 m, the planned missing sample remains, and an in-range planted
  LiDAR 4 outlier at step 405 is rejected. This validates agreement with that
  reference setup, not the physical vehicle geometry.

### Driver and shadow-navigation plumbing

- Added a navigation-sample accessor in `src/lidar.rs` exposing sensor ID,
  sample ID, range in meters, quality, and receive time.
- `src/main.rs` forwards only new samples whose quality is `Ok` and whose age
  is under 350 ms; repeat samples are suppressed using ID and receive time.
- Added `Input::Lidar` and the estimator update path in `src/shadow_nav.rs`.
- Added `use_lidar` (default `false`) and `lidar_mounts` settings. Startup
  validation checks IDs 1-4, duplicate IDs, finite geometry, unit beam
  directions, and requires four mounts when fusion is enabled.
- Added four LiDAR flags to the terminal status line. The CSV already has
  four acceptance and NIS columns.
- Updated `flight2/README.md` with the settings shape and warnings.

## Remaining LiDAR work

### Required before enabling fusion

1. **Resolve COM versus IMU frames.** David's model defines sensor position
   relative to the COM and uses `height_com`; shadow-nav position is described
   relative to the IMU. The current config/model wiring follows the
   COM-relative model. Confirm whether the state/model needs a conversion or
   whether the state convention should change. Do not silently substitute
   IMU-relative values for COM-relative values.
2. **Measure all four mounts and height.** Obtain each beam direction and
   sensor position in the agreed body frame, confirm COM height and physical
   sensor-ID mapping, and replace all placeholder/example values.
3. **Configure endpoints for LiDAR 3 and 4.** `src/lidar.rs` currently lists
   only IDs 1 and 2 at `192.168.1.200:2000` and `192.168.1.201:2000`. Add the
   real remaining endpoint addresses only after they are confirmed.
4. **Check signs and geometry on the vehicle.** In a safe stationary test,
   lift the vehicle: range should increase and NED down should become more
   negative. Also check the predicted response when tilted. Stop and
   investigate if ranges gate unexpectedly or the estimate shifts while the
   vehicle is still.

The README JSON values are schema examples only, not usable mount
measurements. Keep `use_lidar` false until items 1-4 have been addressed.

### Software verification

1. **Rust tests passed in WSL/Linux:** `cargo test --manifest-path
   flight2/Cargo.toml` completed with 110 passed, 0 failed and 2 ignored on
   2026-11-21. The focused covariance tests also passed. A Windows test attempt
   still fails in `rppal` because that dependency uses Unix APIs; this is a
   platform limitation, not a Rust test result.
2. **End-to-end bridge test remains open.** Current tests cover the accessor,
   model, wrapper, and reference sequence, but do not exercise a TCP frame all
   the way through `main.rs` and the shadow-nav thread. Add replay/simulation
   coverage or test against the adapters once available. Verify quality/stale
   rejection, repeat suppression, ID-to-mount mapping, and CSV/status results.

## How to run the offline tests

From a Linux or WSL shell with Rust installed, at the repository root:

```bash
cargo test --manifest-path flight2/Cargo.toml lidar
cargo test --manifest-path flight2/Cargo.toml
cargo fmt --manifest-path flight2/Cargo.toml -- --check
```

The first command selects tests with `lidar` in their test name. The full
suite is also necessary because sequence tests that exercise LiDAR may not
have `lidar` in their names. In WSL, a checkout on the Windows C: drive is
commonly at `/mnt/c/Users/jayby/repos/luna`; adjust if yours is elsewhere.

The full Rust test suite and focused covariance tests passed in WSL. `rustfmt
--check` passed for the changed GMEKF Rust files, and `git diff --check` passed.

## Files changed for this update

- `src/navigate/gmekf/math.rs` — range model and Jacobian.
- `src/navigate/gmekf.rs` — mounts, range update, timing, noise, gate, and
  diagnostics.
- `src/navigate/gmekf/tests.rs` — model, wrapper, and four-sensor sequence
  coverage.
- `src/navigate/gmekf/testdata/make_reference.m` — MATLAB reference generator.
- `src/navigate/gmekf/testdata/unit_cases.json` and `sequence.json` — generated
  reference fixtures.
- `src/lidar.rs` — sample accessor and its unit test.
- `src/main.rs` — forwarding of fresh, valid, non-repeated samples.
- `src/shadow_nav.rs` — LiDAR config, validation, input, update, and status
  reporting.
- `README.md` — configuration example and operational caveats.
