# Local progress: abort and sequence validation

## M0 — setup and investigation (2026-11-09)

### Completed work
- Confirmed a clean `main` worktree and created local branch `fix/abort-sequence-validation` from `main` (`2d2a4f6`). No remote writes.
- Environment: Windows 10 build 26200; Git 2.47.1, Rust/Cargo 1.98.0, Docker 27.5.1, Python 3.13.1, Node 22.13.1, npm 10.9.2. Nix and GitHub CLI are unavailable.
- Inspected the requested code paths, existing GUI Playwright smoke test, Isolab harness, four remote reference branches, and PR #186. PR #186 is an open GUI setup/config selection change; its diff affects `SystemPages.tsx` setup dropdown and must be considered during M4.

### Findings (hypotheses, not yet reproduced)
- `sam/src/command.rs::safe_valves` adds the one-second `HEARTBEAT_TIME_LIMIT` to elapsed time for every abort. `SamControlMessage::Abort` records `Instant::now()` first, so commanded aborts may actuate valves with timers at or below one second immediately. `sam/src/state.rs` separately enters abort after heartbeat timeout and records its own abort instant. Timing semantics need independent tests.
- GUI `FlightControlMessage::Abort` and sequence `SequenceDomainCommand::Abort` converge on `flight2/src/device.rs::send_sams_abort`; Flight also calls it on Servo disconnect. SAM heartbeat loss takes a separate local path.
- `CreateAbortStage` rebuilds Flight's saved stage, while `SetAbortStage` copies it into active state and sends per-SAM `AbortStageValveStates`. Recreating an active stage does not visibly refresh active state or SAMs. This could leave stale settings but is unproven.
- `flight2/src/sequence.rs` prefixes Python with `common` imports and globals generated from mappings, then executes with `python3 -c`. `servo/src/server/routes/sequence.rs` saves scripts and dispatches stored scripts without static validation; its `force` field is documented for config mismatch, but the route currently has a TODO for that check.
- `dev/AVI-440-unit-test-framework` offers mock GPIO and HIL test ideas, but is based on older SAM interfaces. `dev/AVI-375-simulator` has a simulated SAM packet consumer. `dev/AVI-291/abort-stage-cleanup` introduced clear-status handling and simplified abort dispatch in older code. `dev/AVI/mithilp/sitl-renode-poc` contains an earlier Isolab harness; current `sitl/isolab` already provides simulated SAMs and Servo/Flight network integration. Reuse ideas selectively.

### Tests and limits
- `cargo test -p sam --no-default-features`: **FAIL** on Windows while compiling Linux-specific `rppal` (233 errors involving missing `libc` Linux APIs); no SAM tests ran. This is a host compatibility issue, not a reproduced abort failure.
- Nix checks skipped: `nix` is unavailable. No hardware or live network used.

### Next milestone: M1
- Add mock-hardware tests against production timer logic for 0/200/500/1000/2000/5000 ms, commanded and heartbeat-loss origins, repeated abort/clear, and valve state/timing across SAMs.
- Add Flight/loopback tests for updating an active stage and abort during an executing sequence. Use Isolab only when it adds coverage beyond unit/loopback tests. Record failing tests and their causes before any production behavior change.
- **Decision awaiting approval after M1:** whether and how to change abort timing and stale-stage fail-safe behavior. Stop before M1 until the user says `continue`.

### Files changed
- `LOCAL_PROGRESS.md` only.

## M1 — reproduction, batch 1 (2026-11-09)

### Completed work and findings
- `docker info --format '{{.OSType}}'` could not connect to Docker Desktop's Linux engine (missing `dockerDesktopLinuxEngine` pipe). No container or live network was started. NixOS/Isolab VM checks are unavailable, not failed.
- Extracted SAM's existing timer decision and configured-valve advancement into `sam/src/abort_timing.rs`; `sam/src/command.rs` calls these same functions. No timer or fail-safe semantics changed.
- Extracted Flight's existing saved-stage replacement into `flight2/src/abort_stage_store.rs`; `flight2/src/device.rs` calls it. No active-stage propagation behavior changed.
- Hardware-free tests use injected elapsed time and capture valve outputs. The two-SAM test covers staged output, clear flags, and repeat. The stage store test covers changes without duplicate names.
- Reproduction: commanded abort with a 200 ms timer fires at 1 ms because the one-second heartbeat allowance is applied. Reproduction: replacing the saved active stage at 5000 ms leaves Flight's cloned active stage and previously dispatched SAM setting at 2000 ms.

### Exact tests
- `rustc --edition 2021 --test sam/src/abort_timing.rs -o target/abort_timing_tests.exe` then `target/abort_timing_tests.exe`: **2 passed, 1 ignored**.
- `target/abort_timing_tests.exe --ignored`: **1 failed as expected**, commanded 200 ms timer at 1 ms.
- `rustc --edition 2021 --test flight2/src/abort_stage_store.rs -o target/abort_stage_store_tests.exe` then `target/abort_stage_store_tests.exe`: **1 passed, 1 ignored**.
- `target/abort_stage_store_tests.exe --ignored`: **1 failed as expected**, active 2000 ms versus saved 5000 ms.
- `git diff --check`: passed.

### Remaining M1 work / exact next commands
- Expand timing assertions for every specified delay and origin, including exact boundaries; run both portable test binaries with and without `--ignored`.
- Inspect/test sequence abort interruption where feasible without Linux; run native GUI build and Playwright smoke test if dependencies install.
- Run `git diff --check`, update this file with final M1 results and limits, and commit task files only. Stop for approval before M2.

## M1 — final checkpoint

### Completed work
- Added an observed commanded-abort timing matrix for 0, 200, 500, 1000, 2000, and 5000 ms. At 1 ms elapsed, current production logic fires the first four timer values. The heartbeat-loss test confirms the existing one-second allowance. Both failing desired-behavior tests remain `#[ignore]` so normal test runs pass, and were run explicitly to demonstrate failure.
- Added `flight2/src/abort_sequence.rs` and wired the existing stop decision into `send_sams_abort`. A portable test confirms ordinary sequences are selected for termination and the `AbortStage` monitor is retained. Actual process termination and UDP delivery require Linux integration and remain unverified.
- Installed GUI dependencies with a workspace-local npm cache after the default npm cache failed with `EPERM`. Built GUI natively. Installed Playwright Chromium, but browser launch on this host failed with `spawn UNKNOWN`.

### Exact final results
- `target/abort_timing_tests.exe`: **3 passed, 1 ignored**. `--ignored`: **1 expected failure** (200 ms commanded timer fires at 1 ms).
- `target/abort_stage_store_tests.exe`: **1 passed, 1 ignored**. `--ignored`: **1 expected failure** (active 2000 ms, saved 5000 ms).
- `target/abort_sequence_tests.exe`: **1 passed**.
- `npm run build` in `gui`: **pass**.
- `npx tsc --noEmit` in `gui`: **fail**, five errors in Tauri/Vite declarations and unrelated AVI source files; none in files changed for M1.
- `npx playwright test --reporter=line`: **unavailable** after browser install because Chromium launch returns `spawn UNKNOWN`. The one smoke test did not run.
- `git diff --check`: **pass**. Docker Linux engine and NixOS/Isolab VM tests remain unavailable.

### Files changed
- `sam/src/abort_timing.rs`, `sam/src/command.rs`, `sam/src/main.rs`
- `flight2/src/abort_stage_store.rs`, `flight2/src/abort_sequence.rs`, `flight2/src/device.rs`, `flight2/src/main.rs`
- `LOCAL_PROGRESS.md`

### Unresolved issues and proposed M2 decision
- Proposed timing change: distinguish commanded abort from heartbeat-loss abort, preserve the 1000 ms heartbeat detection limit, and apply elapsed time from the correct origin so commanded delays are honored. Exact zero/boundary comparison needs review before implementation.
- Proposed active-stage change: when replacing the active stage, refresh Flight's active copy and redispatch to all affected SAMs, or explicitly reject an update until the operator reselects it. Review the fail-safe choice before implementation.
- Test coverage uses production decision functions and mock outputs, but Linux SAM/Flight compilation, real process killing, packet delivery, and hardware verification remain outstanding.
- **Await user approval before M2 safety behavior changes.** Do not run M2 until `continue`.
