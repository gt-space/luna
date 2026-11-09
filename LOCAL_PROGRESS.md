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

## M2 — commanded timing and explicit Apply (2026-11-09)

### Changes
- SAM now records whether an abort came from heartbeat loss. Commanded aborts use zero heartbeat allowance and fire at the configured delay boundary, measured from receipt of `SamControlMessage::Abort`. Heartbeat-loss detection remains 1000 ms and its existing strict `elapsed + 1000 ms > timer` rule is unchanged.
- Editing a saved stage continues to leave Flight's active copy and SAM configuration unchanged. The GUI persists a pending marker per server/stage across navigation and refresh, shows a red border and warning, and offers an Apply button using Servo's existing `SetAbortStage` route. A failed request retains the warning. Servo now rejects Apply when Flight is disconnected.
- The GUI clears the pending marker only after Servo accepts the dispatch request and displays that SAM delivery is unacknowledged. There are no SAM acknowledgments; this is not confirmation of valve configuration delivery.

### Exact results
- `rustc --edition 2021 --test sam/src/abort_timing.rs -o target/abort_timing_tests.exe` and `target/abort_timing_tests.exe`: **4 passed, 0 ignored**. Includes 0/200/500/1000/2000/5000 ms, before/at/after boundaries, heartbeat behavior, and two-SAM clear/repeat progression.
- `rustc --edition 2021 --test flight2/src/abort_stage_store.rs -o target/abort_stage_store_tests.exe` and `target/abort_stage_store_tests.exe`: **2 passed, 0 ignored**.
- `target/abort_sequence_tests.exe`: **1 passed** at M1; no M2 change to that code.
- `npm run build` in `gui`: **pass**.
- `npx tsc --noEmit` in `gui`: **fail** with the same five pre-existing Tauri/Vite declaration and AVI source errors observed at M1; none are in M2 files.
- `cargo test -p sam --no-default-features`: previously blocked on Windows by Linux-only `rppal`; Docker Linux engine and NixOS/Isolab VM tests remain unavailable. Playwright remains unavailable because Chromium launch returned `spawn UNKNOWN` at M1.

### Limits and next commands
- Local pending state cannot detect edits made by other clients or independently confirm Flight/SAM state. The GUI conservatively marks every edited saved stage pending, including inactive stages. Apply success means Servo accepted a dispatch request; physical delivery is unconfirmed.
- On a Linux test environment, run `cargo test --workspace --all-features` and relevant Isolab checks; verify Flight/SAM UDP delivery and abort timing with simulated boards before hardware verification.
- Next milestone M3: Python preflight validation and dispatch enforcement. Start with `git status --short --branch` and this file. Do not start until user says `continue`.

## M2.1a — protocol and SAM prepare/activate (2026-11-09)

### Completed work
- Added append-only `PrepareAbortStage { revision, valve_states }` and `ActivateAbortStage { revision }` control variants and `AbortStageAck { revision, phase }` telemetry. Existing wire variant indices remain unchanged.
- SAM stores prepared valves separately and acknowledges preparation in repeated telemetry. Matching activation moves those valves into the existing active abort storage and acknowledges activation. Duplicate/stale revisions do not replace active valves; abort cancels preparation and reports rejection. Legacy stage configuration cancels any pending preparation.
- Extracted the existing empty-stage channel range into a production-used helper and tested that it depowers channels 1–6. Existing commanded and heartbeat timing tests still pass.
- Flight currently ignores the new telemetry ACK variant; coordination is explicitly deferred to M2.1b. No GUI/Servo status changes yet.

### Exact tests
- Checkpoint diff: **9 files, 185 lines added, 5 deleted** (`git diff --stat HEAD^ HEAD`).
- `rustc --edition 2021 --test sam/src/stage_transition.rs -o target/stage_transition_tests.exe` then `target/stage_transition_tests.exe`: **4 passed**.
- `rustc --edition 2021 --test sam/src/abort_timing.rs -o target/abort_timing_tests.exe` then `target/abort_timing_tests.exe`: **5 passed**.
- `git diff --check`: **pass**.
- Full SAM/Flight Cargo tests remain unavailable on Windows because `rppal` requires Linux; Docker Linux engine and Nix are unavailable. No hardware or network tests were run.

### Remaining limits and next commands
- Flight must generate strictly increasing, boot-safe revisions, coordinate every registered SAM, reject stale ACKs, handle timeouts and partial activation, and propagate status. Older SAMs will not understand new variants and must time out without confirmation.
- Two-phase UDP activation is not atomic. Do not equate prepared or partially activated status with a safe confirmed transition. Preserve emergency abort during all phases.
- M2.1b next: read this file and `git status`; implement Flight coordinator and revision-correlated Python sequence completion, then test with simulated messages. STOP after its local checkpoint until user says `continue`.

## M2.1b — Flight coordination and sequence confirmation (2026-11-09)

### Completed work
- Flight now derives required SAM IDs from configured mappings, previous and requested stage assignments, and any superseded transition. Every required SAM, including an omitted board, receives PREPARE with its full intended configuration or an empty vector. Unregistered/offline boards remain required and cannot be counted as confirmed.
- Flight generates a boot-time revision epoch with a random counter, sends idempotent retries at 250 ms, waits up to 5 s, validates ACK revision/board/phase, and activates only after all PREPARE ACKs. It reports confirmed only after every ACTIVATE ACK. Rejection, timeout, or emergency abort ends the transition; partial activation is reported as `partially_applied` with no rollback. A SAM reconnect after confirmation invalidates confirmation.
- The latest transition and per-SAM phases are published in Flight's `VehicleState`, already transported to Servo and sequence shared memory. Flight's active stage changes only after confirmation. Unknown stages report failed status. Legacy reconnect sync now sends an empty config to SAMs omitted from the active stage and is suppressed during a transition or known partial activation.
- Python `set_abort_stage()` sends a unique request ID and waits up to 7 s for a matching confirmed status, returning an explicit error on failure, supersession, or timeout. The old IPC command remains for compatibility, but the Python API uses the new variant.

### Exact verification
- Checkpoint diff: **11 files, 491 lines added, 28 deleted** (`git diff --stat HEAD^ HEAD`; includes this line).
- `rustc --edition 2021 --test flight2/src/abort_transition.rs -o target/abort_transition_tests.exe` then `target/abort_transition_tests.exe`: **6 passed**, covering multi-SAM success, offline/removed boards, rejection, duplicate/stale/out-of-order ACKs, rapid requests, timeout, emergency abort, and partial activation.
- `rustc --edition 2021 --test common/src/sequence/transition_wait.rs -o target/transition_wait_tests.exe` then `target/transition_wait_tests.exe`: **1 passed**, including wrong-request, failure, partial, and superseded outcomes.
- `rustc --edition 2021 --test sam/src/stage_transition.rs -o target/stage_transition_tests.exe` then `target/stage_transition_tests.exe`: **4 passed** (M2.1a regression).
- `cargo metadata --no-deps --format-version 1`: **pass**. `rustfmt --emit stdout` parsed changed Rust files without modifying them. `git diff --check`: **pass**.
- `cargo check -p flight-computer --no-default-features`: **blocked on Windows** before Flight compilation by Linux-only `spidev` (27 compile errors). Nix, Docker Linux, and Playwright are unavailable as previously recorded. No physical hardware used.

### Remaining limits and next commands
- Full cross-crate type checking and live IPC/UDP tests require Linux. Boot-time revisions use wall-clock nanoseconds plus random counter; a clock rollback across Flight reboots can cause SAMs to reject newer requests safely until resolved. Older SAM firmware cannot ACK and times out; it is never confirmed.
- Servo currently acknowledges request receipt before Flight confirmation, and GUI still uses local pending markers. M2.1c must consume the authoritative `abort_transition` status, expose critical partial state and per-SAM phases, and avoid treating Servo's HTTP 200 as application success.
- A partial activation leaves Flight's previous stage active while SAMs may differ; no automatic rollback or valve action occurs. Hardware safety response remains a policy decision requiring review before hardware use.
- An emergency abort still kills ordinary sequence processes as before, so a sequence killed mid-wait cannot receive a Python exception; Flight marks the transition failed/partial before dispatching abort. Scripts that catch and suppress the explicit Python exception can still proceed by their own choice.
- M2.1c next commands: read `LOCAL_PROGRESS.md` and `git status`; implement Servo/GUI status flow and tests, then run relevant checks and diff review. STOP after its local checkpoint until user says `continue`. Do not start M3.

## M2.1c — Servo/GUI confirmation display (2026-11-09)

### Changes
- GUI Apply sends a 128-bit request ID through Servo's existing route. HTTP 200 means only that Servo dispatched the request. Flight echoes the ID as an exact hex string in authoritative `abort_transition` telemetry, while the Python sequence wait still compares its specific request.
- GUI displays UNSENT, PREPARING, ACTIVATING, CONFIRMED, FAILED, and PARTIALLY APPLIED plus per-SAM phases from Flight. A red border remains until a live Flight report confirms, recent umbilical telemetry is available, and no GUI request is outstanding. Cached Flight status and an outstanding request survive navigation/reload, but cached confirmation is labeled historical and does not clear the border. HTTP failure and a 7-second wait timeout remain visible. The button now says **Activate and Send to SAMs**.
- Editing an active saved stage causes Flight to report UNSENT without modifying its active stage or SAMs. The old client-local pending-stage markers are no longer used. No changes were made to abort timers, heartbeat behavior, or valve actions.

### Verification
- `node --test gui/tests/abortStatus.test.cjs`: **2 passed** (all displayed phases, live confirmation gate, request pending, failure, timeout, partial).
- Portable production tests: `flight2/src/abort_transition.rs` **6 passed**; `common/src/sequence/transition_wait.rs` **1 passed**; `sam/src/stage_transition.rs` **4 passed**.
- `npm run build` in `gui`: **pass**. `npx tsc --noEmit`: **five pre-existing errors** in Tauri/Vite declarations and unrelated AVI sources; none in M2.1c files. `git diff --check`: **pass**.
- `cargo check -p common --no-default-features`: **platform blocked** in Linux-only `rppal` on Windows before checking changed code. Docker Linux, Nix/Isolab, and Playwright remain unavailable. No hardware testing.
- Reviewed the actual M2.1c diff and full M2.1 diff. M2.1c changes are limited to the existing protocol, Flight, Servo, and abort-stage GUI paths plus one focused GUI status helper/test. No unrelated formatting or PR #186 configuration-selection edits.

### Safety findings requiring approval
- Flight revision epoch is based on wall-clock nanoseconds (`flight2/src/device.rs`). After clock rollback and reboot, SAM's monotonic revision check can reject future requests until the clock catches up. Smallest durable fix needs a persistent monotonic epoch or an explicitly accepted boot/session protocol; this changes revision safety semantics.
- Legacy reconnect sync (`flight2/src/main.rs` heartbeat 20) has no V2 ACK and can send a legacy stage command when no transition is in progress. SAM's legacy command cancels prepared/ACK state. Identity reconnect after a confirmed transition marks it partially applied and suppresses sync, but other legacy sync paths are not confirmation safe. Proposed policy: suppress legacy sync after V2 use and require a new acknowledged transition on reconnect; approve before changing hardware behavior.
- A new stage request can replace a `partially_applied` Flight status. This can hide historical unsynchronization while a new attempt is preparing, although HTTP 200 itself no longer clears the GUI warning. Proposed policy: latch critical unsynchronized state until a fully confirmed all-SAM transition (or explicit operator resolution); approve before changing coordinator behavior.
- Two-phase UDP activation remains non-atomic. A partial transition has no automatic rollback or valve action. Older SAM firmware times out and is never confirmed.

### Next commands
- `git status --short --branch`; review the safety findings above with the operator before changing revision, reconnect, or partial-state policy. On a Linux test environment, run `cargo test --workspace --all-features` and simulated Flight/Servo/SAM integration tests. Do not start M3 until M2.1 is reviewed and approved.

## M2.1d checkpoint 1 — durable revision epochs (2026-11-09)

- Flight reserves a strictly increasing boot epoch in `LUNA_ABORT_EPOCH_DIR` (default `/var/lib/luna/abort-revisions`) before stage transitions. Creation and sync errors leave transitions failed; emergency abort remains available. This directory must persist across reboot and be writable by Flight. No clock ordering remains in Flight revisions.
- `rustc --edition 2021 --test flight2/src/revision_epoch.rs -o target/revision_epoch_tests.exe; .\target\revision_epoch_tests.exe`: **2 passed**, including repeated boots and corrupt storage failure. `git diff --check`: **pass**. Full Flight compilation remains blocked by Linux-only dependencies on Windows.
- Next: implement V2 reconnect protection, test it, update this file, and commit locally. Then implement the partial activation latch. No hardware use.
