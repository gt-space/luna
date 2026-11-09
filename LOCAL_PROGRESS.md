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
