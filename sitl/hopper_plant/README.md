# hopper_plant: the hopper sim's vehicle plant for flight software

The plant from the hopper repo's variant harness (`sims/hopper_6dof_sizer/models/hopper_6dof.slx` with `CONTROLLER=fsw`), as generated C, wrapped for Rust. flight2's `sim-plant` feature uses it for the CTV FC-in-the-loop bench test: the FC flies the simulated vehicle, and the same commands drive the real engine controller (EC).

Not a workspace member: it can't build without the generated code, which isn't committed anywhere. Without `HOPPER_PLANT_CODE` it builds as a stub whose `Plant::new` returns `Error::NotBuilt`, so the workspace's `cargo clippy --all-features` still works.

## 1. Generate the plant (MATLAB R2026a, hopper repo)

```matlab
openProject('sims/hopper_6dof_sizer');
build_plant_c();                         % CTV, CONTROLLER=fsw, ARM64 (the FC)
build_plant_c('Target', 'x86_64');       % desktop Linux / WSL
export_plant_gains();                    % CTV LQR tables -> fsw_bridge/build/gains/ctv
verify_plant_c();                        % generated C vs Simulink, bit for bit (WSL gcc)
```

`build_plant_c` writes `fsw_bridge/build/plant/`: `src/` (every .c/.h, no MATLAB needed), `sources.txt`, `defines.txt` and `plant.json` (vehicle, commit, step, target). Copy that folder and the gains folder to the machine that builds flight2.

## 2. Build flight2 with the plant

```sh
export HOPPER_PLANT_CODE=/path/to/build/plant
cargo build --release -p ctv-flight-computer --features sim-plant
```

For the Raspberry Pi from x86_64 Linux: `rustup target add aarch64-unknown-linux-gnu`, install `gcc-aarch64-linux-gnu`, then add `--target aarch64-unknown-linux-gnu` with `CC_aarch64_unknown_linux_gnu=aarch64-linux-gnu-gcc` and `CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc`. Generate the plant with the default `Target` (`arm64`) for that build.

## 3. Run the bench test

For the bench, build the package instead (section 4) and follow `bench/BENCH_CHECKLIST.md`. By hand:

```sh
FC_EC_ADDRESS=<EC ip:port> ./ctv-flight-computer --sim-plant <gains dir> [--sim-config bench.json] [--sim-log run.csv]
```

It runs nothing else (no servo, sensors or sequences). Type commands on stdin:

| Command | What happens |
|---|---|
| (at launch) | IDLE: EC session up, EC disarmed, safe demand every 10 ms (throttle 0, gimbal centered, fans off) |
| `start` | Arms the EC; once it reports Armed, the flight starts at t = 0 (RUN) |
| (touchdown, end of schedule, position limit) | LANDED: safe demand, plant frozen at its last state, EC still armed |
| `stop` | Terminate: the EC latches safing (SAFED) |
| `reset` | ClearSafing, plant back on the pad (IDLE) |
| `status` | Phase, sim time, EC state |
| `quit`, Ctrl-C, SIGTERM | Terminate, wait for Safing, flush the log, exit. A panic also sends Terminate. |

If the FC dies without sending anything, the EC's own demand timeout safes it.

**The bench config** (`--sim-config`, `bench/bench.json`; `flight2/src/bench_config.rs`) holds every value that may change at the bench: the caps, EDF full thrust (ESC taken as linear), RCS fan thrust and arm, and the PA-12 mounting geometry per axis. Fields left out keep their defaults (the sim's CTV numbers, placeholder PA-12 geometry); unknown fields and out-of-range values stop the program with the field's name. Everything loaded is printed at start.

**Caps** limit only what reaches the EC; the plant flies the controller's own command, so the flight stays nominal. Default: 70% on everything, thrust included (bench guardrail). The TVC cap is 70% of the actuator travel left on each side of center, an assumption about what "70%" means there. The nominal hop needs 0.68-0.79 throttle, so the throttle cap clips it: the run summary printed at landing says how often each cap acted.

**The log** has one row per 10 ms tick: phase, lateness, controller and plant compute time, sim time, the plant state, the controller command, the demand before and after caps (with a flag per cap that acted), and the EC's connection, state, safing reason and applied outputs.

## 4. The bench package

```sh
sitl/hopper_plant/bench/package_bench.sh --plant <build_plant_c folder> --gains <gains folder> --target aarch64
```

builds flight2 with `sim-plant` and writes `target/bench-aarch64/` (and a `.tar.gz`): the binary, `gains/`, `plant.json`, `bench.json`, `bench.env` (EC address), `run.sh`, `BENCH_CHECKLIST.md` and `MANIFEST.txt` (commits, target, sha256 of every file). `run.sh` starts the test with those files; if the FC's OS needs a special launch (for example through the Nix store loader on NixOS), put a `launch.sh` next to it and `run.sh` runs that instead, with the binary's path first.

## Desktop check with the fake EC

```sh
cd firmware/fc_ec_link && make fake_ec && ./fake_ec          # terminal 1
FC_EC_ADDRESS=127.0.0.1:8378 ./ctv-flight-computer --sim-plant <gains dir>   # terminal 2
```

## Known limits

- The bench config's defaults are the sim's CTV numbers (7.875 lbf at full throttle, 16 N per roll fan on a 4.25 in arm), both ASSUMPTIONs there, and placeholder PA-12 geometry. Which actuator is A1 and the fan pairing still need the wiring.
- The controller flies on the plant's true state; there's no estimator in the loop yet.
- The plant has no battery model; the BMS measures power on its own.
