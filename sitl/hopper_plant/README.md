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

**The log** has one row per 10 ms tick: phase, lateness, controller and plant compute time, sim time, the plant state, the controller command, the demand before and after caps (with a flag per cap that acted), the EC's connection, state, safing reason and applied outputs, and the longest gap so far between demand sends to the EC. The link sends the newest demand every 5 ms while declaring a 10 ms period, so a send that's late by up to a period still beats the EC's 30 ms demand timeout.

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

## SIL suite: flight logic, plant and EC firmware logic together

`bench/sil_suite.sh` flies the hop once per scenario and scores it. What is in the loop:

| In the loop, real | In the loop, simulated | Not in the loop |
|---|---|---|
| the flight logic (`gnc.rs`: state machine, faults, touchdown cutoff), the frame conversion (`nav_frame.rs`), the controller (`CtvLqrController`, checked against Simulink's own output), the allocation, the FC-EC UDP link, the EC firmware logic (`ec_side.c`, `esc.c`, `pa12_drv.c`, `measurement.c`, built for the host as `sil_ec`) | the generated hopper plant (truth state, no sensor noise), the EC's UART bus and PA-12 actuators, its ESC timer, the navigation estimate (`SilNav`: truth in a different frame, with no noise or delay) | the main loop (servo, SAMs, sequences), the sensors and the GMEKF, the STM32 itself |

```
HOPPER_BUILD=<hopper>/sims/hopper_6dof_sizer/fsw_bridge/build bench/sil_suite.sh [name-filter]
```

It needs the folder that `build_plant_c.m`, `verify_plant_c.m` and `export_plant_gains.m` wrote (`plant` or `verify/plant`, and `gains/ctv`), Linux or WSL, a C compiler, `make` and `cargo`. The bench is `ctv-flight-computer --sim-plant <gains> --sim-auto`; run it by hand for one scenario (`--sim-fault`, `--sim-expect`, `--sim-heading-offset-deg`, `--sim-origin-offset`, `--sim-imu-lever`, `--sim-rcs`; `sil_ec` takes `--drop-after-ms`, `--pa12-silent`, `--pa12-stuck`, see its `--help`).

Things to know:

- The flight logic runs on the plant's clock, so a run is the same every time (four runs agree over every tick). The EC link and its demand timeout stay on real time, so a stall of the host of more than 30 ms can safe the EC mid-flight; that has happened once in about twenty runs under WSL.
- `bench/sil.json` has no throttle cap. The default bench caps (70% throttle, for powered EC tests) are below the vehicle's weight and it never lifts off.
- The plant gets no roll torque by default, because the FC does not drive the RCS fans (`DRIVEN_CHANNELS` in `ec_link.rs`): that is what the integrated vehicle would get. `--sim-rcs` gives it the torque the controller asks for. The nominal hop is the same either way, because with no disturbance the controller asks for none.
- A scenario marked KNOWN-FAIL is a documented gap. The suite fails if it starts to pass, so fixing the gap shows up as an edit here.
- The plant's actuators are its own models. The EC's simulated actuator positions are logged but not fed to the plant, so a stuck or slow PA-12 is not reflected in the flight.

