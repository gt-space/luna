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

```sh
FC_EC_ADDRESS=<EC ip:port> ./ctv-flight-computer --sim-plant <gains dir> [--sim-caps caps.json] [--sim-log run.csv]
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

**Caps** (printed at start) limit only what reaches the EC; the plant flies the controller's own command, so the flight stays nominal. Defaults: throttle uncapped, fan duty 70%, TVC 70% of the actuator travel left on each side of center (an assumption about what "70%" means for the TVC). A caps file overrides any of them:

```json
{ "throttle": 0.7, "fan_duty": 0.7, "tvc_travel": 0.7 }
```

**The log** has one row per 10 ms tick: phase, lateness, controller and plant compute time, sim time, the plant state, the controller command, the demand before and after caps (with a flag per cap that acted), and the EC's connection, state, safing reason and applied outputs.

## Desktop check with the fake EC

```sh
cd firmware/fc_ec_link && make fake_ec && ./fake_ec          # terminal 1
FC_EC_ADDRESS=127.0.0.1:8378 ./ctv-flight-computer --sim-plant <gains dir>   # terminal 2
```

## Known limits

- The allocation (`Allocation::CTV_SIM`) uses the sim's CTV numbers: 7.875 lbf at full throttle and the sim's roll authority, both ASSUMPTIONs there, and the placeholder TVC geometry. Which actuator is A1 and the fan pairing still need the wiring.
- The controller flies on the plant's true state; there's no estimator in the loop yet.
- The plant has no battery model; the BMS measures power on its own.
