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
- Scenarios named `GMEKF: ...` fly on the navigation filter instead of on truth: the real GMEKF (`--sim-gmekf <nav.json>`, the config from `bench/nav_config.py`) is fed the plant's own sensors, stepped with the plant, on the plant's clock. There are no sockets, threads or wall clock between the sensors and the estimate, so what is left is the filter and the loop. The plant has no ground: it stops at touchdown, and the run then goes on for up to a second with the sensors reading rest, so the flight logic can see the vehicle down and cut the engine itself (it cuts when the vehicle is below 3 cm and slower than 5 cm/s for 0.2 s, or below the 5 mm cutoff).
- On the GMEKF the hop climbs, holds and lands about 0.1 m from the pad (0.00 to 0.12 m over seven noise seeds; 0.38 m/s impact in the earlier 2 cm-GPS plant, 2 mm/s now), with the roll torque given to the plant (without it the roll rate grows from the noise and the vehicle is lost; the FC drives no roll fans yet). The plant's GPS is non-RTK (1.5 m, white at 5 Hz), and the landing miss is the filter's position error at touchdown, so it depends on how the filter places the GPS frame: the GPS origin is the mean over the first `gps_origin_seconds` (10 s) of fixes against the filter's own antenna position, with GPS velocity alone used until then (`navigate::gps_origin`, in `shadow_nav` and in this bench). From the first fix alone, as it was, the origin carried that fix's 1.5 m noise for the whole flight and every later fix pulled the estimate toward it: the miss was 0.19 to 0.91 m over the same seeds (0.44 m on the plant's own seed), growing steadily. Flying without GPS position at all landed 5 cm off on the worst of them, so what GPS position adds here is mostly the origin's error; a real receiver's error is a slow wander, which a longer window or a filter state for the offset would deal with better. The plant's noise seed is built into its C (`Sensor_Noise_Seed`, `sensor_params.seed` + 0:25); other seeds can be tried on a copy of the plant folder with that array edited.
- The filter port follows hopper's ef5c7fb (CTV GMEKF, PR #50): the gyro is a predict input (the filter takes no torque and does no gyro correction), the covariance propagates with the discrete transition and noise Jacobians, and, as of ef5c7fb, the acceleration and its noise Jacobian use the start-of-step attitude (the two lines of `math.rs` that changed since luna's first port, from ed077f9). Hopper's process noise is the plant's per-axis IMU sigmas squared plus its bias and baro-bias PSDs (all `Qc_scale` 1), which `bench/nav_config.py` now passes through. Its retuned P0 is for a filter started without leveling; luna starts from the leveled pad estimate with tighter priors, so it is not applied. The reference data of the filter tests (`navigate/gmekf/testdata`) was regenerated from ef5c7fb with MATLAB (`matlab -batch "make_reference('<hopper>/sims/hopper_6dof_sizer/subsystems/navigation/Src_filter', '<out>')"`); only the Jacobian, predict and closed-loop cases changed. The landing on the seeds above is the same with the change (0.00 to 0.12 m).
- `--sim-command-delay-periods N` makes the plant fly each command N control periods (10 ms) late, and `--sim-tvc-rate-dps` limits the gimbal's speed. The hopper plant already has its own 10 ms of TVC latency and a 200 deg/s gimbal, so these are on top. The delay applies from the first command of the flight (the plant flies the safe command until the first one has come down the path). With hopper's ef5c7fb plant (its IMU and GPS noise) the hop lands within 0.21 m of the pad at 10 and 30 ms with the touchdown tilt inside its 2 degrees; at 50 ms it lands 0.43 m off and tips over on its legs after the touchdown, so that scenario is a known gap. (A run of the suite had it passing at 50 ms for a while: the delay did not apply to the first commands then.) With the earlier plant the peak tilt grew with the delay (0.7 degrees, 2.4 at +10 ms, 4.0 at +30 ms, 6.0 at +50 ms), as it does in hopper's own sweep, and the touchdown tilt missed its 2 degrees from about +40 ms; those tilts were not measured again. The gimbal at 45 deg/s, about what the PA-12's 36 mm/s gives on this geometry, flies the hop, but with 10 ms of delay on top the vehicle is lost.
- `--sim-missed-period-fraction F` makes a fraction of the control periods missed (the plant flies the previous command through them). With 15% missed, a gimbal at 125 deg/s and 40 ms of delay the hop still lands (peak tilt 11 degrees, touchdown tilt 2.0), so the delay, the gimbal speed and the host's jitter that the plant-time SIL can model do not by themselves explain the real-time SIL's loss of the vehicle.
- Hopper builds a plant with a ground too (`build_plant_c('Ground', 'strut')`, `build/plant_strut`): the vehicle stands on its legs, the touchdown stop is off, and the IMU sees the impact. Point `HOPPER_PLANT_CODE` at it. The plant-time hop lands on it as on the other plant, and the real-time bridge ends a flight when the vehicle has been up and is down and still (`ON_THE_GROUND_*` in `sil_bridge.rs`) instead of at a touchdown stop. Two things on a plant with a ground: the plant's own engine cut below 5 mm does not latch, and thrust below its 7 N floor is raised to 7 N, so the flight logic's cut shows as the thrust dropping when the plant decides. With the legs, a vehicle on the pad that is thrust up and aborted tips and creeps (the "abort at 0.5 s" ops scenario stays a known gap).
- On the ground the LiDAR reads about 10 cm, below the 20 cm the filter's noise calibration starts at: `lidar_min_range_m` (0.05 here, the sensor's) lets the filter use those beams, and without it the last 20 cm of a landing and the vehicle at rest are measured by the barometer and GPS alone, a few centimeters off (`bench/nav_config.py` sets it).
- The filter's body rate is the gyro minus bias, as the filter reports it. Smoothing it (a Kalman filter on the rate with hopper's torque process noise, 36 ms time constant) was tried: the lag makes the pitch oscillate, and the raw rate flies clean.
- The filter's origin is the IMU. The GPS antenna lever and the LiDAR mounts are given relative to the IMU, and the LiDAR height is the IMU's (`height_imu`); the controller's state is the filter's transformed to the controller's reference point, the CG, by the lever arm in the flight logic (`--gnc-imu-lever`, `--sim-imu-lever`). `bench/nav_config.py` writes both from the plant: hopper gives the antenna and the mounts from the base and `cg_height` (the CG above the base), and its sensor models put the IMU at the CG, so here the lever is zero (`CTV_IMU_FROM_CG` says otherwise for a what-if, but the plant's own IMU stays at the CG). With the geometry from the base the filter saw a velocity error of rate times 0.35 m, rejected the GPS as outliers and drifted meters.
- The plant's actuators are its own models. The EC's simulated actuator positions are logged but not fed to the plant, so a stuck or slow PA-12 is not reflected in the flight.

## Operations SIL: the whole flight computer, servo and the operator

`bench/sil_ops.sh` runs the real flight computer main loop (not the stand-in loop of `sil_suite.sh`) with everything around it as separate processes on one machine:

```
ctv_ops (operator)  --HTTP/websocket-->  servo  --TCP-->  flight computer (main loop, sequences, GMEKF, flight logic)
                                                           |  UDP demands            ^ simulated sensors (UDP, --sil-feed)
                                                           v                         |
                                                       sil_ec --applied outputs--> bridge (--sil-bridge: the generated plant)
```

```
HOPPER_BUILD=<hopper>/sims/hopper_6dof_sizer/fsw_bridge/build bench/sil_ops.sh [name-filter]
```

- The plant must have sensor outports (hopper's build_plant_c from the sensor-outport commit on). The sensors are hopper's own Sensors block, built into the plant; the navigation filter is configured from that plant's `sensor_params` (its noise, GPS antenna and GPS velocity), because its own defaults are for the vehicle's receiver and IMU.
- The plant has no ground, so the bridge keeps the vehicle on the pad until the EC applies throttle, and until then sends pad-rest sensors from the same sensor settings (including the GPS antenna's offset, without which the filter's origin is half a meter off). The navigation filter levels during that time, so the first minute of a run is the filter starting up.
- The operator is `sitl/isolab/src/bin/ctv_ops.rs`. It waits for the pre-flight checks, arms the EC, starts the flight and follows the status servo forwards, either with the GUI's `POST /operator/ctv-command` or with a Python sequence (`--via sequence`).
- It runs in real time. Runs are not deterministic: a stall of the host, and the noise of the sensors, move a flight around.
- Build it in release (the script does): the GMEKF steps 500 times a second, and in a debug build it cannot keep up, so the estimate falls a tenth of a second behind the truth and the vehicle is lost within three seconds.
- The nominal hop still does not land in this configuration, and the scenarios that show that are marked KNOWN-FAIL. With the filter keeping up, the vehicle flies the hop's climb and is lost around 8 s: the pitch oscillates at 0.8 rad/s and the thrust hunts, as the plant-time flights do with 50 ms or more of delay and gimbal speed limits. The real chain is the FC's 10 ms tick, UDP, the EC's frames and the bridge's steps, with the jitter of a laptop (a few control periods are missed per second under WSL). The flight computer prints how long the EC takes to act (`GNC: EC applies the A1 demand ... after it is sent`): about 20 ms from the demand going out to the applied value coming back in the EC's status, with the throttle immediate. That is the EC's side only; the PA-12's own speed and the link to the plant come on top. Do not read the control performance of this run as the vehicle's: it is a check that the pieces work together, and it needs the real hardware's timing.
- Roll torque: the flight computer does not drive the RCS fans (`ec_link::DRIVEN_CHANNELS`), so by default the plant here has no roll torque, as the vehicle has none. That leaves little margin: on the strut plant, in-process with the real filter and no roll torque, the hop lands with up to 10 ms of added command delay and with 10% of the control periods missed, but is lost with 20 to 30 ms of delay, or with 10% missed periods plus 10 ms (the pitch rings, the gimbal sits on its stops, and the uncontrolled roll runs away). With roll torque (`sil_suite.sh` always passes `--sim-rcs`) the same 30 ms delay with 10% missed periods lands. To fly the multi-process SIL with the roll axis held, give the plant the duties the flight logic asks for, straight from the flight computer (it is optimistic: no fan lag, and it skips the EC): `BRIDGE_EXTRA="--sil-roll-port 5602" FC_EXTRA="--sil-roll-to 127.0.0.1:5602" bench/sil_ops.sh`.
- Flying by hand from the GUI: the bridge exits when the flight is over, after which no sensors reach the flight computer and its estimate goes unhealthy (`no new IMU step (stale)`) within a fraction of a second. `--sil-bridge --sil-stay` keeps it sending the sensors of the vehicle at rest instead, so the page stays live; it still prints its `BRIDGE RESULT` once. A run is still one flight: the plant is not reset, so restart the stack to fly again. Landing used to leave the estimate drifting, about a metre in 30 s on the strut plant: the touchdown is a shock the IMU cannot follow (it clips at its 157 m/s^2 full scale and samples every 2 ms, and the legs ring at 250 Hz after contact), so the velocity the filter integrates is off by a few tenths of a metre per second, and with its position uncertain by only 1 to 3 cm the GPS gate (NIS 20 to 3000 against 16.8) then rejects every fix. The flight computer now levels the navigation filter again 1.5 s after a flight ends (`GNC: the flight is over; the navigation filter levels again ...`), as the operator's re-initialize does, so the estimate is good again about 4 s later, at the new spot, and stays within a few centimetres. The filter itself is unchanged (it follows hopper's MATLAB, gate included).

## Starting the stack for the GUI

`bench/ctv_stack.sh` (Linux/WSL) and `bench/ctv_stack.ps1` (Windows, runs the shell script in WSL) start, stop and inspect the multi-process stack the GUI connects to: servo, `sil_ec`, the plant bridge (`--sil-stay`) and the flight computer on the navigation filter.

```
HOPPER_BUILD=<hopper>/sims/hopper_6dof_sizer/fsw_bridge/build bench/ctv_stack.sh start   # or restart
bench/ctv_stack.sh status | stop | logs [servo|ec|bridge|fc] | gui
```

```
.\bench\ctv_stack.ps1 start -HopperBuild C:\...\fsw_bridge\build    # or restart, status, stop, logs fc
```

Then connect the GUI to servo at 127.0.0.1, choose the Umbilical source and open Views > CTV; the filter needs a few seconds to level before the page reads READY. `start` builds first (slow the first time), leaves everything running in the background and returns. It is one flight per start, so `restart` before the next. Options: `HOPPER_PLANT_CODE` (default `plant_strut`, then `plant`, then `verify/plant`), `GAINS_DIR`, `CTV_STACK_DIR` (logs, pid files and the built `sil_ec`, default `~/ctv_stack`; `start` refuses a folder that holds other files), `CARGO_TARGET_DIR` (default `~/ctv-sil-target`; building under `/mnt/c` is slow), `SIL_TRUTH_NAV=1` (`-TruthNav`), `SKIP_BUILD=1` (`-SkipBuild`). It runs in real time, so a loaded host can lose a flight (one of two runs ended Safed with a NavLost at 18 s while the machine was busy; the next landed).

## Replaying hopper's recorded navigation flight

`navigate::gmekf::replay` (an ignored test) feeds the sensor readings hopper recorded from its nominal GMEKF flight (`fsw_bridge/build/nav_replay/ctv`, written by hopper) through this repo's GMEKF and scores the result against the truth and against the estimate hopper's MATLAB filter gave:

```
HOPPER_NAV_REPLAY=<hopper>/sims/hopper_6dof_sizer/fsw_bridge/build/nav_replay/ctv   cargo test -p ctv-flight-computer --release gmekf::replay -- --ignored --nocapture
```

With the LiDAR beams fused out to 6 m (`lidar_max_range`; the nav team's noise calibration ends at 4 m and the CTV hops to 4.5 m) it matches hopper's filter: position within 1 cm horizontally and 2 mm vertically, velocity 1 cm/s, attitude 0.05 to 0.14 degrees. Without LiDAR the vertical position and velocity are 7 times worse. The LiDAR mount positions are given from the centre of mass (hopper gives them from the base).
