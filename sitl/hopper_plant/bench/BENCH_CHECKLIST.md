# CTV FC-in-the-loop bench checklist

The FC flies the precompiled CTV sim (Aniket's LQR gains) and sends the same commands to the EC, capped (default 70% on throttle, fans and TVC travel). The sim flies the uncapped command. Everything below runs from the package folder on the FC.

## 1. Copy the package over

- [ ] `scp bench-aarch64.tar.gz <user>@flight.local:~` then `tar xzf bench-aarch64.tar.gz && cd bench-aarch64`
- [ ] `head -6 MANIFEST.txt`: right luna and plant commits, binary is `ARM aarch64`
- [ ] `sha256sum -c <(tail -n +7 MANIFEST.txt)` all OK
- [ ] OS-specific launcher in place if this FC needs one (`launch.sh`, see `run.sh`)

## 2. Settings

- [ ] `bench.env`: `FC_EC_ADDRESS` is the EC's IP:port (`192.168.1.50:8378` by default)
- [ ] `ping` the EC's IP from the FC
- [ ] `bench.json`: caps, EDF/RCS thrust and PA-12 geometry as agreed for this session. Any field left out keeps its default; a bad value stops the program with the field's name.

## 3. Dry run while safed

- [ ] Motor restrained, props/fans clear, everyone knows who types `stop`
- [ ] `./run.sh`: the startup printout shows the plant, gains, every bench value, the log path, then `IDLE (EC disarmed, safe outputs)` and `EC link: connected ... EC is Disarmed`
- [ ] Read the caps line out loud. Look for `WARNING` lines (a PA-12 that can't reach the gimbal limit)
- [ ] `status` -> `IDLE ... EC connected true state Some(Disarmed)`
- [ ] `quit` -> `quitting; terminating the EC`, program exits

## 4. Full run

- [ ] `./run.sh`, wait for IDLE, then `start`: `Arm accepted`, `EC armed; RUN, t = 0`
- [ ] The run lands by itself (~16 s): `LANDED at t = ... (touchdown); safe outputs, plant frozen`, then the run summary (how often each cap acted; lateness, overruns)
- [ ] `stop` -> `EC Armed -> Safing (FcTerminate)`, `SAFED`
- [ ] Next run: `reset` (clears safing, plant back on the pad), then `start` again
- [ ] Done: `quit`

## 5. Check every way out safes the EC

Do each once, with the EC armed (after `start`):

- [ ] `stop`: EC goes to Safing (FcTerminate), outputs safe
- [ ] Ctrl-C: `quitting; terminating the EC`, EC goes to Safing
- [ ] Kill the FC side hard (`pkill -9 ctv-flight` from a second terminal, or pull the Ethernet cable): the EC safes on its own within its 30 ms demand timeout
- [ ] After each: EC outputs at rest (ESC minimum throttle, PA-12s centered, fans off)

## 6. Logs

- [ ] One CSV per run in `logs/run_<date>.csv`, a row every 10 ms: phase, lateness, controller/plant time, sim state, controller command, demand before and after caps (with a flag per cap that acted), and the EC's state, safing reason and applied outputs
- [ ] Copy `logs/` off the FC after the session, with `bench.json` and `MANIFEST.txt` next to it
- [ ] The BMS logs power and battery itself. With no prop on the motor its numbers are far below a real flight load; note that with the results.
