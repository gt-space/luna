#!/usr/bin/env bash
# The CTV operations SIL: the real flight computer main loop (servo link,
# Python sequences, the GMEKF, the flight logic) flying the generated hopper
# plant, through the real engine-controller firmware logic, operated the way the
# GUI operates it.
#
#   HOPPER_BUILD=<hopper>/sims/hopper_6dof_sizer/fsw_bridge/build ./sil_ops.sh [name-filter]
#
# Processes, all on this machine:
#
#   ctv_ops (the operator)  --HTTP/websocket-->  servo  --TCP-->  flight computer
#                                                                   |        ^
#                                                       UDP demands |        | simulated sensors (UDP)
#                                                                   v        |
#                                             sil_ec --applied outputs--> bridge (the plant)
#
# `sil_suite.sh` is the quicker, deterministic version with the flight logic and
# the plant in one process. This one adds the pieces that suite leaves out: the
# sensors and the real GMEKF, the main loop, servo, and the operator's commands.
# It runs in real time, so a stall of the host can safe the EC mid-flight (its
# demand timeout is 30 ms).
#
# The plant has to have sensor outports (hopper's build_plant_c from the
# sensor-outport commit on). Linux or WSL, with a C compiler, make, cargo and
# python3.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LUNA="$(cd "$HERE/../../.." && pwd)"
HOPPER_BUILD="${HOPPER_BUILD:?set HOPPER_BUILD to the fsw_bridge/build folder of the hopper repo}"
PLANT="${HOPPER_PLANT_CODE:-}"
if [ -z "$PLANT" ]; then
  for candidate in "$HOPPER_BUILD/plant" "$HOPPER_BUILD/verify/plant"; do
    [ -f "$candidate/plant.json" ] && PLANT="$candidate" && break
  done
fi
[ -f "$PLANT/plant.json" ] || { echo "no plant.json under $HOPPER_BUILD (set HOPPER_PLANT_CODE)" >&2; exit 2; }
GAINS="${GAINS_DIR:-$HOPPER_BUILD/gains/ctv}"
CONFIG="${SIL_CONFIG:-$HERE/sil.json}"
FILTER="${1:-}"
WORK="$(mktemp -d)"
PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null; done
  wait 2>/dev/null
  [ -n "${KEEP_LOGS:-}" ] && { mkdir -p "$KEEP_LOGS" && cp -r "$WORK"/run "$KEEP_LOGS"/ 2>/dev/null; }
  rm -rf "$WORK"
}
trap cleanup EXIT

echo "plant:  $PLANT"
echo "gains:  $GAINS"
echo "config: $CONFIG"

# What to build: the host EC, and the flight computer (also the bridge) with the
# generated plant, servo and the operator
cp -r "$LUNA/firmware/fc_ec_link" "$WORK/ec"
(cd "$WORK/ec" && make sil_ec >/dev/null) || { echo "sil_ec did not build" >&2; exit 2; }
export HOPPER_PLANT_CODE="$PLANT"
# A release build: the GMEKF steps 500 times a second, and a debug build cannot keep
# up, so the estimate falls a tenth of a second behind and the loop is lost
PROFILE="${SIL_PROFILE:-release}"
PROFILE_FLAG="--release"
[ "$PROFILE" = debug ] && PROFILE_FLAG=""
(cd "$LUNA" && cargo build -q $PROFILE_FLAG -p ctv-flight-computer --features sim-plant && cargo build -q $PROFILE_FLAG -p servo -p isolab) || exit 2
TARGET="${CARGO_TARGET_DIR:-$LUNA/target}/$PROFILE"
FC="$TARGET/ctv-flight-computer"
SERVO="$TARGET/servo"
OPS="$TARGET/ctv_ops"

# The navigation filter, tuned to the sensors of the plant that was built: its
# measurement noise, the GPS antenna's place on the vehicle, GPS velocity (whose
# noise is white in this plant), the Earth's field it uses, the four LiDAR beams,
# and the smoothing of the body rate. The filter's own defaults are for the
# receiver and IMU on the vehicle.
python3 "$HERE/nav_config.py" "$PLANT/plant.json" > "$WORK/nav.json" || exit 2
# The filter's origin is the IMU; the flight logic transforms its state to the
# controller's reference point (the CG) by this lever
LEVER="$(python3 "$HERE/nav_config.py" "$PLANT/plant.json" --lever)" || exit 2

pass=0 fail=0 total=0

# name | operator args | EC args | how it should end: PASS or KNOWN-FAIL
scenarios=(
  "start by GUI commands, terminate at 0.5 s|--terminate-at 0.5 --expect safed --expect-fault ManualTerminate|--speed 100| PASS"
  "re-initialize navigation first, then terminate at 0.5 s|--reinit-nav --terminate-at 0.5 --expect safed --expect-fault ManualTerminate|--speed 100| PASS"
  "start by Python sequence, terminate at 0.5 s|--via sequence --terminate-at 0.5 --expect safed --expect-fault ManualTerminate|--speed 100| PASS"
  "start by GUI commands, abort at 0.5 s|--abort-at 0.5 --expect abort-land --expect-fault ManualAbort|--speed 100| KNOWN-FAIL"
  "nominal hop, PA-12 at 100 mm/s|--expect landed|--speed 100| KNOWN-FAIL"
  "nominal hop, PA-12 at its datasheet 36 mm/s|--expect landed|--speed 36| KNOWN-FAIL"
)

run_one() {
  local ops_args="$1" ec_args="$2"
  local dir="$WORK/run"
  rm -rf "$dir"; mkdir -p "$dir/home" "$dir/logs"
  PIDS=()

  ( cd "$dir"
    HOME="$dir/home" "$SERVO" serve --volatile --quiet > servo.log 2>&1 &
    echo $! > servo.pid
    "$WORK/ec/sil_ec" -p 8378 --applied-udp 127.0.0.1:5601 $ec_args ${EC_ARGS_EXTRA:-} > ec.log 2>&1 &
    echo $! > ec.pid
    "$FC" --sil-bridge --sil-feed-to 127.0.0.1:5600 --sil-applied-port 5601 \
      --sim-config "$CONFIG" --sim-log "$dir/bridge.csv" ${BRIDGE_EXTRA:-} > bridge.log 2>&1 &
    echo $! > bridge.pid
    sleep 1
    FC_EC_ADDRESS=127.0.0.1:8378 FC_SERVO_ADDRESS=127.0.0.1:5025 \
      "$FC" desktop --sil-feed 5600 --gnc-gains "$GAINS" --gnc-vehicle-config "$CONFIG" \
      --gnc-allow-dirty --gnc-imu-lever="$LEVER" --shadow-nav-config "$WORK/nav.json" --disable-file-logging \
      --log-dirs "$dir/logs" ${FC_EXTRA:-} > fc.log 2>&1 &
    echo $! > fc.pid
    "$OPS" --servo 127.0.0.1:7200 $ops_args > ops.log 2>&1
    echo $? > code
    for p in fc bridge ec servo; do kill "$(cat $p.pid)" 2>/dev/null; done
    wait 2>/dev/null )
}

for entry in "${scenarios[@]}"; do
  IFS='|' read -r name ops_args ec_args want <<< "$entry"
  want="${want# }"
  [ -n "$FILTER" ] && [[ "$name" != *"$FILTER"* ]] && continue
  total=$((total + 1))
  run_one "$ops_args" "$ec_args"
  code="$(cat "$WORK/run/code" 2>/dev/null || echo 99)"
  result="$(grep -E "OPS RESULT" "$WORK/run/ops.log" | tail -1 | sed -E "s/OPS RESULT: //")"
  bridge="$(grep -E "BRIDGE RESULT" "$WORK/run/bridge.log" | tail -1 | sed -E 's/.*apex_m=([0-9.]+).*impact_speed_mps=([0-9.na]+).*/plant: apex \1 m, impact \2 m\/s/')"
  [ -z "$result" ] && result="(no result: exit $code)"
  if [ "$code" = 0 ]; then got=PASS; else got=FAIL; fi
  if [ "$want" = PASS ] && [ "$got" = PASS ]; then verdict="ok"; pass=$((pass + 1))
  elif [ "$want" = KNOWN-FAIL ] && [ "$got" = FAIL ]; then verdict="known gap"; pass=$((pass + 1))
  elif [ "$want" = KNOWN-FAIL ] && [ "$got" = PASS ]; then verdict="NOW PASSES: update the suite"; fail=$((fail + 1))
  else verdict="UNEXPECTED $got"; fail=$((fail + 1)); fi
  printf '%-48s [%s]\n    %s\n    %s\n' "$name" "$verdict" "$result" "$bridge"
  if [ "$verdict" != ok ] && [ "$verdict" != "known gap" ]; then
    grep -E "OPS (VERDICT|ERROR)" "$WORK/run/ops.log" | head -3 | sed 's/^/    /'
    echo "    (set KEEP_LOGS=<dir> to keep the logs of every process)"
  fi
done

echo
echo "$pass of $total scenarios ended as listed, $fail did not"
[ "$fail" = 0 ]
