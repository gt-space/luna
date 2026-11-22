#!/usr/bin/env bash
# The CTV software-in-the-loop suite: the flight logic (Gnc), the generated
# hopper plant and the real engine-controller firmware logic (sil_ec), over the
# real FC-EC link, one flight per scenario, scored.
#
#   HOPPER_BUILD=<hopper>/sims/hopper_6dof_sizer/fsw_bridge/build ./sil_suite.sh [name-filter]
#
# HOPPER_BUILD is the folder build_plant_c.m / verify_plant_c.m / export_plant_gains.m
# wrote: it needs `verify/plant` (or `plant`) and `gains/ctv`. Linux or WSL, with
# a C compiler, make and cargo. Nothing here touches hardware.
#
# Each scenario says how it should end. A scenario listed as KNOWN-FAIL is a
# gap the suite documents: it must still fail, so that fixing the gap shows up
# as a change here. The exit status is 0 only if every scenario ended as listed.

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
trap 'rm -rf "$WORK"' EXIT

echo "plant:  $PLANT"
echo "gains:  $GAINS"
echo "config: $CONFIG"

# The host EC: the firmware's own logic with simulated hardware
cp -r "$LUNA/firmware/fc_ec_link" "$WORK/ec"
(cd "$WORK/ec" && make sil_ec >/dev/null) || { echo "sil_ec did not build" >&2; exit 2; }

# The flight computer with the SIL bench, linked to the generated plant
export HOPPER_PLANT_CODE="$PLANT"
(cd "$LUNA" && cargo build -q -p ctv-flight-computer --features sim-plant) || exit 2
BIN="${CARGO_TARGET_DIR:-$LUNA/target}/debug/ctv-flight-computer"

pass=0 fail=0 total=0

# name | flight computer args | EC args | how it should end: PASS or KNOWN-FAIL
scenarios=(
  "nominal hop||| PASS"
  "nav frame offsets (heading 137 deg, origin shifted, IMU lever)|--sim-heading-offset-deg 137 --sim-origin-offset 35,-12,-3 --sim-imu-lever=-0.3,0.02,-0.01|| PASS"
  "with the roll torque given to the plant|--sim-rcs|| PASS"
  "nav estimate 50 ms stale at 5 s|--sim-fault nav-stale:5:0.05|| PASS"
  "nav lost for 100 ms at 5 s|--sim-fault nav-loss:5:0.1|| PASS"
  "nav lost for good at 5 s: terminate|--sim-fault nav-loss:5 --sim-expect safed|| PASS"
  "NaN estimate for good at 5 s: terminate|--sim-fault nav-nan:5 --sim-expect safed|| PASS"
  "operator terminate at 5 s|--sim-fault terminate:5 --sim-expect safed|| PASS"
  "operator abort at 3 s (ascending): land|--sim-fault abort:3 --sim-expect abort-land|| PASS"
  "operator abort at 9 s (descending): land|--sim-fault abort:9 --sim-expect abort-land|| PASS"
  "EC stops answering 4.5 s into flight|--sim-expect safed|--drop-after-ms 10000| PASS"
  "PA-12 A1 stops answering 4.5 s into flight|--sim-expect safed|--pa12-silent a1 --inject-at-ms 10000| PASS"
  "attitude estimate biased 30 deg at 5 s|--sim-fault tilt-bias:5:30 --sim-expect abort-land|| KNOWN-FAIL"
)

run_one() {
  local name="$1" fc_args="$2" ec_args="$3"
  local dir="$WORK/run"
  rm -rf "$dir"; mkdir -p "$dir"
  ( cd "$dir"
    "$WORK/ec/sil_ec" -p 8378 $ec_args > ec.log 2>&1 &
    local ec=$!
    sleep 0.5
    FC_EC_ADDRESS=127.0.0.1:8378 timeout 120 "$BIN" --sim-plant "$GAINS" --sim-allow-dirty --sim-auto \
      --sim-log "$dir/run.csv" --sim-config "$CONFIG" $fc_args < /dev/null > fc.log 2>&1
    echo $? > code
    kill "$ec" 2>/dev/null; wait "$ec" 2>/dev/null )
}

for entry in "${scenarios[@]}"; do
  IFS='|' read -r name fc_args ec_args want <<< "$entry"
  want="${want# }"
  [ -n "$FILTER" ] && [[ "$name" != *"$FILTER"* ]] && continue
  total=$((total + 1))
  run_one "$name" "$fc_args" "$ec_args"
  code="$(cat "$WORK/run/code")"
  result="$(grep -E "SIL RESULT" "$WORK/run/fc.log" | tail -1 | sed -E 's/.*mode=Some\(([A-Za-z]*)\).*time_s=([0-9.]+).*apex_m=([0-9.]+).*landing_offset_m=([0-9.na/]+) impact_speed_mps=([0-9.na/]+).*faults=(\[.*\])/\1 t=\2 apex=\3 off=\4 impact=\5 \6/')"
  [ -z "$result" ] && result="(no result: exit $code)"
  if [ "$code" = 0 ]; then got=PASS; else got=FAIL; fi
  if [ "$want" = PASS ] && [ "$got" = PASS ]; then verdict="ok"; pass=$((pass + 1))
  elif [ "$want" = KNOWN-FAIL ] && [ "$got" = FAIL ]; then verdict="known gap"; pass=$((pass + 1))
  elif [ "$want" = KNOWN-FAIL ] && [ "$got" = PASS ]; then verdict="NOW PASSES: update the suite"; fail=$((fail + 1))
  else verdict="UNEXPECTED $got"; fail=$((fail + 1)); fi
  printf '%-62s %-10s %s\n' "$name" "[$verdict]" "$result"
  [ "$verdict" != ok ] && [ "$verdict" != "known gap" ] && grep -E "SIL VERDICT|rror|panick" "$WORK/run/fc.log" | head -3 | sed 's/^/    /'
done

echo
echo "$pass of $total scenarios ended as listed, $fail did not"
[ "$fail" = 0 ]
