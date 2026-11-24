#!/usr/bin/env bash
# Starts, stops and inspects the simulated CTV stack the GUI connects to:
# servo, the EC logic (sil_ec), the plant bridge and the flight computer.
#
#   HOPPER_BUILD=<hopper>/sims/hopper_6dof_sizer/fsw_bridge/build ./ctv_stack.sh start
#   ./ctv_stack.sh status | stop | restart | logs [servo|ec|bridge|fc] | gui
#
# After `start`, open the GUI: connect to servo at 127.0.0.1, pick the Umbilical
# telemetry source, open Views > CTV. One flight per start: the simulated vehicle
# is not reset after landing, so `restart` before the next one.
#
# Environment (all optional except HOPPER_BUILD for start/restart):
#   HOPPER_BUILD       the hopper repo's fsw_bridge/build (holds gains/ctv and the plant)
#   HOPPER_PLANT_CODE  the plant folder to use (default: plant_strut, plant or verify/plant under HOPPER_BUILD)
#   GAINS_DIR          the gains folder (default: $HOPPER_BUILD/gains/ctv)
#   CTV_STACK_DIR      where logs, pid files and the built sil_ec go (default: $HOME/ctv_stack);
#                      start only replaces the files it writes there, nothing else
#   CARGO_TARGET_DIR   where cargo builds (default: $HOME/ctv-sil-target; building under
#                      /mnt/c is slow on WSL)
#   SIL_TRUTH_NAV=1    fly on the plant's true state instead of the navigation filter
#   SKIP_BUILD=1       do not run cargo/make first
#
# Linux or WSL with a C compiler, make, cargo and python3. Nothing here touches hardware.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LUNA="$(cd "$HERE/../../.." && pwd)"
export PATH="$HOME/.cargo/bin:$PATH"
# Every path is made absolute here, from the caller's folder: the script changes
# folder later, and a relative path would then point somewhere else
abspath() { realpath -m -- "$1"; }
export CARGO_TARGET_DIR="$(abspath "${CARGO_TARGET_DIR:-$HOME/ctv-sil-target}")"
D="$(abspath "${CTV_STACK_DIR:-$HOME/ctv_stack}")"
T="$CARGO_TARGET_DIR/release"
PIDS="$D/pids"
SERVO_PORT=7200

die() { echo "ctv_stack: $*" >&2; exit 2; }

# Is the process whose pid is in $PIDS/$1 alive?
alive() { [ -f "$PIDS/$1" ] && kill -0 "$(cat "$PIDS/$1")" 2>/dev/null; }

# Starts a command in the background with its own session, remembering its pid
launch() {
  local name="$1" log="$2"; shift 2
  setsid nohup "$@" > "$log" 2>&1 < /dev/null &
  echo $! > "$PIDS/$name"
}

stop_stack() {
  local name pid stopped=0
  for name in fc bridge ec servo; do
    if alive "$name"; then
      pid="$(cat "$PIDS/$name")"
      kill "$pid" 2>/dev/null
      for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$pid" 2>/dev/null || break; sleep 0.3; done
      kill -9 "$pid" 2>/dev/null
      stopped=$((stopped + 1))
    fi
    rm -f "$PIDS/$name"
  done
  # Anything this script started and lost track of (a crashed run), and only that:
  # matched by the full path of our own build output
  pkill -f "$T/servo" 2>/dev/null
  pkill -f "$T/ctv-flight-computer" 2>/dev/null
  pkill -f "$D/ec/sil_ec" 2>/dev/null
  sleep 0.5
  echo "stopped ($stopped tracked processes)"
}

status_stack() {
  local name up=0
  for name in servo ec bridge fc; do
    if alive "$name"; then printf '%-8s running  (pid %s)\n' "$name" "$(cat "$PIDS/$name")"; up=$((up + 1))
    else printf '%-8s not running\n' "$name"; fi
  done
  if [ "$up" = 4 ]; then
    echo "stack is up: GUI -> connect to 127.0.0.1, source Umbilical, Views > CTV (servo on port $SERVO_PORT)"
    [ -f "$D/fc.log" ] && echo "last flight computer line: $(tail -n 1 "$D/fc.log" | cut -c1-120)"
    return 0
  fi
  return 1
}

start_stack() {
  [ -n "${HOPPER_BUILD:-}" ] || die "set HOPPER_BUILD to the hopper repo's fsw_bridge/build folder"
  HOPPER_BUILD="$(abspath "$HOPPER_BUILD")"
  local plant=""
  [ -n "${HOPPER_PLANT_CODE:-}" ] && plant="$(abspath "$HOPPER_PLANT_CODE")"
  if [ -z "$plant" ]; then
    for c in "$HOPPER_BUILD/plant_strut" "$HOPPER_BUILD/plant" "$HOPPER_BUILD/verify/plant"; do
      [ -f "$c/plant.json" ] && plant="$c" && break
    done
  fi
  [ -f "$plant/plant.json" ] || die "no plant.json under $HOPPER_BUILD (set HOPPER_PLANT_CODE)"
  local gains
  gains="$(abspath "${GAINS_DIR:-$HOPPER_BUILD/gains/ctv}")"
  [ -d "$gains" ] || die "no gains folder at $gains"
  command -v cargo >/dev/null || die "cargo not found (install Rust, see rust-toolchain.toml)"

  stop_stack >/dev/null
  # Clear only what this script writes, never the folder itself: CTV_STACK_DIR may
  # hold other files. The built sil_ec in $D/ec is kept so SKIP_BUILD can reuse it.
  rm -rf "$D/home" "$D/logs" "$PIDS"
  rm -f "$D/servo.log" "$D/ec.log" "$D/bridge.log" "$D/fc.log" "$D/bridge.csv" "$D/nav.json"
  mkdir -p "$D/home" "$D/logs" "$D/ec" "$PIDS"
  cd "$D" || exit 2
  export HOPPER_PLANT_CODE="$plant"
  echo "plant:  $plant"
  echo "gains:  $gains"
  echo "dir:    $D"

  if [ -z "${SKIP_BUILD:-}" ]; then
    echo "building (the first time takes a few minutes)..."
    cp -r "$LUNA/firmware/fc_ec_link/." ec/
    (cd ec && make sil_ec >/dev/null) || die "sil_ec did not build"
    (cd "$LUNA" && cargo build -q --release -p ctv-flight-computer --features sim-plant \
      && cargo build -q --release -p servo) || die "cargo build failed"
  fi
  [ -x "$T/servo" ] && [ -x "$T/ctv-flight-computer" ] && [ -x ec/sil_ec ] \
    || die "binaries missing (run once without SKIP_BUILD)"

  python3 "$HERE/nav_config.py" "$plant/plant.json" > nav.json || die "nav_config.py failed"
  local lever
  lever="$(python3 "$HERE/nav_config.py" "$plant/plant.json" --lever)" || die "nav_config.py --lever failed"

  HOME="$D/home" launch servo "$D/servo.log" "$T/servo" serve --volatile --quiet
  launch ec "$D/ec.log" ./ec/sil_ec -p 8378 --applied-udp 127.0.0.1:5601 --speed 100
  launch bridge "$D/bridge.log" "$T/ctv-flight-computer" --sil-bridge --sil-feed-to 127.0.0.1:5600 \
    --sil-applied-port 5601 --sil-roll-port 5602 --sil-stay \
    --sim-config "$HERE/sil.json" --sim-log "$D/bridge.csv"
  sleep 1
  FC_EC_ADDRESS=127.0.0.1:8378 FC_SERVO_ADDRESS=127.0.0.1:5025 \
  launch fc "$D/fc.log" "$T/ctv-flight-computer" desktop --sil-feed 5600 --sil-roll-to 127.0.0.1:5602 \
    --gnc-gains "$gains" --gnc-vehicle-config "$HERE/sil.json" --gnc-allow-dirty \
    --gnc-imu-lever="$lever" ${SIL_TRUTH_NAV:+--sil-truth-nav} --shadow-nav-config nav.json \
    --disable-file-logging --log-dirs "$D/logs"

  # Up when all four are alive and servo answers; the filter then needs a few
  # more seconds to level before the page shows READY
  local i
  for i in $(seq 1 40); do
    sleep 0.5
    for n in servo ec bridge fc; do
      alive "$n" || { echo "ctv_stack: $n exited at start-up; its log:" >&2; tail -n 15 "$D/$n.log" >&2; stop_stack >/dev/null; exit 1; }
    done
    if (exec 3<>"/dev/tcp/127.0.0.1/$SERVO_PORT") 2>/dev/null; then break; fi
  done
  echo "stack up"
  status_stack
  echo "the navigation filter needs a few seconds to level before the page shows READY"
}

case "${1:-start}" in
  start)   start_stack ;;
  stop)    stop_stack ;;
  restart) start_stack ;;
  status)  status_stack ;;
  logs)
    f="$D/${2:-fc}.log"
    [ -f "$f" ] || die "no log $f (is the stack started?)"
    tail -n "${3:-40}" -f "$f" ;;
  gui)
    cd "$LUNA/gui" || exit 2
    command -v npm >/dev/null || die "npm not found (install Node.js)"
    npm install && exec npm run tauri dev ;;
  *) die "usage: $0 start|stop|restart|status|logs [servo|ec|bridge|fc]|gui" ;;
esac
