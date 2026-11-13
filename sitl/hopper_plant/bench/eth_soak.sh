#!/usr/bin/env bash
# Soak test against a fake EC on ANOTHER machine, over the real Ethernet:
# connect, sit in IDLE for a while, then fly several runs, and print a table.
#
#   ./eth_soak.sh --fake-ec-at 192.168.137.1:8378 [--idle-min 30] [--runs 5] [--yes]
#
# Start the fake EC on the other machine first (`./fake_ec | tee fake_ec.log`)
# and keep its log: its once-a-second line and its "link totals" line on exit
# are the only place lost packets and demand arrival gaps show up.
#
# Safety:
#   - Only ever for a FAKE EC. The address is given explicitly (bench.env is
#     never read) and must not be a vehicle board: 192.168.1.130-139
#     (including the proposed real EC at .138) and flight2's default EC
#     192.168.1.50 are refused.
#   - Asks you to confirm before anything is armed, unless --yes.
#   - Writes only inside this folder plus one results file in ~.
#
# The manual checks (stop, Ctrl-C, kill -9, pulling the cable) stay manual;
# see the Ethernet test plan.

set -u
cd "$(dirname "$0")" || exit 1
HERE=$(pwd)

usage() {
    echo "usage: $0 --fake-ec-at IP:PORT [--idle-min N | --idle-sec N] [--runs N] [--yes]" >&2
    exit 2
}

EC_ADDR="" IDLE_S=1800 RUNS=5 YES=no
while [[ $# -gt 0 ]]; do
    case "$1" in
        --fake-ec-at) EC_ADDR="${2:-}"; shift 2 ;;
        --idle-min) [[ ${2:-} =~ ^[0-9]+$ ]] || usage; IDLE_S=$(( $2 * 60 )); shift 2 ;;
        --idle-sec) [[ ${2:-} =~ ^[0-9]+$ ]] || usage; IDLE_S=$2; shift 2 ;;
        --runs) RUNS="${2:-}"; shift 2 ;;
        --yes) YES=yes; shift ;;
        *) usage ;;
    esac
done
[[ $EC_ADDR =~ ^([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3}):([0-9]{1,5})$ ]] || {
    echo "--fake-ec-at must be a numeric IP:PORT (the FC can't use a hostname)"; usage; }
o4=$(( 10#${BASH_REMATCH[4]} ))   # read before the next =~ overwrites BASH_REMATCH
ip=${EC_ADDR%:*}
[[ $RUNS =~ ^[0-9]+$ ]] || usage
if [[ $ip == 192.168.1.* ]] && (( o4 >= 130 && o4 <= 139 )); then
    echo "refusing: $ip is in the vehicle-board range 192.168.1.130-139 (the real EC is proposed at .138)"
    exit 1
fi
if [[ $ip == 192.168.1.50 ]]; then
    echo "refusing: 192.168.1.50 is flight2's default EC address"
    exit 1
fi

export FC_EC_ADDRESS=$EC_ADDR
BIN=./ctv-flight-computer
STAMP=$(date +%Y%m%d_%H%M%S)
HOST=$(hostname 2>/dev/null || echo unknown)
OUT="$HERE/eth_soak_results_$STAMP"
RESULTS="$HOME/eth_soak-$HOST-$STAMP.tar.gz"
LOG="$OUT/session.log"
mkdir -p "$OUT"

FC_PID=""
cleanup() {
    exec 3>&- 2>/dev/null
    [[ -n "$FC_PID" ]] && kill "$FC_PID" 2>/dev/null
    wait 2>/dev/null
    rm -f "$OUT/cmd"
}
trap cleanup EXIT

say() { echo "$*" | tee -a "$OUT/eth_soak.log"; }
wait_for() { # file pattern start_line timeout_s
    local file=$1 pattern=$2 from=${3:-0} limit=$(( ${4:-10} * 10 )) i=0
    while (( i < limit )); do
        tail -n +"$((from + 1))" "$file" 2>/dev/null | grep -q -- "$pattern" && return 0
        sleep 0.1
        i=$((i + 1))
    done
    return 1
}
lines() { wc -l < "$1" 2>/dev/null || echo 0; }
RECONNECT='reconnecting|EC dropped the session|EC rebooted'
reconnects() { tail -n +"$(( $2 + 1 ))" "$1" 2>/dev/null | grep -cE "$RECONNECT"; }

launch() {
    [[ $FC_EC_ADDRESS == "$EC_ADDR" ]] || { echo "refusing: EC address changed"; exit 1; }
    if [[ -x ./launch.sh ]]; then exec ./launch.sh "$BIN" "$@"; fi
    exec "$BIN" "$@"
}

say "== eth_soak $STAMP on $HOST: fake EC at $EC_ADDR, IDLE ${IDLE_S} s, then $RUNS runs"
say "   results: $RESULTS"
if [[ $YES != yes ]]; then
    echo "This arms whatever answers at $EC_ADDR. It must be the FAKE EC on the other machine."
    read -r -p "Type 'fake' to go on: " answer < /dev/tty
    [[ $answer == fake ]] || { say "not confirmed; nothing started"; exit 1; }
fi

mkfifo "$OUT/cmd"
(launch --sim-plant gains --sim-config bench.json --sim-log "$OUT/runs.csv") < "$OUT/cmd" > "$LOG" 2>&1 &
FC_PID=$!
exec 3> "$OUT/cmd"

CONNECT=FAIL
IDLE_RECON="-"
declare -a R_TD R_OVR R_LATE R_GAP R_SAFE
if wait_for "$LOG" "EC is Disarmed" 0 20; then
    CONNECT=PASS
    say "connected: $(grep -m1 'EC link: connected' "$LOG")"

    # ---- IDLE soak ----------------------------------------------------------
    from=$(lines "$LOG")
    end=$(( SECONDS + IDLE_S )); next_status=$SECONDS
    while (( SECONDS < end )); do
        if (( SECONDS >= next_status )); then
            echo status >&3; next_status=$(( SECONDS + 60 ))
        fi
        kill -0 "$FC_PID" 2>/dev/null || { say "the flight program exited during IDLE"; break; }
        sleep 1
    done
    IDLE_RECON=$(reconnects "$LOG" "$from")
    say "IDLE soak done: ${IDLE_S}s, reconnect events $IDLE_RECON, last status: $(grep 'IDLE t =' "$LOG" | tail -1)"

    # ---- Runs ---------------------------------------------------------------
    for n in $(seq 1 "$RUNS"); do
        from=$(lines "$LOG")
        echo start >&3
        if ! wait_for "$LOG" "RUN, t = 0" "$from" 10; then
            R_TD[n]="no RUN"; R_SAFE[n]="?"; say "run $n: didn't start"; break
        fi
        wait_for "$LOG" "longest gap between demand sends" "$from" 60
        seg=$(tail -n +"$((from + 1))" "$LOG")
        echo "$seg" > "$OUT/run_$n.txt"
        R_TD[n]=$(echo "$seg" | grep -oE 'LANDED at t = [0-9.]+ s \(touchdown\)' | grep -oE '[0-9.]+ s' | head -1)
        [[ -z ${R_TD[n]} ]] && R_TD[n]="no touchdown"
        R_OVR[n]=$(echo "$seg" | grep -oE '[0-9]+ overruns' | grep -oE '^[0-9]+' | head -1)
        R_LATE[n]=$(echo "$seg" | grep -oE 'tick lateness max [0-9]+' | grep -oE '[0-9]+$' | head -1)
        R_GAP[n]=$(echo "$seg" | grep -oE 'longest gap between demand sends [0-9.]+' | grep -oE '[0-9.]+$' | head -1)
        if echo "$seg" | grep -q "EC left Armed"; then R_SAFE[n]=YES; else R_SAFE[n]=no; fi
        say "run $n: touchdown ${R_TD[n]}, overruns ${R_OVR[n]:-?}, lateness max ${R_LATE[n]:-?} us, longest gap ${R_GAP[n]:-?} ms, mid-run safing ${R_SAFE[n]}"
        if [[ ${R_SAFE[n]} == no ]]; then
            from=$(lines "$LOG"); echo stop >&3
            wait_for "$LOG" "SAFED" "$from" 5 || say "run $n: stop didn't reach SAFED"
        fi
        from=$(lines "$LOG"); echo reset >&3; sleep 0.5; echo status >&3
        wait_for "$LOG" "IDLE t =" "$from" 5 || { say "run $n: reset didn't return to IDLE"; break; }
        sleep 0.5
    done
else
    say "the program never connected to the fake EC at $EC_ADDR (see session.log)"
fi
echo quit >&3; exec 3>&-
wait_for "$LOG" "quitting" 0 5; sleep 1
kill "$FC_PID" 2>/dev/null; wait "$FC_PID" 2>/dev/null; FC_PID=""
ALL_RECON=$(reconnects "$LOG" 0)

# ---- Table ----------------------------------------------------------------------
num_check() { # value pass_if_below unit
    [[ -z $1 ]] && { echo "?"; return; }
    if awk "BEGIN{exit !($1 < $2)}"; then echo "PASS ($1 $3)"; else echo "FAIL ($1 $3)"; fi
}
{
    echo "=============== eth_soak summary ($HOST, $STAMP) ==============="
    printf "%-22s %s\n" "fake EC" "$EC_ADDR"
    printf "%-22s %s\n" "connect" "$CONNECT"
    if [[ $IDLE_RECON == 0 ]]; then r="PASS (0 reconnects)"; else r="FAIL ($IDLE_RECON reconnects)"; fi
    [[ $IDLE_RECON == "-" ]] && r="not run"
    printf "%-22s %s\n" "IDLE ${IDLE_S}s" "$r"
    for n in $(seq 1 "$RUNS"); do
        [[ -z ${R_TD[n]:-} ]] && break
        td=${R_TD[n]}
        if [[ $td == *s ]] && awk "BEGIN{v=${td% s}; exit !(v >= 14 && v <= 18)}"; then td="PASS ($td)"; else td="FAIL ($td)"; fi
        ovr=${R_OVR[n]:-?}; [[ $ovr == 0 ]] && ovr="PASS (0)" || ovr="FAIL ($ovr)"
        saf=${R_SAFE[n]:-?}; [[ $saf == no ]] && saf="PASS" || saf="FAIL ($saf)"
        printf "run %-18s touchdown %-15s overruns %-9s lateness %-17s send gap %-16s mid-run safing %s\n" \
            "$n" "$td" "$ovr" "$(num_check "${R_LATE[n]:-}" 1000 us)" "$(num_check "${R_GAP[n]:-}" 15 ms)" "$saf"
    done
    if [[ $ALL_RECON == 0 ]]; then r="PASS (0)"; else r="FAIL ($ALL_RECON, see session.log)"; fi
    printf "%-22s %s\n" "reconnects, whole run" "$r"
    echo "Also check the fake EC's log on the other machine:"
    echo "  crc=0 rej=0 stale=0 old=0, lost=0, and 'link totals' longest demand gap < 20 ms."
    echo "Results file: $RESULTS"
} | tee "$OUT/summary.txt"

cp MANIFEST.txt bench.json plant.json "$OUT/" 2>/dev/null
tar -C "$HERE" -czf "$RESULTS" "$(basename "$OUT")" && echo "Packed: $RESULTS"
