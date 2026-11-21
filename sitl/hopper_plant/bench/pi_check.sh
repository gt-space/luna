#!/usr/bin/env bash
# CTV flight computer check: runs the sim-plant bench against a fake engine
# controller on this machine only, and packs the results to send back.
#
#   ./pi_check.sh            no input needed; takes about 3 minutes
#
# What it does:
#   1. Records the system: /etc/os-release, uname, glibc, program loader,
#      and checks every file against MANIFEST.txt.
#   2. Checks that the flight program starts at all. If it doesn't (wrong OS
#      or glibc), it says so, still packs the system info, and stops.
#   3. Starts the bundled fake_ec on 127.0.0.1:8378 and flies 5 full runs
#      (start -> land -> stop -> reset), then one run killed with kill -9 to
#      check that the fake EC safes itself on its demand timeout.
#   4. Packs everything into ~/pi_results-<hostname>-<date>.tar.gz and
#      prints a PASS/FAIL table.
#
# Safety: the program only ever talks to 127.0.0.1 (this machine). The real
# EC, bench.env and the network are never used. Nothing is installed, and
# nothing outside this folder is touched except the one results file in ~.

set -u
cd "$(dirname "$0")" || exit 1
HERE=$(pwd)

EC_ADDR=127.0.0.1:8378       # fake EC on this machine; never a network address
export FC_EC_ADDRESS=$EC_ADDR   # what the program reads; bench.env is never sourced
RUNS=5
BIN=./ctv-flight-computer
STAMP=$(date +%Y%m%d_%H%M%S)
HOST=$(hostname 2>/dev/null || echo unknown)
OUT="$HERE/pi_check_results_$STAMP"
RESULTS="$HOME/pi_results-$HOST-$STAMP.tar.gz"
mkdir -p "$OUT"

FC_PID="" EC_PID=""
cleanup() {
    exec 3>&- 2>/dev/null
    [[ -n "$FC_PID" ]] && kill "$FC_PID" 2>/dev/null
    [[ -n "$EC_PID" ]] && kill "$EC_PID" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT

say() { echo "$*" | tee -a "$OUT/pi_check.log"; }

# Wait until a log file has a line matching a pattern (after line $3)
wait_for() { # file pattern start_line timeout_s
    local file=$1 pattern=$2 from=${3:-0} limit=$(( ${4:-10} * 10 )) i=0
    while (( i < limit )); do
        if tail -n +"$((from + 1))" "$file" 2>/dev/null | grep -q -- "$pattern"; then
            return 0
        fi
        sleep 0.1
        i=$((i + 1))
    done
    return 1
}
lines() { wc -l < "$1" 2>/dev/null || echo 0; }

# Launch the program the way run.sh would, but always against the fake EC.
# Always called in a subshell; exec makes a backgrounded launch's PID the
# program's own, so the kill -9 test hits the program.
launch() {
    [[ $FC_EC_ADDRESS == "$EC_ADDR" ]] || { echo "refusing: EC address is not $EC_ADDR"; exit 1; }
    if [[ -x ./launch.sh ]]; then
        exec ./launch.sh "$BIN" "$@"
    else
        exec "$BIN" "$@"
    fi
}

start_fake_ec() { # log name
    ./fake_ec > "$OUT/$1" 2>&1 &
    EC_PID=$!
    sleep 1
    kill -0 "$EC_PID" 2>/dev/null
}
stop_fake_ec() { [[ -n "$EC_PID" ]] && kill "$EC_PID" 2>/dev/null; wait "$EC_PID" 2>/dev/null; EC_PID=""; }

say "== pi_check $STAMP on $HOST (results: $RESULTS)"

# ---- 1. System info ---------------------------------------------------------
{
    echo "== /etc/os-release"; cat /etc/os-release 2>&1
    echo; echo "== uname -a"; uname -a 2>&1
    echo; echo "== ldd --version"; ldd --version 2>&1 | head -2
    echo; echo "== program loaders"
    for l in /lib/ld-linux-aarch64.so.1 /lib64/ld-linux-x86-64.so.2; do
        if [[ -e $l ]]; then echo "$l: present"; else echo "$l: MISSING"; fi
    done
    echo; echo "== nproc / memory"; nproc 2>&1; head -3 /proc/meminfo 2>&1
    echo; echo "== bundled programs"
    (file ctv-flight-computer fake_ec 2>/dev/null || ls -l ctv-flight-computer fake_ec) 2>&1
    echo; echo "== other processes on the bench ports"
    (ss -uanp 2>/dev/null || netstat -uan 2>/dev/null) | grep -E ':(8378|4574)\b' || echo "none"
    echo; echo "== MANIFEST check"
    sha256sum -c <(tail -n +7 MANIFEST.txt) 2>&1
} > "$OUT/system.txt" 2>&1

OS=$(. /etc/os-release 2>/dev/null; echo "${PRETTY_NAME:-unknown}")
GLIBC=$(ldd --version 2>&1 | head -1 | grep -oE '[0-9]+\.[0-9]+$' || echo "?")
ARCH=$(uname -m)
if grep -q "FAILED" "$OUT/system.txt"; then MANIFEST=FAIL; else MANIFEST=PASS; fi
case "$ARCH" in
    aarch64) LOADER=/lib/ld-linux-aarch64.so.1 ;;
    x86_64)  LOADER=/lib64/ld-linux-x86-64.so.2 ;;
    *)       LOADER=unknown ;;
esac
[[ -e $LOADER ]] && LOADER_OK=present || LOADER_OK=MISSING
say "system: $OS, glibc $GLIBC, $ARCH, loader $LOADER $LOADER_OK, manifest $MANIFEST"

# ---- 2. Does the program start? ---------------------------------------------
STARTS=FAIL START_NOTE=""
START_OUT=$( (launch --help) 2>&1)
echo "$START_OUT" > "$OUT/program_start.txt"
if echo "$START_OUT" | grep -q -- "--sim-plant"; then
    STARTS=PASS
else
    START_NOTE=$(echo "$START_OUT" | head -2 | tr '\n' ' ')
fi
say "program starts: $STARTS ${START_NOTE:+($START_NOTE)}"

declare -a R_TD R_OVR R_GAP R_SAFE
KILL9="not run"

if [[ $STARTS == PASS ]]; then
    # ---- 3. Five full runs in one session ------------------------------------
    LOG="$OUT/runs.log"
    if start_fake_ec fake_ec_runs.log; then
        mkfifo "$OUT/cmd"
        (launch --sim-plant gains --sim-config bench.json --sim-log "$OUT/runs.csv") \
            < "$OUT/cmd" > "$LOG" 2>&1 &
        FC_PID=$!
        exec 3> "$OUT/cmd"
        if wait_for "$LOG" "EC is Disarmed" 0 15; then
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
                R_GAP[n]=$(echo "$seg" | grep -oE 'longest gap between demand sends [0-9.]+' | grep -oE '[0-9.]+$' | head -1)
                if echo "$seg" | grep -q "EC left Armed"; then R_SAFE[n]=YES; else R_SAFE[n]=no; fi
                say "run $n: touchdown ${R_TD[n]}, overruns ${R_OVR[n]:-?}, longest gap ${R_GAP[n]:-?} ms, mid-run safing ${R_SAFE[n]}"
                if [[ ${R_SAFE[n]} == no ]]; then
                    from=$(lines "$LOG"); echo stop >&3
                    wait_for "$LOG" "SAFED" "$from" 5 || say "run $n: stop didn't reach SAFED"
                fi
                from=$(lines "$LOG"); echo reset >&3; sleep 0.5; echo status >&3
                wait_for "$LOG" "IDLE t =" "$from" 5 || { say "run $n: reset didn't return to IDLE"; break; }
                sleep 0.5
            done
        else
            say "the program never connected to the fake EC (see runs.log)"
        fi
        echo quit >&3; exec 3>&-
        wait_for "$LOG" "quitting" 0 5; sleep 1
        kill "$FC_PID" 2>/dev/null; wait "$FC_PID" 2>/dev/null; FC_PID=""
        rm -f "$OUT/cmd"
        stop_fake_ec
    else
        say "fake_ec didn't start (see fake_ec_runs.log)"
    fi

    # ---- 4. kill -9 mid-run: the fake EC must safe on its demand timeout -----
    LOG9="$OUT/kill9.log"
    if start_fake_ec fake_ec_kill9.log; then
        mkfifo "$OUT/cmd9"
        (launch --sim-plant gains --sim-config bench.json --sim-log "$OUT/kill9.csv") \
            < "$OUT/cmd9" > "$LOG9" 2>&1 &
        FC_PID=$!
        exec 3> "$OUT/cmd9"
        if wait_for "$LOG9" "EC is Disarmed" 0 15 && { echo start >&3; wait_for "$LOG9" "RUN, t = 0" 0 10; }; then
            sleep 3
            kill -9 "$FC_PID" 2>/dev/null; wait "$FC_PID" 2>/dev/null; FC_PID=""
            sleep 1
            if grep -q "ARMED -> SAFING (demand timeout)" "$OUT/fake_ec_kill9.log"; then
                KILL9="PASS (fake EC safed on its demand timeout)"
            else
                KILL9="FAIL (fake EC didn't safe on its own)"
            fi
        else
            KILL9="FAIL (run didn't start)"
        fi
        exec 3>&-
        [[ -n "$FC_PID" ]] && { kill "$FC_PID" 2>/dev/null; wait "$FC_PID" 2>/dev/null; FC_PID=""; }
        rm -f "$OUT/cmd9"
        stop_fake_ec
    else
        KILL9="FAIL (fake_ec didn't start)"
    fi
    say "kill -9: $KILL9"
fi

# ---- 5. Table ------------------------------------------------------------------
verdict_gap() { # ms
    local g=$1
    [[ -z $g ]] && { echo "?"; return; }
    if awk "BEGIN{exit !($g >= 30)}"; then echo "FAIL ($g ms)"
    elif awk "BEGIN{exit !($g > 15)}"; then echo "PASS, FLAG ($g ms > 15)"
    else echo "PASS ($g ms)"; fi
}
{
    echo "=============== pi_check summary ($HOST, $STAMP) ==============="
    printf "%-26s %s\n" "OS" "$OS ($ARCH)"
    printf "%-26s %s\n" "glibc" "$GLIBC (the program needs >= 2.39)"
    printf "%-26s %s\n" "program loader" "$LOADER $LOADER_OK"
    printf "%-26s %s\n" "files (MANIFEST)" "$MANIFEST"
    printf "%-26s %s\n" "program starts" "$STARTS ${START_NOTE:+- $START_NOTE}"
    if [[ $STARTS == PASS ]]; then
        for n in $(seq 1 "$RUNS"); do
            [[ -z ${R_TD[n]:-} ]] && break
            td=${R_TD[n]}
            if [[ $td == *s ]] && awk "BEGIN{v=${td% s}; exit !(v >= 14 && v <= 18)}"; then
                td="PASS ($td)"   # nominal hop lands at ~15.97 s
            else
                td="FAIL ($td)"
            fi
            ovr=${R_OVR[n]:-?}; [[ $ovr == 0 ]] && ovr="PASS (0)" || ovr="CHECK ($ovr)"
            saf=${R_SAFE[n]:-?}; [[ $saf == no ]] && saf="PASS (none)" || saf="FAIL ($saf)"
            printf "run %-22s touchdown %-16s overruns %-10s demand gap %-22s mid-run safing %s\n" \
                "$n" "$td" "$ovr" "$(verdict_gap "${R_GAP[n]:-}")" "$saf"
        done
        printf "%-26s %s\n" "kill -9" "$KILL9"
    else
        echo "Runs skipped: the program doesn't start on this system. Send the results file back as is."
    fi
    echo "Results file: $RESULTS"
} | tee "$OUT/summary.txt"

cp MANIFEST.txt bench.json plant.json "$OUT/" 2>/dev/null
tar -C "$HERE" -czf "$RESULTS" "$(basename "$OUT")" && echo "Packed: $RESULTS"
