#!/usr/bin/env bash
# Starts the sim-plant bench test from this package folder.
#   ./run.sh                 uses bench.env, bench.json, gains/
#   ./run.sh --sim-log x.csv extra flight2 arguments go through
#
# OS hook: if launch.sh exists here (and is executable), it is run instead
# of the binary, with the binary's path first and the arguments after it.
# Use it for anything the OS needs, e.g. starting the binary through the Nix
# store loader on NixOS. Without it the binary runs directly.
set -euo pipefail
cd "$(dirname "$0")"
set -a
# shellcheck disable=SC1091
. ./bench.env
set +a
mkdir -p logs
log="logs/run_$(date +%Y%m%d_%H%M%S).csv"
args=(--sim-plant gains --sim-config bench.json --sim-log "$log" "$@")
echo "EC address: ${FC_EC_ADDRESS}   log: ${log}"
if [[ -x ./launch.sh ]]; then
    exec ./launch.sh ./ctv-flight-computer "${args[@]}"
fi
exec ./ctv-flight-computer "${args[@]}"
