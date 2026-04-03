#!/usr/bin/env bash

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

if [[ $# -ne 1 ]]; then
  echo "Usage: sudo $0 <up|down>" >&2
  exit 1
fi

case "$1" in
  up)
    ip link set umb-rkt up
    ip link set umb-gnd up
    echo "Umbilical link is up."
    ;;
  down)
    ip link set umb-rkt down
    ip link set umb-gnd down
    echo "Umbilical link is down."
    ;;
  *)
    echo "Usage: sudo $0 <up|down>" >&2
    exit 1
    ;;
esac
