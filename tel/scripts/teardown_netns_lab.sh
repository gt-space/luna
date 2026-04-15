#!/usr/bin/env bash

set -euo pipefail

readonly BRIDGE_NF_STATE_FILE="/tmp/luna-netns-lab-bridge-nf.env"

restore_bridge_netfilter() {
  if [[ ! -f "${BRIDGE_NF_STATE_FILE}" ]]; then
    return
  fi

  # shellcheck disable=SC1090
  source "${BRIDGE_NF_STATE_FILE}"

  [[ -n "${BRIDGE_NF_CALL_IPTABLES:-}" ]] && \
    sysctl -q -w net.bridge.bridge-nf-call-iptables="${BRIDGE_NF_CALL_IPTABLES}"
  [[ -n "${BRIDGE_NF_CALL_ARPTABLES:-}" ]] && \
    sysctl -q -w net.bridge.bridge-nf-call-arptables="${BRIDGE_NF_CALL_ARPTABLES}"
  [[ -n "${BRIDGE_NF_CALL_IP6TABLES:-}" ]] && \
    sysctl -q -w net.bridge.bridge-nf-call-ip6tables="${BRIDGE_NF_CALL_IP6TABLES}"

  rm -f "${BRIDGE_NF_STATE_FILE}"
}

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

ip link del br-rocket 2>/dev/null || true
ip link del br-ground 2>/dev/null || true
ip link del umb-rkt 2>/dev/null || true
ip link del ftel-radio 2>/dev/null || true

ip netns del flight 2>/dev/null || true
ip netns del servo 2>/dev/null || true
ip netns del ftel 2>/dev/null || true
ip netns del gtel 2>/dev/null || true

restore_bridge_netfilter

echo "Namespace lab removed."
