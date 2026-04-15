#!/usr/bin/env bash

set -euo pipefail

# Installs policy routing so DSCP-46 telemetry packets destined for Servo are
# forced to use FTEL as the next hop, even when Servo is otherwise directly
# reachable over the umbilical Ethernet.

SERVO_IP="${SERVO_IP:-192.168.1.10}"
FTEL_IP="${FTEL_IP:-192.168.1.132}"
TABLE_ID="${TABLE_ID:-246}"
TABLE_NAME="${TABLE_NAME:-tel_radio}"
RULE_PRIORITY="${RULE_PRIORITY:-246}"
FWMARK="${FWMARK:-246}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

if ! command -v ip >/dev/null 2>&1; then
  echo "'ip' command not found." >&2
  exit 1
fi

if ! command -v iptables >/dev/null 2>&1; then
  echo "'iptables' command not found." >&2
  exit 1
fi

route_to_ftel="$(ip -4 route get "${FTEL_IP}")"
dev="$(awk '{for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit }}' <<<"${route_to_ftel}")"
src_ip="$(awk '{for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit }}' <<<"${route_to_ftel}")"

if [[ -z "${dev}" ]]; then
  echo "Could not determine the network interface that reaches ${FTEL_IP}." >&2
  exit 1
fi

if [[ -z "${src_ip}" ]]; then
  echo "Could not determine the source IP used to reach ${FTEL_IP}." >&2
  exit 1
fi

if ! grep -Eq "^[[:space:]]*${TABLE_ID}[[:space:]]+${TABLE_NAME}$" /etc/iproute2/rt_tables; then
  echo "${TABLE_ID} ${TABLE_NAME}" >> /etc/iproute2/rt_tables
fi

# Original Table Configuration
ip route replace table "${TABLE_NAME}" "${FTEL_IP}/32" dev "${dev}" src "${src_ip}"
ip route replace table "${TABLE_NAME}" "${SERVO_IP}/32" via "${FTEL_IP}" dev "${dev}" src "${src_ip}"

# Mangle Rule (Ensures DSCP 46 is marked with FWMARK)
while iptables -t mangle -C OUTPUT -d "${SERVO_IP}" -m dscp --dscp 46 -j MARK --set-mark "${FWMARK}" 2>/dev/null; do
  iptables -t mangle -D OUTPUT -d "${SERVO_IP}" -m dscp --dscp 46 -j MARK --set-mark "${FWMARK}"
done
iptables -t mangle -A OUTPUT -d "${SERVO_IP}" -m dscp --dscp 46 -j MARK --set-mark "${FWMARK}"

# Priority Rule for Table 246
while ip rule show | grep -Fq "priority ${RULE_PRIORITY} "; do
  ip rule del priority "${RULE_PRIORITY}"
done

ip rule add \
  priority "${RULE_PRIORITY}" \
  fwmark "${FWMARK}" \
  lookup "${TABLE_NAME}"

# Add explicit Table 69 routing and rule
ip rule add fwmark 246 to 192.168.1.10/32 lookup 69 priority 100
ip route replace 192.168.1.10/32 via 192.168.1.132 dev eth0 table 69

# Disable ICMP redirects
sysctl -w net.ipv4.conf.all.accept_redirects=0
sysctl -w net.ipv4.conf.default.accept_redirects=0

ip route flush cache

cat <<EOF
Installed TEL policy routing:
  Servo IP:      ${SERVO_IP}
  FTEL next hop: ${FTEL_IP}
  Interface:     ${dev}
  Source IP:     ${src_ip}
  Routing table: ${TABLE_NAME} (${TABLE_ID})
  FW mark:       ${FWMARK}
  Rule:          fwmark ${FWMARK} -> ${TABLE_NAME}
  Mangle rule:   OUTPUT to ${SERVO_IP} with DSCP 46 gets mark ${FWMARK}
  
Additional Config:
  Table 69:      192.168.1.10/32 via 192.168.1.132 (Priority 100)
  Sysctl:        accept_redirects disabled
EOF
