#!/usr/bin/env bash

set -euo pipefail

# Creates four isolated namespaces that model:
#   - flight
#   - servo
#   - ftel
#   - gtel
#
# Topology:
#   flight <-> rocket bridge <-> ftel
#                    |
#                 umbilical
#                    |
#   servo  <-> ground bridge <-> gtel
#
# And a separate point-to-point radio link:
#   ftel radio0 <-> gtel radio0
#
# The radio link MTU is capped at 255 bytes total.

readonly NS_FLIGHT="flight"
readonly NS_SERVO="servo"
readonly NS_FTEL="ftel"
readonly NS_GTEL="gtel"

readonly BR_ROCKET="br-rocket"
readonly BR_GROUND="br-ground"
readonly BRIDGE_NF_STATE_FILE="/tmp/luna-netns-lab-bridge-nf.env"

cleanup() {
  ip link del "${BR_ROCKET}" 2>/dev/null || true
  ip link del "${BR_GROUND}" 2>/dev/null || true
  ip link del "umb-rkt" 2>/dev/null || true
  ip link del "ftel-radio" 2>/dev/null || true

  ip netns del "${NS_FLIGHT}" 2>/dev/null || true
  ip netns del "${NS_SERVO}" 2>/dev/null || true
  ip netns del "${NS_FTEL}" 2>/dev/null || true
  ip netns del "${NS_GTEL}" 2>/dev/null || true
}

save_and_disable_bridge_netfilter() {
  local iptables_value=""
  local arptables_value=""
  local ip6tables_value=""

  iptables_value="$(sysctl -n net.bridge.bridge-nf-call-iptables 2>/dev/null || true)"
  arptables_value="$(sysctl -n net.bridge.bridge-nf-call-arptables 2>/dev/null || true)"
  ip6tables_value="$(sysctl -n net.bridge.bridge-nf-call-ip6tables 2>/dev/null || true)"

  cat > "${BRIDGE_NF_STATE_FILE}" <<EOF
BRIDGE_NF_CALL_IPTABLES=${iptables_value}
BRIDGE_NF_CALL_ARPTABLES=${arptables_value}
BRIDGE_NF_CALL_IP6TABLES=${ip6tables_value}
EOF

  [[ -n "${iptables_value}" ]] && sysctl -q -w net.bridge.bridge-nf-call-iptables=0
  [[ -n "${arptables_value}" ]] && sysctl -q -w net.bridge.bridge-nf-call-arptables=0
  [[ -n "${ip6tables_value}" ]] && sysctl -q -w net.bridge.bridge-nf-call-ip6tables=0
}

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must be run as root." >&2
  exit 1
fi

save_and_disable_bridge_netfilter
cleanup

ip netns add "${NS_FLIGHT}"
ip netns add "${NS_SERVO}"
ip netns add "${NS_FTEL}"
ip netns add "${NS_GTEL}"

ip link add "${BR_ROCKET}" type bridge
ip link add "${BR_GROUND}" type bridge
ip link set "${BR_ROCKET}" up
ip link set "${BR_GROUND}" up

create_ethernet_node() {
  local ns="$1"
  local ns_if="$2"
  local peer_if="$3"
  local bridge="$4"
  local addr="$5"

  ip link add "${peer_if}" type veth peer name "${ns_if}"
  ip link set "${ns_if}" netns "${ns}"
  ip link set "${peer_if}" master "${bridge}"
  ip link set "${peer_if}" up

  ip netns exec "${ns}" ip link set lo up
  ip netns exec "${ns}" ip link set "${ns_if}" name eth0
  ip netns exec "${ns}" ip addr add "${addr}" dev eth0
  ip netns exec "${ns}" ip link set eth0 up
}

create_ethernet_node "${NS_FLIGHT}" "flight-eth" "p-flight" "${BR_ROCKET}" "192.168.1.11/24"
create_ethernet_node "${NS_FTEL}"   "ftel-eth"   "p-ftel"   "${BR_ROCKET}" "192.168.1.132/24"
create_ethernet_node "${NS_SERVO}"  "servo-eth"  "p-servo"  "${BR_GROUND}" "192.168.1.10/24"
create_ethernet_node "${NS_GTEL}"   "gtel-eth"   "p-gtel"   "${BR_GROUND}" "192.168.1.140/24"

# Umbilical cable joining the two bridges while connected.
ip link add "umb-rkt" type veth peer name "umb-gnd"
ip link set "umb-rkt" master "${BR_ROCKET}"
ip link set "umb-gnd" master "${BR_GROUND}"
ip link set "umb-rkt" up
ip link set "umb-gnd" up

# Point-to-point radio link with 255-byte MTU.
ip link add "ftel-radio" type veth peer name "gtel-radio"
ip link set "ftel-radio" netns "${NS_FTEL}"
ip link set "gtel-radio" netns "${NS_GTEL}"

ip netns exec "${NS_FTEL}" ip link set "ftel-radio" name radio0
ip netns exec "${NS_GTEL}" ip link set "gtel-radio" name radio0
ip netns exec "${NS_FTEL}" ip link set radio0 mtu 255
ip netns exec "${NS_GTEL}" ip link set radio0 mtu 255
ip netns exec "${NS_FTEL}" ip addr add 10.8.8.0/31 dev radio0
ip netns exec "${NS_GTEL}" ip addr add 10.8.8.1/31 dev radio0
ip netns exec "${NS_FTEL}" ip link set radio0 up
ip netns exec "${NS_GTEL}" ip link set radio0 up

if ip netns exec "${NS_FTEL}" sh -c 'command -v ethtool >/dev/null 2>&1'; then
  ip netns exec "${NS_FTEL}" ethtool -K radio0 tx off rx off >/dev/null
  ip netns exec "${NS_GTEL}" ethtool -K radio0 tx off rx off >/dev/null
fi

ip netns exec "${NS_FTEL}" sysctl -q -w net.ipv4.ip_forward=1
ip netns exec "${NS_GTEL}" sysctl -q -w net.ipv4.ip_forward=1

if ip netns exec "${NS_FTEL}" sh -c 'command -v nft >/dev/null 2>&1'; then
  ip netns exec "${NS_FTEL}" nft -f - <<'EOF'
table ip nat {
  chain prerouting {
    type nat hook prerouting priority -100; policy accept;
    ip saddr 192.168.1.11 ip daddr 192.168.1.10 ip dscp 46 dnat to 10.8.8.1
  }

  chain postrouting {
    type nat hook postrouting priority 100; policy accept;
    oifname "radio0" snat to 10.8.8.0
  }
}
EOF

  ip netns exec "${NS_GTEL}" nft -f - <<'EOF'
table ip nat {
  chain prerouting {
    type nat hook prerouting priority -100; policy accept;
    iifname "radio0" dnat to 192.168.1.10
  }

  chain postrouting {
    type nat hook postrouting priority 100; policy accept;
    oifname "eth0" ip daddr 192.168.1.10 snat to 192.168.1.140
  }
}
EOF
else
  if ! ip netns exec "${NS_FTEL}" sh -c 'command -v iptables >/dev/null 2>&1'; then
    echo "Neither nft nor iptables is available inside the namespaces." >&2
    exit 1
  fi

  ip netns exec "${NS_FTEL}" iptables -t nat -A PREROUTING \
    -s 192.168.1.11 -d 192.168.1.10 -m dscp --dscp 46 \
    -j DNAT --to-destination 10.8.8.1
  ip netns exec "${NS_FTEL}" iptables -t nat -A POSTROUTING \
    -o radio0 -j SNAT --to-source 10.8.8.0

  ip netns exec "${NS_GTEL}" iptables -t nat -A PREROUTING \
    -i radio0 -j DNAT --to-destination 192.168.1.10
  ip netns exec "${NS_GTEL}" iptables -t nat -A POSTROUTING \
    -o eth0 -d 192.168.1.10 -j SNAT --to-source 192.168.1.140
fi

# Policy-route DSCP-46 packets destined for Servo through FTEL.
ip netns exec "${NS_FLIGHT}" ip route replace table 246 192.168.1.132/32 dev eth0 src 192.168.1.11
ip netns exec "${NS_FLIGHT}" ip route replace table 246 192.168.1.10/32 via 192.168.1.132 dev eth0 src 192.168.1.11
ip netns exec "${NS_FLIGHT}" iptables -t mangle -F OUTPUT
ip netns exec "${NS_FLIGHT}" iptables -t mangle -A OUTPUT \
  -d 192.168.1.10 -m dscp --dscp 46 -j MARK --set-mark 246
while ip netns exec "${NS_FLIGHT}" ip rule show | grep -Fq "priority 246 "; do
  ip netns exec "${NS_FLIGHT}" ip rule del priority 246
done
ip netns exec "${NS_FLIGHT}" ip rule add priority 246 fwmark 246 lookup 246
ip netns exec "${NS_FLIGHT}" ip route flush cache

cat <<'EOF'
Namespace lab is ready.

Open terminals like:
  sudo ip netns exec servo bash
  sudo ip netns exec flight bash
  sudo ip netns exec ftel bash
  sudo ip netns exec gtel bash

Suggested manual flow:
  1. In the servo namespace shell:
       cargo run -p servo -- serve
  2. In the flight namespace shell:
       cargo run -p flight-computer
  3. While both are running, disconnect the umbilical with:
       sudo tel/scripts/toggle_umbilical.sh down
  4. Reconnect it with:
       sudo tel/scripts/toggle_umbilical.sh up

When the umbilical is up, Servo should receive both umbilical and radio telemetry.
When it is down, only radio telemetry should continue.
EOF
