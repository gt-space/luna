#!/usr/bin/bash
set -e

IFACE="$1"

if [ -z "$IFICE" ]; then
  echo "Error: No interface specified."
  echo "Usage: sudo $0 <interface>"
  exit 1
fi

# Clean up potentially existing namespaces.
ip netns del flight 2>/dev/null || true
ip netns del servo 2>/dev/null || true

# Create network namespaces for flight and servo.
ip netns add flight
ip netns add servo

# Create corresponding virtual interfaces.
ip link add flight link "$IFACE" type macvlan mode bridge
ip link add servo link "$IFACE" type macvlan mode bridge

# Move the virtual interfaces into their namespaces.
ip link set flight netns flight
ip link set servo netns servo

# Configure flight interface address and bring it up.
ip netns exec flight ip addr add 192.168.1.11/24 dev flight
ip netns exec flight ip link set flight up
ip netns exec flight ip link set lo up

# Configure servo interface address and bring it up.
ip netns exec servo ip addr add 192.168.1.10/24 dev servo
ip netns exec servo ip link set servo up
ip netns exec servo ip link set lo up

# Explicit flight routing rules allowing servo + ftel communication.
ip netns exec flight iptables -A OUTPUT -d 192.168.1.10 -j ACCEPT
ip netns exec flight iptables -A OUTPUT -d 192.168.1.132 -j ACCEPT
ip netns exec flight iptables -A OUTPUT -d 192.168.1.0/24 -j DROP

# Explicit servo routing rules allowing flight + gtel communication.
ip netns exec servo iptables -A OUTPUT -d 192.168.1.11 -j ACCEPT
ip netns exec servo iptables -A OUTPUT -d 192.168.1.140 -j ACCEPT
ip netns exec servo iptables -A OUTPUT -d 192.168.1.0/24 -j DROP

# Put the interface into promiscuous mode so it doesn't drop packets.
ip link set "$IFACE" promisc on
