#!/usr/bin/env bash
# One-time setup for the HITL test board's CM5. Safe to run again.
#
#   sudo ./setup-cm5.sh --key meerkat.pub   set up the board
#   ./setup-cm5.sh --check                  check the buses and the service
#
# --key is the Meerkat runner's public SSH key. CI logs in as the hitl user
# with it to copy a new build to /opt/hitl and restart the service.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
devices=(/dev/spidev0.0 /dev/spidev1.0 /dev/spidev5.0 /dev/i2c-1)

check() {
	local ok=0
	for device in "${devices[@]}"; do
		if [[ -e $device ]]; then
			echo "ok      $device"
		else
			echo "missing $device"
			ok=1
		fi
	done
	if systemctl is-active --quiet hitl.service; then
		echo "ok      hitl.service is running"
	else
		echo "missing hitl.service is not running"
		ok=1
	fi
	return $ok
}

key=""
while [[ $# -gt 0 ]]; do
	case $1 in
		--check)
			check
			exit
			;;
		--key)
			key=$2
			shift 2
			;;
		*)
			echo "usage: sudo $0 [--key meerkat.pub] | $0 --check" >&2
			exit 2
			;;
	esac
done

if [[ $EUID -ne 0 ]]; then
	echo "run with sudo" >&2
	exit 1
fi

# Buses. Each line is added once.
config=/boot/firmware/config.txt
[[ -f $config ]] || config=/boot/config.txt
reboot=0
for line in dtparam=i2c_arm=on dtoverlay=spi0-1cs dtoverlay=spi1-1cs dtoverlay=spi5-1cs; do
	if ! grep -qxF "$line" "$config"; then
		echo "$line" >>"$config"
		echo "added $line to $config"
		reboot=1
	fi
done

# The user the service runs as and CI deploys as.
for group in spi i2c gpio; do
	getent group "$group" >/dev/null || groupadd --system "$group"
done
if ! id hitl &>/dev/null; then
	useradd --create-home --shell /bin/bash --groups spi,i2c,gpio hitl
	echo "created user hitl"
fi
usermod --append --groups spi,i2c,gpio hitl

install -d -o hitl -g hitl /opt/hitl

if [[ -n $key ]]; then
	install -d -o hitl -g hitl -m 700 ~hitl/.ssh
	touch ~hitl/.ssh/authorized_keys
	if ! grep -qxF "$(cat "$key")" ~hitl/.ssh/authorized_keys; then
		cat "$key" >>~hitl/.ssh/authorized_keys
		echo "added the Meerkat key"
	fi
	chown hitl:hitl ~hitl/.ssh/authorized_keys
	chmod 600 ~hitl/.ssh/authorized_keys
fi

# CI may restart the service and nothing else.
cat >/etc/sudoers.d/hitl <<'EOF'
hitl ALL=(root) NOPASSWD: /usr/bin/systemctl restart hitl.service, /usr/bin/systemctl stop hitl.service
EOF
chmod 440 /etc/sudoers.d/hitl
visudo -cf /etc/sudoers.d/hitl >/dev/null

install -m 644 "$here/hitl.service" /etc/systemd/system/hitl.service
systemctl daemon-reload
systemctl enable hitl.service

if [[ ! -x /opt/hitl/hitl ]]; then
	echo "no build in /opt/hitl yet; the first CI run or a manual copy puts one there"
fi
if [[ $reboot -eq 1 ]]; then
	echo "reboot for the bus changes, then run $0 --check"
fi
