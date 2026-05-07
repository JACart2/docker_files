#!/usr/bin/env bash
set -euo pipefail

# Configure a static IP on the Velodyne (VLP-16) subnet and make it persistent.
# Defaults can be overridden via environment variables:
#   IFACE    (e.g., enp0s31f6)
#   HOST_IP  (e.g., 192.168.1.254/24)

HOST_IP="${HOST_IP:-192.168.1.254/24}"
IFACE="${IFACE:-}"

if [[ -z "$IFACE" ]]; then
  for devpath in /sys/class/net/*; do
    dev="$(basename "$devpath")"
    [[ "$dev" == "lo" ]] && continue
    [[ "$dev" == docker* ]] && continue
    [[ "$dev" == veth* ]] && continue
    [[ "$dev" == br-* ]] && continue
    [[ "$dev" == wl* ]] && continue
    [[ -f "/sys/class/net/$dev/type" ]] || continue
    if [[ "$(cat "/sys/class/net/$dev/type")" == "1" ]]; then
      IFACE="$dev"
      break
    fi
  done
fi

if [[ -z "$IFACE" ]]; then
  echo "ERROR: No ethernet interface found. Set IFACE explicitly." >&2
  exit 1
fi

# Remove any existing velodyne static connection so we can recreate it cleanly.
sudo nmcli con delete velodyne-static 2>/dev/null || true

sudo nmcli con add \
  type ethernet \
  con-name velodyne-static \
  ifname "$IFACE" \
  ipv4.method manual \
  ipv4.addresses "$HOST_IP" \
  ipv6.method ignore \
  connection.autoconnect yes

sudo nmcli con up velodyne-static

echo "Velodyne network configured on $IFACE with $HOST_IP (persistent via NetworkManager)."
