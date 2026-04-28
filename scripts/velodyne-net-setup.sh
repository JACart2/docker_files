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

sudo tee /usr/local/bin/velodyne-net.sh >/dev/null <<EOF
#!/usr/bin/env bash
set -euo pipefail
IFACE="$IFACE"
HOST_IP="$HOST_IP"

/sbin/ip addr replace "$HOST_IP" dev "$IFACE"
/sbin/ip link set "$IFACE" up
EOF

sudo chmod +x /usr/local/bin/velodyne-net.sh

sudo tee /etc/systemd/system/velodyne-net.service >/dev/null <<'EOF'
[Unit]
Description=Configure Velodyne network interface
After=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/velodyne-net.sh
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now velodyne-net.service

echo "Velodyne network configured on $IFACE with $HOST_IP (persistent)."
