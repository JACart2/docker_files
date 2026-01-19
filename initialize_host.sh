# allow the containers to access the X server (so that rviz can be shown).
sudo xhost local:root
# assign static ip (to all ethernet interfaces) to recieve velodyne data. For more information see page 23, 4.2.1 Network Setup in Isolation of the Velodyne Puck (VLP-16) User Manual (https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf).
# sudo ip addr add 192.168.1.254/24 dev $(nmcli device status | awk '$2 == "ethernet" {print $1}') 


# --- Velodyne network configuration ---
# VLP-16 factory default IP: 192.168.1.201
# Host should be on same subnet (192.168.1.0/24), different IP
VEL_IP="192.168.1.201"
HOST_IP="192.168.1.254/24"
IFACE="enp0s31f6"

# Sanity check: interface exists
if ! ip link show "$IFACE" &>/dev/null; then
  echo "ERROR: Network interface $IFACE not found."
  exit 1
fi

# Assign (or replace) static IP on the ethernet interface
sudo ip addr replace "$HOST_IP" dev "$IFACE"
sudo ip link set "$IFACE" up

echo "Assigned $HOST_IP to $IFACE"

# Optional: verify connectivity if sensor is plugged in
if ping -c 1 -W 1 "$VEL_IP" &>/dev/null; then
  echo "Velodyne reachable at $VEL_IP"
else
  echo "WARNING: Velodyne not reachable at $VEL_IP (is it powered and plugged in?)"
fi

echo "Host initialization complete."
