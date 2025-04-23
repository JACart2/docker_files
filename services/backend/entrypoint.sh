#!/bin/bash
set -e

# Primary ROS environment
source "/opt/ros/humble/setup.bash"

# Source dependency workspace first
if [ -f /dependency_ws/install/setup.bash ]; then
  source "/dependency_ws/install/setup.bash"
else
  echo "⚠️ /dependency_ws/install/setup.bash not found! Some dependencies may be missing." >&2
fi

# Build and source dev_ws
cd /dev_ws
if [ ! -f install/setup.bash ]; then
  echo "⚙️ Building dev workspace..."
  rosdep update >/dev/null
  rosdep install --from-paths src --ignore-src -r -y >/dev/null
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

if [ -f install/setup.bash ]; then
  source "install/setup.bash"
else
  echo "❌ /dev_ws/install/setup.bash not found after build!" >&2
  exit 1
fi

# Verify LIO-SAM is found
if ! ros2 pkg list | grep -q lio_sam; then
  echo "❌ LIO-SAM package not found in workspaces!" >&2
  echo "Workspace paths:" >&2
  echo "$AMENT_PREFIX_PATH" | tr ':' '\n' >&2
  exit 1
fi

# Environment persistence
{
  echo 'source "/opt/ros/humble/setup.bash"'
  echo 'source "/dev_ws/install/setup.bash"'
  echo "export LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/dev_ws/install/lib:\$LD_LIBRARY_PATH"
} >> ~/.bashrc

# UDEV rules (keep your existing)
PKG="/dev_ws/src/ai-navigation/motor_control"
cp "${PKG}/resource/99-usb-serial.rules" "/etc/udev/rules.d"
pip install -r ${PKG}/resource/requirements.txt >/dev/null

# Execute command
if [[ -n "$BACKEND_COMMAND" ]]; then
  echo "🚀 Executing custom command: $BACKEND_COMMAND"
  exec bash -c "$BACKEND_COMMAND"
else
  echo "🚀 Executing default command: $@"
  exec "$@"
fi