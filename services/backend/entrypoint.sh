#!/bin/bash
set -e

# IMPORTANT DEV NOTE - THE IMAGE NEEDS TO BE REBUILT IN ORDER FOR CHANGES HERE TO BE REFLECTED

# Primary ROS environment
source "/opt/ros/humble/setup.bash"

# Source dependency workspace first
source "/dependency_ws/install/setup.bash"

# Build and source dev_ws
cd /dev_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source "install/setup.bash"

# Add to bashrc for easier development inside container
{
  echo 'source "/opt/ros/humble/setup.bash"'
  echo 'source "/dev_ws/install/setup.bash"'
} >> ~/.bashrc

# Execute command
if [[ -n "$BACKEND_COMMAND" ]]; then
  echo "🚀 Executing custom command: $BACKEND_COMMAND"
  exec bash -c "$BACKEND_COMMAND"
else
  echo "🚀 Executing default command: $@"
  exec "$@"
fi
