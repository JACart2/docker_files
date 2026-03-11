#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/jazzy/setup.bash" --
source "/opt/ros_ws/install/setup.bash" --
if [ -f "/root/dev_ws/install/setup.bash" ]; then
    source "/root/dev_ws/install/setup.bash" --
fi

# If the mounted dev workspace exists but its setup files don't register
# cart_launch, rebuild it in-container so ROS can discover the package.
if [ -d "/root/dev_ws/src/ai-navigation/cart_control/cart_launch" ] && ! ros2 pkg prefix cart_launch >/dev/null 2>&1; then
    echo "cart_launch not found in the current ROS environment. Rebuilding /root/dev_ws..."
    cd /root/dev_ws
    colcon build --symlink-install --base-paths src
    source "/root/dev_ws/install/setup.bash" --
fi

echo 'source "/opt/ros/jazzy/setup.bash"' >> ~/.bashrc 
echo 'source "/opt/ros_ws/install/setup.bash"' >> ~/.bashrc 
echo 'if [ -f "/root/dev_ws/install/setup.bash" ]; then source "/root/dev_ws/install/setup.bash"; fi' >> ~/.bashrc 
# take in command line input and execute

if [[ -z "$BACKEND_COMMAND" ]]; then
  exec "$@"
else
  bash -c "$BACKEND_COMMAND"
fi
