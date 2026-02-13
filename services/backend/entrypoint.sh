#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/jazzy/setup.bash" --
source "/opt/ros_ws/install/setup.bash" --
if [ -f "/root/dev_ws/install/setup.bash" ]; then
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

