#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/jazzy/setup.bash" --
source "/opt/ros_ws/install/setup.bash" --
source "/dev_ws/install/setup.bash" --
echo 'source "/opt/ros/jazzy/setup.bash" ' >> ~/.bashrc 
echo 'source "/opt/ros_ws/install/setup.bash" ' >> ~/.bashrc 
echo 'source "/dev_ws/install/setup.bash" ' >> ~/.bashrc 
# take in command line input and execute

if [[ -z "$BACKEND_COMMAND" ]]; then
  exec "$@"
else
  bash -c "$BACKEND_COMMAND"
fi

