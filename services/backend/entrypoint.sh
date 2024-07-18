#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
source "/dev_ws/install/setup.bash" --
echo 'source "/opt/ros/humble/setup.bash" ' >> ~/.bashrc 
echo 'source "/dev_ws/install/setup.bash" ' >> ~/.bashrc 
# take in command line input and execute
echo "Successfully started JACart container. You can run a ros2 launch here."
exec "$@"