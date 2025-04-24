#!/bin/sh
export OA_SECRET=$(cat /mnt/OA_SECRET.txt)

echo "OA_SECRET is set to: $OA_SECRET"

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
source "/dev_ws/install/setup.bash" --
echo 'source "/opt/ros/humble/setup.bash" ' >> ~/.bashrc 
echo 'source "/dev_ws/install/setup.bash" ' >> ~/.bashrc 

# take in command line input and execute

# if no FER_COMMAND, default to compose.yml CMD. Otherwise, use FER_COMMAND
if [[ -z "$FER_COMMAND" ]]; then
  exec "$@"
else
  bash -c "$FER_COMMAND"
fi

