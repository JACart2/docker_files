#!/bin/bash
export OA_SECRET=$(cat /mnt/OA_SECRET.txt)

echo "OA_SECRET loaded (length: ${#OA_SECRET})"

# setup ros2 environment
source "/opt/ros/jazzy/setup.bash" 
source "/dev_ws/install/setup.bash" 

# if no FER_COMMAND, default to compose.yml CMD. Otherwise, use FER_COMMAND
if [[ -z "$FER_COMMAND" ]]; then
  exec "$@"
else
  exec bash -lc "$FER_COMMAND"
fi

