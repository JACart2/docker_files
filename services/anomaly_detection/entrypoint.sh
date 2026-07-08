#!/bin/bash
source "/opt/ros/jazzy/setup.bash" --
if [ -f "/opt/ros_ws/install/setup.bash" ]; then
    source "/opt/ros_ws/install/setup.bash" --
fi
if [ -f "/root/dev_ws/install/setup.bash" ]; then
    source "/root/dev_ws/install/setup.bash" --
fi
grep -qxF 'source "/opt/ros/jazzy/setup.bash"' ~/.bashrc || \
    echo 'source "/opt/ros/jazzy/setup.bash"' >> ~/.bashrc
grep -qxF 'if [ -f "/opt/ros_ws/install/setup.bash" ]; then source "/opt/ros_ws/install/setup.bash"; fi' ~/.bashrc || \
    echo 'if [ -f "/opt/ros_ws/install/setup.bash" ]; then source "/opt/ros_ws/install/setup.bash"; fi' >> ~/.bashrc
grep -qxF 'if [ -f "/root/dev_ws/install/setup.bash" ]; then source "/root/dev_ws/install/setup.bash"; fi' ~/.bashrc || \
    echo 'if [ -f "/root/dev_ws/install/setup.bash" ]; then source "/root/dev_ws/install/setup.bash"; fi' >> ~/.bashrc

if [ -e "/root/dev_ws/src/anomaly_detection/COLCON_IGNORE" ]; then
    rm -f "/root/dev_ws/src/anomaly_detection/COLCON_IGNORE"
fi


# if no ANOMALY_DETECTION_COMMAND, default to compose.yml CMD. Otherwise, use ANOMALY_DETECTION_COMMAND
if [[ -z "$ANOMALY_DETECTION_COMMAND" ]]; then
  exec "$@"
else
  exec bash -lc "$ANOMALY_DETECTION_COMMAND"
fi
