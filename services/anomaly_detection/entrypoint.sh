#!/bin/bash
source "/opt/ros/jazzy/setup.bash" --
if [ -f "/root/dev_ws/install/setup.bash" ]; then
    source "/root/dev_ws/install/setup.bash" --
fi


# if no ANOMALY_DETECTION_COMMAND, default to compose.yml CMD. Otherwise, use ANOMALY_DETECTION_COMMAND
if [[ -z "$ANOMALY_DETECTION_COMMAND" ]]; then
  exec "$@"
else
  exec bash -lc "$ANOMALY_DETECTION_COMMAND"
fi

