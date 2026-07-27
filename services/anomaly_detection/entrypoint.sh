#!/bin/bash
set -e

source "/opt/ros/jazzy/setup.bash" --
source "/opt/ros_ws/install/setup.bash" --
if [ -f "/root/dev_ws/install/setup.bash" ]; then
    source "/root/dev_ws/install/setup.bash" --
fi

if [[ "${ANOMALY_AUTOSTART:-true}" != "true" ]]; then
  echo "Anomaly detection autostart is disabled; container is ready for manual commands."
  exec tail -f /dev/null
fi

if [[ -n "${ROS_BAG_PATH:-}" ]]; then
  if [[ ! -e "$ROS_BAG_PATH" ]]; then
    echo "Configured ROS bag does not exist: $ROS_BAG_PATH" >&2
    exit 1
  fi

  bag_args=("$ROS_BAG_PATH" "--rate" "${ROS_BAG_RATE:-1.0}")
  if [[ "${ROS_BAG_LOOP:-true}" == "true" ]]; then
    bag_args+=("--loop")
  fi

  ros2 bag play "${bag_args[@]}" &
  bag_pid=$!
  if kill -0 "$bag_pid" 2>/dev/null; then
    echo "ROS bag started: $ROS_BAG_PATH (pid=$bag_pid)"
  else
    echo "ROS bag failed to start: $ROS_BAG_PATH" >&2
    exit 1
  fi
fi


# if no ANOMALY_DETECTION_COMMAND, default to compose.yml CMD. Otherwise, use ANOMALY_DETECTION_COMMAND
if [[ -z "$ANOMALY_DETECTION_COMMAND" ]]; then
  exec "$@"
else
  exec bash -lc "$ANOMALY_DETECTION_COMMAND"
fi
