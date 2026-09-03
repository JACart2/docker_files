#!/bin/bash
set -euo pipefail

# Usage:
#   ./dev-run-anomaly-detection.sh
#   ./dev-run-anomaly-detection.sh james
#   ./dev-run-anomaly-detection.sh madison
#   ./dev-run-anomaly-detection.sh mycart 7
#   CART_NAME=mycart ROS_DOMAIN_ID=7 ./dev-run-anomaly-detection.sh
#
# Execution Modes (ANOMALY_MODE):
#   ANOMALY_MODE=bag_ui ./dev-run-anomaly-detection.sh   # Run Bag Recorder UI & launch Chrome on :5000
#   ANOMALY_MODE=node   ./dev-run-anomaly-detection.sh   # Run standard anomaly detection node
#   ANOMALY_MODE=launch ./dev-run-anomaly-detection.sh   # Run full anomaly detection launchfile
#
# Optional:
#   ANOMALY_AUTOSTART=true ./dev-run-anomaly-detection.sh
#   RECORDER_PORT=5000 ./dev-run-anomaly-detection.sh
#
# Dashboard overrides:
#   SERVER_IP=10.247.225.41 API_PORT=8000 ./dev-run-anomaly-detection.sh
#   DASHBOARD_SCHEME=http ./dev-run-anomaly-detection.sh

source ./dashboard-api.sh

dashboard_configure "$@"
dashboard_start_registration

RECORDER_PORT="${RECORDER_PORT:-5000}"

echo "Starting anomaly detection with:"
echo "  CART_NAME=${CART_NAME}"
echo "  CART_ID=${CART_ID}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  DASHBOARD_ROOT=${DASHBOARD_ROOT}"
echo "  CART_PORT=${CART_PORT}"
echo "  RECORDER_PORT=${RECORDER_PORT}"

###############################################################################
# Background-process cleanup
###############################################################################

BROWSER_PID=""

cleanup() {
  echo
  echo "Cleaning up launcher background processes..."

  if [ -n "${BROWSER_PID:-}" ]; then
    kill "$BROWSER_PID" 2>/dev/null || true
    wait "$BROWSER_PID" 2>/dev/null || true
  fi

  dashboard_stop_registration
}

trap cleanup EXIT
trap 'exit 130' SIGINT SIGTERM

###############################################################################
# Host initialization
###############################################################################

bash ./initialize_host.sh

###############################################################################
# Chrome / Browser Helper
###############################################################################

wait_for_service() {
  local port="$1"

  until curl -fsS \
    --connect-timeout 2 \
    --max-time 5 \
    "http://localhost:${port}" \
    >/dev/null 2>&1
  do
    sleep 1
  done
}

open_chrome_when_ready() {
  local port="$1"
  local target_url="http://localhost:${port}"

  wait_for_service "$port"

  echo "Launching browser for Bag Recorder UI at ${target_url}..."

  if command -v google-chrome >/dev/null 2>&1; then
    google-chrome --new-window "$target_url" >/dev/null 2>&1 &
  elif command -v google-chrome-stable >/dev/null 2>&1; then
    google-chrome-stable --new-window "$target_url" >/dev/null 2>&1 &
  elif command -v chromium-browser >/dev/null 2>&1; then
    chromium-browser --new-window "$target_url" >/dev/null 2>&1 &
  elif command -v chromium >/dev/null 2>&1; then
    chromium --new-window "$target_url" >/dev/null 2>&1 &
  elif command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$target_url" >/dev/null 2>&1 &
  elif command -v open >/dev/null 2>&1; then
    open "$target_url" >/dev/null 2>&1 &
  else
    echo "========================================================"
    echo " Bag Recorder UI ready at: ${target_url}"
    echo "========================================================"
  fi
}

###############################################################################
# Docker Compose configuration
###############################################################################

check_internet() {
  curl -fsS \
    --head \
    --connect-timeout 3 \
    --max-time 5 \
    https://auth.docker.io \
    >/dev/null 2>&1
}

COMPOSE_FLAGS=(
  "--remove-orphans"
  "--force-recreate"
)

if check_internet; then
  echo "Internet detected: Docker images may be rebuilt."
  COMPOSE_FLAGS+=("--build")
else
  echo "No internet detected: using locally cached Docker images."
  COMPOSE_FLAGS+=("--pull" "never")
fi

###############################################################################
# Execution Mode & Command Setup
###############################################################################

# Default mode is bag_ui for standalone recording without LLM analysis
ANOMALY_MODE="${ANOMALY_MODE:-bag_ui}"

# Autostart by default for launch and bag_ui modes
if [ "$ANOMALY_MODE" = "launch" ] || [ "$ANOMALY_MODE" = "bag_ui" ] || [ "$ANOMALY_MODE" = "recorder" ]; then
  ANOMALY_AUTOSTART="${ANOMALY_AUTOSTART:-true}"
else
  ANOMALY_AUTOSTART="${ANOMALY_AUTOSTART:-false}"
fi

ANOMALY_PACKAGE_PATHS=(
  "src/anomaly_detection/anomaly_msg"
  "src/anomaly_detection/anomaly_detection"
  "src/anomaly_detection/tester"
)

ANOMALY_BASE_PATHS="${ANOMALY_PACKAGE_PATHS[*]}"

ANOMALY_BUILD_AND_SOURCE="\
cd /root/dev_ws && \
colcon build \
  --symlink-install \
  --base-paths ${ANOMALY_BASE_PATHS} && \
source install/setup.bash"

BAG_RECORDER_UI_NODE="\
${ANOMALY_BUILD_AND_SOURCE} && \
ros2 run anomaly_detection bag_recorder_ui_node"

ANOMALY_DETECTION_NODE="\
${ANOMALY_BUILD_AND_SOURCE} && \
ros2 run anomaly_detection anomaly_detection_node"

ANOMALY_DETECTION_LAUNCH="\
${ANOMALY_BUILD_AND_SOURCE} && \
ros2 launch anomaly_detection anomaly_detection.launch.py"

case "$ANOMALY_AUTOSTART" in
  true)
    case "$ANOMALY_MODE" in
      bag_ui|recorder)
        ANOMALY_COMMAND="$BAG_RECORDER_UI_NODE"
        ;;
      node)
        ANOMALY_COMMAND="$ANOMALY_DETECTION_NODE"
        ;;
      launch)
        ANOMALY_COMMAND="$ANOMALY_DETECTION_LAUNCH"
        ;;
      *)
        echo "Invalid ANOMALY_MODE '${ANOMALY_MODE}'."
        echo "Valid values are: 'bag_ui', 'node', 'launch'."
        exit 1
        ;;
    esac
    ;;
  false)
    ANOMALY_COMMAND=""
    ;;
  *)
    echo "Invalid ANOMALY_AUTOSTART '${ANOMALY_AUTOSTART}'."
    echo "Valid values are 'true' and 'false'."
    exit 1
    ;;
esac

export ANOMALY_AUTOSTART
export ANOMALY_MODE
export ANOMALY_DETECTION_COMMAND="$ANOMALY_COMMAND"

echo "  ANOMALY_AUTOSTART=${ANOMALY_AUTOSTART}"
echo "  ANOMALY_MODE=${ANOMALY_MODE}"
if [ "$ANOMALY_AUTOSTART" = "true" ]; then
  echo "  ANOMALY_COMMAND=${ANOMALY_COMMAND}"
else
  echo "  ANOMALY_COMMAND=disabled (run commands manually in the container)"
fi

###############################################################################
# Start Container & Launch Browser
###############################################################################

docker compose up \
  -d \
  "${COMPOSE_FLAGS[@]}" \
  anomaly_detection

if [ "$ANOMALY_AUTOSTART" = "true" ] && { [ "$ANOMALY_MODE" = "bag_ui" ] || [ "$ANOMALY_MODE" = "recorder" ]; }; then
  open_chrome_when_ready "$RECORDER_PORT" &
  BROWSER_PID=$!
elif [ "$ANOMALY_AUTOSTART" = "true" ] && [ "$ANOMALY_MODE" = "launch" ]; then
  open_chrome_when_ready 5001 &
  BROWSER_PID=$!
fi

###############################################################################
# Attach Interactive Terminal
###############################################################################

###############################################################################
# Attach Interactive Terminal
###############################################################################

docker compose exec \
  -it \
  -w /root/dev_ws \
  anomaly_detection \
  bash -c '
    source /opt/ros/jazzy/setup.bash
    source /opt/ros_ws/install/setup.bash

    if [ -f /root/dev_ws/install/setup.bash ]; then
      source /root/dev_ws/install/setup.bash
    fi

    exec bash
  '