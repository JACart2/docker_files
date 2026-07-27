#!/bin/bash
set -euo pipefail

# Usage:
#   ./dev-run-anomaly-detection.sh
#   ./dev-run-anomaly-detection.sh james
#   ./dev-run-anomaly-detection.sh madison
#   ./dev-run-anomaly-detection.sh mycart 7
#   CART_NAME=mycart ROS_DOMAIN_ID=7 ./dev-run-anomaly-detection.sh
#
# Optional:
#   ANOMALY_AUTOSTART=true ./dev-run-anomaly-detection.sh
#   ANOMALY_MODE=node ./dev-run-anomaly-detection.sh
#   ANOMALY_MODE=launch ./dev-run-anomaly-detection.sh
#
# Dashboard overrides:
#   SERVER_IP=10.247.225.41 API_PORT=8000 ./dev-run-anomaly-detection.sh
#   DASHBOARD_SCHEME=http ./dev-run-anomaly-detection.sh

source ./dashboard-api.sh

dashboard_configure "$@"
dashboard_start_registration

echo "Starting anomaly detection with:"
echo "  CART_NAME=${CART_NAME}"
echo "  CART_ID=${CART_ID}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  DASHBOARD_ROOT=${DASHBOARD_ROOT}"
echo "  CART_PORT=${CART_PORT}"

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
# Browser helper
###############################################################################

wait_for_anomaly_frontend() {
  local port="$1"

  until curl -fsS \
    --connect-timeout 2 \
    --max-time 5 \
    "http://localhost:${port}" \
    >/dev/null 2>&1
  do
    sleep 2
  done
}

open_browser_when_ready() {
  local port="$1"
  local frontend_url="http://localhost:${port}"

  wait_for_anomaly_frontend "$port"

  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$frontend_url" >/dev/null 2>&1 || true
  elif command -v open >/dev/null 2>&1; then
    open "$frontend_url" >/dev/null 2>&1 || true
  else
    echo "Anomaly interface is available at:"
    echo "  ${frontend_url}"
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
# Anomaly detection command
###############################################################################

ANOMALY_AUTOSTART="${ANOMALY_AUTOSTART:-false}"

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

ANOMALY_DETECTION_NODE="\
${ANOMALY_BUILD_AND_SOURCE} && \
ros2 run anomaly_detection anomaly_detection_node"

ANOMALY_DETECTION_LAUNCH="\
${ANOMALY_BUILD_AND_SOURCE} && \
ros2 launch anomaly_detection anomaly_detection.launch.py"

ANOMALY_MODE="${ANOMALY_MODE:-launch}"

case "$ANOMALY_AUTOSTART" in
  true)
    case "$ANOMALY_MODE" in
      node)
        ANOMALY_COMMAND="$ANOMALY_DETECTION_NODE"
        ;;
      launch)
        ANOMALY_COMMAND="$ANOMALY_DETECTION_LAUNCH"
        ;;
      *)
        echo "Invalid ANOMALY_MODE '${ANOMALY_MODE}'."
        echo "Valid values are 'node' and 'launch'."
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
# Start anomaly detection
###############################################################################

docker compose up \
  -d \
  "${COMPOSE_FLAGS[@]}" \
  anomaly_detection

if [ "$ANOMALY_AUTOSTART" = "true" ] && [ "$ANOMALY_MODE" = "launch" ]; then
  open_browser_when_ready 5000 &
  BROWSER_PID=$!
fi

###############################################################################
# Open VS Code attached to the anomaly container
###############################################################################

if command -v code >/dev/null 2>&1; then
  CONTAINER_ID="$(
    docker compose ps -q anomaly_detection
  )"

  if [ -n "$CONTAINER_ID" ]; then
    CONTAINER_NAME="$(
      docker inspect \
        --format '{{.Name}}' \
        "$CONTAINER_ID" |
        sed 's|^/||'
    )"

    if [ -n "$CONTAINER_NAME" ]; then
      HEX_NAME="$(
        printf '%s' "$CONTAINER_NAME" |
          od -A n -t x1 |
          tr -d ' \n'
      )"

      URI="vscode-remote://attached-container+${HEX_NAME}/root/dev_ws"

      echo "Opening VS Code attached to ${CONTAINER_NAME}..."
      code --folder-uri "$URI"
    fi
  fi
fi

###############################################################################
# Attach terminal
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
