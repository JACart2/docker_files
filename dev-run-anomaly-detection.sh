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
#   ANOMALY_MODE=node ./dev-run-anomaly-detection.sh
#   ANOMALY_MODE=launch ./dev-run-anomaly-detection.sh
#
# Dashboard overrides:
#   SERVER_IP=10.247.225.41 API_PORT=8000 ./dev-run-anomaly-detection.sh
#   DASHBOARD_SCHEME=http ./dev-run-anomaly-detection.sh

###############################################################################
# Cart and dashboard configuration
###############################################################################

CART_NAME="${CART_NAME:-${1:-james}}"
CLI_ROS_DOMAIN_ID="${2:-}"

SERVER_IP="${SERVER_IP:-10.247.225.41}"
CART_PORT="${CART_PORT:-9090}"
API_PORT="${API_PORT:-8000}"
DASHBOARD_SCHEME="${DASHBOARD_SCHEME:-https}"

CART_NAME_LOWER="$(
  printf '%s' "$CART_NAME" |
    tr '[:upper:]' '[:lower:]'
)"

# ROS_DOMAIN_ID priority:
#   1. Existing ROS_DOMAIN_ID environment variable
#   2. Second command-line argument
#   3. Known cart default
#   4. Domain 0 fallback
if [ -n "${ROS_DOMAIN_ID:-}" ]; then
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"
elif [ -n "$CLI_ROS_DOMAIN_ID" ]; then
  ROS_DOMAIN_ID="$CLI_ROS_DOMAIN_ID"
else
  case "$CART_NAME_LOWER" in
    james)
      ROS_DOMAIN_ID="0"
      ;;
    madison)
      ROS_DOMAIN_ID="1"
      ;;
    *)
      ROS_DOMAIN_ID="0"
      ;;
  esac
fi

if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]]; then
  echo "Invalid ROS_DOMAIN_ID '${ROS_DOMAIN_ID}'. It must be a number."
  exit 1
fi

if (( ROS_DOMAIN_ID < 0 || ROS_DOMAIN_ID > 232 )); then
  echo "Invalid ROS_DOMAIN_ID '${ROS_DOMAIN_ID}'. Use a value between 0 and 232."
  exit 1
fi

if ! [[ "$CART_PORT" =~ ^[0-9]+$ ]]; then
  echo "Invalid CART_PORT '${CART_PORT}'. It must be a number."
  exit 1
fi

if ! [[ "$API_PORT" =~ ^[0-9]+$ ]]; then
  echo "Invalid API_PORT '${API_PORT}'. It must be a number."
  exit 1
fi

CART_ID="${CART_ID:-$CART_NAME}"

DASHBOARD_ROOT="${DASHBOARD_SCHEME}://${SERVER_IP}:${API_PORT}"

export CART_NAME
export CART_ID
export ROS_DOMAIN_ID
export SERVER_IP
export CART_PORT
export API_PORT
export DASHBOARD_SCHEME
export DASHBOARD_ROOT

# Used by the Vite frontend when a frontend service is built from this
# Compose project.
export VITE_CART_NAME="$CART_NAME"
export VITE_DASHBOARD_API_ROOT="${DASHBOARD_ROOT}/"

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
REREGISTER_PID=""

cleanup() {
  echo
  echo "Cleaning up launcher background processes..."

  if [ -n "$BROWSER_PID" ]; then
    kill "$BROWSER_PID" 2>/dev/null || true
  fi

  if [ -n "$REREGISTER_PID" ]; then
    kill "$REREGISTER_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT
trap 'exit 130' SIGINT SIGTERM

###############################################################################
# Host initialization
###############################################################################

bash ./initialize_host.sh

###############################################################################
# Dashboard registration
###############################################################################

REREGISTER_INTERVAL_SEC="${REREGISTER_INTERVAL_SEC:-15}"
REGISTER_COOLDOWN_SEC="${REGISTER_COOLDOWN_SEC:-15}"

dashboard_up() {
  # -k allows the dashboard's self-signed HTTPS certificate.
  curl -k -fsS \
    --connect-timeout 3 \
    --max-time 5 \
    "${DASHBOARD_ROOT}/" \
    >/dev/null 2>&1
}

register_cart() {
  local payload

  payload="$(
    printf \
      '{"name":"%s","port":%s}' \
      "$CART_NAME" \
      "$CART_PORT"
  )"

  curl -k -fsS \
    --connect-timeout 3 \
    --max-time 5 \
    -X POST \
    "${DASHBOARD_ROOT}/api/vehicles/register" \
    -H "Content-Type: application/json" \
    -d "$payload" \
    >/dev/null 2>&1
}

reregister_loop() {
  local last_ok=0
  local dashboard_was_up=false

  while true; do
    if dashboard_up; then
      local now
      now="$(date +%s)"

      if [ "$dashboard_was_up" = false ]; then
        echo "[Dashboard] Connected to ${DASHBOARD_ROOT}"
        dashboard_was_up=true
      fi

      if (( now - last_ok >= REGISTER_COOLDOWN_SEC )); then
        if register_cart; then
          last_ok="$now"
        else
          echo "[Dashboard] Cart registration failed"
        fi
      fi
    else
      if [ "$dashboard_was_up" = true ]; then
        echo "[Dashboard] Connection lost"
      fi

      dashboard_was_up=false
    fi

    sleep "$REREGISTER_INTERVAL_SEC"
  done
}

reregister_loop &
REREGISTER_PID=$!

###############################################################################
# Browser helper
###############################################################################

wait_for_anomaly_frontend() {
  local port="$1"

  until curl -fsS \
    --connect-timeout 2 \
    "http://localhost:${port}" \
    >/dev/null 2>&1
  do
    sleep 2
  done
}

open_browser_when_ready() {
  local port="$1"

  wait_for_anomaly_frontend "$port"

  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "http://localhost:${port}" >/dev/null 2>&1
  elif command -v open >/dev/null 2>&1; then
    open "http://localhost:${port}"
  else
    echo "Anomaly interface is available at:"
    echo "  http://localhost:${port}"
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

export ANOMALY_MODE
export ANOMALY_DETECTION_COMMAND="$ANOMALY_COMMAND"

echo "  ANOMALY_MODE=${ANOMALY_MODE}"
echo "  ANOMALY_COMMAND=${ANOMALY_COMMAND}"

###############################################################################
# Start anomaly detection
###############################################################################

docker compose up \
  -d \
  "${COMPOSE_FLAGS[@]}" \
  anomaly_detection

if [ "$ANOMALY_MODE" = "launch" ]; then
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