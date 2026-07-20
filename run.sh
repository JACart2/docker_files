#!/bin/bash

# Usage:
#   ./run.sh                         # defaults to james, ROS_DOMAIN_ID 0
#   ./run.sh james                   # james, ROS_DOMAIN_ID 0
#   ./run.sh madison                 # madison, ROS_DOMAIN_ID 1
#   ./run.sh mycart 7                # mycart, ROS_DOMAIN_ID 7
#   CART_NAME=mycart ROS_DOMAIN_ID=7 ./run.sh
#   ENABLE_ANOMALY_DETECTION=true ./run.sh  # also start the full AAD/LLM service

# Config (override via env / args)
CART_NAME="${CART_NAME:-${1:-james}}"
CLI_ROS_DOMAIN_ID="${2:-}"

SERVER_IP="${SERVER_IP:-10.247.225.41}"   # Dashboard server
CART_PORT="${CART_PORT:-9090}"
API_PORT="${API_PORT:-8000}"
DASHBOARD_SCHEME="${DASHBOARD_SCHEME:-https}"
ENABLE_ANOMALY_DETECTION="${ENABLE_ANOMALY_DETECTION:-false}"

case "${ENABLE_ANOMALY_DETECTION,,}" in
  true|1|yes|on)
    ENABLE_ANOMALY_DETECTION=true
    ;;
  false|0|no|off)
    ENABLE_ANOMALY_DETECTION=false
    ;;
  *)
    echo "Invalid ENABLE_ANOMALY_DETECTION '${ENABLE_ANOMALY_DETECTION}'."
    echo "Use true or false."
    exit 1
    ;;
esac

# Normalize only for matching known cart defaults.
# Keep CART_NAME itself as the actual display/registration name.
CART_NAME_LOWER="$(printf '%s' "$CART_NAME" | tr '[:upper:]' '[:lower:]')"

# Pick ROS_DOMAIN_ID in this priority:
# 1. Existing ROS_DOMAIN_ID env var
# 2. Second CLI argument
# 3. Known cart default
# 4. Fallback default
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

# Validate ROS_DOMAIN_ID is numeric
if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]]; then
  echo "Invalid ROS_DOMAIN_ID '${ROS_DOMAIN_ID}'. It must be a number."
  exit 1
fi

# Optional: ROS 2 domain IDs are commonly kept in the 0-232 range.
if (( ROS_DOMAIN_ID < 0 || ROS_DOMAIN_ID > 232 )); then
  echo "Invalid ROS_DOMAIN_ID '${ROS_DOMAIN_ID}'. Use a value between 0 and 232."
  exit 1
fi

# CART_ID defaults to CART_NAME unless explicitly overridden
CART_ID="${CART_ID:-$CART_NAME}"

export CART_NAME
export CART_ID
export ROS_DOMAIN_ID
export SERVER_IP
export API_PORT
export CART_PORT
export DASHBOARD_SCHEME
export ENABLE_ANOMALY_DETECTION

DASHBOARD_ROOT="${DASHBOARD_SCHEME}://${SERVER_IP}:${API_PORT}"

# Variables passed into the Vite frontend
export VITE_CART_NAME="${CART_NAME}"
export VITE_DASHBOARD_API_ROOT="${DASHBOARD_ROOT}/"

echo "Starting cart UI/helper with:"
echo "  CART_NAME=${CART_NAME}"
echo "  CART_ID=${CART_ID}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  DASHBOARD_ROOT=${DASHBOARD_ROOT}"
echo "  ENABLE_ANOMALY_DETECTION=${ENABLE_ANOMALY_DETECTION}"

#Termination signal to run.sh cleans all child processes
cleanup() {
  echo "Cleaning up..."
  kill $COMPOSE_PID 2>/dev/null || true
  kill %% 2>/dev/null || true  # Kill background jobs
  exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# How often to *probe* the dashboard when it's down / between attempts
REREGISTER_INTERVAL_SEC="${REREGISTER_INTERVAL_SEC:-15}"
# Minimum time between successful registrations
REGISTER_COOLDOWN_SEC="${REGISTER_COOLDOWN_SEC:-15}"   # 15 seconds

bash ./initialize_host.sh

wait_for_frontend () {
  until curl -fsS http://localhost:5173 >/dev/null 2>&1; do
    sleep 2
  done
}

dashboard_up () {
  # -k allows self-signed HTTPS certificates
  curl -k -fsS "${DASHBOARD_ROOT}/" >/dev/null 2>&1
}

register_cart () {
  # -k allows self-signed HTTPS certificates
  curl -k -fsS -X POST "${DASHBOARD_ROOT}/api/vehicles/register" \
    -H "Content-Type: application/json" \
    -d "{\"name\":\"${CART_NAME}\",\"port\":${CART_PORT}}" >/dev/null 2>&1 || true
}

reregister_loop () {
  local last_ok=0
  while true; do
    if dashboard_up; then
      local now
      now="$(date +%s)"

      # only re-register if we haven't done so recently
      if (( now - last_ok >= REGISTER_COOLDOWN_SEC )); then
        register_cart
        last_ok="$now"
      fi
    fi

    sleep "$REREGISTER_INTERVAL_SEC"
  done
}

# Start the cart and browser UI. The autonomy launch already provides rosbridge
# and anomaly-producing/logging nodes; the LLM-backed AAD service is optional.
COMPOSE_SERVICES=(backend frontend)

if [ "$ENABLE_ANOMALY_DETECTION" = true ]; then
  COMPOSE_SERVICES+=(anomaly_detection)
else
  # Stop a service left running by an earlier full-AI launch. Merely omitting a
  # Compose service does not stop an already-running container.
  docker compose stop anomaly_detection >/dev/null 2>&1 || true
fi

docker compose up "${COMPOSE_SERVICES[@]}" --build --remove-orphans --force-recreate &
COMPOSE_PID=$!

# Wait for frontend then open browser
wait_for_frontend
open "http://localhost:5173"

# Start periodic re-registration in background
reregister_loop &

# Keep attached to compose
wait "$COMPOSE_PID"
