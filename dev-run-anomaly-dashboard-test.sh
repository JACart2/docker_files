#!/usr/bin/env bash
set -euo pipefail

# Run rosbridge plus synthetic anomaly logging/alert publishers without the
# anomaly detector or autonomy stack.
#
# Usage:
#   ./dev-run-anomaly-dashboard-test.sh
#   ./dev-run-anomaly-dashboard-test.sh madison
#   ./dev-run-anomaly-dashboard-test.sh mycart 7
#
# Environment overrides:
#   CART_NAME, ROS_DOMAIN_ID, CART_PORT, SERVER_IP, API_PORT,
#   DASHBOARD_SCHEME, AAD_TEST_PUBLISH_PERIOD

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CART_NAME="${CART_NAME:-${1:-james}}"
CLI_ROS_DOMAIN_ID="${2:-}"
CART_NAME_LOWER="$(printf '%s' "$CART_NAME" | tr '[:upper:]' '[:lower:]')"

if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
elif [[ -n "$CLI_ROS_DOMAIN_ID" ]]; then
  ROS_DOMAIN_ID="$CLI_ROS_DOMAIN_ID"
elif [[ "$CART_NAME_LOWER" == "madison" ]]; then
  ROS_DOMAIN_ID="1"
else
  ROS_DOMAIN_ID="0"
fi

CART_ID="${CART_ID:-$CART_NAME}"
CART_PORT="${CART_PORT:-9090}"
SERVER_IP="${SERVER_IP:-10.247.225.41}"
API_PORT="${API_PORT:-8000}"
DASHBOARD_SCHEME="${DASHBOARD_SCHEME:-https}"
AAD_TEST_PUBLISH_PERIOD="${AAD_TEST_PUBLISH_PERIOD:-2.0}"
DASHBOARD_ROOT="${DASHBOARD_SCHEME}://${SERVER_IP}:${API_PORT}"

if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || ((ROS_DOMAIN_ID < 0 || ROS_DOMAIN_ID > 232)); then
  echo "Invalid ROS_DOMAIN_ID '$ROS_DOMAIN_ID'; expected an integer from 0 to 232." >&2
  exit 2
fi
if ! [[ "$CART_PORT" =~ ^[0-9]+$ ]] || ((CART_PORT < 1 || CART_PORT > 65535)); then
  echo "Invalid CART_PORT '$CART_PORT'; expected an integer from 1 to 65535." >&2
  exit 2
fi
if ! [[ "$API_PORT" =~ ^[0-9]+$ ]] || ((API_PORT < 1 || API_PORT > 65535)); then
  echo "Invalid API_PORT '$API_PORT'; expected an integer from 1 to 65535." >&2
  exit 2
fi
if ! [[ "$AAD_TEST_PUBLISH_PERIOD" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
   ! awk -v period="$AAD_TEST_PUBLISH_PERIOD" 'BEGIN { exit !(period > 0) }'; then
  echo "Invalid AAD_TEST_PUBLISH_PERIOD '$AAD_TEST_PUBLISH_PERIOD'; expected a positive number." >&2
  exit 2
fi

export CART_NAME CART_ID ROS_DOMAIN_ID CART_PORT SERVER_IP API_PORT
export DASHBOARD_SCHEME AAD_TEST_PUBLISH_PERIOD DASHBOARD_ROOT

REREGISTER_INTERVAL_SEC="${REREGISTER_INTERVAL_SEC:-15}"
REGISTER_COOLDOWN_SEC="${REGISTER_COOLDOWN_SEC:-15}"
REREGISTER_PID=""

cleanup() {
  if [[ -n "$REREGISTER_PID" ]]; then
    kill "$REREGISTER_PID" 2>/dev/null || true
    wait "$REREGISTER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

dashboard_up() {
  curl -k -fsS --connect-timeout 3 --max-time 5 "$DASHBOARD_ROOT/" \
    >/dev/null 2>&1
}

register_cart() {
  local payload
  payload="$(printf '{\"name\":\"%s\",\"port\":%s}' "$CART_NAME" "$CART_PORT")"
  curl -k -fsS --connect-timeout 3 --max-time 5 \
    -X POST "$DASHBOARD_ROOT/api/vehicles/register" \
    -H "Content-Type: application/json" \
    -d "$payload" >/dev/null 2>&1
}

reregister_loop() {
  local last_ok=0
  local connected=false
  while true; do
    if dashboard_up; then
      local now
      now="$(date +%s)"
      if [[ "$connected" == false ]]; then
        echo "[Dashboard] Connected to $DASHBOARD_ROOT"
        connected=true
      fi
      if ((now - last_ok >= REGISTER_COOLDOWN_SEC)); then
        if register_cart; then
          last_ok="$now"
          echo "[Dashboard] Registered $CART_NAME on websocket port $CART_PORT"
        else
          echo "[Dashboard] Cart registration failed; retrying"
        fi
      fi
    else
      if [[ "$connected" == true ]]; then
        echo "[Dashboard] Connection lost; retrying"
      fi
      connected=false
    fi
    sleep "$REREGISTER_INTERVAL_SEC"
  done
}

echo "Starting standalone anomaly dashboard test:"
echo "  CART_NAME=$CART_NAME"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  DASHBOARD_ROOT=$DASHBOARD_ROOT"
echo "  ROSBRIDGE_PORT=$CART_PORT"
echo "  LOGGING_TOPIC=/ai_anomaly_logging"
echo "  ALERT_TOPIC=/aad/alerts"
echo "  PUBLISH_PERIOD=${AAD_TEST_PUBLISH_PERIOD}s"
echo "Press Ctrl-C to stop."

reregister_loop &
REREGISTER_PID=$!

docker compose run --rm --no-deps --build \
  -e BACKEND_COMMAND= \
  -e CART_ID="$CART_ID" \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e CART_PORT="$CART_PORT" \
  -e AAD_TEST_PUBLISH_PERIOD="$AAD_TEST_PUBLISH_PERIOD" \
  backend bash -lc '
    set -euo pipefail
    cd /root/dev_ws
    colcon build --symlink-install --base-paths \
      src/anomaly_detection/tester
    source install/setup.bash
    exec ros2 launch tester dashboard_anomaly_test.launch.py \
      websocket_port:="$CART_PORT" \
      publish_period:="$AAD_TEST_PUBLISH_PERIOD"
  '
