#!/bin/bash
set -e

# Config (override via env)
CART_NAME="${CART_NAME:-james}"
SERVER_IP="${SERVER_IP:-10.247.225.41}"   # Dashboard server
CART_PORT="${CART_PORT:-9090}"
API_PORT="${API_PORT:-8000}"
DASHBOARD_SCHEME="${DASHBOARD_SCHEME:-https}"

export CART_NAME
export SERVER_IP
export API_PORT
export CART_PORT
export DASHBOARD_SCHEME

DASHBOARD_ROOT="${DASHBOARD_SCHEME}://${SERVER_IP}:${API_PORT}"

# Variables passed into the Vite frontend
export VITE_CART_NAME="${CART_NAME}"
export VITE_DASHBOARD_API_ROOT="${DASHBOARD_ROOT}/"

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

# Start containers
docker compose up backend frontend --build --remove-orphans --force-recreate &
COMPOSE_PID=$!

# Wait for frontend then open browser
wait_for_frontend
open "http://localhost:5173"

# Start periodic re-registration in background
reregister_loop &

# Keep attached to compose
wait "$COMPOSE_PID"
