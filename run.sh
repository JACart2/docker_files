#!/bin/bash
set -e

# -------------------------
# Config (override via env)
# -------------------------
CART_NAME="${CART_NAME:-james}"
SERVER_IP="${SERVER_IP:-10.247.225.41}"   # Dashboard server (external)
CART_PORT="${CART_PORT:-9090}"
API_PORT="${API_PORT:-8000}"

# How often to *probe* the dashboard when it's down / between attempts
REREGISTER_INTERVAL_SEC="${REREGISTER_INTERVAL_SEC:-30}"
# Minimum time between successful registrations
REGISTER_COOLDOWN_SEC="${REGISTER_COOLDOWN_SEC:-300}"   # 5 minutes

bash ./initialize_host.sh

wait_for_frontend () {
  until curl -fsS http://localhost:5173 >/dev/null; do
    sleep 2
  done
}

dashboard_up () {
  curl -fsS "http://${SERVER_IP}:${API_PORT}/" >/dev/null
}

register_cart () {
  curl -fsS -X POST "http://${SERVER_IP}:${API_PORT}/api/vehicles/register" \
    -H "Content-Type: application/json" \
    -d "{\"name\":\"${CART_NAME}\",\"port\":${CART_PORT}}"
}

# Background loop: if dashboard is up, try (re)registering occasionally
reregister_loop () {
  local last_ok=0
  while true; do
    if dashboard_up; then
      local now
      now="$(date +%s)"

      # only re-register if we haven't done so recently
      if (( now - last_ok >= REGISTER_COOLDOWN_SEC )); then
        if register_cart; then
          last_ok="$now"
        fi
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
