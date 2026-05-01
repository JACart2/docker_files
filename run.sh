#!/bin/bash
set -e

# -------------------------
# Config (override via env)
# -------------------------
CART_NAME="${CART_NAME:-james}"
SERVER_IP="${SERVER_IP:-10.247.225.41}"   # Dashboard server (external)
CART_PORT="${CART_PORT:-9090}"            # rosbridge port
API_PORT="${API_PORT:-8000}"              # dashboard server port

bash ./initialize_host.sh

wait_for_frontend () {
  until curl -fsS http://localhost:5173 >/dev/null; do
    sleep 2
  done
}

wait_for_dashboard () {
  # external dashboard server availability check
  until curl -fsS "http://${SERVER_IP}:${API_PORT}/" >/dev/null; do
    sleep 2
  done
}

register_cart () {
  curl -fsS -X POST "http://${SERVER_IP}:${API_PORT}/api/vehicles/register" \
    -H "Content-Type: application/json" \
    -d "{\"name\":\"${CART_NAME}\",\"port\":${CART_PORT}}"
}

# Start containers
docker compose up backend frontend --build --remove-orphans --force-recreate &
COMPOSE_PID=$!

# Wait for the frontend dev server, then open browser
wait_for_frontend
open "http://localhost:5173"

# After browser opens: wait for dashboard, then register (without blocking compose logs)
(
  wait_for_dashboard
  # optionally retry registration until it works
  until register_cart; do
    sleep 2
  done
) &

# Keep attached to docker compose
wait "$COMPOSE_PID"
