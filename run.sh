#!/bin/bash
set -e

# -------------------------
# Config (override via env)
# -------------------------
CART_NAME="${CART_NAME:-james}"
SERVER_IP="${SERVER_IP:-10.247.225.41}"
CART_PORT="${CART_PORT:-9090}"
API_PORT="${API_PORT:-8000}"

bash ./initialize_host.sh

wait_for_frontend () {
  until curl -fsS http://localhost:5173 >/dev/null; do
    sleep 2
  done
}

wait_for_backend () {
  # waits until the backend API is answering
  until curl -fsS "http://${SERVER_IP}:${API_PORT}/" >/dev/null; do
    sleep 2
  done
}

register_cart () {
  curl -fsS -X POST "http://${SERVER_IP}:${API_PORT}/api/vehicles/register" \
    -H "Content-Type: application/json" \
    -d "{\"name\":\"${CART_NAME}\",\"port\":${CART_PORT}}"
}

# Start containers, but keep the script running
docker compose up backend frontend --build --remove-orphans --force-recreate &
COMPOSE_PID=$!

# Wait specifically for backend to be usable (not for docker compose to "finish", because `up` runs forever)
wait_for_backend

# Optionally also wait for frontend before opening browser
wait_for_frontend
open "http://localhost:5173"

# Now that backend is up, do the API call
register_cart

# Keep streaming compose logs / keep containers attached like normal
wait "$COMPOSE_PID"
