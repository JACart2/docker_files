#!/bin/bash

# -------------------------
# Config (override via env)
# -------------------------
CART_NAME="${CART_NAME:-james}"
SERVER_IP="${SERVER_IP:-10.247.225.41}" # Current ZeroTier IP for dashboard
CART_PORT="${CART_PORT:-9090}" # Default rosbridge port
API_PORT="${API_PORT:-8000}" # Port for dashboard server

bash ./initialize_host.sh

open_browser_when_ready () {
	until curl -s http://localhost:5173 > /dev/null
	do
	  sleep 2
	done
	open http://localhost:5173
}

register_cart_when_ready () {
  # Wait until the server answers on port 8000 (adjust path if needed)
  until curl -s "http://${SERVER_IP}:${API_PORT}/" > /dev/null
  do
    sleep 2
  done

  curl -X POST "http://${SERVER_IP}:${API_PORT}/api/vehicles/register" \
    -H "Content-Type: application/json" \
    -d "{\"name\":\"${CART_NAME}\",\"port\":${CART_PORT}}"
}

open_browser_when_ready &
register_cart_when_ready &

docker compose up backend frontend --build --remove-orphans --force-recreate
