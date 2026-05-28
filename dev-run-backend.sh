#!/bin/bash

# -------------------------
# Config (override via env)
# -------------------------
CART_NAME="${CART_NAME:-james}"
SERVER_IP="${SERVER_IP:-10.247.225.41}"   # Dashboard server (external)
CART_PORT="${CART_PORT:-9090}"
API_PORT="${API_PORT:-8000}"

# How often to *probe* the dashboard when it's down / between attempts
REREGISTER_INTERVAL_SEC="${REREGISTER_INTERVAL_SEC:-15}"
# Minimum time between successful registrations
REGISTER_COOLDOWN_SEC="${REGISTER_COOLDOWN_SEC:-15}"   # 15 seconds

bash ./initialize_host.sh

wait_for_frontend () {
    until curl -fsS http://localhost:5173 >/dev/null 2>&1
    do
        sleep 2
    done
}

dashboard_up () {
  # Only check, don't fail script if dashboard is unreachable
  curl -fsS "http://${SERVER_IP}:${API_PORT}/" >/dev/null 2>&1
}

register_cart () {
  # Ignore failure, always return true
  curl -fsS -X POST "http://${SERVER_IP}:${API_PORT}/api/vehicles/register" \
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

open_browser_when_ready () {
    wait_for_frontend
    if command -v open &> /dev/null; then
        open http://localhost:5173
    else
        xdg-open http://localhost:5173
    fi
}

# Start opening browser when frontend is ready (background)
open_browser_when_ready &

# Start periodic re-registration in background
reregister_loop &

# Start the frontend with its default command, and stall the backend.
BACKEND_COMMAND="tail -f /dev/null" docker compose up frontend backend -d --build --remove-orphans --force-recreate

# Launch VS Code attached to the container
if command -v code &> /dev/null; then
    CONTAINER_ID=$(docker compose ps -q backend)
    if [ -n "$CONTAINER_ID" ]; then
        CONTAINER_NAME=$(docker inspect --format '{{.Name}}' $CONTAINER_ID | sed 's/^\///')
        if [ -n "$CONTAINER_NAME" ]; then
             HEX_NAME=$(printf "$CONTAINER_NAME" | od -A n -t x1 | tr -d ' \n')
             URI="vscode-remote://attached-container+${HEX_NAME}/root/dev_ws"
             echo "Opening VS Code attached to ${CONTAINER_NAME}..."
             code --folder-uri "$URI"
        fi
    fi
fi

# Attach a terminal to the backend
docker compose exec -it -w /root/dev_ws backend bash -c \
  'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && ([ -f /root/dev_ws/install/setup.bash ] && source /root/dev_ws/install/setup.bash); exec bash'
