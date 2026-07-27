#!/bin/bash
set -euo pipefail

# Shared by another script
source ./dashboard-api.sh

dashboard_configure "$@"
dashboard_start_registration

echo "Starting cart UI/helper with:"
echo "  CART_NAME=${CART_NAME}"
echo "  CART_ID=${CART_ID}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  DASHBOARD_ROOT=${DASHBOARD_ROOT}"

#Termination signal to run.sh cleans all child processes
cleanup() {
    echo "Cleaning up..."

    dashboard_stop_registration

    if [ -n "${COMPOSE_PID:-}" ]; then
        kill "$COMPOSE_PID" 2>/dev/null || true
    fi

    wait 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 130' SIGINT SIGTERM

bash ./initialize_host.sh

wait_for_frontend () {
  until curl -fsS http://localhost:5173 >/dev/null 2>&1; do
    sleep 2
  done
}

# Start containers
docker compose up backend frontend --build --remove-orphans --force-recreate &
COMPOSE_PID=$!

# Wait for frontend then open browser
wait_for_frontend
open "http://localhost:5173"

# Keep attached to compose
wait "$COMPOSE_PID"
