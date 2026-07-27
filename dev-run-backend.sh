#!/bin/bash
set -euo pipefail

source ./dashboard-api.sh

dashboard_configure "$@"
dashboard_start_registration

echo "Starting backend development environment with:"
echo "  CART_NAME=${CART_NAME}"
echo "  CART_ID=${CART_ID}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  DASHBOARD_ROOT=${DASHBOARD_ROOT}"

BROWSER_PID=""

cleanup() {
    echo "Cleaning up..."

    dashboard_stop_registration

    if [ -n "${BROWSER_PID:-}" ]; then
        kill "$BROWSER_PID" 2>/dev/null || true
        wait "$BROWSER_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT
trap 'exit 130' SIGINT SIGTERM

bash ./initialize_host.sh

wait_for_frontend() {
    until curl -fsS http://localhost:5173 >/dev/null 2>&1; do
        sleep 2
    done
}

open_browser_when_ready() {
    wait_for_frontend

    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open "http://localhost:5173" >/dev/null 2>&1 || true
    elif command -v open >/dev/null 2>&1; then
        open "http://localhost:5173" >/dev/null 2>&1 || true
    else
        echo "Frontend available at http://localhost:5173"
    fi
}

open_browser_when_ready &
BROWSER_PID=$!

# Start the frontend normally and keep the backend container alive.
BACKEND_COMMAND="tail -f /dev/null" \
docker compose up \
    frontend \
    backend \
    -d \
    --build \
    --remove-orphans \
    --force-recreate

# Launch VS Code attached to the backend container.
if command -v code >/dev/null 2>&1; then
    CONTAINER_ID="$(docker compose ps -q backend)"

    if [ -n "$CONTAINER_ID" ]; then
        CONTAINER_NAME="$(
            docker inspect --format '{{.Name}}' "$CONTAINER_ID" |
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

# Attach a terminal to the backend container.
docker compose exec \
    -it \
    -w /root/dev_ws \
    backend \
    bash -c '
        source /opt/ros/jazzy/setup.bash
        source /opt/ros_ws/install/setup.bash

        if [ -f /root/dev_ws/install/setup.bash ]; then
            source /root/dev_ws/install/setup.bash
        fi

        exec bash
    '