#!/usr/bin/env bash

# This file is intended to be sourced by the launcher scripts.
#
# Usage:
#   source "$(dirname "${BASH_SOURCE[0]}")/dashboard-api.sh"
#   dashboard_configure "$@"
#   dashboard_start_registration

DASHBOARD_REGISTRATION_PID=""

dashboard_configure() {
    local cart_argument="${1:-}"
    local domain_argument="${2:-}"

    CART_NAME="${CART_NAME:-${cart_argument:-james}}"

    SERVER_IP="${SERVER_IP:-10.247.225.41}"
    CART_PORT="${CART_PORT:-9090}"
    API_PORT="${API_PORT:-8000}"
    DASHBOARD_SCHEME="${DASHBOARD_SCHEME:-https}"

    local cart_name_lower
    cart_name_lower="$(
        printf '%s' "$CART_NAME" |
            tr '[:upper:]' '[:lower:]'
    )"

    # ROS_DOMAIN_ID priority:
    # 1. Existing environment variable
    # 2. Second command-line argument
    # 3. Known cart default
    # 4. Domain 0 fallback
    if [ -n "${ROS_DOMAIN_ID:-}" ]; then
        ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
    elif [ -n "$domain_argument" ]; then
        ROS_DOMAIN_ID="$domain_argument"
    else
        case "$cart_name_lower" in
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

    if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]]; then
        echo "Invalid ROS_DOMAIN_ID '${ROS_DOMAIN_ID}'. It must be a number."
        return 1
    fi

    if ((ROS_DOMAIN_ID < 0 || ROS_DOMAIN_ID > 232)); then
        echo "Invalid ROS_DOMAIN_ID '${ROS_DOMAIN_ID}'. Use a value between 0 and 232."
        return 1
    fi

    if ! [[ "$CART_PORT" =~ ^[0-9]+$ ]]; then
        echo "Invalid CART_PORT '${CART_PORT}'. It must be a number."
        return 1
    fi

    if ! [[ "$API_PORT" =~ ^[0-9]+$ ]]; then
        echo "Invalid API_PORT '${API_PORT}'. It must be a number."
        return 1
    fi

    CART_ID="${CART_ID:-$CART_NAME}"
    DASHBOARD_ROOT="${DASHBOARD_SCHEME}://${SERVER_IP}:${API_PORT}"

    REREGISTER_INTERVAL_SEC="${REREGISTER_INTERVAL_SEC:-15}"
    REGISTER_COOLDOWN_SEC="${REGISTER_COOLDOWN_SEC:-15}"

    export CART_NAME
    export CART_ID
    export ROS_DOMAIN_ID
    export SERVER_IP
    export CART_PORT
    export API_PORT
    export DASHBOARD_SCHEME
    export DASHBOARD_ROOT
    export REREGISTER_INTERVAL_SEC
    export REGISTER_COOLDOWN_SEC

    # Frontend build/runtime variables.
    export VITE_CART_NAME="$CART_NAME"
    export VITE_DASHBOARD_API_ROOT="${DASHBOARD_ROOT}/"

    echo "Cart dashboard configuration:"
    echo "  CART_NAME=${CART_NAME}"
    echo "  CART_ID=${CART_ID}"
    echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "  DASHBOARD_ROOT=${DASHBOARD_ROOT}"
    echo "  CART_PORT=${CART_PORT}"
}

dashboard_is_available() {
    curl \
        -k \
        -fsS \
        --connect-timeout 3 \
        --max-time 5 \
        "${DASHBOARD_ROOT}/" \
        >/dev/null 2>&1
}

dashboard_register_cart() {
    local payload

    payload="$(
        printf \
            '{"name":"%s","port":%s}' \
            "$CART_NAME" \
            "$CART_PORT"
    )"

    curl \
        -k \
        -fsS \
        --connect-timeout 3 \
        --max-time 5 \
        -X POST \
        "${DASHBOARD_ROOT}/api/vehicles/register" \
        -H "Content-Type: application/json" \
        -d "$payload" \
        >/dev/null 2>&1
}

dashboard_registration_loop() {
    local last_success=0
    local dashboard_was_available=false

    while true; do
        if dashboard_is_available; then
            local current_time
            current_time="$(date +%s)"

            if [ "$dashboard_was_available" = false ]; then
                echo "[Dashboard] Connected to ${DASHBOARD_ROOT}"
                dashboard_was_available=true
            fi

            if ((current_time - last_success >= REGISTER_COOLDOWN_SEC)); then
                if dashboard_register_cart; then
                    last_success="$current_time"
                    echo "[Dashboard] Registered ${CART_NAME}"
                else
                    echo "[Dashboard] Registration failed for ${CART_NAME}"
                fi
            fi
        else
            if [ "$dashboard_was_available" = true ]; then
                echo "[Dashboard] Connection lost"
            fi

            dashboard_was_available=false
        fi

        sleep "$REREGISTER_INTERVAL_SEC"
    done
}

dashboard_start_registration() {
    if [ -n "${DASHBOARD_REGISTRATION_PID:-}" ] &&
        kill -0 "$DASHBOARD_REGISTRATION_PID" 2>/dev/null; then
        return 0
    fi

    dashboard_registration_loop &
    DASHBOARD_REGISTRATION_PID=$!
    export DASHBOARD_REGISTRATION_PID
}

dashboard_stop_registration() {
    if [ -n "${DASHBOARD_REGISTRATION_PID:-}" ]; then
        kill "$DASHBOARD_REGISTRATION_PID" 2>/dev/null || true
        wait "$DASHBOARD_REGISTRATION_PID" 2>/dev/null || true
        DASHBOARD_REGISTRATION_PID=""
    fi
}