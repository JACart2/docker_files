#!/bin/bash
set -e

bash ./initialize_host.sh


open_browser_when_ready() {
    local port=$1
	until curl -s http://localhost:$port > /dev/null
	do
	# This is just waiting for the application to start. If the container is not up and running, this will wait forever.
	#   echo "Waiting for port 5173 to open."
	  sleep 2
	done
	open http://localhost:$port
}
check_internet() {
    # Try reaching Docker Hub (fast + relevant to your issue)
    curl -s --head https://auth.docker.io > /dev/null
    return $?
}

# Decide compose flags based on connectivity
if check_internet; then
    echo "Internet detected: using normal docker compose build"
    COMPOSE_FLAGS="--build --remove-orphans --force-recreate"
else
    echo "No internet detected: using cached images only"
    COMPOSE_FLAGS=" --pull=never --remove-orphans --force-recreate"
fi


ANOMALY_PACKAGE_PATHS="src/anomaly_detection/anomaly_msg src/anomaly_detection/anomaly_detection src/anomaly_detection/tester"
ANOMALY_BUILD_AND_SOURCE="cd ~/dev_ws && colcon build --symlink-install --base-paths ${ANOMALY_PACKAGE_PATHS} && source install/setup.bash"
ANOMALY_DETECTION_NODE="${ANOMALY_BUILD_AND_SOURCE} && ros2 run anomaly_detection anomaly_detection_node"
ANOMALY_DETECTION_LAUNCH="${ANOMALY_BUILD_AND_SOURCE} && ros2 launch anomaly_detection anomaly_detection.launch.py"
ANOMALY_MODE="${ANOMALY_MODE:-launch}"

if [ "$ANOMALY_MODE" = "node" ]; then
    ANOMALY_COMMAND="$ANOMALY_DETECTION_NODE"
else
    ANOMALY_COMMAND="$ANOMALY_DETECTION_LAUNCH"
fi

BACKEND_ROS_DOMAIN_ID="$(docker compose exec -T backend printenv ROS_DOMAIN_ID 2>/dev/null || true)"
export ROS_DOMAIN_ID="${BACKEND_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-0}}"
echo "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID} for anomaly_detection"
echo "Using anomaly command: ${ANOMALY_COMMAND}"

ANOMALY_DETECTION_COMMAND="$ANOMALY_COMMAND" docker compose up -d $COMPOSE_FLAGS anomaly_detection

# Launch VS Code attached to the container
if command -v code &> /dev/null; then
    CONTAINER_ID=$(docker compose ps -q anomaly_detection)
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

# Attach a terminal to the anomaly_detection
if [ "$ANOMALY_MODE" != "node" ]; then
    open_browser_when_ready 5000 &
fi
docker compose exec -it -w /root/dev_ws anomaly_detection bash -c 'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && ([ -f /root/dev_ws/install/setup.bash ] && source /root/dev_ws/install/setup.bash); exec bash'
