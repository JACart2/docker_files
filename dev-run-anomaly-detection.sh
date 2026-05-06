#!/bin/bash

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


STALL="tail -f /dev/null"
MADISON_CONFIG="cd ~/dev_ws && ros2 launch cart_launch autonomous_launcher.launch.py cart_config_path:=./src/ai-navigation/cart_control/cart_launch/config/cart_madison.yaml"
JAMES_CONFIG="cd ~/dev_ws && ros2 launch cart_launch autonomous_launcher.launch.py "

ANOMALY_LAUNCH="cd ~/dev_ws && ros2 launch anomaly_detection anomaly_detection.launch.py "

ANOMALY_DETECTION_COMMAND=$ANOMALY_LAUNCH  BACKEND_COMMAND=$JAMES_CONFIG docker compose up $COMPOSE_FLAGS

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
open_browser_when_ready 5173 & open_browser_when_ready 5000 & docker compose exec -it -w /root/dev_ws anomaly_detection bash -c 'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && ([ -f /root/dev_ws/install/setup.bash ] && source /root/dev_ws/install/setup.bash); exec bash'



