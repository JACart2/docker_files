#!/bin/bash

bash ./initialize_host.sh


open_browser_when_ready () {
	until curl -s http://localhost:5173 > /dev/null
	do
	# This is just waiting for the application to start. If the container is not up and running, this will wait forever.
	#   echo "Waiting for port 5173 to open."
	  sleep 2
	done
	open http://localhost:5173
}

STALL="tail -f /dev/null"
MADISON_CONFIG="cd ~/dev_ws && ros2 launch cart_launch autonomous_launcher.launch.py cart_config_path:=./src/ai-navigation/cart_control/cart_launch/config/cart_madison.yaml"
ANOMALY_LAUNCH="cd ~/dev_ws && ros2 launch anomaly_detection anomaly_detection.launch.py"

ANOMALY_DETECTION_COMMAND=$ANOMALY_LAUNCH   docker compose up --build --remove-orphans --force-recreate 

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
open_browser_when_ready & docker compose exec -it -w /root/dev_ws anomaly_detection bash -c 'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && ([ -f /root/dev_ws/install/setup.bash ] && source /root/dev_ws/install/setup.bash); exec bash'



