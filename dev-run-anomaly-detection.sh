#!/bin/bash

bash ./initialize_host.sh

STALL="tail -f /dev/null"

FRONTEND_COMMAND=$STALL ANOMALY_DETECTION_COMMAND=$STALL docker compose up --build --remove-orphans --force-recreate 

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
docker compose exec -it -w /root/dev_ws anomaly_detection bash -c 'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && ([ -f /root/dev_ws/install/setup.bash ] && source /root/dev_ws/install/setup.bash); exec bash'



