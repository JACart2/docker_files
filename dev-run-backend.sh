#!/bin/bash

bash ./initialize_host.sh

open_browser_when_ready () {
	until curl -s http://localhost:5173 > /dev/null
	do
	# This is just waiting for the application to start. If the container is not up and running, this will wait forever.
	#   echo "Waiting for port 5173 to open."
	  sleep 2
	done
	if command -v open &> /dev/null; then
		open http://localhost:5173
	else
		xdg-open http://localhost:5173
	fi
}

open_browser_when_ready &

# Start the frontend with its default command, and stall the backend.
# We set FRONTEND_COMMAND to empty (or unset) to trigger the default behavior in entrypoint.sh/compose.yaml
BACKEND_COMMAND="tail -f /dev/null" docker compose up frontend backend -d --build --remove-orphans --force-recreate

# Launch VS Code attached to the container
if command -v code &> /dev/null; then
    CONTAINER_ID=$(docker compose ps -q backend)
    if [ -n "$CONTAINER_ID" ]; then
        CONTAINER_NAME=$(docker inspect --format '{{.Name}}' $CONTAINER_ID | sed 's/^\///')
        if [ -n "$CONTAINER_NAME" ]; then
             HEX_NAME=$(printf "$CONTAINER_NAME" | od -A n -t x1 | tr -d ' \n')
             URI="vscode-remote://attached-container+${HEX_NAME}/dev_ws"
             echo "Opening VS Code attached to ${CONTAINER_NAME}..."
             code --folder-uri "$URI"
        fi
    fi
fi

# Attach a terminal to the backend
docker compose exec -it -w /dev_ws backend bash -c 'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && ([ -f /dev_ws/install/setup.bash ] && source /dev_ws/install/setup.bash); exec bash'
