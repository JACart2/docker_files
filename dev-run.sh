#!/bin/bash

bash ./initialize_host.sh

STALL="tail -f /dev/null"

FRONTEND_COMMAND=$STALL BACKEND_COMMAND=$STALL docker compose up --build --remove-orphans --force-recreate -d

# Attach a terminal to the backend
docker compose exec -it -w /dev_ws backend bash -c 'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && ([ -f /dev_ws/install/setup.bash ] && source /dev_ws/install/setup.bash); exec bash'


