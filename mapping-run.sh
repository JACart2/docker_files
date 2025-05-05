#!/bin/bash

bash ./initialize_host.sh

RUN="ros2 launch mapping_launch mapping.launch.py"

BACKEND_COMMAND=$RUN docker compose up --build --remove-orphans --force-recreate 

