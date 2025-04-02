#!/bin/bash

bash ./initialize_host.sh

STALL="tail -f /dev/null"

FRONTEND_COMMAND=$STALL BACKEND_COMMAND=$STALL docker compose up --build --remove-orphans --force-recreate 

