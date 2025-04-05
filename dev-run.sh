#!/bin/bash

bash ./initialize_host.sh

STALL="tail -f /dev/null"

# update to true
RUN_FER="false"

docker compose create --build --remove-orphans --force-recreate fer
# temp disabled to check on FER
#FRONTEND_COMMAND=$STALL BACKEND_COMMAND=$STALL docker compose create --build --remove-orphans --force-recreate 


