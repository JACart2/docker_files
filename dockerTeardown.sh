#!/bin/bash

# run the docker list command
DOCKER_RESULTS="$(docker container list)"

# extracting individual container line info
IFS=$'\n'
DOCKER_ARRAY=($DOCKER_RESULTS)
CONTAINER_LINE_ONE="${DOCKER_ARRAY[1]}"
CONTAINER_LINE_TWO="${DOCKER_ARRAY[2]}"
CONTAINER_LINE_THREE="${DOCKER_ARRAY[3]}"

# extracting individual container IDs
IFS=$' '
DOCKER_ID_ONE=($CONTAINER_LINE_ONE)
DOCKER_ID_TWO=($CONTAINER_LINE_TWO)
DOCKER_ID_THREE=($CONTAINER_LINE_THREE)

# -- debugging: verbose --
echo "Shutting down: ${DOCKER_ID_ONE} ..."
echo "Shutting down: ${DOCKER_ID_TWO} ..."
echo "Shutting down: ${DOCKER_ID_THREE} ..."
echo ""

docker stop ${DOCKER_ID_ONE}
docker stop ${DOCKER_ID_TWO}
docker stop ${DOCKER_ID_THREE}

# confirm docker container was stopped 
echo "Currently running containers:"
echo ""
docker ps
