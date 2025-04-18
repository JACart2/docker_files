#!/bin/bash

bash ./initialize_host.sh
RUN_FER="false"

open_browser_when_ready () {
	until curl -s http://localhost:5173 > /dev/null
	do
	# This is just waiting for the application to start. If the container is not up and running, this will wait forever.
	#   echo "Waiting for port 5173 to open."
	  sleep 2
	done
	open http://localhost:5173
}

open_browser_when_ready & docker compose up --remove-orphans --force-recreate 


