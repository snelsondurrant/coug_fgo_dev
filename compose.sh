#!/bin/bash
# Created by Nelson Durrant, Nov 2024
#
# Sets up a quick CoUGARs development environment

function printInfo {
    # print blue
    echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
    # print yellow
    echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
    # print red
    echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

# Mapviz doesn't work on aarch64
if [ ! $(uname -m) == "aarch64" ]; then
	# Check if the mapproxy container is already running
	if [ $(docker ps | grep danielsnider/mapproxy | wc -l) -eq 0 ]; then
		# https://github.com/danielsnider/docker-mapproxy-googlemaps/tree/master
		printWarning "Starting the mapproxy container..."
		docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
	fi
fi

case $1 in
  	"down")
    	printWarning "Stopping the cougars-ct container..."
    	docker compose -f docker/docker-compose.yaml down
    	;;
  	*)
		# Allow container to forward graphical displays to host
		xhost +

    	printInfo "Loading the cougars-ct container..."
        docker compose -f docker/docker-compose.yaml up -d

		# Check if a 'coug_dev' tmux session already exists
		if [ "$(docker exec -it cougars-ct tmux list-sessions | grep coug_dev)" == "" ]; then

			# If not, create a new 'coug_dev' tmux session
			printWarning "Creating a new 'coug_dev' tmux session..."
			docker exec -it cougars-ct tmux new-session -d -s coug_dev -n main -c "~"
			docker exec -it cougars-ct tmux send-keys -t coug_dev:main.0 "clear && cat /startup/introduction.txt" C-m

		fi
		# Attach to the 'coug_dev' tmux session
		docker exec -it cougars-ct tmux attach -t coug_dev
    ;;
esac
