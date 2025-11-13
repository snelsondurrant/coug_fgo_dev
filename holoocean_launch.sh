#!/bin/bash
# Created by Nelson Durrant, Nov 2025
#
# Launches the specified scenario in the HoloOcean Docker container
# Use the '-b' flag to launch the BlueROV2

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

# Start the Docker container if not already running
if [ $(docker ps | grep holoocean-ct | wc -l) -eq 0 ]; then
	printWarning "Starting the holoocean-ct container..."
	cd $(dirname "$(readlink -f "$0")")/docker/holoocean && docker compose up -d
fi

# Launch HoloOcean with the param files in 'holoocean_bridge'
if [ "$1" == "-b" ]; then
  docker exec -it holoocean-ct /bin/bash -c "source ~/ros2_ws/install/setup.bash \
    && ros2 run holoocean_main holoocean_node --ros-args --params-file /home/ue4/config/bluerov2_holoocean_params.yaml"
else
  docker exec -it holoocean-ct /bin/bash -c "source ~/ros2_ws/install/setup.bash \
    && ros2 run holoocean_main holoocean_node --ros-args --params-file /home/ue4/config/coug_holoocean_params.yaml"
fi
