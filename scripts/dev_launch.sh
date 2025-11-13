#!/bin/bash
# Created by Nelson Durrant, Nov 2025
#
# Launches FGO localization with simulation and visualization tools
# Use the '-b' flag to launch the BlueROV2

source ~/coug_ws/install/setup.bash

if [ "$1" == "-b" ]; then
  ros2 launch coug_bringup dev.launch.py urdf_file:="urdf/bluerov2_holoocean/bluerov2_holoocean.urdf.xacro"
else
  ros2 launch coug_bringup dev.launch.py urdf_file:="urdf/couguv_holoocean.urdf.xacro"
fi
