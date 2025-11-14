#!/bin/bash
# Created by Nelson Durrant, Nov 2025
#
# Launches FGO localization on the BlueROV2 for HITL testing

source ~/coug_ws/install/setup.bash
ros2 launch coug_bringup bluerov2_hitl.launch.py urdf_file:="urdf/bluerov2_holoocean/bluerov2_holoocean.urdf.xacro"
