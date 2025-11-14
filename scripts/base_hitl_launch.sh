#!/bin/bash
# Created by Nelson Durrant, Nov 2025
#
# Launches visualization tools on the base station for HITL testing

source ~/coug_ws/install/setup.bash
ros2 launch coug_bringup base_hitl.launch.py
