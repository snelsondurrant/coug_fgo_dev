#!/bin/bash
# Created by Nelson Durrant, Nov 2025
# 
# Drives the CougUV or BlueROV2 using the keyboard

source ~/coug_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
