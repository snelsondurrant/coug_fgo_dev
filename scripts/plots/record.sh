#!/bin/bash
# Created by Nelson Durrant, Nov 2025
# 
# Records and plots data from a ROS2 bag

read -p "Enter desired bag name: " bag_name
BAG_NAME=${bag_name:-latest_run}
BAG_PATH="$HOME/scripts/plots/$BAG_NAME"

trap "sudo chmod -R a+w /home/frostlab-docker/scripts && python3 $HOME/scripts/plots/plot.py $BAG_PATH" EXIT

ros2 bag record \
  /odometry/truth \
  /odometry/global \
  /odometry/global_ekf \
  /odometry/global_ukf \
  /factor_graph_node/velocity \
  /factor_graph_node/imu_bias \
  /imu/data \
  /gps/fix \
  /odometry/depth \
  /imu/heading \
  /dvl/data \
  /origin \
  /odometry/gps \
  /smoothed_path \
  /tf \
  /tf_static \
  -o "$BAG_PATH"
