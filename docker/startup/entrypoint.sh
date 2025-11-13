#!/bin/bash
# Created by Nelson Durrant, Nov 2025
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs cougars-ct'

set -e

# Fix initial write permission errors on Linux computers
sudo chmod -R a+w /home/frostlab-docker/coug_ws
sudo chmod -R a+w /home/frostlab-docker/scripts
sudo chmod -R a+w /startup
echo "Successfully applied write permissions patch."

# Copy the .tmux.conf file to the home directory
cp /startup/.tmux.conf /home/frostlab-docker/.tmux.conf
echo "Copied .tmux.conf to home directory."

# Any tasks that need to be run on startup should be added here

exec /bin/bash
