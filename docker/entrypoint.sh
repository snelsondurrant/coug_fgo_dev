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
echo "Successfully applied write permissions patch."

exec /bin/bash
