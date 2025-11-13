#!/bin/bash
# Created by Nelson Durrant, Nov 2025

set -e

# Fix initial write permission errors on Linux computers
sudo chmod -R a+w /home/ue4/config
echo "Successfully applied write permissions patch."

exec /bin/bash
