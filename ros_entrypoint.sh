#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/iron/setup.bash"
source "/ros2_ws/ceb/install/setup.bash"
exec "$@"
