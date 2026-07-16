#!/usr/bin/env bash
# Source ROS and the workspace overlay before running the container command
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

exec "$@"
