#!/usr/bin/env bash
# Source ROS and the workspace overlay before running the container command
set -e

# Named volumes mount as root on first use, take ownership of the workspace
for workspace_dir in /ws/build /ws/install /ws/log; do
    if [ -d "${workspace_dir}" ] && [ ! -w "${workspace_dir}" ]; then
        sudo chown ubuntu:ubuntu "${workspace_dir}"
    fi
done

source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

exec "$@"
