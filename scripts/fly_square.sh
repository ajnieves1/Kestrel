#!/usr/bin/env bash
# Take off, fly a ten meter square, and land, exercising the full flight loop
set -euo pipefail

ros2 service call /kestrel/cmd/takeoff kestrel_msgs/srv/Takeoff "{altitude: 5.0}"
ros2 service call /kestrel/cmd/goto kestrel_msgs/srv/GotoLocal "{north: 10.0, east: 0.0, altitude: 5.0, yaw_deg: 0.0}"
ros2 service call /kestrel/cmd/goto kestrel_msgs/srv/GotoLocal "{north: 10.0, east: 10.0, altitude: 5.0, yaw_deg: 0.0}"
ros2 service call /kestrel/cmd/goto kestrel_msgs/srv/GotoLocal "{north: 0.0, east: 10.0, altitude: 5.0, yaw_deg: 0.0}"
ros2 service call /kestrel/cmd/goto kestrel_msgs/srv/GotoLocal "{north: 0.0, east: 0.0, altitude: 5.0, yaw_deg: 0.0}"
ros2 service call /kestrel/cmd/land std_srvs/srv/Trigger "{}"
