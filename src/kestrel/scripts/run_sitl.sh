#!/usr/bin/env bash
# Boot ArduCopter SITL with MAVLink on udp 14550, extra args pass through
set -euo pipefail

# ArduPilot opens arducopter in an xterm when DISPLAY is set, we never want a
# window, clear it so the launcher logs to a file and runs in the background
unset DISPLAY

# Keep SITL state out of the source tree
STATE_DIR=/tmp/kestrel_sitl
mkdir -p "${STATE_DIR}"
cd "${STATE_DIR}"

exec sim_vehicle.py -v ArduCopter --no-rebuild --out=127.0.0.1:14551 "$@"
