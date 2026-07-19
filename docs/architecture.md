# Architecture

## Nodes

| Node | Subscribes | Publishes | Services |
|---|---|---|---|
| `telemetry_monitor` | `/mavros/state`, `/mavros/battery`, `/mavros/local_position/pose` | none | none |
| `flight_commander` | `/mavros/state`, `/mavros/local_position/pose`, `/mavros/home_position/home` | `/mavros/setpoint_position/local` | offers `/kestrel/cmd/takeoff`, `/kestrel/cmd/goto`, `/kestrel/cmd/land` |
| `safety_guard` | `/mavros/local_position/pose`, `/mavros/battery`, `/mavros/state` | none | offers `/kestrel/abort` |
| `defect_detector` | `/camera/image_raw`, `/camera/camera_info`, `/mavros/local_position/pose` | `/kestrel/detections`, `/kestrel/defect_events` | none |
| `mission_director` | `/kestrel/defect_events` | `/kestrel/mission_state` | calls `/kestrel/cmd/takeoff`, `/kestrel/cmd/goto`, `/kestrel/cmd/land` |
| `report_writer` | `/kestrel/defect_events`, `/kestrel/mission_state` | none | none |
| `mcp_server` (node name `mcp_bridge`) | `/mavros/state`, `/mavros/local_position/pose`, `/mavros/battery`, `/kestrel/mission_state` | none | calls `/kestrel/cmd/takeoff`, `/kestrel/cmd/goto`, `/kestrel/cmd/land`, `/kestrel/abort` |

`inspection_planner` is a pure Python module with waypoint geometry
functions. It is not a ROS node. `mission_director` calls it directly, in
the same process.

## Topics and services

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `/kestrel/detections` | `vision_msgs/Detection2DArray` | detector out | defect boxes per frame |
| `/kestrel/defect_events` | custom `DefectEvent.msg` | detector out | fires once per new confirmed defect, with an estimated world position |
| `/kestrel/mission_state` | `std_msgs/String` | director out | current state machine state |
| `/kestrel/cmd/takeoff` | service `Takeoff.srv` | commander in | altitude in, success out |
| `/kestrel/cmd/goto` | service `GotoLocal.srv` | commander in | local north east target in, success out |
| `/kestrel/cmd/land` | service `std_srvs/Trigger` | commander in | land now |
| `/kestrel/abort` | service `std_srvs/Trigger` | guard in | any node can call this service, the guard then forces RTL |

## Detector backends

`defect_detector` supports two backends. The parameter `detector_backend`
selects the backend. The node contract and the topics stay the same for
both backends.

- `yolo` (the default backend): a YOLOv8n ONNX model runs with
  `onnxruntime`. The model trains on real corrosion photos. There is no
  known marker geometry to solve a pose from, so this backend uses a fixed
  depth assumption instead (`assumed_depth_m`, the survey orbit radius
  minus the structure radius) and a ray through the center of the
  detection box. Debounce and dedupe logic track world position clusters
  within `dedupe_radius_m`, not a marker ID.
- `aruco`: the original OpenCV ArUco detector. This backend uses
  `cv2.solvePnP` against the known 0.5 meter board geometry to find an
  exact pose. Debounce logic and a set of fired IDs use the marker ID as
  the key.

Both backends feed the same `estimate_world_position` function. This
function converts a position from the optical frame to a world position
in north, east, and altitude values.

## MCP server

The command `ros2 run kestrel mcp_server` starts one process. This
process holds two parts: an rclpy node named `mcp_bridge` that spins on a
background executor thread, and a FastMCP server that speaks MCP over
stdio in the main thread. An LLM client launches the process with
`docker exec -i kestrel ...`. This command pipes stdio directly into the
running simulation container.

Six tools wrap the guarded services. No tool publishes a setpoint
directly.

| Tool | Wraps | Validation |
|---|---|---|
| `takeoff(altitude)` | `/kestrel/cmd/takeoff` | altitude from 1 to 30 m |
| `goto(north, east, altitude, yaw_deg)` | `/kestrel/cmd/goto` | inside 95 m of home, altitude from 1 to 30 m |
| `land()` | `/kestrel/cmd/land` | none |
| `abort()` | `/kestrel/abort` | none, this action latches RTL |
| `get_telemetry()` | reads stored topic data | none |
| `get_mission_state()` | reads stored topic data | none |

Two resources expose finished reports. `kestrel://reports` lists the
report directories. `kestrel://reports/{timestamp}` returns the markdown
text of one report.

MCP over stdio owns the standard output stream. For this reason, the
server must never print to standard output. All log messages go to
standard error instead. The safety guard node runs on its own and keeps
final authority. An `abort` call latches RTL in the same way for
conversational control and for the autonomous mission.

## Frames

The MAVROS local frame uses ENU coordinates. Every `kestrel` service uses
compass north and east values instead. One function converts between the
two systems: `build_enu_setpoint` in `flight_commander.py`. No other part
of the project performs this conversion. The function uses these rules:
ENU `x = east`, `y = north`, `z = altitude`, and yaw in radians
`= radians(90 - yaw_deg)`.
