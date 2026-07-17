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

`inspection_planner` is a pure Python module of waypoint geometry functions,
not a ROS node. `mission_director` calls it directly, in process.

## Topics and services

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `/kestrel/detections` | `vision_msgs/Detection2DArray` | detector out | defect boxes per frame |
| `/kestrel/defect_events` | custom `DefectEvent.msg` | detector out | fired once per new confirmed defect, with estimated world position |
| `/kestrel/mission_state` | `std_msgs/String` | director out | current state machine state |
| `/kestrel/cmd/takeoff` | service `Takeoff.srv` | commander in | altitude in, success out |
| `/kestrel/cmd/goto` | service `GotoLocal.srv` | commander in | local north east target in, success out |
| `/kestrel/cmd/land` | service `std_srvs/Trigger` | commander in | land now |
| `/kestrel/abort` | service `std_srvs/Trigger` | guard in | anything may call, guard forces RTL |

## Frames

MAVROS's local frame is ENU while every `kestrel` service speaks compass
north and east. That one conversion lives in a single function,
`build_enu_setpoint` in `flight_commander.py`, and nowhere else in the
project: ENU `x = east`, `y = north`, `z = altitude`, yaw radians
`= radians(90 - yaw_deg)`.
