# ROS Project

![ci](https://github.com/ajnieves1/Kestrel/actions/workflows/ci.yaml/badge.svg)

![Mission flight](docs/mission.gif)

Copter flies a structure, finds defects with onboard vision AI, flies closer
to confirm, and writes a plain language inspection report.

## How it works

```mermaid
flowchart TB
    subgraph sim[Gazebo Harmonic]
        world[World: pylon plus defect markers]
        copter[Copter model plus camera]
    end

    sitl[ArduCopter SITL]
    mavros[MAVROS 2]

    subgraph graph[ROS 2 graph, ROS Project nodes]
        telemetry[telemetry_monitor: watches state and battery]
        commander[flight_commander: arm, takeoff, goto, land]
        guard[safety_guard: geofence, battery, RTL]
        detector[defect_detector: ArUco detection on camera feed]
        planner[inspection_planner: survey grid plus closer look]
        director[mission_director: state machine, runs a job]
        writer[report_writer: LLM API, writes report]
    end

    sim -- physics and motors --> sitl
    sim -- camera frames --> detector
    sitl -- MAVLink udp 14550 --> mavros
    mavros -- ROS 2 topics and services --> graph
    director --> planner
    director --> commander
    detector --> director
    director --> writer
```

One mission: `mission_director` asks `inspection_planner` for a survey path
around the structure, feeds waypoints to `flight_commander`, `defect_detector`
streams detections the whole time, on a detection the director pauses the
survey and requests a closer orbit of that point, photos and detections
accumulate, after landing `report_writer` turns them into a markdown report
with photos embedded.

## Quick start

```bash
git clone https://github.com/ajnieves1/Kestrel.git
docker compose -f docker/compose.yaml build dev
docker compose -f docker/compose.yaml run --rm dev ros2 launch kestrel mission.launch.py headless:=true
```

For the Gazebo GUI instead of headless, drop `headless:=true` and run
`xhost +local:` on the host once first.

## Sample report

A full mission run with no LLM API key set writes an appendix only report:
[docs/sample_report.md](docs/sample_report.md).

![Marker view](docs/marker_view.png)

## Stack

| Piece | Role |
|---|---|
| ROS 2 Jazzy | Node graph, topics, services |
| ArduPilot SITL | Flight controller simulation |
| MAVROS 2 | MAVLink bridge into ROS 2 |
| Gazebo Harmonic | Physics sim and camera |
| OpenCV ArUco | v1 defect detector |
| LLM report writer | Claude, OpenAI, or Gemini, provider chosen by a parameter |
| Docker | One dev image on every machine |
| GitHub Actions | CI with a real SITL flight test on every push |

## License

MIT
