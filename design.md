# ROS Project: Implementation Handoff (Fable plans, you implement)

> **Read this first.** Fable did architecture and planning. You (Sonnet, Opus, or
> whoever picks this up) implement. Your job: turn each TASK below into working
> code. Do **not** redesign. Do **not** add features beyond the task. If a task is
> ambiguous or the spec seems wrong, **stop and ask** instead of guessing.

---

## 0. How to use this doc

- Work **one TASK at a time, in order**. Each task has: `Goal`, `Files`, `Steps`, `Verify`, `Done`.
- A task is **Done** only when its `Verify` command passes. Run it. Paste the result.
- `Contract` blocks are the designed API. Implement the bodies, keep the signatures
  as written unless you hit a concrete blocker, then ask.
- Stay inside the task's `Files`. Do not edit other modules to improve them.
- After each task: report (a) files changed, (b) verify output, (c) anything you had to assume.

## 1. Working protocol

1. Match the style rules in section 4 exactly. No exceptions.
2. No new dependencies unless the task names them. Ask before adding any.
3. No speculative abstraction, config, or error handling for impossible cases.
   Keep every node as simple as it can be while doing its job.
4. If two interpretations exist, pick none, ask.
5. Prefer the smallest diff that passes `Verify`.
6. Commit messages: subject line plus at most one short sentence of body. Make sure commit's do not ever include signatures by Claude or Anthropic. Every commit message should not say anything about claude, anthropic, or AI. Claude should not show as a contributer to the repo. 
7. Anything that commands the vehicle goes through the safety guard once it
   exists (task 8). Never publish setpoints from a new node directly after that
   point, ask if unsure.

## 2. Project facts (fixed, do not change)

| Thing | Value |
|---|---|
| Name | **ROS Project** (working name, not final) |
| Type | Autonomous inspection drone, simulation first, hardware later |
| Pitch | Copter flies a structure, finds defects with onboard vision AI, flies closer to confirm, and writes a plain language inspection report |
| Audience | Robotics recruiters reading the GitHub repo |
| Languages | Python 3.12 (rclpy) for all nodes in v1, C++ only if a task names it |
| ROS distro | **ROS 2 Jazzy Jalisco** (LTS, Ubuntu 24.04 base) |
| Autopilot | **ArduPilot Copter** (latest stable), SITL for all development |
| MAVLink bridge | **MAVROS 2** (the MAVLink requirement of this project) |
| Simulator | **Gazebo Harmonic** with the ArduPilot Gazebo plugin |
| Vision | YOLO model exported to ONNX, run with `onnxruntime` |
| Report AI | Claude API (model `claude-sonnet-5`), key via `ANTHROPIC_API_KEY` env var |
| Build | colcon workspace inside a Docker container |
| License | MIT |
| Repo root | `spaceproject/` (rename to `kestrel` when pushed to GitHub) |

## 3. Dev environment (both machines, one container)

The user develops on a Windows 11 laptop now and a Fedora Linux desktop as the
main machine. ROS 2 Jazzy targets Ubuntu 24.04, so neither machine runs it
natively. Everything runs inside one Docker image so both machines behave the
same.

- **Windows 11**: Docker Desktop with the WSL2 backend. GUI apps (Gazebo, rviz2)
  display through WSLg, which Windows 11 ships with. Run all commands from a
  WSL2 shell, not PowerShell, once the container exists.
- **Fedora**: Docker Engine or Podman. For GUI apps pass the Wayland or X11
  socket into the container, the compose file handles it.
- Headless mode must always work, CI has no display. Every launch file takes a
  `headless:=true` argument.

The image is defined in `docker/Dockerfile` (task 1). It contains ROS 2 Jazzy
desktop, Gazebo Harmonic, MAVROS, ArduPilot source with SITL built, and the
Python deps. The source tree mounts into the container at `/ws/src/kestrel`, so
edits on the host appear inside instantly.

Note on hyphens: the style rules below ban hyphenated words in everything we
write. Upstream identifiers we cannot rename (apt package names like
`ros-jazzy-mavros`, CLI flags like `--ros-args`) keep their hyphens, that is
tool syntax, not our prose.

## 4. Code style (enforced, every file, and in chat)

- **No hyphenated words, anywhere, full stop.** Not in code comments, docs,
  commit messages, PR titles or bodies, release notes, or chat replies. Use a
  synonym or rephrase instead of reaching for a hyphen. Only exception:
  upstream tool syntax as noted in section 3.
- **No em dashes** anywhere in the project (code, comments, YAML, docs). Use a
  comma or colon instead.
- Never leave variable names ambiguous. `b` becomes `byte`, `msg` becomes
  `stateMessage` or similar. Keep variables as obviously named as possible.
- **K&R braces** in any braced language: opening brace on the same line as the
  statement, closing brace on its own line.
- **Always brace** `if`/`for`/`while`/`do`/`switch` bodies. No single line
  `if (x) doThing();`.
- **Comments on the line above** the code they describe. Never trailing or inline.
- **One simple comment above every function**, including trivial ones. In Python
  this is a single `#` line directly above the `def`, one plain line.
- **No trailing periods** at the end of comments.
- Keep comments as simple as possible. Keep code as simple as possible.
- Python naming: `snake_case` functions and variables, `PascalCase` classes,
  `UPPER_SNAKE_CASE` constants. ROS package and node names in `snake_case`.
- Every Python file starts with a comment stating the file purpose in one line.

Example of required comment and naming style:

```python
# Send the copter to a local position and wait until it arrives
def fly_to_local_position(self, target_north, target_east, target_altitude):
    # Build the setpoint message once, reuse it while waiting
    setpoint_message = PoseStamped()
```

## 5. Architecture

```
                         +---------------------------+
                         |     Gazebo Harmonic        |
                         |  world: pylon + defects    |
                         |  copter model + camera     |
                         +------+-------------+------+
                                |             |
                        physics + motors   camera frames
                                |             |
+---------------+   MAVLink    |             |
| ArduCopter    +--------------+             |
| SITL          |                            |
+------+--------+                            |
       | MAVLink (udp 14550)                 |
+------+--------+                            |
| MAVROS 2      |                            |
+------+--------+                            |
       | ROS 2 topics and services           |
       |                                     |
  +----+-------------------------------------+----+
  |             ROS 2 graph (ROS Project nodes)    |
  |                                                |
  |  telemetry_monitor   watches state and battery |
  |  flight_commander    arm, takeoff, goto, land  |
  |  safety_guard        geofence, battery, RTL    |
  |  defect_detector     YOLO ONNX on camera feed  |
  |  inspection_planner  survey grid + closer look |
  |  mission_director    state machine, runs a job |
  |  report_writer       Claude API, writes report |
  +------------------------------------------------+
```

Data flow of one mission: `mission_director` asks `inspection_planner` for a
survey path around the structure, feeds waypoints to `flight_commander`,
`defect_detector` streams detections the whole time, on a detection the
director pauses the survey and requests a closer orbit of that point, photos
and detections accumulate, after landing `report_writer` turns them into a
markdown report with photos embedded.

### Key topics and services (our names, all under `/kestrel`)

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `/kestrel/detections` | `vision_msgs/Detection2DArray` | detector out | defect boxes per frame |
| `/kestrel/defect_events` | custom `DefectEvent.msg` | detector out | fired once per new confirmed defect, with estimated world position |
| `/kestrel/mission_state` | `std_msgs/String` | director out | current state machine state |
| `/kestrel/cmd/takeoff` | service `Takeoff.srv` | commander in | altitude in, success out |
| `/kestrel/cmd/goto` | service `GotoLocal.srv` | commander in | local NED target in, success out |
| `/kestrel/cmd/land` | service `std_srvs/Trigger` | commander in | land now |
| `/kestrel/abort` | service `std_srvs/Trigger` | guard in | anything may call, guard forces RTL |

MAVROS side (given, do not rename): `/mavros/state`, `/mavros/battery`,
`/mavros/local_position/pose`, `/mavros/setpoint_position/local`, services
`/mavros/cmd/arming`, `/mavros/set_mode`, `/mavros/cmd/takeoff`.

ArduPilot note: ArduCopter uses **GUIDED** mode for external control, not the
PX4 style OFFBOARD. Setpoints only act in GUIDED. `flight_commander` owns mode
switching.

## 6. Repo layout (create as you go, full target shape)

```
kestrel/
  README.md                 # pitch, GIF, architecture, quick start
  LICENSE                   # MIT text
  design.md                 # this file
  .gitignore
  docker/
    Dockerfile              # the one dev image
    compose.yaml            # dev service with GUI and headless profiles
  .github/workflows/
    ci.yaml                 # build image, colcon build, lint, SITL smoke test
  src/kestrel/              # single ROS 2 Python package for v1
    package.xml
    setup.py
    setup.cfg
    resource/kestrel
    kestrel/
      __init__.py
      telemetry_monitor.py
      flight_commander.py
      safety_guard.py
      defect_detector.py
      inspection_planner.py
      mission_director.py
      report_writer.py
    launch/
      sitl.launch.py        # SITL + MAVROS only
      sim.launch.py         # Gazebo world + SITL + MAVROS
      mission.launch.py     # everything, runs a full inspection
    config/
      kestrel_params.yaml   # every tunable in one file
    worlds/
      pylon_world.sdf       # inspection target world
    models/                 # copter with camera, defect markers
    test/
      test_smoke_sitl.py    # arm, takeoff, land, assert, headless
  src/kestrel_msgs/         # custom messages and services, ament_cmake package
    msg/DefectEvent.msg
    srv/Takeoff.srv
    srv/GotoLocal.srv
  models/vision/            # ONNX file lives here, git lfs or download script
  reports/                  # mission output lands here, gitignored
```

## 7. Phases and tasks

### Phase 0: scaffold and environment

**TASK 0: repo scaffold**
- Goal: empty but correct repo shape.
- Files: `README.md` (stub with pitch), `LICENSE`, `.gitignore` (Python, ROS,
  `reports/`, `models/vision/*.onnx`), `git init` on branch `main`.
- Steps: create files, first commit.
- Verify: `git log --oneline` shows one commit, `git status` clean.
- Done: repo exists and is clean.

**TASK 1: Docker dev image**
- Goal: one image that runs everything on both machines.
- Files: `docker/Dockerfile`, `docker/compose.yaml`.
- Steps:
  1. Base `ros:jazzy` (Ubuntu 24.04). Install `ros-jazzy-desktop`,
     `ros-jazzy-mavros`, `ros-jazzy-mavros-extras`, `gz-harmonic`,
     `ros-jazzy-ros-gzharmonic`, `ros-jazzy-vision-msgs`.
  2. Run the MAVROS geographiclib dataset install script, MAVROS fails
     silently without it.
  3. Clone ArduPilot at the latest stable Copter tag, run its prereq script,
     build SITL once so first launch is fast.
  4. Clone the ArduPilot Gazebo plugin and build it against Harmonic.
  5. `pip install onnxruntime anthropic pymavlink MAVProxy`.
  6. compose service `dev` mounts repo at `/ws/src/kestrel`, passes the
     display socket, plus a `headless` profile without it.
- Verify: `docker compose run dev ros2 doctor` reports no error, and
  `docker compose run dev sim_vehicle.py --help` prints usage.
- Done: both verify commands pass on the Windows machine.

**TASK 2: CI skeleton**
- Goal: GitHub Actions builds the image and the workspace on every push.
- Files: `.github/workflows/ci.yaml`.
- Steps: job 1 builds the Docker image with layer caching, job 2 runs
  `colcon build` and `colcon test` inside it. No SITL test yet, that is task 18.
- Verify: push a branch, Actions run is green.
- Done: green check on GitHub.

### Phase 1: bringup

**TASK 3: SITL runs**
- Goal: ArduCopter SITL boots inside the container and speaks MAVLink.
- Files: `docker/` tweaks only if needed, plus a `scripts/run_sitl.sh`.
- Steps: script wraps `sim_vehicle.py -v ArduCopter --no-rebuild` with output on
  udp 14550. No Gazebo yet, plain SITL physics.
- Verify: `mavproxy.py --master=udp:127.0.0.1:14550` connects, `mode GUIDED`
  then `arm throttle` succeeds after EKF settles.
- Done: manual arm through mavproxy works.

**TASK 4: MAVROS bridge**
- Goal: MAVROS connects to SITL and ROS 2 topics flow.
- Files: `src/kestrel/launch/sitl.launch.py`, `src/kestrel/config/kestrel_params.yaml`,
  package skeleton files (`package.xml`, `setup.py`, `setup.cfg`, `resource/`).
- Steps: launch file starts SITL (as an ExecuteProcess) and the MAVROS node
  pointed at udp 14550. Put the fcu url in the params yaml, not hardcoded.
- Verify: `ros2 topic echo /mavros/state --once` shows `connected: true`.
- Done: verify passes from a fresh container.

**TASK 5: telemetry monitor node**
- Goal: first ROS Project node, proves the rclpy plumbing.
- Files: `src/kestrel/kestrel/telemetry_monitor.py`, entry point in `setup.py`.
- Contract:
```python
# Watch vehicle state and battery and log a one line summary each second
class TelemetryMonitor(Node):
    # Subscribe to mavros state, battery, and local position
    def __init__(self):
    # Log mode, armed flag, battery percent, and altitude
    def log_summary(self):
```
- Verify: `ros2 run kestrel telemetry_monitor` prints a sane line per second
  while SITL runs.
- Done: output shows live changing altitude during a mavproxy commanded takeoff.

### Phase 2: control

**TASK 6: flight commander, arm and takeoff**
- Goal: programmatic arm and takeoff through services.
- Files: `src/kestrel/kestrel/flight_commander.py`, `src/kestrel_msgs/` package
  with `Takeoff.srv` (`float32 altitude` in, `bool success, string message` out).
- Steps: commander offers `/kestrel/cmd/takeoff`. Implementation: set mode
  GUIDED, wait for EKF ready via `/mavros/state` and home position set, call
  arming service, call MAVROS takeoff service, poll altitude until within 0.5 m
  of target or 30 s timeout.
- Verify: `ros2 service call /kestrel/cmd/takeoff kestrel_msgs/srv/Takeoff "{altitude: 5.0}"`
  returns success and telemetry monitor shows 5 m.
- Done: cold start to hover with two commands.

**TASK 7: goto and land**
- Goal: full basic flight loop.
- Files: `flight_commander.py`, `GotoLocal.srv` (`float32 north, float32 east,
  float32 altitude` in, `bool success, string message` out).
- Steps: goto publishes setpoints to `/mavros/setpoint_position/local` at 10 Hz
  until within 0.5 m, land calls MAVROS land and waits for disarm. Mind the
  ENU vs NED difference: MAVROS local frame is ENU, our service speaks north
  and east, convert inside the commander and put one comment above the
  conversion explaining it.
- Verify: `scripts/fly_square.sh`, a script that takes off, flies a 10 m square,
  lands. Watch positions in telemetry output.
- Done: square flight completes and vehicle disarms.

**TASK 8: safety guard**
- Goal: independent node that can always bring the vehicle home.
- Files: `src/kestrel/kestrel/safety_guard.py`, params in `kestrel_params.yaml`
  (`geofence_radius_m: 100.0`, `battery_floor_percent: 25.0`).
- Steps: subscribes to position and battery, offers `/kestrel/abort`. On fence
  breach, low battery, or abort call: set mode RTL directly via MAVROS and
  latch, further setpoints are pointless once RTL holds, log loudly.
- Verify: set `geofence_radius_m: 5.0`, fly the square, guard triggers RTL
  before the first corner.
- Done: RTL triggers and the vehicle returns and lands.

### Phase 3: simulation world

**TASK 9: Gazebo world and camera copter**
- Goal: visual world worth putting in a demo GIF.
- Files: `worlds/pylon_world.sdf`, `models/`, `launch/sim.launch.py`.
- Steps: world contains a transmission pylon or similar tall structure (source
  a free model, cite it in the README), ground plane, good lighting. Copter is
  the ArduPilot Gazebo iris model plus a gimbal fixed camera publishing
  640x480 at 15 fps. `sim.launch.py` = Gazebo + SITL with the Gazebo frame
  model + MAVROS, honors `headless:=true`.
- Verify: `ros2 launch kestrel sim.launch.py` shows the world, takeoff service
  still works, `ros2 topic hz /camera/image_raw` reports about 15 Hz.
- Done: flight around the pylon with live camera in `rqt_image_view`.

**TASK 10: defect markers in world**
- Goal: things for the detector to find.
- Files: `models/defect_marker/`, world edits.
- Steps: place four flat visual markers on the pylon (rust texture patches, or
  ArUco boards as stand ins). Positions listed in `kestrel_params.yaml` as
  ground truth for later scoring.
- Verify: markers visible in the camera feed when flying past.
- Done: screenshot captured for the README.

### Phase 4: perception

**TASK 11: defect detector node**
- Goal: ONNX inference on the camera stream.
- Files: `src/kestrel/kestrel/defect_detector.py`, `models/vision/download_model.sh`,
  `DefectEvent.msg` (`std_msgs/Header header, string label, float32 confidence,
  geometry_msgs/Point world_position, string image_path`).
- Steps: v1 uses a pretrained YOLO exported to ONNX, detecting the stand in
  markers (an ArUco detector via OpenCV is an acceptable v1 if YOLO on rust
  textures needs training we do not have data for: ask before choosing). Node
  subscribes to the image, runs inference at most 5 Hz, publishes
  `Detection2DArray` on `/kestrel/detections`. A defect seen in 5 consecutive
  frames fires one `DefectEvent` on `/kestrel/defect_events` with a world
  position estimated from vehicle pose plus camera geometry (rough is fine,
  document the assumption), and saves the frame to `reports/current/photos/`.
- Verify: fly past a marker manually, exactly one event fires per marker, image
  saved.
- Done: all four markers produce events in one flight.

### Phase 5: autonomy

**TASK 12: inspection planner**
- Goal: generate a survey path around a structure.
- Files: `src/kestrel/kestrel/inspection_planner.py`.
- Contract:
```python
# Build a vertical helix of waypoints around a structure for survey flight
def build_survey_path(center_north, center_east, structure_height, orbit_radius, climb_step):
# Build a small circle of waypoints around one point for a closer look
def build_orbit_path(point, orbit_radius, waypoint_count):
```
  Pure functions returning waypoint lists, unit tested, no ROS in this file.
- Verify: `colcon test` runs the new unit tests green.
- Done: tests cover path shape, spacing, and altitude bounds.

**TASK 13: mission director**
- Goal: the state machine that runs a whole inspection.
- Files: `src/kestrel/kestrel/mission_director.py`, `launch/mission.launch.py`.
- Steps: states `IDLE, TAKEOFF, SURVEY, INVESTIGATE, RESUME, RETURN, LANDED`.
  Survey follows the helix, on a `DefectEvent` push the current survey index,
  fly the orbit path around the defect, then resume. After the last waypoint,
  return and land. Publish state on `/kestrel/mission_state`. All calls go
  through the commander services, never raw setpoints.
- Verify: `ros2 launch kestrel mission.launch.py` completes a full autonomous
  inspection in sim with zero manual input, events for all markers, clean
  landing.
- Done: full mission recorded as a screen capture for the README GIF.

### Phase 6: AI report

**TASK 14: report writer**
- Goal: turn the mission log into a human readable inspection report.
- Files: `src/kestrel/kestrel/report_writer.py`.
- Steps: subscribes to defect events and mission state, on `LANDED` builds a
  prompt containing mission stats and each defect (label, confidence, world
  position, photo), sends photos plus prompt to the Claude API
  (`claude-sonnet-5`, the `anthropic` Python package), writes
  `reports/<timestamp>/report.md` with the model's findings section embedded
  above a machine generated appendix of raw detections. No API key means: skip
  the model call, write the appendix only, log a warning, never crash the
  mission.
- Verify: run the full mission with a key set, report file exists, findings
  reference the actual defects.
- Done: a sample report committed to `docs/sample_report.md` (photos included).

### Phase 7: polish and proof

**TASK 15: SITL smoke test in CI**
- Goal: the differentiator, autonomous flight tested on every push.
- Files: `src/kestrel/test/test_smoke_sitl.py`, `ci.yaml` update.
- Steps: pytest launches headless SITL + MAVROS, calls takeoff to 5 m, asserts
  altitude, lands, asserts disarm, hard 5 minute timeout. Runs in CI inside the
  image. Keep it plain SITL without Gazebo so CI stays fast.
- Verify: green Actions run including the smoke test.
- Done: badge in README.

**TASK 16: README and demo**
- Goal: the recruiter facing surface.
- Files: `README.md`, `docs/architecture.md`, GIF under `docs/`.
- Steps: pitch paragraph, mission GIF at the top, architecture diagram
  (mermaid), quick start (three commands: clone, compose build, launch
  mission), sample report link, roadmap section listing the stretch goals.
- Verify: a fresh reader can run the sim from README alone, test it in a clean
  container.
- Done: user reviews and approves the README.

### Stretch (post v1, do not start unless asked)

- Replace MAVROS with ArduPilot native DDS (`AP_DDS`) and compare latency.
- Train a real defect model on public corrosion datasets, drop the stand ins.
- Hardware target: Pixhawk 6C class controller on a small quad, companion
  computer runs the same nodes.
- Live video stream and report upload during flight.

## 8. Known traps (read before touching flight code)

- ArduPilot needs the EKF to converge before arming, expect roughly 30 s after
  SITL boot. Poll, never sleep a fixed time.
- ArduCopter external control is GUIDED mode. Setpoints in any other mode are
  ignored silently.
- MAVROS local frame is ENU, MAVLink and ArduPilot think NED. Every position
  bug in this project will be a frame mixup, convert in exactly one place
  (flight commander) and nowhere else.
- MAVROS without the geographiclib datasets connects but misbehaves, the
  Dockerfile must install them (task 1 step 2).
- Gazebo and SITL must agree on the vehicle frame: SITL runs with the Gazebo
  frame flag (`-f gazebo-iris` style, exact flag per ArduPilot Gazebo plugin
  docs) or physics fight each other.
- SITL speedup is tempting but breaks camera frame timing in Gazebo, run at
  real time for perception work.
- CI runners have no GPU, onnxruntime CPU provider only, keep the model small.

## 9. Verification philosophy

Every task ends in an observable behavior, a topic echo, a completed flight, a
file on disk, a green CI run. If you finish a task and the only evidence is
that code exists, the task is not done. When a verify step needs the sim, say
so in your report and include the log lines that prove the behavior.
