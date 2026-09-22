# Ribitics Robot

Ribitics is a local-first, upgradeable physical robot stack. The software is split into replaceable layers so the robot can keep its memory, identity, dashboard, integrations, and behavior while the computer, sensors, drivetrain, or AI model change.

## What works now

- FastAPI robot brain and responsive control dashboard
- Persistent SQLite long-term memory and recent conversation context
- Local Ollama conversation model with a basic offline fallback
- Robot-side wake phrase and offline Vosk speech recognition
- Local speaker output
- Optional USB camera with live frames and local motion detection
- Optional vendor-neutral 2D LiDAR with live radar visualization
- Fused obstacle awareness from bumpers, front proximity, and LiDAR
- Host-side forward-motion safety gate plus optional ESP32 bumper/ultrasonic gate
- ESP32-S3 + Cytron MDD10A differential-drive bridge and command watchdog
- Signed wheel-encoder telemetry for forward and reverse motion
- Differential-drive wheel odometry
- Local LiDAR occupancy map
- A* route planning with robot-radius obstacle inflation
- Low-speed supervised waypoint following
- Manual driving and STOP always cancel an active route
- Safe recovery workflow: stop first, then replan; no automatic reverse
- Session-local charging-dock approach foundation
- Simulation mode with virtual sensor states and map obstacles
- GitHub Actions tests for Python, dashboard structure, and dashboard JavaScript syntax

## Architecture

~~~text
                    RIBITICS
                       |
              FastAPI robot brain
          /         /       \          \
      memory     local AI   voice    integrations
      SQLite      Ollama    Vosk      your software
          \         |         /
               live context
                    |
         perception + localization
        /          |            \
    camera      2D LiDAR      encoders
   OpenCV          |              |
                obstacle       odometry
                sensing          |
                   \             /
                    local occupancy map
                            |
                    A* route planner
                            |
                 supervised navigator
                            |
                    spatial safety gate
                            |
                    robot controller
                            |
                         ESP32-S3
                            |
                      Cytron MDD10A
                       /         \
                  left motors  right motors
~~~

The ESP32 owns time-critical motor and safety behavior. The main computer owns conversation, local perception, dead reckoning, mapping, route planning, UI, and integrations.

## Run in simulation first

Python 3.11+ is recommended.

~~~bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux/macOS: source .venv/bin/activate
pip install -e ".[dev]"
cp .env.example .env
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
~~~

Open http://localhost:8000.

RIBITICS_MODE=simulation is the default. Simulation produces signed encoder ticks, so the same odometry and navigation code can be exercised without a physical chassis.

The dashboard lets you inject sensor states, place a virtual obstacle, reset the local pose, plan a destination, watch A* route around the obstacle, start/cancel supervised simulated motion, verify manual override, and mark a session-local dock.

## Local AI, voice, camera, and LiDAR

For fuller local conversation, install Ollama and set:

~~~env
RIBITICS_OLLAMA_URL=http://127.0.0.1:11434
RIBITICS_OLLAMA_MODEL=qwen2.5:3b
~~~

For robot-side voice on Linux / Raspberry Pi OS:

~~~bash
sudo apt update
sudo apt install -y libportaudio2 espeak-ng
pip install -e ".[voice]"
python scripts/download-vosk-model.py
~~~

Then:

~~~env
RIBITICS_ENABLE_VOICE_LOOP=true
RIBITICS_ENABLE_LOCAL_TTS=true
~~~

Camera:

~~~bash
pip install -e ".[vision]"
~~~

~~~env
RIBITICS_ENABLE_CAMERA=true
RIBITICS_CAMERA_DEVICE=0
~~~

2D LiDAR:

~~~bash
pip install -e ".[lidar]"
~~~

Default RPLIDAR A1 configuration:

~~~env
RIBITICS_ENABLE_LIDAR=true
RIBITICS_LIDAR_MODEL=RPLIDAR-A1
RIBITICS_LIDAR_PORT=/dev/ttyUSB0
RIBITICS_LIDAR_FORWARD_ANGLE_DEG=0
RIBITICS_LIDAR_FRONT_ARC_DEG=35
~~~

The LiDAR adapter is vendor-neutral through lds2d. Install voice, vision, and LiDAR together with:

~~~bash
pip install -e ".[robot]"
~~~

## v0.4 odometry

Ribitics now estimates a local x/y/heading pose from signed left/right encoder ticks.

This is dead reckoning, not global SLAM. Wheel slip, tire compression, uneven floors, inaccurate wheel diameter, and inaccurate wheel spacing all create drift.

The starter single-channel encoder firmware signs pulses from the commanded wheel direction. This is useful for the starter chassis, but true quadrature encoders are a future accuracy upgrade.

### Calibrate before physical navigation

The values in .env.example are development placeholders. Do not set RIBITICS_ODOMETRY_CALIBRATED=true until the actual robot has been measured and tested.

1. Lift the drive wheels off the floor and verify left/right motor direction and encoder activity.
2. Verify encoder sign. Forward must increase signed distance and reverse must decrease it. Correct LEFT_ENCODER_INVERT / RIGHT_ENCODER_INVERT in the ESP32 firmware if needed.
3. Measure loaded wheel diameter with the robot's normal weight on the tires.
4. Measure effective wheel base center-to-center between left and right wheel tracks.
5. Measure ticks per wheel revolution. Mark a wheel and record the signed tick change across one or preferably several exact revolutions.
6. Configure the measured values:

~~~env
RIBITICS_WHEEL_DIAMETER_CM=<measured>
RIBITICS_WHEEL_BASE_CM=<measured>
RIBITICS_ENCODER_TICKS_PER_REVOLUTION=<measured>
~~~

7. Manually drive a measured straight distance such as 100 cm at low speed. Compare real distance with dashboard odometry and refine the scale.
8. Perform a carefully measured in-place rotation and refine wheel-base calibration until estimated heading is reasonably close.
9. Repeat forward, reverse, left-turn, and right-turn tests on the actual floor.
10. Only after validation set:

~~~env
RIBITICS_ODOMETRY_CALIBRATED=true
~~~

Encoder odometry will still drift over distance. Calibration makes it useful for short local routes; it does not turn it into absolute positioning.

## Local occupancy mapping

When LiDAR and odometry are healthy, Ribitics ray-traces each scan into a local occupancy grid.

~~~env
RIBITICS_ENABLE_MAPPING=true
RIBITICS_MAP_RESOLUTION_CM=5
RIBITICS_MAP_SIZE_CM=1200
RIBITICS_MAP_ROBOT_RADIUS_CM=20
~~~

The map is centered on the local startup/reset origin. Occupied cells are inflated by the robot radius before planning so the route does not squeeze the robot through a gap only its center point can fit.

The v0.4 map is intentionally session-local. It does not yet perform loop closure or global scan-matching relocalization after a reboot.

## A* route planning

The dashboard can plan a local x/y destination against the occupancy grid.

The planner rejects out-of-map and occupied goals, uses eight-direction A*, inflates obstacles by robot radius, blocks unknown space on physical hardware by default, and produces local waypoints for the supervised follower.

The default goal-distance limit is 500 cm.

## Supervised navigation safety gate

Physical route execution is OFF by default:

~~~env
RIBITICS_ENABLE_SUPERVISED_NAVIGATION=false
~~~

Changing that to true is only one prerequisite. Physical execution still refuses to start unless:

- odometry is explicitly marked calibrated
- encoder odometry is ready and not stale
- 2D LiDAR is live
- the local occupancy map is ready
- spatial safety is not in a stop condition
- the goal is within the configured local-distance limit

Low normalized speed limits are used:

~~~env
RIBITICS_NAVIGATION_MAX_LINEAR=0.22
RIBITICS_NAVIGATION_MAX_ANGULAR=0.28
~~~

Every navigation movement still goes through the normal RobotController safety gate.

### Manual override

Manual control is always authoritative.

- touching manual drive cancels active navigation
- STOP cancels active navigation
- clearing the map cancels active navigation
- resetting odometry cancels active navigation
- a close obstacle stops navigation
- stale odometry stops navigation
- losing LiDAR during physical navigation stops navigation
- the ESP32 watchdog remains underneath the host software
- the physical latching E-stop remains the independent hard stop

## Recovery behavior

v0.4 does not automatically reverse or improvise when blocked.

If spatial safety blocks a route:

1. motors stop
2. navigation enters blocked
3. the dashboard shows recovery/replan availability
4. after the hazard or map changes, the operator can choose Replan stopped route
5. Ribitics computes a new route while remaining stopped
6. the operator must explicitly start the replanned route

## Charging-dock foundation

The dashboard can mark the robot's current local pose as the dock for the current session.

Ribitics computes an approach point behind the dock heading:

~~~env
RIBITICS_DOCK_APPROACH_DISTANCE_CM=60
~~~

You can plan and supervise a return to that approach point through the normal navigation gates.

v0.4 does not automatically make charging contact. Final alignment still requires dedicated close-range hardware such as fiducials, IR/beacon sensing, contact switches, or another verified method.

The dock pose is deliberately not persisted as a global coordinate yet because encoder dead reckoning does not provide a reliable cross-reboot global reference.

## Connect the physical ESP32

Flash firmware/esp32_motor_controller using PlatformIO. Verify every pin in src/main.cpp before applying motor power.

~~~env
RIBITICS_MODE=esp32
RIBITICS_SERIAL_PORT=/dev/ttyACM0
RIBITICS_SERIAL_BAUD=115200
~~~

Current telemetry includes signed encoder ticks and the direction mode:

~~~json
{
  "type": "telemetry",
  "estop": false,
  "left_ticks": 120,
  "right_ticks": 118,
  "encoder_direction_mode": "command_signed_single_channel",
  "left": 40,
  "right": 40
}
~~~

## Safety rules

Software is not the emergency stop.

- Use a latching physical E-stop that removes motor power through appropriately rated hardware.
- Fuse motor and computer power branches appropriately.
- Verify the E-stop before every physical navigation test.
- First powered motor tests should be done with wheels lifted.
- Keep the robot within line of sight during all v0.4 navigation tests.
- Keep people, pets, stairs, roads, loading docks, traffic, and fragile objects outside the test area.
- Start with an open, flat, controlled test area and very low speed.
- Never treat LiDAR, camera vision, odometry, FastAPI, Wi-Fi, or the ESP32 as a certified safety system.
- Do not enable physical supervised navigation with placeholder odometry values.
- Do not treat the local occupancy map as globally accurate.
- Do not enable unattended or public-space autonomous operation from this release.

## Tests

~~~bash
pytest -q
python -m compileall -q app
~~~

GitHub Actions also extracts the dashboard JavaScript and runs node --check so browser syntax regressions fail CI.

Coverage includes memory/API behavior, motor mixing, signed odometry, differential heading math, spatial safety, camera/LiDAR lazy loading, occupancy mapping, obstacle inflation, A* detours, supervised navigation, STOP cancellation, stopped recovery replanning, dock approach planning, and dashboard structure.

## Security before LAN use

Set a real control token:

~~~env
RIBITICS_CONTROL_TOKEN=a-long-random-secret
~~~

Movement, navigation, map mutation, camera snapshots, LiDAR scans, and other control operations use the same control-token protection.

## Next safe autonomy stage

The main missing piece is global localization, not more speed.

A future v0.5 should focus on LiDAR scan matching to correct wheel-odometry drift, persistent maps, relocalization after reboot, quadrature encoder support where hardware permits, pose-confidence scoring, route replanning from corrected pose, and dedicated dock fiducial/beacon/contact alignment.

Only after those are physically validated should unattended warehouse missions be considered.
