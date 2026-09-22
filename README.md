# Ribitics Robot

Ribitics Robot is a local-first, upgradeable robot software stack designed to start cheap and grow with the hardware.

The goal is **not** to lock the robot to one Raspberry Pi, one AI model, one camera, or one chassis. The software is split so the robot can keep its identity, memory, dashboard, integrations, and behavior while you replace physical parts over time.

## What works now

- FastAPI robot brain and control API
- Persistent SQLite long-term memory
- Persistent recent conversation context across restarts
- Natural `remember that ...` teaching command
- Optional **local Ollama** conversation model (no paid cloud API required)
- Basic offline fallback responder when Ollama is not running
- Browser microphone input and spoken replies
- Robot-side always-listening USB microphone loop
- Offline Vosk wake phrase + speech recognition (`Hey Ribitics` by default)
- Local robot speaker output through `espeak-ng` or another command
- Dashboard voice state, last-heard text, last reply, errors, and start/stop controls
- Optional USB camera with live dashboard frames and local motion detection
- Optional vendor-neutral 2D LiDAR service with a live radar view
- Fused spatial awareness from front proximity, bumpers, and LiDAR
- Host-side forward-motion safety gate with configurable stop/warning distances
- Matching optional ESP32 front bumper / ultrasonic safety gate
- Simulation controls for testing clear, warning, blocked, and bumper states without hardware
- Responsive phone/tablet robot dashboard
- Manual differential-drive controls
- Simulation mode so development can start without hardware
- ESP32-S3 serial hardware adapter
- Starter ESP32-S3 + Cytron MDD10A firmware
- Motor command watchdog that stops if commands disappear
- Physical E-stop telemetry input
- Wheel encoder telemetry hooks
- Replaceable integration interface for external software

## Architecture

```text
Phone / tablet / robot touchscreen
             |
        Web dashboard
             |
                 FastAPI robot brain
          /            |             \
      memory         local AI      integrations
      SQLite          Ollama       your software
          \             |             /
             live robot context
             /              \
       camera/OpenCV      spatial awareness
                         /       |       \
                   proximity  bumpers   2D LiDAR
                         \       |       /
                           safety gate
                               |
                       robot controller
                               |
                      USB serial protocol
                               |
                            ESP32-S3
                               |
                         Cytron MDD10A
                         /           \
                    left motors   right motors
```

The ESP32 owns time-critical motor/sensor behavior. The main computer owns conversation, memory, UI, vision, and software integrations.

## Run it before buying hardware

Python 3.11+ is recommended.

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux/macOS: source .venv/bin/activate
pip install -e ".[dev]"
cp .env.example .env
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

Open <http://localhost:8000>.

The default `RIBITICS_MODE=simulation` means movement commands are simulated and shown in the dashboard.

## Give Ribitics a local AI model

The app is intentionally usable without a model. For fuller conversation, install Ollama on the robot computer and pull a small model that fits your hardware, then update `.env` if needed:

```env
RIBITICS_OLLAMA_URL=http://127.0.0.1:11434
RIBITICS_OLLAMA_MODEL=qwen2.5:3b
```

If Ollama is not available, Ribitics automatically uses a small built-in fallback responder; memory and robot control continue to work.

## Turn on fully standalone voice

The standalone voice path is designed for Raspberry Pi-class or used mini-PC hardware and does not require a paid speech API.

On Linux / Raspberry Pi OS:

~~~bash
sudo apt update
sudo apt install -y libportaudio2 espeak-ng
pip install -e ".[voice]"
python scripts/download-vosk-model.py
~~~

Then enable the robot microphone and speaker in `.env`:

~~~env
RIBITICS_ENABLE_VOICE_LOOP=true
RIBITICS_ENABLE_LOCAL_TTS=true
RIBITICS_WAKE_PHRASE=hey ribitics
RIBITICS_VOSK_MODEL_PATH=models/vosk-model-small-en-us-0.15
~~~

Start it with:

~~~bash
bash scripts/run-voice.sh
~~~

On Windows, use `scripts/run-voice.ps1`.

You can speak the wake phrase and command together — “Hey Ribitics what do you remember about the warehouse?” — or pause after “Hey Ribitics” and then give the command.

If the wrong microphone is selected, set `RIBITICS_MICROPHONE_DEVICE` to a sounddevice device number or device name. The dashboard exposes the microphone/model state and any startup error instead of failing silently.

## Turn on camera vision

Camera support stays optional so the rest of the robot can run without OpenCV:

~~~bash
pip install -e ".[vision]"
~~~

Then configure:

~~~env
RIBITICS_ENABLE_CAMERA=true
RIBITICS_CAMERA_DEVICE=0
RIBITICS_CAMERA_WIDTH=640
RIBITICS_CAMERA_HEIGHT=480
RIBITICS_CAMERA_FPS=12
~~~

The dashboard shows the latest local camera frame, motion state, resolution, frame rate, and camera errors. Motion detection is local and lightweight. Ribitics does **not** claim to recognize objects unless a separate object-recognition model is added later.

## Turn on 2D LiDAR

The LiDAR service uses the vendor-neutral `lds2d` driver layer. Install it with:

~~~bash
pip install -e ".[lidar]"
~~~

The default configuration is ready for an RPLIDAR A1:

~~~env
RIBITICS_ENABLE_LIDAR=true
RIBITICS_LIDAR_MODEL=RPLIDAR-A1
RIBITICS_LIDAR_PORT=/dev/ttyUSB0
RIBITICS_LIDAR_FORWARD_ANGLE_DEG=0
RIBITICS_LIDAR_FRONT_ARC_DEG=35
RIBITICS_LIDAR_MAX_DISTANCE_MM=6000
~~~

Other supported model names include `RPLIDAR-C1`, `YDLIDAR-X4`, and `LDROBOT-LD14P`. The dashboard renders a live 2D radar view. The nearest valid LiDAR point inside the configured forward arc participates in the same forward-motion safety gate as the bumpers and front proximity sensor.

To install voice + vision + LiDAR together:

~~~bash
pip install -e ".[robot]"
~~~

## Test spatial safety without hardware

Simulation mode can inject sensor states through the dashboard. The built-in shortcuts include:

- clear path at 150 cm
- warning zone at 55 cm
- blocking obstacle at 20 cm
- pressed front bumper
- no proximity sensor

With the default 35 cm stop threshold, a 20 cm simulated obstacle returns HTTP 409 for a forward drive request, stops the motors, and still allows reverse so the robot can be recovered.

## Teach it

Say or type:

```text
Remember that the white resin chairs are stored in aisle 3.
```

The fact is saved to SQLite and is available after reboots. Recent user/assistant turns are also stored locally so follow-up questions can keep context when Ollama is running.

## Connect the physical ESP32

Flash `firmware/esp32_motor_controller` using PlatformIO. **Verify every pin in `src/main.cpp` before applying motor power.**

Then configure:

```env
RIBITICS_MODE=esp32
RIBITICS_SERIAL_PORT=/dev/ttyACM0
RIBITICS_SERIAL_BAUD=115200
```

Windows ports look like `COM3`, `COM4`, etc.

The host sends newline-delimited JSON such as:

```json
{"cmd":"drive","left":40,"right":40}
{"cmd":"stop"}
```

The ESP32 sends telemetry:

```json
{"type":"telemetry","estop":false,"left_ticks":120,"right_ticks":118,"left":40,"right":40}
```

## Safety rules

Software is **not** the emergency stop.

- Wire a latching physical E-stop so it removes motor power through an appropriately rated relay/contactor.
- Fuse the battery branch and computer branch separately.
- Test with the wheels lifted off the floor first.
- Begin with low motor limits; the default software limit is 65%.
- Keep manual control within line of sight.
- Confirm motor polarity and encoder direction before floor testing.
- Never rely on Wi-Fi, the browser, voice recognition, camera vision, LiDAR, FastAPI, or the ESP32 alone to stop a dangerous machine.
- The proximity/LiDAR gate is supplemental software safety, not a certified collision-avoidance system.
- Voice currently handles conversation, not unsupervised driving. Autonomous navigation remains disabled until wheel odometry, mapping, localization, braking distance, and collision behavior are physically validated.

## External software integrations

`app/integrations/base.py` defines the adapter contract. Add integrations behind this interface instead of mixing business-specific code into motor control.

Examples we can add next:

- rental orders and inventory
- schedules and delivery routes
- warehouse tasks
- barcode / QR inventory scans
- virtual-office agents
- maintenance reminders

## Hardware upgrade path

### V1
- used mini-PC or Raspberry Pi-class computer
- ESP32-S3
- Cytron MDD10A
- differential drive base
- USB camera
- USB microphone + speaker
- physical E-stop

### V1.5
- wheel encoders fully calibrated
- bumper sensors
- ultrasonic / ToF sensors
- USB camera
- bigger speaker / mic array
- tablet or touchscreen body

### V2
- 2D LiDAR (software adapter and dashboard are now ready)
- wheel odometry calibration
- local occupancy map / localization
- ROS 2 navigation adapter if needed
- optional heavier Whisper/faster-whisper speech-to-text upgrade
- camera object detection
- automatic charging dock

### V3
- Jetson-class AI computer if needed
- robotic arm / gripper
- RFID / barcode / UWB hardware
- autonomous warehouse tasks

## Tests

```bash
pytest -q
```

## Security before LAN use

Set a real control token in `.env`:

```env
RIBITICS_CONTROL_TOKEN=a-long-random-secret
```

Movement and voice start/stop endpoints require the token when the default value is changed. The dashboard reads a token from browser `localStorage` under `ribiticsToken`.

## Project status

Ribitics now has a local standalone perception-and-control foundation: simulation, persistent memory, local AI, robot-side voice, USB camera, 2D LiDAR, fused obstacle awareness, dashboard controls, and the ESP32 motor bridge are separate layers. The next safe autonomy stage is calibrated wheel odometry + mapping/localization; autonomous driving is intentionally not enabled yet.
