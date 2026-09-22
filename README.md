# Ribitics Robot

Ribitics Robot is a local-first, upgradeable robot software stack designed to start cheap and grow with the hardware.

The goal is **not** to lock the robot to one Raspberry Pi, one AI model, one camera, or one chassis. The software is split so the robot can keep its identity, memory, dashboard, integrations, and behavior while you replace physical parts over time.

## What works now

- FastAPI robot brain and control API
- Persistent SQLite memory
- Natural `remember that ...` teaching command
- Optional **local Ollama** conversation model (no paid cloud API required)
- Basic offline fallback responder when Ollama is not running
- Browser microphone input and spoken replies
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
        /       |       \
 memory     local AI    integrations
 SQLite      Ollama     your software
             |
       robot controller
             |
    USB serial JSON protocol
             |
          ESP32-S3
             |
       Cytron MDD10A
        /          \
 left motors    right motors
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

## Teach it

Say or type:

```text
Remember that the white resin chairs are stored in aisle 3.
```

The fact is saved to SQLite and is available after reboots.

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
- Never rely on Wi-Fi, the browser, FastAPI, or the ESP32 alone to stop a dangerous machine.

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
- bigger speaker / mic array
- tablet or touchscreen body

### V2
- 2D LiDAR
- ROS 2 navigation adapter
- local speech-to-text (Whisper/faster-whisper)
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

Movement endpoints require the token when the default value is changed. The dashboard currently reads a token from browser `localStorage` under `ribiticsToken`; a setup screen can be added next.

## Project status

This is the first usable foundation. It intentionally avoids hard-coding a specific chassis or AI vendor so the robot can evolve instead of being rebuilt from scratch.
