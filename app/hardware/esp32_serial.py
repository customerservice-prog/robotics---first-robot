from __future__ import annotations

import json
import threading
import time

import serial

from app.hardware.base import RobotHardware
from app.models import RobotStatus


class ESP32SerialHardware(RobotHardware):
    """JSON-lines protocol to the ESP32-S3 motor and safety controller."""

    def __init__(self, name: str, port: str, baud: int):
        self.name = name
        self.port = port
        self.baud = baud
        self._serial: serial.Serial | None = None
        self._lock = threading.Lock()
        self._telemetry: dict = {}
        self._left = 0
        self._right = 0
        self._last_message = "Disconnected"
        self._open()

    def _open(self) -> None:
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.05)
            self._last_message = f"Connected to {self.port}"
            threading.Thread(target=self._reader, daemon=True).start()
        except serial.SerialException as exc:
            self._serial = None
            self._last_message = f"Serial error: {exc}"

    def _reader(self) -> None:
        assert self._serial is not None
        while self._serial and self._serial.is_open:
            try:
                raw = self._serial.readline().decode("utf-8", errors="ignore").strip()
                if raw:
                    payload = json.loads(raw)
                    if payload.get("type") == "telemetry":
                        self._telemetry = payload
            except (json.JSONDecodeError, serial.SerialException):
                time.sleep(0.05)

    def _send(self, payload: dict) -> None:
        if not self._serial or not self._serial.is_open:
            self._last_message = "ESP32 is not connected"
            return
        wire = (json.dumps(payload, separators=(",", ":")) + "\n").encode()
        with self._lock:
            self._serial.write(wire)

    def drive(self, left: int, right: int) -> None:
        self._left = int(max(-100, min(100, left)))
        self._right = int(max(-100, min(100, right)))
        self._send({"cmd": "drive", "left": self._left, "right": self._right})
        self._last_message = f"Drive L={self._left} R={self._right}"

    def stop(self) -> None:
        self._left = self._right = 0
        self._send({"cmd": "stop"})
        self._last_message = "Stop command sent"

    def status(self) -> RobotStatus:
        connected = bool(self._serial and self._serial.is_open)
        return RobotStatus(
            name=self.name,
            mode="esp32",
            connected=connected,
            stopped=self._left == 0 and self._right == 0,
            estop=bool(self._telemetry.get("estop", False)),
            left_motor=self._left,
            right_motor=self._right,
            battery_voltage=self._telemetry.get("battery_voltage"),
            left_ticks=self._telemetry.get("left_ticks"),
            right_ticks=self._telemetry.get("right_ticks"),
            front_distance_cm=self._telemetry.get("front_distance_cm"),
            left_distance_cm=self._telemetry.get("left_distance_cm"),
            right_distance_cm=self._telemetry.get("right_distance_cm"),
            front_bumper_left=bool(self._telemetry.get("front_bumper_left", False)),
            front_bumper_right=bool(self._telemetry.get("front_bumper_right", False)),
            lidar_connected=bool(self._telemetry.get("lidar_connected", False)),
            lidar_min_distance_cm=self._telemetry.get("lidar_min_distance_cm"),
            last_message=self._last_message,
        )

    def close(self) -> None:
        self.stop()
        if self._serial and self._serial.is_open:
            self._serial.close()
