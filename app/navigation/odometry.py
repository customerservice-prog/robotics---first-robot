from __future__ import annotations

import math
import threading
import time
from datetime import datetime, timezone

from app.hardware.base import RobotHardware
from app.models import OdometryStatus, Pose2D


def normalize_heading_deg(value: float) -> float:
    return ((value + 180.0) % 360.0) - 180.0


class DifferentialOdometry:
    """Encoder-only local dead reckoning for a differential-drive robot.

    This is intentionally not presented as global localization. Wheel slip and encoder
    calibration errors accumulate until a later scan-matching/localization layer corrects them.
    """

    def __init__(
        self,
        hardware: RobotHardware,
        *,
        wheel_diameter_cm: float,
        wheel_base_cm: float,
        ticks_per_revolution: float,
        poll_hz: float = 20.0,
        stale_seconds: float = 1.0,
        calibrated: bool = False,
    ) -> None:
        self.hardware = hardware
        self.wheel_diameter_cm = max(0.1, float(wheel_diameter_cm))
        self.wheel_base_cm = max(0.1, float(wheel_base_cm))
        self.ticks_per_revolution = max(1.0, float(ticks_per_revolution))
        self.poll_hz = max(1.0, float(poll_hz))
        self.stale_seconds = max(0.1, float(stale_seconds))
        self.config_calibrated = bool(calibrated)

        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False
        self._ready = False
        self._pose = Pose2D()
        self._left_ticks: int | None = None
        self._right_ticks: int | None = None
        self._last_tick_time: float | None = None
        self._last_update_monotonic: float | None = None
        self._last_update_at: datetime | None = None
        self._linear_velocity = 0.0
        self._angular_velocity = 0.0
        self._distance_traveled = 0.0
        self._last_error = ""

    def start(self) -> OdometryStatus:
        if self._thread and self._thread.is_alive():
            return self.status()
        self._stop_event.clear()
        with self._lock:
            self._running = True
            self._last_error = ""
        self._thread = threading.Thread(target=self._run, name="ribitics-odometry", daemon=True)
        self._thread.start()
        return self.status()

    def stop(self) -> OdometryStatus:
        self._stop_event.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._lock:
            self._running = False
            self._linear_velocity = 0.0
            self._angular_velocity = 0.0
        return self.status()

    def reset(self, pose: Pose2D | None = None, *, reset_hardware: bool = False) -> OdometryStatus:
        if reset_hardware:
            self.hardware.reset_encoders()
        raw = self.hardware.status()
        with self._lock:
            self._pose = Pose2D.model_validate((pose or Pose2D()).model_dump())
            self._pose.heading_deg = normalize_heading_deg(self._pose.heading_deg)
            self._left_ticks = raw.left_ticks
            self._right_ticks = raw.right_ticks
            now = time.monotonic()
            self._last_tick_time = now
            self._last_update_monotonic = now if raw.left_ticks is not None else None
            self._last_update_at = datetime.now(timezone.utc) if raw.left_ticks is not None else None
            self._ready = raw.left_ticks is not None and raw.right_ticks is not None
            self._linear_velocity = 0.0
            self._angular_velocity = 0.0
            self._distance_traveled = 0.0
            self._last_error = ""
        return self.status()

    def update_once(self) -> OdometryStatus:
        raw = self.hardware.status()
        left_ticks = raw.left_ticks
        right_ticks = raw.right_ticks
        if left_ticks is None or right_ticks is None:
            with self._lock:
                self._ready = False
                self._last_error = "Wheel encoder telemetry is unavailable"
            return self.status()

        now_mono = time.monotonic()
        now_dt = datetime.now(timezone.utc)

        with self._lock:
            if self._left_ticks is None or self._right_ticks is None or self._last_tick_time is None:
                self._left_ticks = left_ticks
                self._right_ticks = right_ticks
                self._last_tick_time = now_mono
                self._last_update_monotonic = now_mono
                self._last_update_at = now_dt
                self._ready = True
                self._last_error = ""
                return self._status_locked(raw.mode)

            delta_left_ticks = left_ticks - self._left_ticks
            delta_right_ticks = right_ticks - self._right_ticks
            elapsed = max(1e-4, now_mono - self._last_tick_time)
            self._left_ticks = left_ticks
            self._right_ticks = right_ticks
            self._last_tick_time = now_mono

            distance_per_tick = math.pi * self.wheel_diameter_cm / self.ticks_per_revolution
            left_cm = delta_left_ticks * distance_per_tick
            right_cm = delta_right_ticks * distance_per_tick
            center_cm = (left_cm + right_cm) / 2.0
            delta_heading_rad = (right_cm - left_cm) / self.wheel_base_cm

            heading_rad = math.radians(self._pose.heading_deg)
            midpoint_heading = heading_rad + delta_heading_rad / 2.0
            self._pose.x_cm += center_cm * math.cos(midpoint_heading)
            self._pose.y_cm += center_cm * math.sin(midpoint_heading)
            self._pose.heading_deg = normalize_heading_deg(
                self._pose.heading_deg + math.degrees(delta_heading_rad)
            )

            self._linear_velocity = center_cm / elapsed
            self._angular_velocity = math.degrees(delta_heading_rad) / elapsed
            self._distance_traveled += (abs(left_cm) + abs(right_cm)) / 2.0
            self._last_update_monotonic = now_mono
            self._last_update_at = now_dt
            self._ready = True
            self._last_error = ""
            return self._status_locked(raw.mode)

    def status(self) -> OdometryStatus:
        raw = self.hardware.status()
        with self._lock:
            return self._status_locked(raw.mode)

    def _status_locked(self, hardware_mode: str) -> OdometryStatus:
        now = time.monotonic()
        stale = (
            self._last_update_monotonic is None
            or now - self._last_update_monotonic > self.stale_seconds
        )
        calibrated = self.config_calibrated or hardware_mode == "simulation"
        return OdometryStatus(
            running=self._running,
            ready=self._ready,
            calibrated=calibrated,
            stale=stale,
            source="simulation_encoder_dead_reckoning"
            if hardware_mode == "simulation"
            else "encoder_dead_reckoning",
            pose=Pose2D.model_validate(self._pose.model_dump()),
            left_ticks=self._left_ticks,
            right_ticks=self._right_ticks,
            wheel_diameter_cm=self.wheel_diameter_cm,
            wheel_base_cm=self.wheel_base_cm,
            ticks_per_revolution=self.ticks_per_revolution,
            linear_velocity_cm_s=self._linear_velocity,
            angular_velocity_deg_s=self._angular_velocity,
            distance_traveled_cm=self._distance_traveled,
            last_update_at=self._last_update_at,
            last_error=self._last_error,
        )

    def _run(self) -> None:
        interval = 1.0 / self.poll_hz
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                self.update_once()
            except Exception as exc:
                with self._lock:
                    self._ready = False
                    self._last_error = f"Odometry update failed: {exc}"
            remaining = interval - (time.monotonic() - started)
            self._stop_event.wait(max(0.0, remaining))
