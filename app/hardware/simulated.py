from __future__ import annotations

import time

from app.hardware.base import RobotHardware
from app.models import RobotStatus, SimulationSensors


class SimulatedHardware(RobotHardware):
    def __init__(self, name: str, ticks_per_second_at_full_power: float = 360.0):
        self.name = name
        self.left = 0
        self.right = 0
        self._estop = False
        self.message = "Simulation ready"
        self._sensors = SimulationSensors()
        self._tick_rate = max(1.0, float(ticks_per_second_at_full_power))
        self._left_ticks = 0.0
        self._right_ticks = 0.0
        self._last_tick_at = time.monotonic()

    def _integrate_ticks(self) -> None:
        now = time.monotonic()
        elapsed = max(0.0, now - self._last_tick_at)
        self._last_tick_at = now
        self._left_ticks += elapsed * self._tick_rate * (self.left / 100.0)
        self._right_ticks += elapsed * self._tick_rate * (self.right / 100.0)

    def drive(self, left: int, right: int) -> None:
        self._integrate_ticks()
        if self._estop:
            self.left = self.right = 0
            self.message = "E-stop active"
            return
        self.left = int(max(-100, min(100, left)))
        self.right = int(max(-100, min(100, right)))
        self.message = f"Simulated drive L={self.left} R={self.right}"

    def stop(self) -> None:
        self._integrate_ticks()
        self.left = self.right = 0
        self.message = "Stopped"

    def set_sensors(self, sensors: SimulationSensors) -> None:
        self._sensors = sensors
        self.message = "Simulation spatial sensors updated"

    def reset_encoders(self) -> None:
        self._integrate_ticks()
        self._left_ticks = 0.0
        self._right_ticks = 0.0
        self.message = "Simulation encoders reset"

    def status(self) -> RobotStatus:
        self._integrate_ticks()
        return RobotStatus(
            name=self.name,
            mode="simulation",
            connected=True,
            stopped=self.left == 0 and self.right == 0,
            estop=self._estop,
            left_motor=self.left,
            right_motor=self.right,
            battery_voltage=12.8,
            left_ticks=int(round(self._left_ticks)),
            right_ticks=int(round(self._right_ticks)),
            encoder_direction_mode="simulation_signed",
            front_distance_cm=self._sensors.front_distance_cm,
            left_distance_cm=self._sensors.left_distance_cm,
            right_distance_cm=self._sensors.right_distance_cm,
            front_bumper_left=self._sensors.front_bumper_left,
            front_bumper_right=self._sensors.front_bumper_right,
            lidar_connected=self._sensors.lidar_connected,
            lidar_min_distance_cm=self._sensors.lidar_min_distance_cm,
            last_message=self.message,
        )
