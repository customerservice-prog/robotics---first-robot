from __future__ import annotations

from app.hardware.base import RobotHardware
from app.perception.spatial import SpatialAwareness


def mix_drive(linear: float, angular: float, limit: int = 65) -> tuple[int, int]:
    """Mix joystick linear/angular values into differential left/right motor percentages."""
    left = linear + angular
    right = linear - angular
    scale = max(1.0, abs(left), abs(right))
    left = left / scale
    right = right / scale
    return round(left * limit), round(right * limit)


class UnsafeDriveError(RuntimeError):
    pass


class RobotController:
    def __init__(
        self,
        hardware: RobotHardware,
        motor_limit: int = 65,
        awareness: SpatialAwareness | None = None,
    ):
        self.hardware = hardware
        self.motor_limit = max(10, min(100, motor_limit))
        self.awareness = awareness

    def drive(self, linear: float, angular: float) -> None:
        if self.awareness is not None:
            spatial = self.awareness.status()
            if self.hardware.status().estop:
                self.hardware.stop()
                raise UnsafeDriveError("Physical E-stop is active")
            if linear > 0.01 and not spatial.clear_to_move_forward:
                self.hardware.stop()
                raise UnsafeDriveError(spatial.reason)

        left, right = mix_drive(linear, angular, self.motor_limit)
        self.hardware.drive(left, right)

    def stop(self) -> None:
        self.hardware.stop()
