from app.hardware.base import RobotHardware


def mix_drive(linear: float, angular: float, limit: int = 65) -> tuple[int, int]:
    """Mix joystick linear/angular values into differential left/right motor percentages."""
    left = linear + angular
    right = linear - angular
    scale = max(1.0, abs(left), abs(right))
    left = left / scale
    right = right / scale
    return round(left * limit), round(right * limit)


class RobotController:
    def __init__(self, hardware: RobotHardware, motor_limit: int = 65):
        self.hardware = hardware
        self.motor_limit = max(10, min(100, motor_limit))

    def drive(self, linear: float, angular: float) -> None:
        left, right = mix_drive(linear, angular, self.motor_limit)
        self.hardware.drive(left, right)

    def stop(self) -> None:
        self.hardware.stop()
