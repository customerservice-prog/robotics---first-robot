from app.hardware.base import RobotHardware
from app.models import RobotStatus


class SimulatedHardware(RobotHardware):
    def __init__(self, name: str):
        self.name = name
        self.left = 0
        self.right = 0
        self._estop = False
        self.message = "Simulation ready"

    def drive(self, left: int, right: int) -> None:
        if self._estop:
            self.left = self.right = 0
            self.message = "E-stop active"
            return
        self.left = int(max(-100, min(100, left)))
        self.right = int(max(-100, min(100, right)))
        self.message = f"Simulated drive L={self.left} R={self.right}"

    def stop(self) -> None:
        self.left = self.right = 0
        self.message = "Stopped"

    def status(self) -> RobotStatus:
        return RobotStatus(
            name=self.name,
            mode="simulation",
            connected=True,
            stopped=self.left == 0 and self.right == 0,
            estop=self._estop,
            left_motor=self.left,
            right_motor=self.right,
            battery_voltage=12.8,
            left_ticks=0,
            right_ticks=0,
            last_message=self.message,
        )
