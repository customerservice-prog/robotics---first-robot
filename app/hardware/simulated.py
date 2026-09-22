from app.hardware.base import RobotHardware
from app.models import RobotStatus, SimulationSensors


class SimulatedHardware(RobotHardware):
    def __init__(self, name: str):
        self.name = name
        self.left = 0
        self.right = 0
        self._estop = False
        self.message = "Simulation ready"
        self._sensors = SimulationSensors()

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

    def set_sensors(self, sensors: SimulationSensors) -> None:
        self._sensors = sensors
        self.message = "Simulation spatial sensors updated"

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
            front_distance_cm=self._sensors.front_distance_cm,
            left_distance_cm=self._sensors.left_distance_cm,
            right_distance_cm=self._sensors.right_distance_cm,
            front_bumper_left=self._sensors.front_bumper_left,
            front_bumper_right=self._sensors.front_bumper_right,
            lidar_connected=self._sensors.lidar_connected,
            lidar_min_distance_cm=self._sensors.lidar_min_distance_cm,
            last_message=self.message,
        )
