import pytest

from app.hardware.simulated import SimulatedHardware
from app.models import LidarStatus, SimulationSensors
from app.perception.spatial import SpatialAwareness
from app.robot import RobotController, UnsafeDriveError


class FakeLidar:
    def __init__(self, front_cm: float | None):
        self.front_cm = front_cm

    def status(self) -> LidarStatus:
        return LidarStatus(
            auto_start=False,
            running=True,
            ready=True,
            state="scanning",
            front_min_distance_cm=self.front_cm,
            left_min_distance_cm=120,
            right_min_distance_cm=140,
        )


def build_robot(*, require_proximity: bool = False, lidar=None):
    hardware = SimulatedHardware("Ribitics")
    awareness = SpatialAwareness(
        hardware,
        stop_cm=35,
        warn_cm=70,
        require_proximity_for_forward=require_proximity,
        lidar=lidar,
    )
    controller = RobotController(hardware, 65, awareness)
    return hardware, awareness, controller


def test_no_sensor_is_unknown_but_manual_forward_allowed_by_default():
    _, awareness, _ = build_robot()
    status = awareness.status()
    assert status.hazard_level == "unknown"
    assert status.clear_to_move_forward is True


def test_strict_mode_blocks_forward_when_proximity_is_missing():
    hardware, awareness, controller = build_robot(require_proximity=True)
    assert awareness.status().clear_to_move_forward is False
    with pytest.raises(UnsafeDriveError):
        controller.drive(1, 0)
    assert hardware.status().stopped is True


def test_close_obstacle_blocks_forward_and_stops():
    hardware, awareness, controller = build_robot()
    hardware.set_sensors(SimulationSensors(front_distance_cm=20))
    assert awareness.status().hazard_level == "stop"
    with pytest.raises(UnsafeDriveError, match="20.0 cm"):
        controller.drive(1, 0)
    assert hardware.status().left_motor == 0
    assert hardware.status().right_motor == 0


def test_warning_distance_allows_manual_forward():
    hardware, awareness, controller = build_robot()
    hardware.set_sensors(SimulationSensors(front_distance_cm=55))
    assert awareness.status().hazard_level == "warning"
    controller.drive(0.5, 0)
    assert hardware.status().left_motor > 0


def test_reverse_remains_available_with_front_obstacle():
    hardware, _, controller = build_robot()
    hardware.set_sensors(SimulationSensors(front_distance_cm=10))
    controller.drive(-0.5, 0)
    assert hardware.status().left_motor < 0
    assert hardware.status().right_motor < 0


def test_front_bumper_blocks_forward_even_without_distance():
    hardware, awareness, controller = build_robot()
    hardware.set_sensors(SimulationSensors(front_bumper_left=True))
    status = awareness.status()
    assert status.hazard_level == "stop"
    assert "bumper" in status.reason.lower()
    with pytest.raises(UnsafeDriveError):
        controller.drive(0.5, 0)


def test_close_usb_lidar_point_blocks_forward_and_populates_side_distances():
    hardware, awareness, controller = build_robot(lidar=FakeLidar(25))
    status = awareness.status()
    assert status.lidar_connected is True
    assert status.lidar_min_distance_cm == 25
    assert status.left_distance_cm == 120
    assert status.right_distance_cm == 140
    assert status.clear_to_move_forward is False
    with pytest.raises(UnsafeDriveError, match="25.0 cm"):
        controller.drive(0.5, 0)
