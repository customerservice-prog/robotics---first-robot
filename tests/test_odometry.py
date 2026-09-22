import math

from app.hardware.base import RobotHardware
from app.models import Pose2D, RobotStatus
from app.navigation.odometry import DifferentialOdometry


class EncoderHardware(RobotHardware):
    def __init__(self):
        self.left = 0
        self.right = 0

    def drive(self, left: int, right: int) -> None:
        pass

    def stop(self) -> None:
        pass

    def status(self) -> RobotStatus:
        return RobotStatus(
            name="test",
            mode="esp32",
            connected=True,
            stopped=True,
            estop=False,
            left_motor=0,
            right_motor=0,
            left_ticks=self.left,
            right_ticks=self.right,
            encoder_direction_mode="test_signed",
        )


def make_odometry(hardware):
    return DifferentialOdometry(
        hardware,
        wheel_diameter_cm=6.5,
        wheel_base_cm=15.0,
        ticks_per_revolution=360,
        poll_hz=20,
        stale_seconds=5,
        calibrated=True,
    )


def test_one_wheel_revolution_moves_forward_one_circumference():
    hardware = EncoderHardware()
    odom = make_odometry(hardware)
    odom.reset(Pose2D())
    hardware.left = 360
    hardware.right = 360
    status = odom.update_once()
    assert status.pose.x_cm == pytest_approx(math.pi * 6.5, 0.05)
    assert abs(status.pose.y_cm) < 0.05
    assert abs(status.pose.heading_deg) < 0.05


def test_signed_reverse_ticks_move_pose_backwards():
    hardware = EncoderHardware()
    odom = make_odometry(hardware)
    odom.reset(Pose2D())
    hardware.left = -180
    hardware.right = -180
    status = odom.update_once()
    assert status.pose.x_cm < -10
    assert abs(status.pose.heading_deg) < 0.05


def test_differential_ticks_change_heading():
    hardware = EncoderHardware()
    odom = make_odometry(hardware)
    odom.reset(Pose2D())
    hardware.left = -180
    hardware.right = 180
    status = odom.update_once()
    assert status.pose.heading_deg > 70
    assert status.pose.heading_deg < 90


def pytest_approx(value: float, tolerance: float):
    class Approx:
        def __eq__(self, other):
            return abs(other - value) <= tolerance

    return Approx()
