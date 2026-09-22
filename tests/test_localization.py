import math
from datetime import datetime, timezone

from app.hardware.simulated import SimulatedHardware
from app.models import LidarPoint, LidarScan, Pose2D, RelocalizeRequest
from app.navigation.localization import CorrelativeLocalizer
from app.navigation.mapping import LocalOccupancyMap
from app.navigation.odometry import DifferentialOdometry


class FakeLidar:
    def __init__(self, scan: LidarScan):
        self._scan = scan

    def scan(self) -> LidarScan:
        return self._scan


def make_scan(true_pose: Pose2D, world_points: list[tuple[float, float]]) -> LidarScan:
    points = []
    for x_cm, y_cm in world_points:
        dx = x_cm - true_pose.x_cm
        dy = y_cm - true_pose.y_cm
        distance = math.hypot(dx, dy)
        global_angle = math.degrees(math.atan2(dy, dx))
        relative = ((global_angle - true_pose.heading_deg + 180) % 360) - 180
        points.append(LidarPoint(angle_deg=relative, distance_cm=distance))
    return LidarScan(points=points, captured_at=datetime.now(timezone.utc))


def build_localizer(true_pose: Pose2D):
    hardware = SimulatedHardware("Ribitics")
    odom = DifferentialOdometry(
        hardware,
        wheel_diameter_cm=6.5,
        wheel_base_cm=15,
        ticks_per_revolution=360,
        stale_seconds=10,
        calibrated=True,
    )
    odom.reset(Pose2D())

    targets = [
        (100.0, 0.0),
        (0.0, 120.0),
        (-100.0, 50.0),
        (80.0, -100.0),
        (-60.0, -120.0),
        (145.0, 85.0),
    ]
    scan = make_scan(true_pose, targets)
    lidar = FakeLidar(scan)
    mapping = LocalOccupancyMap(
        lidar,
        odom,
        resolution_cm=5,
        size_cm=600,
        robot_radius_cm=15,
    )
    for target in targets:
        mapping.add_virtual_obstacle(target[0], target[1], 5)
    mapping.set_learning(False)

    localizer = CorrelativeLocalizer(
        mapping,
        lidar,
        odom,
        search_xy_cm=20,
        search_heading_deg=12,
        xy_step_cm=5,
        heading_step_deg=3,
        min_points=5,
        min_confidence=0.35,
        max_correction_cm=30,
        max_correction_deg=15,
        confidence_decay_distance_cm=500,
        confidence_decay_seconds=60,
    )
    return localizer, odom


def test_scan_match_recovers_known_pose_offset():
    true_pose = Pose2D(x_cm=10, y_cm=-5, heading_deg=6)
    localizer, _ = build_localizer(true_pose)
    result = localizer.match_scan(
        localizer.lidar.scan(),
        Pose2D(),
        search_xy_cm=20,
        search_heading_deg=12,
    )
    assert result.matched is True
    assert result.confidence >= 0.35
    assert abs(result.pose.x_cm - true_pose.x_cm) <= 5
    assert abs(result.pose.y_cm - true_pose.y_cm) <= 5
    assert abs(result.pose.heading_deg - true_pose.heading_deg) <= 3


def test_explicit_relocalization_applies_pose_correction():
    true_pose = Pose2D(x_cm=10, y_cm=-5, heading_deg=6)
    localizer, odom = build_localizer(true_pose)
    result = localizer.relocalize(
        RelocalizeRequest(
            hint_pose=Pose2D(),
            search_xy_cm=20,
            search_heading_deg=12,
        )
    )
    assert result.matched is True
    status = localizer.status()
    assert status.ready is True
    assert status.correction_count == 1
    assert odom.status().correction_count == 1
    assert abs(odom.status().pose.x_cm - true_pose.x_cm) <= 5


def test_match_rejects_scan_with_too_few_points():
    true_pose = Pose2D(x_cm=10, y_cm=-5, heading_deg=6)
    localizer, _ = build_localizer(true_pose)
    weak = LidarScan(
        points=[LidarPoint(angle_deg=0, distance_cm=100)],
        captured_at=datetime.now(timezone.utc),
    )
    result = localizer.match_scan(weak, Pose2D())
    assert result.matched is False
    assert "need at least" in result.reason
