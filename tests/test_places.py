import math
from datetime import datetime, timezone

from app.hardware.simulated import SimulatedHardware
from app.models import LidarPoint, LidarScan, Pose2D
from app.navigation.localization import CorrelativeLocalizer
from app.navigation.mapping import LocalOccupancyMap
from app.navigation.odometry import DifferentialOdometry
from app.navigation.places import PlaceRecognizer


class FakeLidar:
    def __init__(self, scan):
        self._scan = scan

    def scan(self):
        return self._scan


def make_world_points():
    points = []
    for index, angle in enumerate(range(-170, 180, 30)):
        radius = 140 + (index % 4) * 35
        radians = math.radians(angle)
        points.append((radius * math.cos(radians), radius * math.sin(radians)))
    return points


def make_scan(pose, world_points):
    points = []
    for x_cm, y_cm in world_points:
        dx = x_cm - pose.x_cm
        dy = y_cm - pose.y_cm
        distance = math.hypot(dx, dy)
        global_angle = math.degrees(math.atan2(dy, dx))
        relative = ((global_angle - pose.heading_deg + 180) % 360) - 180
        points.append(LidarPoint(angle_deg=relative, distance_cm=distance))
    return LidarScan(points=points, captured_at=datetime.now(timezone.utc))


def build_system(tmp_path, anchor_pose=Pose2D()):
    hardware = SimulatedHardware("Ribitics")
    odom = DifferentialOdometry(
        hardware,
        wheel_diameter_cm=6.5,
        wheel_base_cm=15,
        ticks_per_revolution=360,
        stale_seconds=10,
        calibrated=True,
    )
    odom.reset(anchor_pose)
    world_points = make_world_points()
    lidar = FakeLidar(make_scan(anchor_pose, world_points))
    mapping = LocalOccupancyMap(
        lidar,
        odom,
        resolution_cm=5,
        size_cm=800,
        robot_radius_cm=15,
        persistence_path=str(tmp_path / "map.json"),
    )
    for x_cm, y_cm in world_points:
        mapping.add_virtual_obstacle(x_cm, y_cm, 5)
    mapping.set_learning(False)

    localizer = CorrelativeLocalizer(
        mapping,
        lidar,
        odom,
        search_xy_cm=30,
        search_heading_deg=15,
        xy_step_cm=5,
        heading_step_deg=5,
        min_points=8,
        min_confidence=0.25,
        max_correction_cm=50,
        max_correction_deg=30,
        confidence_decay_distance_cm=500,
        confidence_decay_seconds=60,
    )
    places = PlaceRecognizer(
        mapping,
        lidar,
        odom,
        localizer,
        persistence_path=str(tmp_path / "places.json"),
        sectors=36,
        max_range_cm=600,
        min_valid_sectors=8,
        min_similarity=0.65,
        min_margin=0.0,
        max_anchors=20,
        verify_search_xy_cm=80,
        verify_search_heading_deg=30,
    )
    return hardware, odom, lidar, mapping, localizer, places, world_points


def test_place_descriptor_recovers_heading_rotation(tmp_path):
    _, _, lidar, _, _, places, world_points = build_system(tmp_path)
    anchor = places.capture_anchor("warehouse start")
    assert anchor.name == "warehouse start"

    live_pose = Pose2D(x_cm=0, y_cm=0, heading_deg=20)
    lidar._scan = make_scan(live_pose, world_points)
    result = places.recognize()
    assert result.recognized is True
    assert result.anchor_id == anchor.anchor_id
    assert result.similarity >= 0.65
    assert abs(result.estimated_pose.heading_deg - 20) <= 10


def test_place_recognition_requires_geometric_verification_before_pose_change(tmp_path):
    _, odom, lidar, _, _, places, world_points = build_system(tmp_path)
    places.capture_anchor("dock aisle")
    live_pose = Pose2D(x_cm=10, y_cm=-5, heading_deg=10)
    lidar._scan = make_scan(live_pose, world_points)

    before = odom.status().pose.model_copy()
    recognized = places.recognize()
    assert recognized.recognized is True
    assert odom.status().pose == before

    verified = places.recognize_and_relocalize()
    assert verified.recognized is True
    assert verified.verified is True
    assert verified.scan_match is not None
    assert verified.scan_match.matched is True
    assert odom.status().correction_count == 1


def test_place_anchor_file_rejects_different_map_id(tmp_path):
    _, _, _, mapping, _, places, _ = build_system(tmp_path)
    places.capture_anchor("known place")
    old_id = mapping.identity().map_id
    assert places.save() is True

    mapping.clear()
    assert mapping.identity().map_id != old_id
    places.invalidate_for_map_change()
    assert places.load() is False
    assert places.list_anchors() == []
    assert "different map ID" in places.status().last_error
