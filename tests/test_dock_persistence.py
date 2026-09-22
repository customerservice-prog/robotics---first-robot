from app.hardware.simulated import SimulatedHardware
from app.models import Pose2D
from app.navigation.docking import DockingFoundation
from app.navigation.mapping import LocalOccupancyMap
from app.navigation.odometry import DifferentialOdometry
from app.perception.lidar import LidarService


class FakeNavigator:
    pass


def build_dock(tmp_path):
    hardware = SimulatedHardware("Ribitics")
    odom = DifferentialOdometry(
        hardware,
        wheel_diameter_cm=6.5,
        wheel_base_cm=15,
        ticks_per_revolution=360,
        stale_seconds=10,
        calibrated=True,
    )
    odom.reset(Pose2D(x_cm=50, y_cm=25, heading_deg=90))
    lidar = LidarService(
        auto_start=False,
        model="RPLIDAR-A1",
        port="/dev/null",
        forward_angle_deg=0,
        front_arc_deg=35,
        max_distance_mm=6000,
    )
    mapping = LocalOccupancyMap(
        lidar,
        odom,
        resolution_cm=5,
        size_cm=600,
        robot_radius_cm=20,
        persistence_path=str(tmp_path / "map.json"),
    )
    mapping.add_virtual_obstacle(200, 200, 10)
    mapping.set_learning(False)
    mapping.save()
    dock_path = tmp_path / "dock.json"
    dock = DockingFoundation(
        odom,
        FakeNavigator(),
        mapping,
        approach_distance_cm=60,
        persistence_path=str(dock_path),
    )
    return odom, mapping, dock, dock_path


def test_dock_round_trip_on_same_map(tmp_path):
    odom, mapping, dock, dock_path = build_dock(tmp_path)
    status = dock.set_current_pose()
    assert status.configured is True
    assert status.persistent is True
    assert status.valid_for_current_map is True
    assert dock_path.exists()

    restored = DockingFoundation(
        odom,
        FakeNavigator(),
        mapping,
        approach_distance_cm=60,
        persistence_path=str(dock_path),
    )
    loaded = restored.load()
    assert loaded.configured is True
    assert loaded.valid_for_current_map is True
    assert loaded.map_id == mapping.identity().map_id


def test_dock_is_blocked_after_map_identity_changes(tmp_path):
    _, mapping, dock, _ = build_dock(tmp_path)
    dock.set_current_pose()
    old_id = mapping.identity().map_id
    mapping.clear()
    assert mapping.identity().map_id != old_id
    status = dock.status()
    assert status.configured is True
    assert status.valid_for_current_map is False
    assert status.approach_goal is None
