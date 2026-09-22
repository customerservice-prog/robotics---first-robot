from app.hardware.simulated import SimulatedHardware
from app.models import NavigationGoal, Pose2D
from app.navigation.mapping import LocalOccupancyMap
from app.navigation.odometry import DifferentialOdometry
from app.navigation.planner import AStarPlanner
from app.perception.lidar import LidarService


def build_map():
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
    )
    return mapping, odom


def test_virtual_obstacle_appears_in_snapshot():
    mapping, _ = build_map()
    mapping.add_virtual_obstacle(100, 0, 20)
    snapshot = mapping.snapshot()
    assert snapshot.occupied
    assert any(abs(cell.x_cm - 100) < 30 for cell in snapshot.occupied)


def test_astar_routes_around_inflated_obstacle():
    mapping, odom = build_map()
    mapping.add_virtual_obstacle(100, 0, 20)
    planner = AStarPlanner(mapping, robot_radius_cm=20, allow_unknown=True)
    plan = planner.plan(
        odom.status().pose,
        NavigationGoal(x_cm=200, y_cm=0),
        allow_unknown=True,
    )
    assert plan.found is True
    assert len(plan.waypoints) >= 2
    assert plan.planned_distance_cm > 180
    assert any(abs(point.y_cm) > 20 for point in plan.waypoints)


def test_goal_inside_inflated_obstacle_is_rejected():
    mapping, odom = build_map()
    mapping.add_virtual_obstacle(100, 0, 20)
    planner = AStarPlanner(mapping, robot_radius_cm=20, allow_unknown=True)
    plan = planner.plan(
        odom.status().pose,
        NavigationGoal(x_cm=100, y_cm=0),
        allow_unknown=True,
    )
    assert plan.found is False
    assert "occupied" in plan.reason.lower()



def test_sparse_map_persistence_round_trip(tmp_path):
    mapping, odom = build_map()
    path = tmp_path / "warehouse-map.json"
    mapping.persistence_path = path
    mapping.add_virtual_obstacle(100, 0, 20)
    mapping.set_learning(False)
    saved = mapping.save()
    assert saved.success is True
    assert path.exists()

    lidar = mapping.lidar
    restored = LocalOccupancyMap(
        lidar,
        odom,
        resolution_cm=5,
        size_cm=600,
        robot_radius_cm=20,
        persistence_path=str(path),
    )
    loaded = restored.load()
    assert loaded.success is True
    status = restored.status()
    assert status.loaded_from_disk is True
    assert status.learning_enabled is False
    assert status.dirty is False
    assert restored.grid_copy() == mapping.grid_copy()



def test_map_identity_and_revision_round_trip(tmp_path):
    mapping, odom = build_map()
    path = tmp_path / "identity-map.json"
    mapping.persistence_path = path
    mapping.add_virtual_obstacle(80, 40, 10)
    original_id = mapping.identity().map_id
    saved = mapping.save()
    assert saved.success is True
    assert saved.map_id == original_id
    assert saved.revision >= 1

    restored = LocalOccupancyMap(
        mapping.lidar,
        odom,
        resolution_cm=5,
        size_cm=600,
        robot_radius_cm=20,
        persistence_path=str(path),
    )
    loaded = restored.load()
    assert loaded.success is True
    assert restored.identity().map_id == original_id
    assert restored.identity().revision == saved.revision
    snapshot = restored.snapshot()
    assert snapshot.map_id == original_id
    assert snapshot.revision == saved.revision

    restored.clear()
    assert restored.identity().map_id != original_id
    assert restored.identity().revision == 0
