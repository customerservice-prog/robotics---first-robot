from app.hardware.simulated import SimulatedHardware
from app.models import NavigationGoal, Pose2D
from app.navigation.mapping import LocalOccupancyMap
from app.navigation.navigator import SupervisedNavigator
from app.navigation.odometry import DifferentialOdometry
from app.navigation.planner import AStarPlanner
from app.perception.lidar import LidarService
from app.perception.spatial import SpatialAwareness
from app.robot import RobotController


def test_simulation_can_plan_supervised_route_without_hardware_enable_flag():
    hardware = SimulatedHardware("Ribitics")
    odom = DifferentialOdometry(
        hardware,
        wheel_diameter_cm=6.5,
        wheel_base_cm=15,
        ticks_per_revolution=360,
        stale_seconds=10,
        calibrated=False,
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
    awareness = SpatialAwareness(hardware, lidar=lidar)
    mapping = LocalOccupancyMap(lidar, odom, resolution_cm=5, size_cm=600, robot_radius_cm=20)
    planner = AStarPlanner(mapping, robot_radius_cm=20, allow_unknown=False)
    robot = RobotController(hardware, 65, awareness)
    navigator = SupervisedNavigator(
        robot=robot,
        odometry=odom,
        occupancy_map=mapping,
        planner=planner,
        awareness=awareness,
        lidar=lidar,
        hardware_mode="simulation",
        hardware_execution_enabled=False,
        allow_unknown=False,
        max_goal_distance_cm=500,
        max_linear=0.22,
        max_angular=0.28,
        waypoint_tolerance_cm=12,
        heading_tolerance_deg=12,
        timeout_seconds=30,
    )

    allowed, _ = navigator.hardware_execution_allowed()
    assert allowed is True
    plan = navigator.plan(NavigationGoal(x_cm=80, y_cm=40))
    assert plan.found is True
    assert plan.uses_unknown_space is True
    assert navigator.status().state == "planned"
