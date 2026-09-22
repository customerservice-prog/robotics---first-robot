from app.perception.lidar import LidarService


def test_lidar_service_is_lazy_and_starts_stopped():
    lidar = LidarService(
        auto_start=False,
        model="RPLIDAR-A1",
        port="/dev/ttyUSB0",
        forward_angle_deg=0,
        front_arc_deg=35,
        max_distance_mm=6000,
    )
    status = lidar.status()
    assert status.running is False
    assert status.ready is False
    assert status.state == "stopped"
    assert status.model == "RPLIDAR-A1"
    assert lidar.scan().points == []


def test_relative_angles_are_robot_centered():
    lidar = LidarService(
        auto_start=False,
        model="RPLIDAR-A1",
        port="/dev/ttyUSB0",
        forward_angle_deg=90,
        front_arc_deg=35,
        max_distance_mm=6000,
    )
    assert lidar._relative_angle(90) == 0
    assert lidar._relative_angle(100) == 10
    assert lidar._relative_angle(80) == -10
