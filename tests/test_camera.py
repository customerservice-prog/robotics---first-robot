from app.perception.camera import CameraService


def test_camera_service_is_lazy_and_starts_stopped():
    camera = CameraService(
        auto_start=False,
        device="0",
        width=640,
        height=480,
        fps=12,
        jpeg_quality=80,
        motion_threshold=12,
    )
    status = camera.status()
    assert status.running is False
    assert status.ready is False
    assert status.state == "stopped"
    assert status.device == "0"
    assert camera.snapshot() is None
