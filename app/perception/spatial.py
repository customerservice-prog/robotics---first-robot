from __future__ import annotations

from app.hardware.base import RobotHardware
from app.models import CameraStatus, SpatialStatus
from app.perception.lidar import LidarService


class SpatialAwareness:
    """Converts proximity, bumper, and LiDAR telemetry into a conservative forward decision."""

    def __init__(
        self,
        hardware: RobotHardware,
        *,
        stop_cm: float = 35.0,
        warn_cm: float = 70.0,
        require_proximity_for_forward: bool = False,
        lidar: LidarService | None = None,
    ) -> None:
        self.hardware = hardware
        self.stop_cm = max(1.0, float(stop_cm))
        self.warn_cm = max(self.stop_cm, float(warn_cm))
        self.require_proximity_for_forward = require_proximity_for_forward
        self.lidar = lidar

    def status(self) -> SpatialStatus:
        raw = self.hardware.status()
        lidar_status = self.lidar.status() if self.lidar is not None else None

        lidar_connected = raw.lidar_connected or bool(lidar_status and lidar_status.ready)
        lidar_front = self._minimum(
            raw.lidar_min_distance_cm,
            lidar_status.front_min_distance_cm if lidar_status else None,
        )
        left_distance = self._minimum(
            raw.left_distance_cm,
            lidar_status.left_min_distance_cm if lidar_status else None,
        )
        right_distance = self._minimum(
            raw.right_distance_cm,
            lidar_status.right_min_distance_cm if lidar_status else None,
        )
        forward_distances = [
            distance
            for distance in (raw.front_distance_cm, lidar_front)
            if distance is not None
        ]
        nearest = min(forward_distances) if forward_distances else None
        sensors_available = bool(
            forward_distances
            or raw.front_bumper_left
            or raw.front_bumper_right
            or lidar_connected
        )

        if raw.estop:
            return self._result(
                raw,
                left_distance,
                right_distance,
                lidar_connected,
                lidar_front,
                sensors_available,
                nearest,
                clear=False,
                level="stop",
                reason="Physical E-stop is active",
            )

        if raw.front_bumper_left or raw.front_bumper_right:
            return self._result(
                raw,
                left_distance,
                right_distance,
                lidar_connected,
                lidar_front,
                sensors_available,
                nearest,
                clear=False,
                level="stop",
                reason="Front bumper is pressed",
            )

        if nearest is not None and nearest <= self.stop_cm:
            return self._result(
                raw,
                left_distance,
                right_distance,
                lidar_connected,
                lidar_front,
                sensors_available,
                nearest,
                clear=False,
                level="stop",
                reason=f"Obstacle is {nearest:.1f} cm ahead",
            )

        if nearest is not None and nearest <= self.warn_cm:
            return self._result(
                raw,
                left_distance,
                right_distance,
                lidar_connected,
                lidar_front,
                sensors_available,
                nearest,
                clear=True,
                level="warning",
                reason=f"Object nearby at {nearest:.1f} cm",
            )

        if nearest is None and self.require_proximity_for_forward:
            return self._result(
                raw,
                left_distance,
                right_distance,
                lidar_connected,
                lidar_front,
                sensors_available,
                nearest,
                clear=False,
                level="unknown",
                reason="Forward proximity sensing is required but unavailable",
            )

        if nearest is None:
            return self._result(
                raw,
                left_distance,
                right_distance,
                lidar_connected,
                lidar_front,
                sensors_available,
                nearest,
                clear=True,
                level="unknown",
                reason="No forward proximity reading; manual line-of-sight control only",
            )

        return self._result(
            raw,
            left_distance,
            right_distance,
            lidar_connected,
            lidar_front,
            sensors_available,
            nearest,
            clear=True,
            level="clear",
            reason=f"Forward path clear to at least {nearest:.1f} cm",
        )

    def context_text(self, camera: CameraStatus | None = None) -> str:
        spatial = self.status()
        parts = [
            f"Forward safety: {spatial.hazard_level} ({spatial.reason}).",
            f"Clear to move forward: {'yes' if spatial.clear_to_move_forward else 'no'}.",
        ]
        if spatial.front_distance_cm is not None:
            parts.append(f"Front proximity distance: {spatial.front_distance_cm:.1f} cm.")
        if spatial.left_distance_cm is not None:
            parts.append(f"Left nearest distance: {spatial.left_distance_cm:.1f} cm.")
        if spatial.right_distance_cm is not None:
            parts.append(f"Right nearest distance: {spatial.right_distance_cm:.1f} cm.")
        if spatial.front_bumper_left or spatial.front_bumper_right:
            parts.append("A front bumper is pressed.")
        if spatial.lidar_connected:
            if spatial.lidar_min_distance_cm is not None:
                parts.append(
                    f"2D LiDAR is connected; nearest point in the forward arc is "
                    f"{spatial.lidar_min_distance_cm:.1f} cm."
                )
            else:
                parts.append("2D LiDAR is connected but has no valid forward point yet.")
        if camera is not None:
            if camera.ready:
                motion = "motion detected" if camera.motion_detected else "no significant motion"
                parts.append(
                    f"Camera is live at {camera.width}x{camera.height}; {motion}. "
                    "No object-recognition model is enabled, so do not identify objects from this status alone."
                )
            else:
                parts.append("Camera is not currently providing a live frame.")
        return " ".join(parts)

    def _result(
        self,
        raw,
        left_distance: float | None,
        right_distance: float | None,
        lidar_connected: bool,
        lidar_front: float | None,
        sensors_available: bool,
        nearest: float | None,
        *,
        clear: bool,
        level: str,
        reason: str,
    ) -> SpatialStatus:
        return SpatialStatus(
            sensors_available=sensors_available,
            clear_to_move_forward=clear,
            hazard_level=level,
            reason=reason,
            obstacle_stop_cm=self.stop_cm,
            obstacle_warn_cm=self.warn_cm,
            front_distance_cm=raw.front_distance_cm,
            left_distance_cm=left_distance,
            right_distance_cm=right_distance,
            nearest_forward_distance_cm=nearest,
            front_bumper_left=raw.front_bumper_left,
            front_bumper_right=raw.front_bumper_right,
            lidar_connected=lidar_connected,
            lidar_min_distance_cm=lidar_front,
        )

    @staticmethod
    def _minimum(*values: float | None) -> float | None:
        present = [value for value in values if value is not None]
        return min(present) if present else None
