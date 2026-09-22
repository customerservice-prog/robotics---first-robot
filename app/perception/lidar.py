from __future__ import annotations

import threading
from datetime import datetime, timezone

from app.models import LidarPoint, LidarScan, LidarStatus


class LidarService:
    """Optional local 2D LiDAR scanner using the vendor-neutral lds2d package."""

    def __init__(
        self,
        *,
        auto_start: bool,
        model: str,
        port: str,
        forward_angle_deg: float,
        front_arc_deg: float,
        max_distance_mm: float,
    ) -> None:
        self.auto_start = auto_start
        self.model = model
        self.port = port
        self.forward_angle_deg = float(forward_angle_deg) % 360.0
        self.front_arc_deg = max(5.0, min(90.0, float(front_arc_deg)))
        self.max_distance_mm = max(250.0, float(max_distance_mm))

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._running = False
        self._ready = False
        self._state = "stopped"
        self._scan_points = 0
        self._scan_hz = 0.0
        self._front_min: float | None = None
        self._left_min: float | None = None
        self._right_min: float | None = None
        self._overall_min: float | None = None
        self._last_scan_at: datetime | None = None
        self._last_error = ""
        self._points: list[LidarPoint] = []

    def start(self) -> LidarStatus:
        if self._thread and self._thread.is_alive():
            return self.status()
        self._stop_event.clear()
        with self._lock:
            self._running = True
            self._ready = False
            self._state = "starting"
            self._last_error = ""
        self._thread = threading.Thread(target=self._run, name="ribitics-lidar", daemon=True)
        self._thread.start()
        return self.status()

    def stop(self) -> LidarStatus:
        self._stop_event.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=3.0)
        with self._lock:
            self._running = False
            self._ready = False
            self._state = "stopped"
        return self.status()

    def status(self) -> LidarStatus:
        with self._lock:
            return LidarStatus(
                auto_start=self.auto_start,
                running=self._running,
                ready=self._ready,
                state=self._state,
                model=self.model,
                port=self.port,
                scan_points=self._scan_points,
                scan_hz=round(self._scan_hz, 2),
                front_min_distance_cm=self._front_min,
                left_min_distance_cm=self._left_min,
                right_min_distance_cm=self._right_min,
                overall_min_distance_cm=self._overall_min,
                last_scan_at=self._last_scan_at,
                last_error=self._last_error,
            )

    def scan(self) -> LidarScan:
        with self._lock:
            return LidarScan(
                points=[point.model_copy() for point in self._points],
                captured_at=self._last_scan_at,
            )

    def _run(self) -> None:
        try:
            try:
                from lds2d import Lidar
            except ImportError as exc:
                raise RuntimeError(
                    "LiDAR support is not installed. Install with: pip install -e '.[lidar]'"
                ) from exc

            with Lidar.open(self.model, self.port) as lidar:
                for scan in lidar.scans():
                    if self._stop_event.is_set():
                        break

                    points: list[LidarPoint] = []
                    front: list[float] = []
                    left: list[float] = []
                    right: list[float] = []
                    all_distances: list[float] = []

                    for raw_point in scan.valid_points:
                        distance_mm = float(raw_point.dist_mm)
                        if distance_mm <= 0 or distance_mm > self.max_distance_mm:
                            continue
                        distance_cm = distance_mm / 10.0
                        relative_angle = self._relative_angle(float(raw_point.angle_deg))
                        quality = getattr(raw_point, "quality", None)
                        points.append(
                            LidarPoint(
                                angle_deg=round(relative_angle, 2),
                                distance_cm=round(distance_cm, 2),
                                quality=float(quality) if quality is not None else None,
                            )
                        )
                        all_distances.append(distance_cm)
                        if abs(relative_angle) <= self.front_arc_deg:
                            front.append(distance_cm)
                        elif self.front_arc_deg < relative_angle <= 120:
                            left.append(distance_cm)
                        elif -120 <= relative_angle < -self.front_arc_deg:
                            right.append(distance_cm)

                    captured_at = datetime.now(timezone.utc)
                    with self._lock:
                        self._points = points[:1080]
                        self._scan_points = len(points)
                        self._scan_hz = float(getattr(scan, "scan_freq_hz", 0.0) or 0.0)
                        self._front_min = self._minimum(front)
                        self._left_min = self._minimum(left)
                        self._right_min = self._minimum(right)
                        self._overall_min = self._minimum(all_distances)
                        self._last_scan_at = captured_at
                        self._running = True
                        self._ready = bool(points)
                        self._state = "scanning" if points else "waiting_for_points"
                        self._last_error = ""

        except Exception as exc:
            with self._lock:
                self._running = False
                self._ready = False
                self._state = "error"
                self._last_error = str(exc)
        finally:
            if self._stop_event.is_set():
                with self._lock:
                    self._running = False
                    self._ready = False
                    self._state = "stopped"

    def _relative_angle(self, angle_deg: float) -> float:
        return ((angle_deg - self.forward_angle_deg + 180.0) % 360.0) - 180.0

    @staticmethod
    def _minimum(values: list[float]) -> float | None:
        return round(min(values), 2) if values else None
