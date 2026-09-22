from __future__ import annotations

import json
import math
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from app.models import (
    LidarScan,
    MapCell,
    MapPersistenceResult,
    MapSnapshot,
    MapStatus,
    Pose2D,
)
from app.navigation.odometry import DifferentialOdometry
from app.perception.lidar import LidarService


class LocalOccupancyMap:
    """Sparse local occupancy grid built from LiDAR + corrected local pose."""

    FORMAT_NAME = "ribitics_sparse_occupancy"
    FORMAT_VERSION = 1

    def __init__(
        self,
        lidar: LidarService,
        odometry: DifferentialOdometry,
        *,
        resolution_cm: float = 5.0,
        size_cm: float = 1200.0,
        robot_radius_cm: float = 20.0,
        poll_hz: float = 5.0,
        persistence_path: str = "data/ribitics-map.json",
    ) -> None:
        self.lidar = lidar
        self.odometry = odometry
        self.resolution_cm = max(1.0, float(resolution_cm))
        self.size_cm = max(self.resolution_cm * 20, float(size_cm))
        self.robot_radius_cm = max(1.0, float(robot_radius_cm))
        self.poll_hz = max(0.5, float(poll_hz))
        self.persistence_path = Path(persistence_path)
        self.width_cells = max(20, int(round(self.size_cm / self.resolution_cm)))
        if self.width_cells % 2:
            self.width_cells += 1
        self.half_cells = self.width_cells // 2

        self._lock = threading.Lock()
        self._scores: dict[tuple[int, int], int] = {}
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False
        self._updates = 0
        self._last_scan_at: datetime | None = None
        self._last_update_at: datetime | None = None
        self._last_saved_at: datetime | None = None
        self._last_loaded_at: datetime | None = None
        self._loaded_from_disk = False
        self._learning_enabled = True
        self._dirty = False
        self._last_error = ""

    def start(self) -> MapStatus:
        if self._thread and self._thread.is_alive():
            return self.status()
        self._stop_event.clear()
        with self._lock:
            self._running = True
            self._last_error = ""
        self._thread = threading.Thread(target=self._run, name="ribitics-mapper", daemon=True)
        self._thread.start()
        return self.status()

    def stop(self) -> MapStatus:
        self._stop_event.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._lock:
            self._running = False
        return self.status()

    def clear(self) -> MapStatus:
        with self._lock:
            self._scores.clear()
            self._updates = 0
            self._last_scan_at = None
            self._last_update_at = datetime.now(timezone.utc)
            self._loaded_from_disk = False
            self._learning_enabled = True
            self._dirty = True
            self._last_error = ""
        return self.status()

    def save(self, path: str | None = None) -> MapPersistenceResult:
        target = Path(path) if path else self.persistence_path
        target.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            entries = [[x, y, score] for (x, y), score in self._scores.items() if score != 0]
            payload = {
                "format": self.FORMAT_NAME,
                "version": self.FORMAT_VERSION,
                "resolution_cm": self.resolution_cm,
                "size_cm": self.size_cm,
                "robot_radius_cm": self.robot_radius_cm,
                "updates": self._updates,
                "scores": entries,
                "saved_at": datetime.now(timezone.utc).isoformat(),
            }

        temp = target.with_suffix(target.suffix + ".tmp")
        try:
            temp.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
            temp.replace(target)
            saved_at = datetime.now(timezone.utc)
            with self._lock:
                self._last_saved_at = saved_at
                self._dirty = False
                self._last_error = ""
            return MapPersistenceResult(
                success=True,
                action="save",
                path=str(target),
                cell_count=len(entries),
                reason="Sparse occupancy map saved atomically",
            )
        except (OSError, ValueError) as exc:
            with self._lock:
                self._last_error = f"Map save failed: {exc}"
            return MapPersistenceResult(
                success=False,
                action="save",
                path=str(target),
                cell_count=0,
                reason=str(exc),
            )
        finally:
            if temp.exists():
                try:
                    temp.unlink()
                except OSError:
                    pass

    def load(self, path: str | None = None) -> MapPersistenceResult:
        target = Path(path) if path else self.persistence_path
        if not target.exists():
            return MapPersistenceResult(
                success=False,
                action="load",
                path=str(target),
                reason="Persistent map file does not exist",
            )
        try:
            payload = json.loads(target.read_text(encoding="utf-8"))
            self._validate_payload(payload)
            loaded_scores: dict[tuple[int, int], int] = {}
            for item in payload.get("scores", []):
                if not isinstance(item, list) or len(item) != 3:
                    continue
                x, y, score = int(item[0]), int(item[1]), int(item[2])
                cell = (x, y)
                if self.in_bounds(cell) and -5 <= score <= 5 and score != 0:
                    loaded_scores[cell] = score
            loaded_at = datetime.now(timezone.utc)
            with self._lock:
                self._scores = loaded_scores
                self._updates = int(payload.get("updates", 0))
                self._last_scan_at = None
                self._last_update_at = loaded_at
                self._last_loaded_at = loaded_at
                self._loaded_from_disk = True
                self._learning_enabled = False
                self._dirty = False
                self._last_error = ""
            return MapPersistenceResult(
                success=True,
                action="load",
                path=str(target),
                cell_count=len(loaded_scores),
                reason=(
                    "Persistent map loaded. Navigation must remain locked until live "
                    "localization confirms the robot pose."
                ),
            )
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            with self._lock:
                self._last_error = f"Map load failed: {exc}"
            return MapPersistenceResult(
                success=False,
                action="load",
                path=str(target),
                reason=str(exc),
            )

    def set_learning(self, enabled: bool) -> MapStatus:
        with self._lock:
            self._learning_enabled = bool(enabled)
            self._last_error = ""
        return self.status()

    def ingest_scan(self, scan: LidarScan, pose: Pose2D) -> bool:
        if not scan.points:
            return False
        robot_cell = self.world_to_cell(pose.x_cm, pose.y_cm)
        if robot_cell is None:
            with self._lock:
                self._last_error = "Robot pose is outside the configured local map"
            return False

        rays: list[tuple[list[tuple[int, int]], tuple[int, int]]] = []
        heading_rad = math.radians(pose.heading_deg)

        for point in scan.points:
            distance = float(point.distance_cm)
            if distance <= self.resolution_cm or distance > self.size_cm * 0.7:
                continue
            angle = heading_rad + math.radians(float(point.angle_deg))
            end_x = pose.x_cm + distance * math.cos(angle)
            end_y = pose.y_cm + distance * math.sin(angle)
            end_cell = self.world_to_cell(end_x, end_y)
            if end_cell is None:
                continue
            ray = self._bresenham(robot_cell, end_cell)
            if len(ray) >= 2:
                rays.append((ray[:-1], end_cell))

        if not rays:
            return False

        with self._lock:
            self._scores[robot_cell] = min(-1, self._scores.get(robot_cell, 0) - 1)
            for free_cells, occupied_cell in rays:
                for cell in free_cells:
                    current = self._scores.get(cell, 0)
                    self._scores[cell] = max(-5, current - 1)
                current = self._scores.get(occupied_cell, 0)
                self._scores[occupied_cell] = min(5, current + 3)
            self._updates += 1
            self._last_scan_at = scan.captured_at
            self._last_update_at = datetime.now(timezone.utc)
            self._dirty = True
            self._last_error = ""
        return True

    def add_virtual_obstacle(self, x_cm: float, y_cm: float, radius_cm: float) -> MapStatus:
        center = self.world_to_cell(x_cm, y_cm)
        if center is None:
            raise ValueError("Virtual obstacle is outside the configured local map")
        radius_cells = max(1, int(math.ceil(radius_cm / self.resolution_cm)))
        origin = self.world_to_cell(0.0, 0.0)
        with self._lock:
            if origin is not None:
                self._scores[origin] = min(-3, self._scores.get(origin, 0) - 3)
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > radius_cells * radius_cells:
                        continue
                    cell = (center[0] + dx, center[1] + dy)
                    if self.in_bounds(cell):
                        self._scores[cell] = 5
            self._updates += 1
            self._last_update_at = datetime.now(timezone.utc)
            self._dirty = True
            self._last_error = ""
        return self.status()

    def status(self) -> MapStatus:
        with self._lock:
            free = sum(1 for score in self._scores.values() if score <= -1)
            occupied = sum(1 for score in self._scores.values() if score >= 2)
            return MapStatus(
                running=self._running,
                ready=self._updates > 0 and (free > 0 or occupied > 0),
                resolution_cm=self.resolution_cm,
                size_cm=self.size_cm,
                updates=self._updates,
                free_cells=free,
                occupied_cells=occupied,
                dirty=self._dirty,
                learning_enabled=self._learning_enabled,
                loaded_from_disk=self._loaded_from_disk,
                persistence_path=str(self.persistence_path),
                last_saved_at=self._last_saved_at,
                last_loaded_at=self._last_loaded_at,
                last_update_at=self._last_update_at,
                last_error=self._last_error,
            )

    def snapshot(self, max_occupied: int = 6000) -> MapSnapshot:
        pose = self.odometry.status().pose
        with self._lock:
            occupied_cells = [
                cell for cell, score in self._scores.items() if score >= 2
            ][: max(1, max_occupied)]
            captured_at = self._last_update_at
        return MapSnapshot(
            resolution_cm=self.resolution_cm,
            size_cm=self.size_cm,
            occupied=[
                MapCell(x_cm=self.cell_to_world(cell)[0], y_cm=self.cell_to_world(cell)[1])
                for cell in occupied_cells
            ],
            robot_pose=pose,
            captured_at=captured_at,
        )

    def grid_copy(self) -> dict[tuple[int, int], int]:
        with self._lock:
            return dict(self._scores)

    def world_to_cell(self, x_cm: float, y_cm: float) -> tuple[int, int] | None:
        gx = int(math.floor(x_cm / self.resolution_cm)) + self.half_cells
        gy = int(math.floor(y_cm / self.resolution_cm)) + self.half_cells
        if 0 <= gx < self.width_cells and 0 <= gy < self.width_cells:
            return gx, gy
        return None

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        gx, gy = cell
        x = (gx - self.half_cells + 0.5) * self.resolution_cm
        y = (gy - self.half_cells + 0.5) * self.resolution_cm
        return x, y

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        return 0 <= cell[0] < self.width_cells and 0 <= cell[1] < self.width_cells

    def _validate_payload(self, payload: dict) -> None:
        if payload.get("format") != self.FORMAT_NAME:
            raise ValueError("Map format is not recognized")
        if int(payload.get("version", -1)) != self.FORMAT_VERSION:
            raise ValueError("Map format version is unsupported")
        stored_resolution = float(payload.get("resolution_cm"))
        stored_size = float(payload.get("size_cm"))
        if abs(stored_resolution - self.resolution_cm) > 1e-6:
            raise ValueError(
                f"Map resolution {stored_resolution} cm does not match configured "
                f"{self.resolution_cm} cm"
            )
        if abs(stored_size - self.size_cm) > 1e-6:
            raise ValueError(
                f"Map size {stored_size} cm does not match configured {self.size_cm} cm"
            )

    @staticmethod
    def _bresenham(
        start: tuple[int, int],
        end: tuple[int, int],
    ) -> list[tuple[int, int]]:
        x0, y0 = start
        x1, y1 = end
        points: list[tuple[int, int]] = []
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        error = dx + dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            twice = 2 * error
            if twice >= dy:
                error += dy
                x0 += sx
            if twice <= dx:
                error += dx
                y0 += sy
        return points

    def _run(self) -> None:
        interval = 1.0 / self.poll_hz
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                with self._lock:
                    learning_enabled = self._learning_enabled
                if not learning_enabled:
                    self._stop_event.wait(interval)
                    continue
                odom = self.odometry.status()
                scan = self.lidar.scan()
                if (
                    odom.ready
                    and not odom.stale
                    and scan.captured_at is not None
                    and scan.captured_at != self._last_scan_at
                ):
                    self.ingest_scan(scan, odom.pose)
            except Exception as exc:
                with self._lock:
                    self._last_error = f"Mapping update failed: {exc}"
            remaining = interval - (time.monotonic() - started)
            self._stop_event.wait(max(0.0, remaining))
