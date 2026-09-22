from __future__ import annotations

import math
import threading
import time
from datetime import datetime, timezone

from app.models import (
    LidarPoint,
    LidarScan,
    LocalizationStatus,
    Pose2D,
    RelocalizeRequest,
    ScanMatchResult,
)
from app.navigation.mapping import LocalOccupancyMap
from app.navigation.odometry import DifferentialOdometry, normalize_heading_deg
from app.perception.lidar import LidarService


class CorrelativeLocalizer:
    """Conservative local scan matcher against a frozen occupancy-map reference.

    This is a bounded correlative search, not full SLAM. It only corrects encoder
    dead-reckoning when enough LiDAR endpoints agree with occupied map cells and the
    resulting confidence exceeds the configured threshold.
    """

    def __init__(
        self,
        occupancy_map: LocalOccupancyMap,
        lidar: LidarService,
        odometry: DifferentialOdometry,
        *,
        poll_hz: float = 2.0,
        search_xy_cm: float = 30.0,
        search_heading_deg: float = 12.0,
        xy_step_cm: float = 5.0,
        heading_step_deg: float = 3.0,
        min_points: int = 25,
        min_confidence: float = 0.45,
        max_correction_cm: float = 35.0,
        max_correction_deg: float = 15.0,
        confidence_decay_distance_cm: float = 250.0,
        confidence_decay_seconds: float = 20.0,
    ) -> None:
        self.map = occupancy_map
        self.lidar = lidar
        self.odometry = odometry
        self.poll_hz = max(0.2, float(poll_hz))
        self.search_xy_cm = max(2.0, float(search_xy_cm))
        self.search_heading_deg = max(1.0, float(search_heading_deg))
        self.xy_step_cm = max(1.0, float(xy_step_cm))
        self.heading_step_deg = max(0.5, float(heading_step_deg))
        self.min_points = max(5, int(min_points))
        self.min_confidence = max(0.05, min(0.95, float(min_confidence)))
        self.max_correction_cm = max(1.0, float(max_correction_cm))
        self.max_correction_deg = max(1.0, float(max_correction_deg))
        self.confidence_decay_distance_cm = max(10.0, float(confidence_decay_distance_cm))
        self.confidence_decay_seconds = max(2.0, float(confidence_decay_seconds))

        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False
        self._state = "odometry_only"
        self._last_match_confidence = 0.0
        self._last_match_at: datetime | None = None
        self._last_match_monotonic: float | None = None
        self._distance_at_match = 0.0
        self._correction_count = 0
        self._last_correction_x = 0.0
        self._last_correction_y = 0.0
        self._last_correction_heading = 0.0
        self._last_scan_at: datetime | None = None
        self._last_error = ""

    def start(self) -> LocalizationStatus:
        if self._thread and self._thread.is_alive():
            return self.status()
        self._stop_event.clear()
        with self._lock:
            self._running = True
            self._last_error = ""
        self._thread = threading.Thread(
            target=self._run,
            name="ribitics-localization",
            daemon=True,
        )
        self._thread.start()
        return self.status()

    def stop(self) -> LocalizationStatus:
        self._stop_event.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._lock:
            self._running = False
        return self.status()

    def invalidate(self, reason: str = "Localization reference changed") -> LocalizationStatus:
        with self._lock:
            self._state = "odometry_only"
            self._last_match_confidence = 0.0
            self._last_match_at = None
            self._last_match_monotonic = None
            self._distance_at_match = self.odometry.status().distance_traveled_cm
            self._last_error = reason
        return self.status()

    def status(self) -> LocalizationStatus:
        odom = self.odometry.status()
        map_status = self.map.status()
        confidence = self._decayed_confidence(odom.distance_traveled_cm)
        with self._lock:
            state = self._state
            if self._last_match_at is not None and confidence < self.min_confidence:
                state = "degraded"
            ready = (
                self._last_match_at is not None
                and confidence >= self.min_confidence
                and map_status.ready
                and not map_status.learning_enabled
            )
            return LocalizationStatus(
                running=self._running,
                ready=ready,
                state=state,
                confidence=confidence,
                pose=odom.pose,
                last_match_confidence=self._last_match_confidence,
                last_match_at=self._last_match_at,
                correction_count=self._correction_count,
                distance_since_correction_cm=max(
                    0.0,
                    odom.distance_traveled_cm - self._distance_at_match,
                ),
                last_correction_x_cm=self._last_correction_x,
                last_correction_y_cm=self._last_correction_y,
                last_correction_heading_deg=self._last_correction_heading,
                map_loaded_from_disk=map_status.loaded_from_disk,
                last_error=self._last_error,
            )

    def match_now(
        self,
        request: RelocalizeRequest | None = None,
        *,
        allow_large_correction: bool = False,
        apply: bool = True,
    ) -> ScanMatchResult:
        request = request or RelocalizeRequest()
        map_status = self.map.status()
        if not map_status.ready:
            return self._failure("Occupancy map is not ready")
        if map_status.learning_enabled:
            return self._failure(
                "Map learning is still enabled. Freeze the reference map before scan matching."
            )

        scan = self.lidar.scan()
        if not scan.points:
            return self._failure("No live LiDAR scan is available")

        odom = self.odometry.status()
        if not odom.ready or odom.stale:
            return self._failure("Odometry is not ready")

        initial = request.hint_pose.model_copy(deep=True) if request.hint_pose else odom.pose
        search_xy = (
            float(request.search_xy_cm)
            if request.search_xy_cm is not None
            else self.search_xy_cm
        )
        search_heading = (
            float(request.search_heading_deg)
            if request.search_heading_deg is not None
            else self.search_heading_deg
        )
        result = self.match_scan(
            scan,
            initial,
            search_xy_cm=search_xy,
            search_heading_deg=search_heading,
        )
        if not result.matched:
            with self._lock:
                self._state = "degraded" if self._last_match_at else "odometry_only"
                self._last_error = result.reason
            return result

        correction_distance = math.hypot(result.correction_x_cm, result.correction_y_cm)
        if not allow_large_correction and (
            correction_distance > self.max_correction_cm
            or abs(result.correction_heading_deg) > self.max_correction_deg
        ):
            result.matched = False
            result.reason = (
                "Scan match exceeded automatic correction bounds: "
                f"{correction_distance:.1f} cm / "
                f"{abs(result.correction_heading_deg):.1f}°"
            )
            with self._lock:
                self._state = "degraded"
                self._last_error = result.reason
            return result

        if apply:
            self.odometry.apply_pose_correction(result.pose)
            odom_after = self.odometry.status()
            now = datetime.now(timezone.utc)
            with self._lock:
                self._state = "tracking"
                self._last_match_confidence = result.confidence
                self._last_match_at = now
                self._last_match_monotonic = time.monotonic()
                self._distance_at_match = odom_after.distance_traveled_cm
                self._correction_count += 1
                self._last_correction_x = result.correction_x_cm
                self._last_correction_y = result.correction_y_cm
                self._last_correction_heading = result.correction_heading_deg
                self._last_scan_at = scan.captured_at
                self._last_error = ""
        return result

    def relocalize(self, request: RelocalizeRequest | None = None) -> ScanMatchResult:
        """Explicit operator relocalization may use a wider correction than background tracking."""
        with self._lock:
            self._state = "relocalizing"
            self._last_error = ""
        result = self.match_now(
            request,
            allow_large_correction=True,
            apply=True,
        )
        if not result.matched:
            with self._lock:
                self._state = "lost"
                self._last_error = result.reason
        return result

    def match_scan(
        self,
        scan: LidarScan,
        initial_pose: Pose2D,
        *,
        search_xy_cm: float | None = None,
        search_heading_deg: float | None = None,
    ) -> ScanMatchResult:
        scores = self.map.grid_copy()
        occupied = {cell for cell, score in scores.items() if score >= 2}
        if len(occupied) < 3:
            return self._failure(
                "Reference map does not contain enough occupied structure",
                initial_pose,
            )

        points = self._prepare_points(scan.points)
        if len(points) < self.min_points:
            return self._failure(
                f"Only {len(points)} usable LiDAR points; need at least {self.min_points}",
                initial_pose,
                points_used=len(points),
            )

        xy_limit = self.search_xy_cm if search_xy_cm is None else max(1.0, search_xy_cm)
        heading_limit = (
            self.search_heading_deg
            if search_heading_deg is None
            else max(0.5, search_heading_deg)
        )

        x_offsets = self._offsets(xy_limit, self.xy_step_cm)
        y_offsets = x_offsets
        heading_offsets = self._offsets(heading_limit, self.heading_step_deg)

        best: tuple[float, int, Pose2D] | None = None
        second_score = -1.0

        for heading_offset in heading_offsets:
            heading = normalize_heading_deg(initial_pose.heading_deg + heading_offset)
            radians = math.radians(heading)
            cos_h = math.cos(radians)
            sin_h = math.sin(radians)
            rotated = [
                (
                    cos_h * local_x - sin_h * local_y,
                    sin_h * local_x + cos_h * local_y,
                )
                for local_x, local_y in points
            ]
            for dx in x_offsets:
                candidate_x = initial_pose.x_cm + dx
                for dy in y_offsets:
                    candidate_y = initial_pose.y_cm + dy
                    score, hits = self._score_candidate(
                        candidate_x,
                        candidate_y,
                        rotated,
                        scores,
                        occupied,
                    )
                    candidate = Pose2D(
                        x_cm=candidate_x,
                        y_cm=candidate_y,
                        heading_deg=heading,
                    )
                    if best is None or score > best[0]:
                        if best is not None:
                            second_score = max(second_score, best[0])
                        best = (score, hits, candidate)
                    elif score > second_score:
                        second_score = score

        if best is None:
            return self._failure("No scan-match candidate could be evaluated", initial_pose)

        best_score, hit_points, best_pose = best
        second_score = max(0.0, second_score)
        margin = max(0.0, best_score - second_score)
        hit_ratio = hit_points / max(1, len(points))
        confidence = min(
            1.0,
            max(0.0, best_score * 0.72 + hit_ratio * 0.23 + min(0.05, margin)),
        )

        correction_x = best_pose.x_cm - initial_pose.x_cm
        correction_y = best_pose.y_cm - initial_pose.y_cm
        correction_heading = normalize_heading_deg(
            best_pose.heading_deg - initial_pose.heading_deg
        )
        matched = confidence >= self.min_confidence
        reason = (
            f"Scan matched with confidence {confidence:.2f}"
            if matched
            else (
                f"Best scan-match confidence {confidence:.2f} is below "
                f"required {self.min_confidence:.2f}"
            )
        )
        return ScanMatchResult(
            matched=matched,
            confidence=confidence,
            pose=best_pose,
            initial_pose=initial_pose,
            correction_x_cm=correction_x,
            correction_y_cm=correction_y,
            correction_heading_deg=correction_heading,
            points_used=len(points),
            hit_points=hit_points,
            best_score=best_score,
            second_best_score=second_score,
            reason=reason,
        )

    def _prepare_points(self, points: list[LidarPoint]) -> list[tuple[float, float]]:
        usable: list[tuple[float, float]] = []
        max_distance = self.map.size_cm * 0.65
        for point in points:
            distance = float(point.distance_cm)
            if distance < self.map.resolution_cm * 2 or distance > max_distance:
                continue
            radians = math.radians(float(point.angle_deg))
            usable.append(
                (
                    distance * math.cos(radians),
                    distance * math.sin(radians),
                )
            )
        if len(usable) <= 180:
            return usable
        stride = max(1, len(usable) // 180)
        return usable[::stride][:180]

    def _score_candidate(
        self,
        candidate_x: float,
        candidate_y: float,
        rotated_points: list[tuple[float, float]],
        scores: dict[tuple[int, int], int],
        occupied: set[tuple[int, int]],
    ) -> tuple[float, int]:
        value = 0.0
        hits = 0
        used = 0
        for offset_x, offset_y in rotated_points:
            cell = self.map.world_to_cell(
                candidate_x + offset_x,
                candidate_y + offset_y,
            )
            if cell is None:
                continue
            used += 1
            score = scores.get(cell, 0)
            if score >= 2:
                hits += 1
                value += 1.0
            elif self._near_occupied(cell, occupied):
                value += 0.35
            elif score <= -1:
                value -= 0.12
        if used == 0:
            return -1.0, 0
        return max(0.0, value / used), hits

    @staticmethod
    def _near_occupied(
        cell: tuple[int, int],
        occupied: set[tuple[int, int]],
    ) -> bool:
        x, y = cell
        return any(
            (x + dx, y + dy) in occupied
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            if dx or dy
        )

    @staticmethod
    def _offsets(limit: float, step: float) -> list[float]:
        if limit <= 0:
            return [0.0]
        count = int(math.floor(limit / step))
        values = [round(index * step, 6) for index in range(-count, count + 1)]
        if not values or 0.0 not in values:
            values.append(0.0)
        return sorted(set(values))

    def _decayed_confidence(self, current_distance: float) -> float:
        with self._lock:
            base = self._last_match_confidence
            last_match_monotonic = self._last_match_monotonic
            distance_at_match = self._distance_at_match
        if base <= 0 or last_match_monotonic is None:
            return 0.0
        distance_delta = max(0.0, current_distance - distance_at_match)
        age = max(0.0, time.monotonic() - last_match_monotonic)
        distance_factor = math.exp(
            -distance_delta / self.confidence_decay_distance_cm
        )
        time_factor = math.exp(-age / self.confidence_decay_seconds)
        return max(0.0, min(1.0, base * distance_factor * time_factor))

    def _failure(
        self,
        reason: str,
        initial_pose: Pose2D | None = None,
        *,
        points_used: int = 0,
    ) -> ScanMatchResult:
        pose = initial_pose or self.odometry.status().pose
        return ScanMatchResult(
            matched=False,
            confidence=0.0,
            pose=pose,
            initial_pose=pose,
            points_used=points_used,
            reason=reason,
        )

    def _run(self) -> None:
        interval = 1.0 / self.poll_hz
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                map_status = self.map.status()
                scan = self.lidar.scan()
                stable_reference = map_status.ready and not map_status.learning_enabled
                if not stable_reference:
                    with self._lock:
                        if self._last_match_at is None:
                            self._state = "mapping" if map_status.ready else "odometry_only"
                    self._stop_event.wait(interval)
                    continue
                if (
                    scan.captured_at is not None
                    and scan.captured_at != self._last_scan_at
                ):
                    self.match_now()
                    with self._lock:
                        self._last_scan_at = scan.captured_at
            except Exception as exc:
                with self._lock:
                    self._state = "error"
                    self._last_error = f"Localization update failed: {exc}"
            remaining = interval - (time.monotonic() - started)
            self._stop_event.wait(max(0.0, remaining))
