from __future__ import annotations

import json
import math
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path

from app.models import (
    LidarScan,
    PlaceAnchor,
    PlaceRecognitionResult,
    PlaceRecognitionStatus,
    Pose2D,
    RelocalizeRequest,
)
from app.navigation.localization import CorrelativeLocalizer
from app.navigation.mapping import LocalOccupancyMap
from app.navigation.odometry import DifferentialOdometry, normalize_heading_deg
from app.perception.lidar import LidarService


class PlaceRecognitionError(RuntimeError):
    pass


class PlaceRecognizer:
    """Persistent LiDAR scan descriptors used only to propose a relocalization hint.

    Descriptor recognition never directly changes the robot pose. A candidate place must
    be verified by the geometric CorrelativeLocalizer before any pose correction is applied.
    """

    FORMAT_NAME = "ribitics_place_anchors"
    FORMAT_VERSION = 1

    def __init__(
        self,
        occupancy_map: LocalOccupancyMap,
        lidar: LidarService,
        odometry: DifferentialOdometry,
        localizer: CorrelativeLocalizer,
        *,
        persistence_path: str,
        sectors: int = 36,
        max_range_cm: float = 600.0,
        min_valid_sectors: int = 10,
        min_similarity: float = 0.72,
        min_margin: float = 0.04,
        max_anchors: int = 100,
        verify_search_xy_cm: float = 100.0,
        verify_search_heading_deg: float = 35.0,
    ) -> None:
        self.map = occupancy_map
        self.lidar = lidar
        self.odometry = odometry
        self.localizer = localizer
        self.persistence_path = Path(persistence_path)
        self.sectors = max(12, min(144, int(sectors)))
        self.max_range_cm = max(100.0, float(max_range_cm))
        self.min_valid_sectors = max(4, min(self.sectors, int(min_valid_sectors)))
        self.min_similarity = max(0.05, min(0.99, float(min_similarity)))
        self.min_margin = max(0.0, min(0.5, float(min_margin)))
        self.max_anchors = max(1, min(1000, int(max_anchors)))
        self.verify_search_xy_cm = max(10.0, float(verify_search_xy_cm))
        self.verify_search_heading_deg = max(5.0, float(verify_search_heading_deg))

        self._lock = threading.Lock()
        self._anchors: list[PlaceAnchor] = []
        self._loaded_map_id = ""
        self._last_recognition_at: datetime | None = None
        self._last_similarity = 0.0
        self._last_anchor_id = ""
        self._last_error = ""

    def status(self) -> PlaceRecognitionStatus:
        identity = self.map.identity()
        with self._lock:
            matching = [anchor for anchor in self._anchors if anchor.map_id == identity.map_id]
            return PlaceRecognitionStatus(
                ready=bool(matching),
                anchor_count=len(matching),
                map_id=identity.map_id,
                persistence_path=str(self.persistence_path),
                last_recognition_at=self._last_recognition_at,
                last_similarity=self._last_similarity,
                last_anchor_id=self._last_anchor_id,
                last_error=self._last_error,
            )

    def list_anchors(self) -> list[PlaceAnchor]:
        identity = self.map.identity()
        with self._lock:
            return [
                anchor.model_copy(deep=True)
                for anchor in self._anchors
                if anchor.map_id == identity.map_id
            ]

    def capture_anchor(self, name: str = "") -> PlaceAnchor:
        map_status = self.map.status()
        if not map_status.ready:
            raise PlaceRecognitionError("Map is not ready")
        if map_status.learning_enabled:
            raise PlaceRecognitionError("Freeze the reference map before saving a place anchor")

        localization = self.localizer.status()
        raw_mode = self.odometry.hardware.status().mode
        if raw_mode != "simulation" and not localization.ready:
            raise PlaceRecognitionError(
                "Physical place anchors require a confident localized pose"
            )

        scan = self.lidar.scan()
        descriptor, valid = self._descriptor(scan)
        if valid < self.min_valid_sectors:
            raise PlaceRecognitionError(
                f"Only {valid} valid LiDAR sectors; need at least {self.min_valid_sectors}"
            )

        identity = self.map.identity()
        pose = self.odometry.status().pose
        anchor = PlaceAnchor(
            anchor_id=str(uuid.uuid4()),
            name=name.strip(),
            map_id=identity.map_id,
            map_revision=identity.revision,
            pose=pose,
            descriptor=descriptor,
            valid_sectors=valid,
            created_at=datetime.now(timezone.utc),
        )
        with self._lock:
            current = [item for item in self._anchors if item.map_id == identity.map_id]
            others = [item for item in self._anchors if item.map_id != identity.map_id]
            current.append(anchor)
            if len(current) > self.max_anchors:
                current = current[-self.max_anchors :]
            self._anchors = others + current
            self._last_error = ""
        self.save()
        return anchor

    def delete_anchor(self, anchor_id: str) -> bool:
        identity = self.map.identity()
        with self._lock:
            before = len(self._anchors)
            self._anchors = [
                anchor
                for anchor in self._anchors
                if not (anchor.anchor_id == anchor_id and anchor.map_id == identity.map_id)
            ]
            deleted = len(self._anchors) != before
        if deleted:
            self.save()
        return deleted

    def recognize(self, scan: LidarScan | None = None) -> PlaceRecognitionResult:
        scan = scan or self.lidar.scan()
        live_descriptor, live_valid = self._descriptor(scan)
        if live_valid < self.min_valid_sectors:
            return self._failure(
                f"Only {live_valid} valid LiDAR sectors; need at least {self.min_valid_sectors}"
            )

        anchors = self.list_anchors()
        if not anchors:
            return self._failure("No place anchors exist for the current map")

        ranked: list[tuple[float, float, PlaceAnchor, int]] = []
        for anchor in anchors:
            similarity, shift = self._best_rotation_similarity(
                live_descriptor,
                anchor.descriptor,
            )
            ranked.append((similarity, abs(shift), anchor, shift))
        ranked.sort(key=lambda item: item[0], reverse=True)

        best_similarity, _, best_anchor, shift = ranked[0]
        second_similarity = ranked[1][0] if len(ranked) > 1 else 0.0
        margin = best_similarity - second_similarity
        heading_offset = normalize_heading_deg(shift * (360.0 / self.sectors))
        estimated_pose = Pose2D(
            x_cm=best_anchor.pose.x_cm,
            y_cm=best_anchor.pose.y_cm,
            heading_deg=normalize_heading_deg(best_anchor.pose.heading_deg + heading_offset),
        )
        recognized = (
            best_similarity >= self.min_similarity
            and (len(ranked) == 1 or margin >= self.min_margin)
        )
        reason = (
            f"Recognized place '{best_anchor.name or best_anchor.anchor_id}' "
            f"with similarity {best_similarity:.2f}"
            if recognized
            else (
                f"Place recognition ambiguous/weak: similarity={best_similarity:.2f}, "
                f"margin={margin:.2f}"
            )
        )
        now = datetime.now(timezone.utc)
        with self._lock:
            self._last_recognition_at = now
            self._last_similarity = best_similarity
            self._last_anchor_id = best_anchor.anchor_id if recognized else ""
            self._last_error = "" if recognized else reason
        return PlaceRecognitionResult(
            recognized=recognized,
            anchor_id=best_anchor.anchor_id,
            anchor_name=best_anchor.name,
            similarity=best_similarity,
            second_similarity=second_similarity,
            margin=margin,
            estimated_pose=estimated_pose,
            heading_offset_deg=heading_offset,
            verified=False,
            reason=reason,
        )

    def recognize_and_relocalize(self) -> PlaceRecognitionResult:
        result = self.recognize()
        if not result.recognized:
            return result

        match = self.localizer.relocalize(
            RelocalizeRequest(
                hint_pose=result.estimated_pose,
                search_xy_cm=min(300.0, self.verify_search_xy_cm),
                search_heading_deg=min(90.0, self.verify_search_heading_deg),
            )
        )
        result.scan_match = match
        result.verified = bool(match.matched)
        if match.matched:
            result.estimated_pose = match.pose
            result.reason = (
                f"Place '{result.anchor_name or result.anchor_id}' recognized and "
                f"geometrically verified at confidence {match.confidence:.2f}"
            )
            with self._lock:
                self._last_error = ""
        else:
            result.reason = (
                "Descriptor place recognition found a candidate, but geometric "
                f"verification failed: {match.reason}"
            )
            with self._lock:
                self._last_error = result.reason
        return result

    def save(self) -> bool:
        identity = self.map.identity()
        anchors = self.list_anchors()
        payload = {
            "format": self.FORMAT_NAME,
            "version": self.FORMAT_VERSION,
            "map_id": identity.map_id,
            "map_revision": identity.revision,
            "sectors": self.sectors,
            "max_range_cm": self.max_range_cm,
            "anchors": [anchor.model_dump(mode="json") for anchor in anchors],
            "saved_at": datetime.now(timezone.utc).isoformat(),
        }
        self.persistence_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.persistence_path.with_suffix(self.persistence_path.suffix + ".tmp")
        try:
            temp.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
            temp.replace(self.persistence_path)
            with self._lock:
                self._loaded_map_id = identity.map_id
                self._last_error = ""
            return True
        except (OSError, ValueError, TypeError) as exc:
            with self._lock:
                self._last_error = f"Place anchor save failed: {exc}"
            return False
        finally:
            if temp.exists():
                try:
                    temp.unlink()
                except OSError:
                    pass

    def load(self) -> bool:
        if not self.persistence_path.exists():
            with self._lock:
                self._anchors = []
                self._loaded_map_id = ""
                self._last_error = "Place anchor file does not exist"
            return False
        try:
            payload = json.loads(self.persistence_path.read_text(encoding="utf-8"))
            if payload.get("format") != self.FORMAT_NAME:
                raise ValueError("Place-anchor format is not recognized")
            if int(payload.get("version", -1)) != self.FORMAT_VERSION:
                raise ValueError("Place-anchor format version is unsupported")
            identity = self.map.identity()
            stored_map_id = str(payload.get("map_id", ""))
            if stored_map_id != identity.map_id:
                raise ValueError(
                    "Place anchors belong to a different map ID and were not loaded"
                )
            if int(payload.get("sectors", self.sectors)) != self.sectors:
                raise ValueError("Place-anchor descriptor sector count does not match configuration")

            anchors = [
                PlaceAnchor.model_validate(item)
                for item in payload.get("anchors", [])
                if isinstance(item, dict)
            ]
            with self._lock:
                self._anchors = anchors[-self.max_anchors :]
                self._loaded_map_id = stored_map_id
                self._last_error = ""
            return True
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            with self._lock:
                self._anchors = []
                self._loaded_map_id = ""
                self._last_error = str(exc)
            return False

    def invalidate_for_map_change(self) -> None:
        identity = self.map.identity()
        with self._lock:
            self._last_anchor_id = ""
            self._last_similarity = 0.0
            if self._loaded_map_id and self._loaded_map_id != identity.map_id:
                self._anchors = []
                self._loaded_map_id = ""
            self._last_error = ""

    def _descriptor(self, scan: LidarScan) -> tuple[list[float], int]:
        descriptor = [-1.0] * self.sectors
        sector_width = 360.0 / self.sectors
        valid = 0
        for point in scan.points:
            distance = float(point.distance_cm)
            if distance <= 0 or distance > self.max_range_cm:
                continue
            angle = ((float(point.angle_deg) + 180.0) % 360.0) - 180.0
            index = int(math.floor((angle + 180.0) / sector_width)) % self.sectors
            normalized = min(1.0, distance / self.max_range_cm)
            if descriptor[index] < 0:
                descriptor[index] = normalized
                valid += 1
            else:
                descriptor[index] = min(descriptor[index], normalized)
        return descriptor, valid

    def _best_rotation_similarity(
        self,
        live: list[float],
        anchor: list[float],
    ) -> tuple[float, int]:
        if len(anchor) != self.sectors or len(live) != self.sectors:
            return 0.0, 0
        best_similarity = 0.0
        best_shift = 0
        for shift in range(self.sectors):
            compared = 0
            error = 0.0
            for index, live_value in enumerate(live):
                anchor_value = anchor[(index + shift) % self.sectors]
                if live_value < 0 or anchor_value < 0:
                    continue
                compared += 1
                error += abs(live_value - anchor_value)
            if compared < self.min_valid_sectors:
                similarity = 0.0
            else:
                mean_error = error / compared
                coverage = min(1.0, compared / max(self.min_valid_sectors, self.sectors * 0.55))
                similarity = max(0.0, 1.0 - mean_error) * coverage
            signed_shift = shift if shift <= self.sectors // 2 else shift - self.sectors
            if similarity > best_similarity:
                best_similarity = similarity
                best_shift = signed_shift
        return best_similarity, best_shift

    def _failure(self, reason: str) -> PlaceRecognitionResult:
        with self._lock:
            self._last_recognition_at = datetime.now(timezone.utc)
            self._last_similarity = 0.0
            self._last_anchor_id = ""
            self._last_error = reason
        return PlaceRecognitionResult(recognized=False, reason=reason)
