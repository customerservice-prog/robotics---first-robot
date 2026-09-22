from __future__ import annotations

import json
import math
import threading
from datetime import datetime, timezone
from pathlib import Path

from app.models import DockStatus, NavigationGoal, NavigationPlan, NavigationStatus, Pose2D
from app.navigation.mapping import LocalOccupancyMap
from app.navigation.navigator import NavigationError, SupervisedNavigator
from app.navigation.odometry import DifferentialOdometry


class DockingFoundation:
    """Persistent dock approach pose tied to a specific occupancy-map identity.

    Final charging-contact alignment remains intentionally outside this class.
    """

    FORMAT_NAME = "ribitics_dock_pose"
    FORMAT_VERSION = 1

    def __init__(
        self,
        odometry: DifferentialOdometry,
        navigator: SupervisedNavigator,
        occupancy_map: LocalOccupancyMap,
        *,
        approach_distance_cm: float = 60.0,
        persistence_path: str = "data/ribitics-dock.json",
    ) -> None:
        self.odometry = odometry
        self.navigator = navigator
        self.map = occupancy_map
        self.approach_distance_cm = max(20.0, float(approach_distance_cm))
        self.persistence_path = Path(persistence_path)
        self._lock = threading.Lock()
        self._dock_pose: Pose2D | None = None
        self._map_id = ""
        self._map_revision = 0
        self._persistent = False
        self._last_error = ""

    def set_pose(self, pose: Pose2D, *, persist: bool = True) -> DockStatus:
        identity = self.map.identity()
        with self._lock:
            self._dock_pose = pose.model_copy(deep=True)
            self._map_id = identity.map_id
            self._map_revision = identity.revision
            self._persistent = False
            self._last_error = ""
        if persist:
            self.save()
        return self.status()

    def set_current_pose(self) -> DockStatus:
        odom = self.odometry.status()
        if not odom.ready or odom.stale:
            raise NavigationError("Odometry is not ready enough to mark the dock")
        return self.set_pose(odom.pose)

    def clear(self) -> DockStatus:
        with self._lock:
            self._dock_pose = None
            self._map_id = ""
            self._map_revision = 0
            self._persistent = False
            self._last_error = ""
        try:
            if self.persistence_path.exists():
                self.persistence_path.unlink()
        except OSError as exc:
            with self._lock:
                self._last_error = f"Could not remove dock file: {exc}"
        return self.status()

    def save(self) -> bool:
        with self._lock:
            pose = self._dock_pose.model_copy(deep=True) if self._dock_pose else None
            map_id = self._map_id
            map_revision = self._map_revision
        if pose is None or not map_id:
            with self._lock:
                self._last_error = "Dock pose is not configured"
            return False

        payload = {
            "format": self.FORMAT_NAME,
            "version": self.FORMAT_VERSION,
            "map_id": map_id,
            "map_revision": map_revision,
            "dock_pose": pose.model_dump(mode="json"),
            "approach_distance_cm": self.approach_distance_cm,
            "saved_at": datetime.now(timezone.utc).isoformat(),
        }
        self.persistence_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.persistence_path.with_suffix(self.persistence_path.suffix + ".tmp")
        try:
            temp.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
            temp.replace(self.persistence_path)
            with self._lock:
                self._persistent = True
                self._last_error = ""
            return True
        except (OSError, ValueError, TypeError) as exc:
            with self._lock:
                self._last_error = f"Dock save failed: {exc}"
            return False
        finally:
            if temp.exists():
                try:
                    temp.unlink()
                except OSError:
                    pass

    def load(self) -> DockStatus:
        if not self.persistence_path.exists():
            with self._lock:
                self._last_error = "Dock file does not exist"
            return self.status()
        try:
            payload = json.loads(self.persistence_path.read_text(encoding="utf-8"))
            if payload.get("format") != self.FORMAT_NAME:
                raise ValueError("Dock file format is not recognized")
            if int(payload.get("version", -1)) != self.FORMAT_VERSION:
                raise ValueError("Dock file version is unsupported")
            identity = self.map.identity()
            stored_map_id = str(payload.get("map_id", ""))
            if stored_map_id != identity.map_id:
                raise ValueError(
                    "Saved dock belongs to a different map ID and was not loaded"
                )
            pose = Pose2D.model_validate(payload.get("dock_pose", {}))
            with self._lock:
                self._dock_pose = pose
                self._map_id = stored_map_id
                self._map_revision = int(payload.get("map_revision", 0))
                self._persistent = True
                self._last_error = ""
            return self.status()
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            with self._lock:
                self._dock_pose = None
                self._map_id = ""
                self._map_revision = 0
                self._persistent = False
                self._last_error = str(exc)
            return self.status()

    def status(self) -> DockStatus:
        odom = self.odometry.status()
        identity = self.map.identity()
        with self._lock:
            pose = self._dock_pose.model_copy(deep=True) if self._dock_pose else None
            stored_map_id = self._map_id
            stored_revision = self._map_revision
            persistent = self._persistent
            error = self._last_error
        if pose is None:
            return DockStatus(
                configured=False,
                map_id=stored_map_id,
                map_revision=stored_revision,
                persistent=persistent,
                valid_for_current_map=False,
                reason=error or "Dock pose is not configured for the current map",
            )
        valid = stored_map_id == identity.map_id
        approach = self._approach_goal(pose) if valid else None
        distance = (
            math.hypot(pose.x_cm - odom.pose.x_cm, pose.y_cm - odom.pose.y_cm)
            if odom.ready and valid
            else None
        )
        return DockStatus(
            configured=True,
            dock_pose=pose,
            approach_goal=approach,
            distance_to_dock_cm=distance,
            map_id=stored_map_id,
            map_revision=stored_revision,
            persistent=persistent,
            valid_for_current_map=valid,
            reason=(
                "Persistent dock approach is valid for the current map. Final charging "
                "contact alignment remains manual."
                if valid
                else "Dock pose belongs to a different map ID and is blocked."
            ),
        )

    def plan_return(self) -> NavigationPlan:
        status = self.status()
        if (
            not status.configured
            or not status.valid_for_current_map
            or status.approach_goal is None
        ):
            raise NavigationError(status.reason or "Dock pose is not valid for the current map")
        return self.navigator.plan(status.approach_goal)

    def start_return(self) -> NavigationStatus:
        status = self.status()
        if (
            not status.configured
            or not status.valid_for_current_map
            or status.approach_goal is None
        ):
            raise NavigationError(status.reason or "Dock pose is not valid for the current map")
        return self.navigator.start(status.approach_goal)

    def _approach_goal(self, pose: Pose2D) -> NavigationGoal:
        heading_rad = math.radians(pose.heading_deg)
        return NavigationGoal(
            x_cm=pose.x_cm - self.approach_distance_cm * math.cos(heading_rad),
            y_cm=pose.y_cm - self.approach_distance_cm * math.sin(heading_rad),
            heading_deg=pose.heading_deg,
        )
