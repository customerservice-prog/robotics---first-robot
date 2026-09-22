from __future__ import annotations

import math
import threading

from app.models import DockStatus, NavigationGoal, NavigationPlan, NavigationStatus, Pose2D
from app.navigation.navigator import NavigationError, SupervisedNavigator
from app.navigation.odometry import DifferentialOdometry


class DockingFoundation:
    """Session-local dock pose and supervised approach planner.

    This intentionally stops at an approach point. Final charging-contact alignment needs
    dedicated short-range sensing/fiducials and is not automated in v0.4.
    """

    def __init__(
        self,
        odometry: DifferentialOdometry,
        navigator: SupervisedNavigator,
        *,
        approach_distance_cm: float = 60.0,
    ) -> None:
        self.odometry = odometry
        self.navigator = navigator
        self.approach_distance_cm = max(20.0, float(approach_distance_cm))
        self._lock = threading.Lock()
        self._dock_pose: Pose2D | None = None

    def set_pose(self, pose: Pose2D) -> DockStatus:
        with self._lock:
            self._dock_pose = pose.model_copy(deep=True)
        return self.status()

    def set_current_pose(self) -> DockStatus:
        odom = self.odometry.status()
        if not odom.ready or odom.stale:
            raise NavigationError("Odometry is not ready enough to mark the dock")
        return self.set_pose(odom.pose)

    def status(self) -> DockStatus:
        odom = self.odometry.status()
        with self._lock:
            pose = self._dock_pose.model_copy(deep=True) if self._dock_pose else None
        if pose is None:
            return DockStatus(
                configured=False,
                reason="Dock pose is not set for this local-map session",
            )
        approach = self._approach_goal(pose)
        distance = (
            math.hypot(pose.x_cm - odom.pose.x_cm, pose.y_cm - odom.pose.y_cm)
            if odom.ready
            else None
        )
        return DockStatus(
            configured=True,
            dock_pose=pose,
            approach_goal=approach,
            distance_to_dock_cm=distance,
            reason=(
                "Dock approach is configured. v0.4 only navigates to the approach point; "
                "final contact alignment remains manual."
            ),
        )

    def plan_return(self) -> NavigationPlan:
        status = self.status()
        if not status.configured or status.approach_goal is None:
            raise NavigationError("Dock pose is not configured")
        return self.navigator.plan(status.approach_goal)

    def start_return(self) -> NavigationStatus:
        status = self.status()
        if not status.configured or status.approach_goal is None:
            raise NavigationError("Dock pose is not configured")
        return self.navigator.start(status.approach_goal)

    def _approach_goal(self, pose: Pose2D) -> NavigationGoal:
        heading_rad = math.radians(pose.heading_deg)
        return NavigationGoal(
            x_cm=pose.x_cm - self.approach_distance_cm * math.cos(heading_rad),
            y_cm=pose.y_cm - self.approach_distance_cm * math.sin(heading_rad),
            heading_deg=pose.heading_deg,
        )
