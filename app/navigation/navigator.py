from __future__ import annotations

import math
import threading
import time
from datetime import datetime, timezone

from app.models import NavigationGoal, NavigationPlan, NavigationStatus
from app.navigation.mapping import LocalOccupancyMap
from app.navigation.odometry import DifferentialOdometry, normalize_heading_deg
from app.navigation.planner import AStarPlanner
from app.perception.lidar import LidarService
from app.perception.spatial import SpatialAwareness
from app.robot import RobotController, UnsafeDriveError


class NavigationError(RuntimeError):
    pass


class SupervisedNavigator:
    """Low-speed local waypoint follower with explicit execution gates.

    Simulation may execute without hardware calibration. Physical execution requires the
    operator to explicitly enable supervised navigation and all live prerequisites to pass.
    """

    def __init__(
        self,
        *,
        robot: RobotController,
        odometry: DifferentialOdometry,
        occupancy_map: LocalOccupancyMap,
        planner: AStarPlanner,
        awareness: SpatialAwareness,
        lidar: LidarService,
        hardware_mode: str,
        hardware_execution_enabled: bool,
        allow_unknown: bool,
        max_goal_distance_cm: float,
        max_linear: float,
        max_angular: float,
        waypoint_tolerance_cm: float,
        heading_tolerance_deg: float,
        timeout_seconds: float,
    ) -> None:
        self.robot = robot
        self.odometry = odometry
        self.map = occupancy_map
        self.planner = planner
        self.awareness = awareness
        self.lidar = lidar
        self.hardware_mode = hardware_mode
        self.hardware_execution_enabled = hardware_execution_enabled
        self.allow_unknown = allow_unknown
        self.max_goal_distance_cm = max(10.0, float(max_goal_distance_cm))
        self.max_linear = max(0.03, min(0.5, float(max_linear)))
        self.max_angular = max(0.05, min(0.6, float(max_angular)))
        self.waypoint_tolerance_cm = max(3.0, float(waypoint_tolerance_cm))
        self.heading_tolerance_deg = max(2.0, float(heading_tolerance_deg))
        self.timeout_seconds = max(5.0, float(timeout_seconds))

        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._cancel_event = threading.Event()
        self._running = False
        self._state = "idle"
        self._reason = ""
        self._goal: NavigationGoal | None = None
        self._plan: NavigationPlan | None = None
        self._waypoint_index = 0
        self._started_at: datetime | None = None
        self._last_update_at: datetime | None = None
        self._last_error = ""

    def hardware_execution_allowed(self) -> tuple[bool, str]:
        if self.hardware_mode == "simulation":
            return True, "Simulation navigation is allowed"

        if not self.hardware_execution_enabled:
            return False, "Hardware supervised navigation is disabled in configuration"

        odom = self.odometry.status()
        if not odom.calibrated:
            return False, "Wheel odometry is not marked calibrated"
        if not odom.ready or odom.stale:
            return False, "Wheel odometry is not healthy"

        lidar = self.lidar.status()
        if not lidar.ready:
            return False, "2D LiDAR must be live for hardware navigation"

        map_status = self.map.status()
        if not map_status.ready:
            return False, "Local occupancy map is not ready"

        spatial = self.awareness.status()
        if spatial.hazard_level == "stop":
            return False, f"Spatial safety stop: {spatial.reason}"

        return True, "Hardware prerequisites passed"

    def plan(self, goal: NavigationGoal) -> NavigationPlan:
        odom = self.odometry.status()
        if not odom.ready or odom.stale:
            raise NavigationError("Odometry is not ready")
        distance = math.hypot(goal.x_cm - odom.pose.x_cm, goal.y_cm - odom.pose.y_cm)
        if distance > self.max_goal_distance_cm:
            raise NavigationError(
                f"Goal is {distance:.1f} cm away; limit is {self.max_goal_distance_cm:.1f} cm"
            )

        permit_unknown = self.hardware_mode == "simulation" or self.allow_unknown
        plan = self.planner.plan(odom.pose, goal, allow_unknown=permit_unknown)
        with self._lock:
            self._goal = goal
            self._plan = plan
            self._waypoint_index = 0
            self._state = "planned" if plan.found else "plan_failed"
            self._reason = plan.reason
            self._last_update_at = datetime.now(timezone.utc)
            self._last_error = ""
        return plan

    def start(self, goal: NavigationGoal) -> NavigationStatus:
        if self._thread and self._thread.is_alive():
            raise NavigationError("Navigation is already running")

        allowed, reason = self.hardware_execution_allowed()
        if not allowed:
            raise NavigationError(reason)

        plan = self.plan(goal)
        if not plan.found or len(plan.waypoints) < 1:
            raise NavigationError(plan.reason or "No route available")

        self._cancel_event.clear()
        with self._lock:
            self._running = True
            self._state = "running"
            self._reason = "Following supervised local route"
            self._started_at = datetime.now(timezone.utc)
            self._last_update_at = self._started_at
            self._last_error = ""
        self._thread = threading.Thread(target=self._run, name="ribitics-navigation", daemon=True)
        self._thread.start()
        return self.status()

    def replan_current_goal(self) -> NavigationPlan:
        with self._lock:
            goal = self._goal
        if goal is None:
            raise NavigationError("No navigation goal is available to recover")
        self.robot.stop()
        plan = self.plan(goal)
        if not plan.found:
            raise NavigationError(plan.reason or "No recovery route was found")
        with self._lock:
            self._state = "replanned"
            self._reason = "Recovery route replanned; operator must explicitly start it"
            self._last_update_at = datetime.now(timezone.utc)
        return plan

    def cancel(self, reason: str = "Navigation cancelled") -> NavigationStatus:
        self._cancel_event.set()
        self.robot.stop()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.5)
        with self._lock:
            self._running = False
            if self._state not in {"arrived", "blocked", "error"}:
                self._state = "cancelled"
            self._reason = reason
            self._last_update_at = datetime.now(timezone.utc)
        return self.status()

    def status(self) -> NavigationStatus:
        allowed, reason = self.hardware_execution_allowed()
        odom = self.odometry.status()
        with self._lock:
            goal = self._goal
            remaining = (
                math.hypot(goal.x_cm - odom.pose.x_cm, goal.y_cm - odom.pose.y_cm)
                if goal is not None
                else None
            )
            plan = self._plan
            return NavigationStatus(
                enabled=self.hardware_mode == "simulation" or self.hardware_execution_enabled,
                hardware_execution_allowed=allowed,
                running=self._running,
                state=self._state,
                reason=self._reason or reason,
                goal=goal,
                waypoint_index=self._waypoint_index,
                waypoint_count=len(plan.waypoints) if plan else 0,
                planned_distance_cm=plan.planned_distance_cm if plan else 0.0,
                distance_remaining_cm=remaining,
                started_at=self._started_at,
                last_update_at=self._last_update_at,
                last_error=self._last_error,
                recovery_available=self._state == "blocked" and goal is not None,
            )

    def current_plan(self) -> NavigationPlan | None:
        with self._lock:
            return self._plan.model_copy(deep=True) if self._plan else None

    def _run(self) -> None:
        started = time.monotonic()
        try:
            plan = self.current_plan()
            if plan is None or not plan.found:
                raise NavigationError("No valid route is loaded")

            for index, waypoint in enumerate(plan.waypoints):
                with self._lock:
                    self._waypoint_index = index
                    self._last_update_at = datetime.now(timezone.utc)

                while not self._cancel_event.is_set():
                    if time.monotonic() - started > self.timeout_seconds:
                        raise NavigationError("Navigation timeout reached")

                    odom = self.odometry.status()
                    if not odom.ready or odom.stale:
                        raise NavigationError("Odometry became stale during navigation")

                    if self.hardware_mode != "simulation":
                        lidar = self.lidar.status()
                        if not lidar.ready:
                            raise NavigationError("LiDAR became unavailable during navigation")

                    spatial = self.awareness.status()
                    if not spatial.clear_to_move_forward:
                        self.robot.stop()
                        self._finish(
                            "blocked",
                            f"Navigation stopped by spatial safety: {spatial.reason}. Replan is available after the hazard/map changes.",
                        )
                        return

                    dx = waypoint.x_cm - odom.pose.x_cm
                    dy = waypoint.y_cm - odom.pose.y_cm
                    distance = math.hypot(dx, dy)
                    if distance <= self.waypoint_tolerance_cm:
                        break

                    target_heading = math.degrees(math.atan2(dy, dx))
                    heading_error = normalize_heading_deg(target_heading - odom.pose.heading_deg)

                    if abs(heading_error) > 25.0:
                        linear = 0.0
                        angular = self._signed_turn(heading_error)
                    else:
                        linear = min(
                            self.max_linear,
                            max(0.08, self.max_linear * min(1.0, distance / 60.0)),
                        )
                        angular = max(
                            -self.max_angular,
                            min(self.max_angular, heading_error / 55.0 * self.max_angular),
                        )

                    try:
                        self.robot.drive(linear, angular)
                    except UnsafeDriveError as exc:
                        self.robot.stop()
                        self._finish("blocked", f"Drive safety rejected route: {exc}. Replan is available after the hazard/map changes.")
                        return

                    with self._lock:
                        self._last_update_at = datetime.now(timezone.utc)
                    self._cancel_event.wait(0.10)

                if self._cancel_event.is_set():
                    self._finish("cancelled", "Navigation cancelled")
                    return

            goal = plan.goal
            if goal.heading_deg is not None and not self._cancel_event.is_set():
                self._rotate_to_heading(goal.heading_deg, started)

            if self._cancel_event.is_set():
                self._finish("cancelled", "Navigation cancelled")
                return

            self.robot.stop()
            self._finish("arrived", "Goal reached")
        except Exception as exc:
            self.robot.stop()
            self._finish("error", str(exc), error=str(exc))

    def _rotate_to_heading(self, heading_deg: float, started: float) -> None:
        while not self._cancel_event.is_set():
            if time.monotonic() - started > self.timeout_seconds:
                raise NavigationError("Navigation timeout reached during final heading")
            odom = self.odometry.status()
            if not odom.ready or odom.stale:
                raise NavigationError("Odometry became stale during final heading")
            error = normalize_heading_deg(heading_deg - odom.pose.heading_deg)
            if abs(error) <= self.heading_tolerance_deg:
                self.robot.stop()
                return
            self.robot.drive(0.0, self._signed_turn(error))
            self._cancel_event.wait(0.10)

    def _signed_turn(self, heading_error: float) -> float:
        magnitude = min(
            self.max_angular,
            max(0.10, abs(heading_error) / 90.0 * self.max_angular),
        )
        return magnitude if heading_error > 0 else -magnitude

    def _finish(self, state: str, reason: str, *, error: str = "") -> None:
        with self._lock:
            self._running = False
            self._state = state
            self._reason = reason
            self._last_error = error
            self._last_update_at = datetime.now(timezone.utc)
