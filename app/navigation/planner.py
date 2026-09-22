from __future__ import annotations

import heapq
import math

from app.models import NavigationGoal, NavigationPlan, Pose2D
from app.navigation.mapping import LocalOccupancyMap


class AStarPlanner:
    def __init__(
        self,
        occupancy_map: LocalOccupancyMap,
        *,
        robot_radius_cm: float,
        allow_unknown: bool = False,
    ) -> None:
        self.map = occupancy_map
        self.robot_radius_cm = max(1.0, float(robot_radius_cm))
        self.allow_unknown = allow_unknown

    def plan(
        self,
        start: Pose2D,
        goal: NavigationGoal,
        *,
        allow_unknown: bool | None = None,
    ) -> NavigationPlan:
        permit_unknown = self.allow_unknown if allow_unknown is None else allow_unknown
        start_cell = self.map.world_to_cell(start.x_cm, start.y_cm)
        goal_cell = self.map.world_to_cell(goal.x_cm, goal.y_cm)
        if start_cell is None:
            return self._failure(goal, "Current pose is outside the local map")
        if goal_cell is None:
            return self._failure(goal, "Goal is outside the local map")

        scores = self.map.grid_copy()
        blocked = self._inflated_obstacles(scores)
        if start_cell in blocked:
            return self._failure(goal, "Robot pose overlaps an occupied safety-inflation cell")
        if goal_cell in blocked:
            return self._failure(goal, "Goal overlaps an occupied safety-inflation cell")

        open_heap: list[tuple[float, int, tuple[int, int]]] = []
        counter = 0
        heapq.heappush(open_heap, (0.0, counter, start_cell))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score = {start_cell: 0.0}
        explored = 0

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            explored += 1
            if current == goal_cell:
                path = self._reconstruct(came_from, current)
                return self._to_plan(path, scores, goal, explored)

            for neighbor, move_cost in self._neighbors(current):
                if not self.map.in_bounds(neighbor) or neighbor in blocked:
                    continue
                state = scores.get(neighbor, 0)
                if state == 0 and not permit_unknown and neighbor != goal_cell:
                    continue
                unknown_penalty = 2.25 if state == 0 else 1.0
                tentative = g_score[current] + move_cost * unknown_penalty
                if tentative >= g_score.get(neighbor, float("inf")):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                counter += 1
                priority = tentative + self._heuristic(neighbor, goal_cell)
                heapq.heappush(open_heap, (priority, counter, neighbor))

        return self._failure(goal, "No traversable path was found", explored)

    def _inflated_obstacles(
        self,
        scores: dict[tuple[int, int], int],
    ) -> set[tuple[int, int]]:
        occupied = [cell for cell, score in scores.items() if score >= 2]
        radius_cells = max(1, int(math.ceil(self.robot_radius_cm / self.map.resolution_cm)))
        offsets = [
            (dx, dy)
            for dx in range(-radius_cells, radius_cells + 1)
            for dy in range(-radius_cells, radius_cells + 1)
            if dx * dx + dy * dy <= radius_cells * radius_cells
        ]
        blocked: set[tuple[int, int]] = set()
        for cell in occupied:
            for dx, dy in offsets:
                inflated = (cell[0] + dx, cell[1] + dy)
                if self.map.in_bounds(inflated):
                    blocked.add(inflated)
        return blocked

    def _to_plan(
        self,
        path: list[tuple[int, int]],
        scores: dict[tuple[int, int], int],
        goal: NavigationGoal,
        explored: int,
    ) -> NavigationPlan:
        if not path:
            return self._failure(goal, "Planner returned an empty path", explored)

        stride = max(1, int(round(20.0 / self.map.resolution_cm)))
        selected = path[::stride]
        if selected[-1] != path[-1]:
            selected.append(path[-1])

        waypoints: list[Pose2D] = []
        for index, cell in enumerate(selected):
            x_cm, y_cm = self.map.cell_to_world(cell)
            heading = 0.0
            if index + 1 < len(selected):
                nx, ny = self.map.cell_to_world(selected[index + 1])
                heading = math.degrees(math.atan2(ny - y_cm, nx - x_cm))
            elif goal.heading_deg is not None:
                heading = goal.heading_deg
            waypoints.append(Pose2D(x_cm=x_cm, y_cm=y_cm, heading_deg=heading))

        planned_distance = 0.0
        for first, second in zip(waypoints, waypoints[1:]):
            planned_distance += math.hypot(
                second.x_cm - first.x_cm,
                second.y_cm - first.y_cm,
            )

        return NavigationPlan(
            found=True,
            reason="Path found",
            goal=goal,
            waypoints=waypoints,
            planned_distance_cm=planned_distance,
            cells_explored=explored,
            uses_unknown_space=any(scores.get(cell, 0) == 0 for cell in path),
        )

    def _neighbors(
        self,
        cell: tuple[int, int],
    ) -> list[tuple[tuple[int, int], float]]:
        x, y = cell
        root2 = math.sqrt(2.0)
        return [
            ((x + 1, y), 1.0),
            ((x - 1, y), 1.0),
            ((x, y + 1), 1.0),
            ((x, y - 1), 1.0),
            ((x + 1, y + 1), root2),
            ((x + 1, y - 1), root2),
            ((x - 1, y + 1), root2),
            ((x - 1, y - 1), root2),
        ]

    @staticmethod
    def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _reconstruct(
        came_from: dict[tuple[int, int], tuple[int, int]],
        current: tuple[int, int],
    ) -> list[tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def _failure(
        goal: NavigationGoal,
        reason: str,
        explored: int = 0,
    ) -> NavigationPlan:
        return NavigationPlan(
            found=False,
            reason=reason,
            goal=goal,
            cells_explored=explored,
        )
