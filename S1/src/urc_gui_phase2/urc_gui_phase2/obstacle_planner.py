"""Small, deterministic obstacle planner in the local REP 103 ENU frame.

This module deliberately has no ROS or Qt dependencies. It models one
circular obstacle and returns either the direct goal or a two-leg detour. The
circle is inflated by a requested clearance so the planned rover center never
touches the physical obstacle boundary.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


Point = Tuple[float, float]
MIN_EXTRA_OFFSET_M = 0.5
SEARCH_STEP_M = 0.25
GEOMETRY_EPSILON_M = 1e-6


class UnreachableTargetError(ValueError):
    """Raised when a start or goal lies inside the inflated obstacle."""


def _finite_point(point: Iterable[float], label: str) -> Point:
    """Return a validated two-dimensional point."""
    try:
        east, north = point
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must contain east and north") from error
    east, north = float(east), float(north)
    if not (math.isfinite(east) and math.isfinite(north)):
        raise ValueError(f"{label} coordinates must be finite")
    return east, north


@dataclass(frozen=True)
class CircleObstacle:
    """Circular obstacle in ENU metres."""

    east_m: float
    north_m: float
    radius_m: float

    def __post_init__(self):
        east, north = _finite_point(
            (self.east_m, self.north_m),
            "obstacle center",
        )
        radius = float(self.radius_m)
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError("obstacle radius must be positive and finite")
        object.__setattr__(self, "east_m", east)
        object.__setattr__(self, "north_m", north)
        object.__setattr__(self, "radius_m", radius)

    @property
    def center(self) -> Point:
        return self.east_m, self.north_m


def point_distance(first: Point, second: Point) -> float:
    """Euclidean distance between two ENU points."""
    return math.hypot(first[0] - second[0], first[1] - second[1])


def point_to_segment_distance(point: Point, start: Point, end: Point) -> float:
    """Shortest distance from a point to a finite line segment."""
    point = _finite_point(point, "point")
    start = _finite_point(start, "segment start")
    end = _finite_point(end, "segment end")
    delta_east = end[0] - start[0]
    delta_north = end[1] - start[1]
    length_squared = delta_east ** 2 + delta_north ** 2
    if length_squared == 0.0:
        return point_distance(point, start)
    fraction = (
        (point[0] - start[0]) * delta_east
        + (point[1] - start[1]) * delta_north
    ) / length_squared
    fraction = max(0.0, min(1.0, fraction))
    closest = (
        start[0] + fraction * delta_east,
        start[1] + fraction * delta_north,
    )
    return point_distance(point, closest)


def route_length(start: Point, route: Iterable[Point]) -> float:
    """Length of a polyline that starts at ``start``."""
    previous = _finite_point(start, "route start")
    total = 0.0
    for index, point in enumerate(route):
        current = _finite_point(point, f"route point {index}")
        total += point_distance(previous, current)
        previous = current
    return total


def plan_route(start: Point, goal: Point, obstacle: CircleObstacle,
               clearance_m: float) -> Tuple[Point, ...]:
    """Plan a direct route or one safe detour around a circular obstacle.

    The planner first inflates the physical radius by ``clearance_m``. If the
    direct segment intersects that inflated circle, candidate waypoints are
    searched on both perpendicular sides of the start-to-goal direction. The
    shortest candidate whose two segments clear the circle is returned.
    """
    start = _finite_point(start, "start")
    goal = _finite_point(goal, "goal")
    clearance = float(clearance_m)
    if not math.isfinite(clearance) or clearance < 0.0:
        raise ValueError("obstacle clearance must be finite and non-negative")

    inflated_radius = obstacle.radius_m + clearance
    for label, point in (("start", start), ("goal", goal)):
        if point_distance(point, obstacle.center) <= inflated_radius:
            raise UnreachableTargetError(
                f"{label} lies inside the obstacle's "
                f"{inflated_radius:.2f} m safety boundary"
            )

    direct_length = point_distance(start, goal)
    if direct_length == 0.0:
        return (goal,)
    direct_clearance = point_to_segment_distance(
        obstacle.center,
        start,
        goal,
    )
    if direct_clearance >= inflated_radius:
        return (goal,)

    direction = (
        (goal[0] - start[0]) / direct_length,
        (goal[1] - start[1]) / direct_length,
    )
    normal = (-direction[1], direction[0])
    initial_offset = inflated_radius + max(clearance, MIN_EXTRA_OFFSET_M)
    max_offset = initial_offset + 2.0 * direct_length + 10.0
    candidates = []

    for side in (-1.0, 1.0):
        offset = initial_offset
        while offset <= max_offset:
            detour = (
                obstacle.east_m + side * normal[0] * offset,
                obstacle.north_m + side * normal[1] * offset,
            )
            first_clearance = point_to_segment_distance(
                obstacle.center,
                start,
                detour,
            )
            second_clearance = point_to_segment_distance(
                obstacle.center,
                detour,
                goal,
            )
            if min(first_clearance, second_clearance) >= (
                    inflated_radius + GEOMETRY_EPSILON_M):
                route = (detour, goal)
                candidates.append((route_length(start, route), route))
                break
            offset += SEARCH_STEP_M

    if not candidates:
        raise UnreachableTargetError(
            "no safe detour was found around the obstacle"
        )
    return min(candidates, key=lambda candidate: candidate[0])[1]


def plan_route_around_obstacles(start: Point, goal: Point,
                                obstacles: Iterable[CircleObstacle],
                                clearance_m: float) -> Tuple[Point, ...]:
    """Plan a route around a separated collection of circular obstacles.

    Each pass finds the first blocked route segment and inserts the safe
    detour produced by :func:`plan_route`. The complete route is checked again
    after every insertion so a detour cannot silently cross another obstacle.
    The presentation course deliberately keeps obstacles separated; heavily
    overlapping circles should be handled by a full path planner instead.
    """
    start = _finite_point(start, "start")
    goal = _finite_point(goal, "goal")
    obstacles = tuple(obstacles)
    if not obstacles:
        return (goal,)

    route = [goal]
    max_insertions = max(8, len(obstacles) * 8)
    for _ in range(max_insertions):
        segment_start = start
        inserted = False
        for segment_index, segment_goal in enumerate(tuple(route)):
            for obstacle in obstacles:
                inflated_radius = obstacle.radius_m + float(clearance_m)
                if point_to_segment_distance(
                        obstacle.center, segment_start, segment_goal
                ) >= inflated_radius:
                    continue
                detour_route = plan_route(
                    segment_start, segment_goal, obstacle, clearance_m
                )
                route.insert(segment_index, detour_route[0])
                inserted = True
                break
            if inserted:
                break
            segment_start = segment_goal
        if not inserted:
            return tuple(route)

    raise UnreachableTargetError(
        "no stable route was found through the obstacle course"
    )
