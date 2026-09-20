"""Geometry tests for the local-frame circular-obstacle planner."""

import math

import pytest

from urc_gui_phase2.obstacle_planner import (
    CircleObstacle,
    UnreachableTargetError,
    plan_route,
    plan_route_around_obstacles,
    point_to_segment_distance,
    route_length,
)


def test_multiple_obstacles_produce_a_multi_leg_safe_route():
    obstacles = (
        CircleObstacle(8.0, 0.0, 2.0),
        CircleObstacle(18.0, -2.0, 2.0),
        CircleObstacle(28.0, 2.0, 2.0),
    )

    route = plan_route_around_obstacles(
        (0.0, 0.0), (38.0, 0.0), obstacles, 1.0
    )

    assert route[-1] == (38.0, 0.0)
    assert len(route) >= 3
    segment_start = (0.0, 0.0)
    for segment_goal in route:
        for obstacle in obstacles:
            assert point_to_segment_distance(
                obstacle.center, segment_start, segment_goal
            ) >= obstacle.radius_m + 1.0
        segment_start = segment_goal


OBSTACLE = CircleObstacle(10.0, 0.0, 2.0)


def test_clear_direct_path_returns_only_the_goal():
    assert plan_route((0.0, 10.0), (20.0, 10.0), OBSTACLE, 1.0) == (
        (20.0, 10.0),
    )


def test_blocked_path_returns_safe_two_leg_detour():
    start = (0.0, 0.0)
    goal = (20.0, 0.0)
    route = plan_route(start, goal, OBSTACLE, 1.0)

    assert len(route) == 2
    detour, returned_goal = route
    assert returned_goal == goal
    assert point_to_segment_distance(OBSTACLE.center, start, detour) > 3.0
    assert point_to_segment_distance(OBSTACLE.center, detour, goal) > 3.0
    assert route_length(start, route) > 20.0


def test_planner_chooses_shorter_side_for_off_center_obstacle():
    obstacle = CircleObstacle(10.0, 2.0, 2.0)
    route = plan_route((0.0, 0.0), (20.0, 0.0), obstacle, 1.0)

    # The obstacle is north of the direct path, so the southern detour is
    # shorter than going farther north around it.
    assert route[0][1] < obstacle.north_m


@pytest.mark.parametrize("point_name", ["start", "goal"])
def test_start_or_goal_inside_safety_boundary_is_unreachable(point_name):
    start, goal = (0.0, 0.0), (20.0, 0.0)
    if point_name == "start":
        start = OBSTACLE.center
    else:
        goal = OBSTACLE.center

    with pytest.raises(UnreachableTargetError, match=point_name):
        plan_route(start, goal, OBSTACLE, 1.0)


def test_zero_length_route_is_valid_when_clear_of_obstacle():
    assert plan_route((0.0, 5.0), (0.0, 5.0), OBSTACLE, 1.0) == (
        (0.0, 5.0),
    )


@pytest.mark.parametrize(
    "obstacle",
    [
        (10.0, 0.0, 0.0),
        (10.0, 0.0, -1.0),
        (float("nan"), 0.0, 1.0),
    ],
)
def test_invalid_obstacle_is_rejected(obstacle):
    with pytest.raises(ValueError):
        CircleObstacle(*obstacle)


def test_segment_distance_uses_segment_endpoints_not_infinite_line():
    distance = point_to_segment_distance(
        (3.0, 4.0),
        (0.0, 0.0),
        (1.0, 0.0),
    )

    assert distance == pytest.approx(math.hypot(2.0, 4.0))
