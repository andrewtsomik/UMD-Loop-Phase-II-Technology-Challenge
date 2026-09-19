"""Unit tests for meaningful path sampling and REP 103 heading math."""

import math

import pytest

from urc_gui_phase2.rover_track import RoverTrack


def test_first_fix_starts_path_without_inventing_a_heading():
    track = RoverTrack(min_step_m=0.5)

    assert track.add_fix(38.0, -110.0, 0.0, 0.0) is True
    assert track.points == ((38.0, -110.0),)
    assert track.heading_deg is None


def test_subthreshold_jitter_does_not_change_path_or_heading():
    track = RoverTrack(min_step_m=0.5)
    track.add_fix(38.0, -110.0, 0.0, 0.0)

    assert track.add_fix(38.000001, -110.000001, 0.2, 0.2) is False
    assert len(track.points) == 1
    assert track.heading_deg is None


@pytest.mark.parametrize(
    'east,north,expected_heading',
    [
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 90.0),
        (0.0, -1.0, 180.0),
        (-1.0, 0.0, 270.0),
    ],
)
def test_heading_uses_compass_convention(east, north, expected_heading):
    track = RoverTrack(min_step_m=0.1)
    track.add_fix(38.0, -110.0, 0.0, 0.0)

    assert track.add_fix(38.1, -109.9, east, north) is True
    assert track.heading_deg == pytest.approx(expected_heading)


def test_reset_clears_path_and_heading():
    track = RoverTrack(min_step_m=0.1)
    track.add_fix(38.0, -110.0, 0.0, 0.0)
    track.add_fix(38.1, -109.9, 1.0, 0.0)

    track.reset()

    assert track.points == ()
    assert track.heading_deg is None


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_non_finite_fix_is_rejected(value):
    track = RoverTrack()

    with pytest.raises(ValueError, match='finite'):
        track.add_fix(value, -110.0, 0.0, 0.0)


@pytest.mark.parametrize('threshold', [0.0, -1.0, math.inf, math.nan])
def test_invalid_threshold_is_rejected(threshold):
    with pytest.raises(ValueError, match='positive finite'):
        RoverTrack(threshold)
