import json
import os

import pytest

from urc_gui_phase2.mission_model import (
    DuplicateWaypointError,
    InvalidCoordinateError,
    InvalidWaypointError,
    MissionError,
    MissionFormatError,
    MissionModel,
    Status,
    TargetType,
    Waypoint,
    WaypointIndexError,
    WaypointNotFoundError,
)


def make_mission(n=3):
    mission = MissionModel('test course')
    for i in range(n):
        mission.add(f'WP{i + 1}', 38.0 + i * 0.001, -76.0 - i * 0.001)
    return mission


def names(mission):
    return [wp.name for wp in mission]


# ---- add ----------------------------------------------------------------

def test_add_valid_waypoint():
    mission = MissionModel()
    wp = mission.add('Post A', 38.9869, -76.9426)
    assert len(mission) == 1
    assert wp.coordinate == (38.9869, -76.9426)
    assert wp.target_type is TargetType.GNSS
    assert wp.status is Status.PENDING
    assert mission.get(wp.id) == wp


def test_add_generates_unique_ids():
    mission = make_mission(50)
    assert len({wp.id for wp in mission}) == 50


def test_add_accepts_range_boundaries():
    mission = MissionModel()
    mission.add('north-east', 90, 180)
    mission.add('south-west', -90, -180)
    assert len(mission) == 2


@pytest.mark.parametrize('lat, lon', [
    (90.0001, 0), (-90.0001, 0), (0, 180.0001), (0, -180.0001), (91, 0), (0, 181),
    (float('nan'), 0), (0, float('nan')), (float('inf'), 0), (0, float('-inf')),
    ('38.9', -76.9), (38.9, None), (True, 0),
])
def test_add_rejects_invalid_coordinate(lat, lon):
    mission = make_mission(1)
    with pytest.raises(InvalidCoordinateError):
        mission.add('bad', lat, lon)
    assert len(mission) == 1  # failed add leaves the mission unchanged


def test_add_rejects_blank_name_and_duplicate_id():
    mission = MissionModel()
    with pytest.raises(InvalidWaypointError):
        mission.add('   ', 0, 0)
    mission.add('a', 0, 0, waypoint_id='x')
    with pytest.raises(DuplicateWaypointError):
        mission.add('b', 1, 1, waypoint_id='x')
    assert len(mission) == 1


def test_only_gnss_type_accepted():
    with pytest.raises(InvalidWaypointError):
        Waypoint('id', 'name', 0, 0, target_type='AR Tag')


# ---- edit ---------------------------------------------------------------

def test_edit_changes_only_given_fields():
    mission = make_mission(2)
    first = mission.waypoints[0]
    edited = mission.edit(first.id, lat_deg=10.5)
    assert edited.lat_deg == 10.5
    assert edited.lon_deg == first.lon_deg
    assert edited.name == first.name
    assert edited.id == first.id
    assert mission.index_of(first.id) == 0


def test_edit_invalid_leaves_waypoint_unchanged():
    mission = make_mission(1)
    original = mission.waypoints[0]
    with pytest.raises(InvalidCoordinateError):
        mission.edit(original.id, name='renamed', lat_deg=123)
    assert mission.get(original.id) == original  # name was not half-applied


def test_edit_missing_id_raises():
    with pytest.raises(WaypointNotFoundError):
        make_mission(1).edit('nope', name='x')


# ---- remove -------------------------------------------------------------

def test_remove_existing_waypoint():
    mission = make_mission(3)
    target = mission.waypoints[1]
    assert mission.remove(target.id) == target
    assert names(mission) == ['WP1', 'WP3']


def test_remove_nonexistent_raises_clear_error():
    mission = make_mission(2)
    with pytest.raises(WaypointNotFoundError, match='nope'):
        mission.remove('nope')
    assert len(mission) == 2


def test_remove_at_bad_index_raises():
    mission = make_mission(2)
    with pytest.raises(WaypointIndexError):
        mission.remove_at(2)
    with pytest.raises(WaypointIndexError):
        mission.remove_at(-1)
    assert mission.remove_at(0).name == 'WP1'


def test_errors_are_catchable_as_mission_error():
    with pytest.raises(MissionError):
        MissionModel().remove('x')


# ---- reorder ------------------------------------------------------------

def test_move_to_index():
    mission = make_mission(4)
    mission.move_to(mission.waypoints[0].id, 2)
    assert names(mission) == ['WP2', 'WP3', 'WP1', 'WP4']
    mission.move_to(mission.waypoints[3].id, 0)
    assert names(mission) == ['WP4', 'WP2', 'WP3', 'WP1']


def test_move_up_and_down():
    mission = make_mission(3)
    last = mission.waypoints[2].id
    mission.move_up(last)
    assert names(mission) == ['WP1', 'WP3', 'WP2']
    mission.move_down(last)
    assert names(mission) == ['WP1', 'WP2', 'WP3']


def test_move_out_of_range_raises_and_changes_nothing():
    mission = make_mission(3)
    with pytest.raises(WaypointIndexError):
        mission.move_up(mission.waypoints[0].id)
    with pytest.raises(WaypointIndexError):
        mission.move_down(mission.waypoints[2].id)
    with pytest.raises(WaypointIndexError):
        mission.move_to(mission.waypoints[0].id, 3)
    assert names(mission) == ['WP1', 'WP2', 'WP3']


def test_reorder_keeps_ids_and_status():
    mission = make_mission(3)
    active = mission.waypoints[0]
    mission.set_status(active.id, Status.ACTIVE)
    mission.move_to(active.id, 2)
    assert mission.get_active_target().id == active.id


# ---- active target / status ---------------------------------------------

def test_no_active_target_by_default():
    assert make_mission(2).get_active_target() is None


def test_only_one_active_at_a_time():
    mission = make_mission(3)
    a, b, _ = mission.waypoints
    mission.set_status(a.id, Status.ACTIVE)
    mission.set_status(b.id, Status.ACTIVE)
    assert mission.get_active_target().id == b.id
    assert mission.get(a.id).status is Status.PENDING
    assert sum(wp.status is Status.ACTIVE for wp in mission) == 1


def test_complete_active_advances():
    mission = make_mission(2)
    a, b = mission.waypoints
    mission.set_status(a.id, Status.ACTIVE)
    assert mission.complete_active().id == b.id
    assert mission.get(a.id).status is Status.DONE
    assert mission.complete_active() is None
    assert mission.get(b.id).status is Status.DONE
    assert mission.get_active_target() is None


def test_complete_active_without_active_raises():
    with pytest.raises(MissionError):
        make_mission(1).complete_active()


def test_removing_active_leaves_no_active():
    mission = make_mission(2)
    a = mission.waypoints[0]
    mission.set_status(a.id, Status.ACTIVE)
    mission.remove(a.id)
    assert mission.get_active_target() is None


def test_waypoints_view_cannot_mutate_mission():
    mission = make_mission(2)
    with pytest.raises(AttributeError):
        mission.waypoints.append('x')
    with pytest.raises(Exception):
        mission.waypoints[0].lat_deg = 5.0  # frozen dataclass


# ---- save / load --------------------------------------------------------

def test_save_load_round_trip_is_identical(tmp_path):
    mission = make_mission(4)
    a, b, c, d = mission.waypoints
    mission.edit(b.id, name='  renamed  ', lat_deg=38.123456789012345)
    mission.set_status(a.id, Status.DONE)
    mission.set_status(b.id, Status.ACTIVE)
    mission.move_to(d.id, 0)
    path = str(tmp_path / 'mission.json')

    mission.save(path)
    loaded = MissionModel.load(path)

    assert loaded == mission
    assert loaded.waypoints == mission.waypoints  # same order, ids, floats, status
    assert loaded.get_active_target().id == b.id
    assert loaded.get(b.id).name == 'renamed'


def test_empty_mission_round_trip(tmp_path):
    path = str(tmp_path / 'empty.json')
    MissionModel('nothing yet').save(path)
    assert MissionModel.load(path) == MissionModel('nothing yet')


def test_save_leaves_no_temp_files_and_overwrites(tmp_path):
    path = str(tmp_path / 'm.json')
    make_mission(1).save(path)
    make_mission(3).save(path)
    assert os.listdir(tmp_path) == ['m.json']
    assert len(MissionModel.load(path)) == 3


def test_failed_save_keeps_previous_file(tmp_path):
    path = str(tmp_path / 'm.json')
    good = make_mission(2)
    good.save(path)
    before = open(path).read()
    broken = make_mission(1)
    broken._waypoints.append(object())  # force serialization to blow up mid-save
    with pytest.raises(Exception):
        broken.save(path)
    assert open(path).read() == before
    assert os.listdir(tmp_path) == ['m.json']


def write_json(tmp_path, data):
    path = tmp_path / 'm.json'
    path.write_text(data if isinstance(data, str) else json.dumps(data))
    return str(path)


def valid_dict():
    return make_mission(2).to_dict()


def test_load_rejects_garbage(tmp_path):
    with pytest.raises(MissionFormatError):
        MissionModel.load(write_json(tmp_path, '{not json'))


def test_load_missing_file_raises_oserror(tmp_path):
    with pytest.raises(FileNotFoundError):
        MissionModel.load(str(tmp_path / 'absent.json'))


def test_load_rejects_wrong_schema_version(tmp_path):
    data = valid_dict()
    data['schema_version'] = 99
    with pytest.raises(MissionFormatError, match='schema_version'):
        MissionModel.load(write_json(tmp_path, data))


def test_load_rejects_invalid_coordinate_in_file(tmp_path):
    data = valid_dict()
    data['waypoints'][0]['lat_deg'] = 500
    with pytest.raises(MissionFormatError, match='out of range'):
        MissionModel.load(write_json(tmp_path, data))


def test_load_rejects_nan_in_file(tmp_path):
    text = json.dumps(valid_dict()).replace('38.0', 'NaN', 1)
    with pytest.raises(MissionFormatError):
        MissionModel.load(write_json(tmp_path, text))


def test_load_rejects_duplicate_ids_and_two_active(tmp_path):
    data = valid_dict()
    data['waypoints'][1]['id'] = data['waypoints'][0]['id']
    with pytest.raises(MissionFormatError, match='duplicate'):
        MissionModel.load(write_json(tmp_path, data))

    data = valid_dict()
    for wp in data['waypoints']:
        wp['status'] = 'active'
    with pytest.raises(MissionFormatError, match='more than one active'):
        MissionModel.load(write_json(tmp_path, data))


def test_load_rejects_missing_fields_and_bad_status(tmp_path):
    data = valid_dict()
    del data['waypoints'][0]['lon_deg']
    with pytest.raises(MissionFormatError, match='missing'):
        MissionModel.load(write_json(tmp_path, data))

    data = valid_dict()
    data['waypoints'][0]['status'] = 'finished'
    with pytest.raises(MissionFormatError):
        MissionModel.load(write_json(tmp_path, data))
