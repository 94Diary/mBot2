import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'launch'))
import world_profiles  # noqa: E402

WORLDS = os.path.join(HERE, '..', 'worlds')


def test_seed7_keeps_original_values():
    result = world_profiles.resolve(WORLDS, 'mbot2_seed7_maze')
    assert result['gz_name'] == 'mbot2_seed7_maze'
    assert result['spawns'][0] == ('mbot1', -1.95, -1.25, 0.0)
    assert result['junction_match_radius'] == 0.25


def test_gazebo_name_comes_from_sdf():
    assert world_profiles.resolve(WORLDS, 'Map1_Walled')['gz_name'] == 'Map1_FullyLined_16x16'
    assert world_profiles.resolve(WORLDS, 'Map2')['gz_name'] == 'Map2_FullyLined_16x16'


def test_map_spawns_on_start_plate():
    assert world_profiles.resolve(WORLDS, 'Map1')['spawns'][0][1:3] == (-3.25, 3.0)


@pytest.mark.parametrize('bad', ['../x', 'nope', '', 'a b', 'Map1.sdf'])
def test_bad_world_rejected(bad):
    with pytest.raises(ValueError):
        world_profiles.resolve(WORLDS, bad)
