# =============================================================================
#  test_kinematics.py - IK on the real robot description
# -----------------------------------------------------------------------------
#  Expands the iiwa_sim xacro, so it needs that package installed.
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import xacro

from ids_experiments.kinematics import IkError, Kinematics, station_targets

HOME = np.array([0.0, 0.5, 0.0, -1.2, 0.0, 0.8, 0.0])


@pytest.fixture(scope='module')
def kinematics():
    path = os.path.join(get_package_share_directory('iiwa_sim'), 'urdf', 'iiwa_sim.urdf.xacro')
    return Kinematics(xacro.process_file(path).toxml())


# ikpy and Pinocchio agree on where the tool is.
def test_forward_kinematics_matches_the_ids_model_at_home(kinematics):
    np.testing.assert_allclose(kinematics.forward(HOME)[:3, 3], [0.6901, 0.0, 0.5535], atol=1e-3)


# Every station pose is reached within 1 mm, tool down.
def test_station_waypoints_are_reached_with_the_tool_pointing_down(kinematics):
    targets = (station_targets('a', (-0.5, 0.0), 0.25, 0.10, 0.11)
               + station_targets('b', (0.0, 0.5), 0.25, 0.10, 0.11))
    solutions = kinematics.chain(targets, HOME)
    for name, position in targets:
        frame = kinematics.forward(solutions[name])
        np.testing.assert_allclose(frame[:3, 3], position, atol=1e-3)
        assert frame[2, 2] == pytest.approx(-1.0, abs=1e-3)


# No wrist flips between poses (repeatable motion).
def test_seeded_chain_keeps_the_wrist_continuous(kinematics):
    targets = (station_targets('a', (-0.5, 0.0), 0.25, 0.10, 0.11)
               + station_targets('b', (0.0, 0.5), 0.25, 0.10, 0.11))
    solutions = list(kinematics.chain(targets, HOME).values())
    for previous, current in zip(solutions, solutions[1:]):
        assert abs(current[6] - previous[6]) < 0.3
        assert abs(current[3]) > 0.2


def test_unreachable_targets_raise(kinematics):
    with pytest.raises(IkError):
        kinematics.inverse((3.0, 0.0, 0.2), HOME)
