import xml.etree.ElementTree as ET

import pytest

from ids_experiments.payload import cube_inertia, payload_sdf


def test_cube_inertia_matches_the_controller_payload_model():
    assert cube_inertia(5.0) == pytest.approx(0.008333, abs=1e-6)


def test_payload_sdf_describes_the_cube_the_detachable_joint_expects():
    root = ET.fromstring(payload_sdf(5.0))
    model = root.find('model')
    assert model.get('name') == 'payload'
    link = model.find('link')
    assert link.get('name') == 'cube_link'
    assert float(link.find('inertial/mass').text) == pytest.approx(5.0)
    expected = cube_inertia(5.0)
    for axis in ('ixx', 'iyy', 'izz'):
        assert float(link.find(f'inertial/inertia/{axis}').text) == pytest.approx(expected)
    assert link.find('collision/geometry/box/size').text == '0.1 0.1 0.1'