import numpy as np
import pytest

from ids_evaluation import layout


def test_mask_round_trip():
    bits = layout.mask_bits([False, True, False, True, False, False, True])
    assert bits == 0b1001010
    assert layout.joints_of(bits) == (1, 3, 6)
    assert layout.joint_label((1, 3)) == 'a2+a4'
    assert layout.joint_label(()) == '-'


def test_kind_index_and_name():
    for i, kind in enumerate(layout.KINDS):
        assert layout.kind_index(kind) == i
        assert layout.kind_name(i) == kind
    assert layout.kind_index('bogus') == -1.0
    assert layout.kind_name(-1) == 'unknown'


def test_status_from_replay_maps_columns():
    w = np.zeros((3, layout.REPLAY_COLUMNS))
    w[:, layout.R_T] = [1.0, 1.01, 1.02]
    w[:, layout.R_MODE] = layout.ARMED
    w[1, layout.R_ALARM] = 1.0
    w[1, layout.R_PRIMARY] = 4
    w[1, layout.R_PHYSICS_ALARM.start + 4] = 1.0
    w[:, layout.R_FAST] = np.arange(7)
    w[:, layout.R_FRICTION] = 2.0
    t, s = layout.status_from_replay(w, 3.5)
    assert np.allclose(t, w[:, 0])
    assert s.shape == (3, layout.STATUS_COLUMNS)
    assert s[1, layout.ALARM] == 1.0 and s[1, layout.PRIMARY] == 4
    assert s[1, layout.MASK] == 16 and s[0, layout.MASK] == 0
    assert np.allclose(s[:, layout.FAST], np.arange(7))
    assert np.allclose(s[:, layout.FRICTION], 2.0)
    assert np.allclose(s[:, layout.LIMIT], 3.5)


def test_pad_status_accepts_legacy_layout():
    legacy = np.ones((4, layout.STATUS_COLUMNS - 1))
    padded = layout.pad_status(legacy)
    assert padded.shape == (4, layout.STATUS_COLUMNS)
    assert np.isnan(padded[:, layout.LIMIT]).all()
    full = np.ones((2, layout.STATUS_COLUMNS))
    assert layout.pad_status(full) is not None
    with pytest.raises(ValueError):
        layout.pad_status(np.ones((2, 10)))