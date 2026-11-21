# =============================================================================
#  test_sequencer.py - the PLC program and its handover order
# -----------------------------------------------------------------------------
#  The handover order is what keeps grabs from causing false alarms, so it is pinned here.
# =============================================================================

import numpy as np

from ids_experiments.sequencer import (Command, Dwell, Move, Sequencer, SetContext,
                                       shuttle_program)

HOME = np.zeros(7)
A = {'hover': np.full(7, 0.10), 'grab': np.full(7, 0.20), 'drop': np.full(7, 0.19)}
B = {'hover': np.full(7, -0.10), 'grab': np.full(7, -0.20), 'drop': np.full(7, -0.19)}


def run(sequencer, seconds, period=0.01):
    outputs = []
    for k in range(int(round(seconds / period)) + 1):
        outputs.append((k * period, sequencer.scan(k * period)))
    return outputs


# The reference never jumps between steps.
def test_move_then_dwell_streams_a_continuous_profile():
    s = Sequencer(HOME, [Move(tuple(A['hover']), 1.0), Dwell(0.5)], passes=1)
    out = run(s, 2.0)
    positions = np.array([o.position for _, o in out])
    assert np.abs(np.diff(positions, axis=0)).max() < 0.01
    np.testing.assert_allclose(out[0][1].position, HOME)
    np.testing.assert_allclose(out[-1][1].position, A['hover'])
    assert out[-1][1].finished
    assert not any(o.finished for t, o in out if t < 1.5)


# Each gripper command is sent once.
def test_commands_are_emitted_exactly_once_per_pass():
    s = Sequencer(HOME, [Command('attach'), Dwell(0.1), Command('detach'), Dwell(0.1)], passes=3)
    commands = [c.name for _, o in run(s, 1.0) for c in o.commands]
    assert commands == ['attach', 'detach'] * 3


# Context on before attach; context off before detach.
def test_shuttle_switches_context_inside_the_handover_order():
    s = Sequencer(HOME, shuttle_program(A, B, 5.0, 1.0, 0.5, 0.2), passes=1)
    events = []
    context = 0
    for t, o in run(s, 12.0):
        if o.context != context:
            events.append((t, f'context {o.context}'))
            context = o.context
            assert o.mass == (5.0 if o.context else 0.0)
        for c in o.commands:
            events.append((t, c.name))
    names = [e for _, e in events]
    assert names == ['context 1', 'attach', 'context 0', 'detach'] * 2
    times = dict()
    for t, e in events[:4]:
        times[e] = t
    assert 0.0 < times['attach'] - times['context 1'] <= 0.02
    assert 0.0 < times['detach'] - times['context 0'] <= 0.02


# The part leaves the floor 20 ms after the context change (B23).
def test_the_lift_starts_on_the_scan_after_the_grab():
    s = Sequencer(HOME, shuttle_program(A, B, 5.0, 1.0, 0.5, 0.2), passes=1)
    out = run(s, 6.0)
    context_on = next(t for t, o in out if o.context == 1)
    attach = next(t for t, o in out for c in o.commands if c.name == 'attach')
    lift = next(t for t, o in out if o.label == 'lift')
    settle = [t for t, o in out if o.label == 'settle at grab']
    assert settle and max(settle) < context_on
    assert 0.0 < attach - context_on <= 0.02
    assert 0.0 < lift - attach <= 0.02


# Never lift with the context still off.
def test_lift_starts_after_the_payload_context_is_on():
    s = Sequencer(HOME, shuttle_program(A, B, 5.0, 1.0, 0.5, 0.2), passes=1)
    previous = None
    for _, o in run(s, 3.5):
        if previous is not None and o.label == 'lift' and previous.label != 'lift':
            assert previous.context == 1
        previous = o


def test_a_finished_program_holds_its_last_pose():
    s = Sequencer(HOME, shuttle_program(A, B, 5.0, 1.0, 0.5, 0.2), passes=1)
    out = run(s, 15.0)
    last = out[-1][1]
    assert last.finished
    np.testing.assert_allclose(last.position, A['hover'])
    np.testing.assert_allclose(last.velocity, 0.0)
    assert last.context == 0
    assert s.completed_passes == 1


def test_set_context_without_payload_reports_zero_mass():
    s = Sequencer(HOME, [SetContext(1, 4.0), Dwell(0.1), SetContext(0, 4.0), Dwell(0.1)], 1)
    masses = {(o.context, o.mass) for _, o in run(s, 0.5)}
    assert masses == {(1, 4.0), (0, 0.0)}


# Re-seating happens after every release.
def test_the_part_is_seated_on_its_pad_after_each_release_when_places_are_given():
    places = ((0.0, 0.5), (-0.5, 0.0))
    s = Sequencer(HOME, shuttle_program(A, B, 5.0, 1.0, 0.5, 0.2, places), passes=1)
    events = [(t, c) for t, o in run(s, 12.0) for c in o.commands]
    assert [c.name for _, c in events] == ['attach', 'detach', 'seat'] * 2
    seats = [(t, c) for t, c in events if c.name == 'seat']
    detaches = [t for t, c in events if c.name == 'detach']
    assert [c.args for _, c in seats] == list(places)
    for (t, _), d in zip(seats, detaches):
        assert t - d >= 0.2 - 1e-9
    plain = Sequencer(HOME, shuttle_program(A, B, 5.0, 1.0, 0.5, 0.2), passes=1)
    assert 'seat' not in [c.name for _, o in run(plain, 12.0) for c in o.commands]
