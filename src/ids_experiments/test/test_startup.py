# =============================================================================
#  test_startup.py - the start-up gate (B16)
# =============================================================================

from ids_experiments.startup import StartupGate


def run(gate, times, ready_inputs=True, states=None):
    states = states or {}
    out = []
    state = None
    for t in times:
        state = states.get(t, state)
        out.append(gate.step(t, ready_inputs, state))
    return out


def test_nothing_happens_before_inputs_or_clock():
    gate = StartupGate(2.0)
    assert gate.step(0.0, True) == (False, False)
    assert gate.step(1.0, False) == (False, False)
    assert not gate.detached


def test_detach_is_repeated_until_the_gripper_reports_detached():
    gate = StartupGate(2.0, detach_period=0.1)
    times = [round(0.01 * k, 2) for k in range(1, 40)]
    out = run(gate, times, states={0.25: 'attached', 0.31: 'detached'})
    published = [t for t, (p, _) in zip(times, out) if p]
    assert published == [0.01, 0.11, 0.21]
    assert gate.detached
    assert not any(r for _, r in out)


def test_settles_after_detach_then_reports_ready_once_due():
    gate = StartupGate(2.0)
    assert gate.step(1.0, True, 'detached') == (False, False)
    assert gate.step(2.99, True, 'detached') == (False, False)
    assert gate.step(3.0, True, 'detached') == (False, True)
    assert gate.step(3.01, True, None) == (False, True)


def test_without_gripper_wait_it_only_settles():
    gate = StartupGate(2.0, wait_for_gripper=False)
    assert gate.detached
    assert gate.step(0.5, True) == (False, False)
    assert gate.step(2.5, True) == (False, True)


def test_an_attached_report_alone_does_not_release_the_start():
    gate = StartupGate(0.5, detach_period=0.1)
    out = run(gate, [round(0.05 * k, 2) for k in range(1, 30)], states={0.1: 'attached'})
    assert not gate.detached
    assert not any(r for _, r in out)
    assert sum(p for p, _ in out) == 15
