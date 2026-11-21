import math

import numpy as np

from conftest import attack_rows, START, status_block
from ids_evaluation import layout, metrics


def _series(duration, spans, primary=3, dt=0.01):
    t = np.arange(0.0, duration, dt)
    alarm = np.zeros(len(t))
    prim = np.full(len(t), -1.0)
    for a, b in spans:
        on = (t >= a) & (t < b)
        alarm[on] = 1.0
        prim[on] = primary
    status = status_block(t + START, alarm, prim, armed_from=2.0)
    return t, status


def test_events_from_truth_closes_and_leaves_open():
    rows = attack_rows([(1, 'step', (3,), 0.02, 10.0, 20.0)])
    rows = np.vstack([rows, [START + 30.0, 1.0, 2, 0b11, 0.5, layout.kind_index('ramp')]])
    events = metrics.events_from_truth(rows, START)
    assert len(events) == 2
    assert events[0] == metrics.Event(1, 'step', (3,), 0.02, 10.0, 20.0)
    assert events[1].kind == 'ramp' and events[1].joints == (0, 1)
    assert math.isinf(events[1].end)


def test_events_from_truth_splits_back_to_back_events():
    rows = np.array([[START + 5, 1, 1, 1, 0.1, 0], [START + 6, 1, 2, 2, 0.1, 0],
                     [START + 7, 0, 0, 0, 0.0, -1]])
    events = metrics.events_from_truth(rows, START)
    assert [(e.event_id, e.start, e.end) for e in events] == [(1, 5.0, 6.0), (2, 6.0, 7.0)]


def test_episodes():
    t = np.arange(10) * 0.1
    alarm = np.array([0, 1, 1, 0, 0, 1, 0, 0, 0, 1])
    primary = np.array([-1, 2, 2, -1, -1, 5, -1, -1, -1, 0])
    eps = metrics.episodes(t, alarm, primary)
    assert [(round(e.start, 2), e.primary) for e in eps] == [(0.1, 2), (0.5, 5), (0.9, 0)]
    assert math.isclose(eps[0].end, 0.3)
    assert metrics.episodes(t, np.zeros(10), primary) == []


def test_detection_latency_and_attribution():
    t, status = _series(60.0, [(30.05, 40.0)], primary=3)
    events = [metrics.Event(1, 'step', (3,), 0.02, 30.0, 40.0)]
    r = metrics.evaluate_instance('v2', t, status, events)
    e = r.events[0]
    assert e.detected and e.attributed and not e.already_alarmed and e.before_end
    assert math.isclose(e.latency, 0.05, abs_tol=1e-9)
    assert r.false_alarms == []
    assert math.isclose(r.detection_start, 2.0 + 3.99)


def test_wrong_joint_is_not_attributed():
    t, status = _series(60.0, [(30.1, 31.0)], primary=6)
    r = metrics.evaluate_instance('v2', t, status,
                                  [metrics.Event(1, 'step', (3,), 0.02, 30.0, 40.0)])
    assert r.events[0].detected and not r.events[0].attributed


def test_missed_event_and_grace():
    t, status = _series(80.0, [(41.5, 42.0)])
    events = [metrics.Event(1, 'step', (3,), 0.02, 30.0, 40.0)]
    r = metrics.evaluate_instance('v2', t, status, events, grace=1.0)
    assert not r.events[0].detected and math.isnan(r.events[0].latency)
    r = metrics.evaluate_instance('v2', t, status, events, grace=2.0)
    assert r.events[0].detected and not r.events[0].before_end


def test_alarm_already_active_counts_as_zero_latency():
    t, status = _series(60.0, [(20.0, 35.0)])
    r = metrics.evaluate_instance('v2', t, status,
                                  [metrics.Event(1, 'step', (3,), 0.02, 30.0, 40.0)])
    e = r.events[0]
    assert e.detected and e.already_alarmed and e.latency == 0.0
    assert len(r.false_alarms) == 1


def test_false_alarms_and_observed_time():
    t, status = _series(3600.0, [(1000.0, 1001.0), (50.5, 51.0), (1.0, 1.5)], dt=0.1)
    events = [metrics.Event(1, 'step', (3,), 0.02, 50.0, 60.0)]
    r = metrics.evaluate_instance('v2', t, status, events, tail=30.0)
    assert [round(e.start) for e in r.false_alarms] == [1000]
    expected = (t[-1] - (2.0 + 3.99)) - 40.0
    assert math.isclose(r.observed_hours * 3600.0, expected, abs_tol=1e-6)
    assert math.isclose(r.false_alarms_per_hour, 1 / r.observed_hours)
    assert r.armed_windows == int((t >= 2.0).sum())
    assert 0.0 < r.alarm_fraction < 0.01


def test_never_armed():
    t = np.arange(0.0, 10.0, 0.01)
    status = status_block(t + START, armed_from=1e9)
    r = metrics.evaluate_instance('v2', t, status, [])
    assert r.armed_windows == 0 and math.isnan(r.false_alarms_per_hour)


def test_tables():
    t, status = _series(60.0, [(30.05, 40.0)])
    events = [metrics.Event(1, 'step', (3,), 0.02, 30.0, 40.0)]
    a = metrics.evaluate_instance('v2', t, status, events)
    b = metrics.evaluate_instance('v2', t, status, events)
    rows = metrics.per_joint_table([('r1', a), ('r2', b)])
    assert len(rows) == 1 and rows[0]['events'] == 2 and rows[0]['detected'] == 2
    assert rows[0]['before_end'] == 2
    assert math.isclose(rows[0]['median_latency'], 0.05, abs_tol=1e-9)
    pooled = metrics.false_alarm_table([('r1', a), ('r2', b)])
    assert pooled[0]['instance'] == 'v2' and pooled[0]['false_alarms'] == 0
    assert math.isclose(pooled[0]['hours'], a.observed_hours * 2)