import json
import math
import os

import numpy as np
import pytest
import yaml

from ids_evaluation import bag, cli, halt, layout, security

START = 100.0


def scenario(**kw):
    return np.array(json.dumps(dict({'name': 's', 'attacks': []}, **kw)))


def reports(**kw):
    return np.array(json.dumps(kw))


def node(role, enabled, counts, matched, enclave=''):
    return {'role': role, 'security_enabled': enabled, 'enclave': enclave, 'counts': counts,
            'matched': {k: {'max': v, 'last': v} for k, v in matched.items()}}


STEP = [{'kind': 'step', 'joint': 4, 'magnitude': 0.02, 'start': 120.0, 'duration': 10.0}]


@pytest.mark.parametrize('scen,rep,foreign,outcome', [
    (scenario(), reports(), [0, 0, 0], 'no attacker'),
    (scenario(attacks=STEP, attacker={'role': 'rogue', 'credentials': 'none'}),
     reports(rogue=node('rogue', False, {'injected_blind': 18000},
                        {'target_subscribers': 0, 'listen_publishers': 0})),
     [0, 0, 0], 'blocked by SROS2'),
    (scenario(attacks=STEP, attacker={'role': 'rogue', 'credentials': 'none'}),
     reports(rogue=node('rogue', False, {'injected_blind': 10, 'received_trusted': 5},
                        {'target_subscribers': 1})),
     [10, 0, 0], 'NOT blocked'),
    (scenario(attacks=STEP, attacker={'role': 'rogue', 'credentials': 'forged'}),
     reports(rogue=node('rogue', True, {'injected_blind': 3}, {'target_subscribers': 0},
                        '/attacker/forged')),
     [np.nan, np.nan, np.nan], 'blocked by SROS2'),
    (scenario(attacks=STEP),
     reports(mitm=node('mitm', True, {'received_trusted': 18000, 'forwarded': 18000},
                       {'output_subscribers': 1}, '/attacker/implant')),
     [0, 0, 0], 'passed SROS2'),
    (scenario(attacks=STEP), reports(), [0, 0, 0], 'no node report'),
    (scenario(attacks=STEP, attacker={'role': 'spoof'}),
     reports(spoof=node('spoof', True, {'received_trusted': 0}, {'output_subscribers': 0})),
     [0, 0, 0], 'did not connect'),
])
def test_security_outcomes(make_export, scen, rep, foreign, outcome):
    e = make_export()
    e['scenario'], e['security'], e['foreign'] = scen, rep, np.array(foreign, dtype=float)
    row = security.security_row(e)
    assert row['outcome'] == outcome
    assert row['v2_alarms'] == 0


def test_security_row_details(make_export):
    e = make_export(alarm_spans=[(20.0, 22.0), (30.0, 31.0)])
    e['scenario'] = scenario(attacks=STEP)
    e['security'] = reports(mitm=node('mitm', True, {'received_trusted': 5, 'forwarded': 4,
                                                     'forwarded_tampered': 1},
                                      {'output_subscribers': 1, 'input_publishers': 1},
                                      '/attacker/implant'))
    e['foreign'] = np.zeros(3)
    row = security.security_row(e)
    assert row['security'] == 'on, /attacker/implant' and row['published'] == 4
    assert row['peers'] == 1 and row['v2_alarms'] == 2 and row['rogue_samples'] == 0.0
    assert security.wanted(e)
    plain = make_export()
    assert not security.wanted(bag.normalise(plain))


def test_reports_are_read_from_the_cell_folder_next_to_the_runs(tmp_path):
    run = tmp_path / 'data' / 'runs' / '20261121-100000_rogue_nocred_s1'
    run.mkdir(parents=True)
    reports_dir = tmp_path / 'data' / 'cell' / 'reports' / run.name
    reports_dir.mkdir(parents=True)
    (reports_dir / 'rogue.yaml').write_text(yaml.safe_dump({'role': 'rogue'}))
    (reports_dir / 'attacker_scenario.yaml').write_text(yaml.safe_dump({'name': 'x'}))
    (reports_dir / 'broken.yaml').write_text('{bad')
    assert security.read_reports(str(run)) == {'rogue': {'role': 'rogue'}}
    assert security.read_reports(str(tmp_path / 'nowhere')) == {}


def test_old_exports_get_unknown_cell_fields(make_export):
    e = bag.normalise(make_export())
    assert np.isnan(e['foreign']).all() and e['alarm_topic'].shape == (0, 2)
    assert json.loads(str(e['security'])) == {}


def measurement(t, halted, speed=0.0, q0=0.0):
    n = len(t)
    m = np.zeros((n, layout.MEASUREMENT_COLUMNS))
    m[:, 0] = START + t
    m[:, 2] = halted
    m[:, halt.V] = speed
    m[:, halt.Q] = q0
    return m


def halt_export(make_export, halted_from=None, requested=True, resume_at=None):
    e = make_export(duration=60.0, alarm_spans=[(30.0, 60.0)])
    t = np.arange(0.0, 60.0, 0.01)
    flags = (t < 0.5).astype(float)
    speed = np.full(len(t), 0.2)
    if halted_from is not None:
        h = t >= halted_from
        if resume_at is not None:
            h &= t < resume_at
        flags[h] = 1.0
        speed[t >= halted_from + 0.5] = 0.001
    e['measurement'] = measurement(t, flags, 0.0)
    e['measurement'][:, halt.V] = speed[:, None]
    if halted_from is not None:
        e['measurement'][t >= halted_from + 0.2, halt.Q.start + 3] = 0.01
    e['scenario'] = scenario(halt_on_alarm=requested)
    e['alarm_topic'] = np.array([[START + 0.1, 0.0], [START + 30.004, 1.0]])
    return bag.normalise(e)


def test_halt_latency_hold_and_speed(make_export):
    row = halt.halt_summary(halt_export(make_export, halted_from=30.01))
    assert row['alarm'] == pytest.approx(30.0) and row['first_halt'] == pytest.approx(30.01)
    assert row['latency'] == pytest.approx(0.01) and row['held_to_end']
    assert row['alarm_message'] == pytest.approx(30.004)
    assert row['max_speed'] == pytest.approx(0.001)
    assert row['halted_s'] == pytest.approx(59.99 - 30.01)
    assert row['hold_drift'] == pytest.approx(0.01)


def test_halt_that_resumes_is_not_held(make_export):
    row = halt.halt_summary(halt_export(make_export, halted_from=30.02, resume_at=40.0))
    assert not row['held_to_end'] and row['halted_s'] == pytest.approx(39.99 - 30.02)


def test_requested_halt_that_never_came(make_export):
    row = halt.halt_summary(halt_export(make_export, halted_from=None))
    assert row['requested'] and math.isnan(row['first_halt']) and not row['held_to_end']


def test_runs_without_a_halt_have_no_row(make_export):
    assert halt.halt_summary(halt_export(make_export, None, requested=False)) is None
    e = bag.normalise(make_export())
    e['measurement'] = np.zeros((0, layout.MEASUREMENT_COLUMNS))
    assert halt.halt_summary(e) is None


def test_evaluate_report_gets_security_and_halt_sections(tmp_path, make_export):
    a = halt_export(make_export, halted_from=30.01)
    a['run'] = np.array('halt_s1')
    a['security'] = reports(mitm=node('mitm', True, {'received_trusted': 1, 'forwarded': 1},
                                      {'output_subscribers': 1}, '/attacker/implant'))
    a['scenario'] = scenario(attacks=STEP, halt_on_alarm=True)
    b = bag.normalise(make_export(run='nominal_s1'))
    paths = []
    for e in (a, b):
        p = str(tmp_path / f"{e['run']}.npz")
        bag.save_export(p, e)
        paths.append(p)
    out = str(tmp_path / 'eval')
    assert cli.evaluate_main([*paths, '--out', out, '--no-plots']) == 0
    text = open(os.path.join(out, 'report.md')).read()
    assert '## Security outcome' in text and 'passed SROS2' in text
    assert '## Halt chain' in text
    assert os.path.isfile(os.path.join(out, 'security.csv'))
    assert os.path.isfile(os.path.join(out, 'halt.csv'))