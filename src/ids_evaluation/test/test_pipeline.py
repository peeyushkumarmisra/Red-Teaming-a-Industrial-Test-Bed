# =============================================================================
#  test_pipeline.py - training, held-out scoring and replay fidelity
# =============================================================================

import json
import math

import numpy as np
import pytest

from ids_evaluation import layout, pipeline


def test_live_series_uses_export_limits(make_export):
    export = make_export()
    series = pipeline.live_series(export)
    assert [s.name for s in series] == ['v2', 'thesis']
    assert series[0].limit == 1.0 and series[0].t[0] == 0.0
    export['limits'] = np.full(4, np.nan)
    series = pipeline.live_series(export)
    assert series[0].limit == 1.0


def test_evaluate_and_held_out(make_export):
    export = make_export(duration=120.0, events=[(1, 'step', (3,), 0.02, 60.0, 70.0)],
                         alarm_spans=[(20.0, 21.0), (60.05, 70.0)])
    events, results = pipeline.evaluate(export, pipeline.live_series(export))
    v2 = results[0]
    assert v2.events[0].detected and math.isclose(v2.events[0].latency, 0.05, abs_tol=1e-6)
    assert len(v2.false_alarms) == 1
    _, results = pipeline.evaluate(export, pipeline.live_series(export), after=30.0)
    assert results[0].false_alarms == []
    assert results[0].detection_start > 30.0


def test_crop_and_training_end(make_export):
    export = make_export(duration=60.0)
    cropped = pipeline.crop(export, 20.0)
    assert cropped['reference'][-1, 0] <= pipeline.start_of(export) + 20.0
    assert len(cropped['measurement']) < len(export['measurement'])
    assert pipeline.training_end(120.0) == 123.0


def test_residual_baselines_detect_a_shift(make_export):
    train = make_export(duration=150.0, seed=1, run='nominal_s1')
    test = make_export(duration=150.0, seed=2, run='step_a4_s1',
                       events=[(1, 'step', (3,), 0.02, 130.0, 140.0)],
                       disturbance_shift=[(3, 130.0, 140.0, 8.0)])
    out, limits = pipeline.residual_baselines(train, [train, test], 123.0)
    assert set(limits) == {'static', 'ewma'}
    for s in out['nominal_s1']:
        held = pipeline.held_out(s, 123.0)
        assert not (held.status[held.t <= 123.0, layout.ALARM] > 0).any()
    _, results = pipeline.evaluate(test, out['step_a4_s1'])
    for r in results:
        assert r.events[0].detected and r.events[0].attributed
        assert r.events[0].latency < 0.2


def test_residual_baselines_need_training_data(make_export):
    short = make_export(duration=4.0)
    with pytest.raises(ValueError):
        pipeline.residual_baselines(short, [short], 123.0)


def test_replay_fidelity(make_export):
    export = make_export()
    live = pipeline.live_series(export)[0]
    f = pipeline.replay_fidelity(live, live)
    assert f['matched'] == f['live']
    assert max(f['relative_rms']) == 0.0 and f['agree_fraction'] == 1.0
    assert np.allclose(f['correlation'], 1.0)
    assert pipeline.replay_fidelity(live, pipeline.Series('x', live.t + 1000.0, live.status,
                                                          1.0)) is None


def test_saturated_runs_are_dropped(make_export):
    ok = make_export(run='a_s1')
    bad = make_export(run='b_s1')
    bad['summary'] = np.array(json.dumps({'warnings': [
        'torque at an effort limit in 5.0% of measurement windows']}))
    assert pipeline.saturated(bad) and not pipeline.saturated(ok)
    kept, dropped = pipeline.keep_runs([ok, bad])
    assert [pipeline.run_name(e) for e in kept] == ['a_s1'] and dropped == ['b_s1']
    assert len(pipeline.keep_runs([ok, bad], include_saturated=True)[0]) == 2


def test_expand_and_is_run_dir(tmp_path):
    run = tmp_path / 'campaign' / '20261120-101010_nominal_s1'
    (run / 'bag').mkdir(parents=True)
    (run / 'summary.yaml').write_text('status: complete\n')
    (tmp_path / 'campaign' / 'notes').mkdir()
    assert pipeline.expand([str(tmp_path / 'campaign')]) == [str(run)]
    assert pipeline.expand([str(run)]) == [str(run)]


def test_legacy_scenarios_from_campaign():
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, '..', '..', 'ids_bringup', 'config', 'campaign.yaml')
    pytest.importorskip('ids_bringup.campaign')
    out = pipeline.legacy_scenarios(['step_a4_s1', 'nominal_s1', 'unknown_s1'], path)
    assert out['step_a4_s1']['attacks'][0]['joint'] == 4
    assert 'unknown_s1' not in out
