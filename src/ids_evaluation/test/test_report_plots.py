import math
import os

import numpy as np

from conftest import START
from ids_evaluation import layout, pipeline, plots, report


def test_fmt():
    assert report.fmt(None) == '-'
    assert report.fmt(True) == 'yes'
    assert report.fmt(math.nan) == '-'
    assert report.fmt(math.inf) == 'open'
    assert report.fmt(0.12345) == '0.123'
    assert report.fmt(3) == '3'


def test_write_report(tmp_path, make_export):
    export = make_export(duration=90.0, events=[(1, 'step', (3,), 0.02, 60.0, 70.0)],
                         alarm_spans=[(60.05, 70.0)])
    _, results = pipeline.evaluate(export, pipeline.live_series(export))
    path = report.write_report(str(tmp_path), 'Test', [('run_s1', r) for r in results],
                               {'v2': 'live', 'thesis': 'live'}, notes=['note'],
                               extra=[('Extra', 'body')])
    text = open(path).read()
    assert '| run_s1 | v2 | step | a4 | 0.020 | 60.000 | yes | yes | 0.050 | yes | a4 |' in text
    assert '## Extra' in text
    for name in ('events.csv', 'runs.csv', 'per_joint.csv'):
        assert os.path.isfile(tmp_path / name)


def test_report_without_events(tmp_path, make_export):
    export = make_export()
    _, results = pipeline.evaluate(export, pipeline.live_series(export))
    text = open(report.write_report(str(tmp_path), 'T', [('r', r) for r in results],
                                    {'v2': 'live'})).read()
    assert 'No attack events' in text
    assert not os.path.exists(tmp_path / 'per_joint.csv')


def test_binned_max_and_intervals():
    x = np.arange(10) * 0.1
    y = np.arange(10, dtype=float)
    xb, yb = plots.binned_max(x, y, 0.5)
    assert list(yb) == [4.0, 9.0]
    assert plots.binned_max(x, y, None)[1] is y
    spans = plots.intervals(x, [0, 1, 1, 0, 0, 0, 0, 0, 1, 1])
    assert len(spans) == 2 and math.isclose(spans[0][0], 0.1)


def test_wear_truth_matches_schedule():
    b = plots.wear_truth(np.array([0.0, 30.0, 130.0, 1e6]))
    assert np.allclose(b[0], plots.WEAR['initial'])
    assert np.allclose(b[2], plots.WEAR['initial'] + 100.0 * plots.WEAR['rate'])
    assert np.allclose(b[3], plots.WEAR['max'])


def test_figures(tmp_path, make_export):
    export = make_export(duration=60.0, events=[(1, 'step', (3,), 0.02, 30.0, 40.0)],
                         alarm_spans=[(30.05, 40.0)])
    s = pipeline.live_series(export)
    panels = [plots.Panel(x.name, x.t + START, x.status, x.limit, x.slow_limit) for x in s]
    events = pipeline.metrics.events_from_truth(export['attack'], START)
    path = str(tmp_path / 'a.png')
    plots.statistics_figure(panels, events, START, 'title', path)
    plots.statistics_figure(panels, events, START, 'zoom', str(tmp_path / 'b.png'), joint=3,
                            xlim=(25.0, 50.0), binw=None)
    errors = plots.friction_figure(s[0].t + START, s[0].status, START, None, None, 't',
                                   str(tmp_path / 'c.png'))
    assert len(errors) == layout.DOF
    for name in ('a.png', 'b.png', 'c.png'):
        assert os.path.getsize(tmp_path / name) > 10000