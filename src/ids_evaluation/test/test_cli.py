# =============================================================================
#  test_cli.py - the commands end to end on synthetic exports
# =============================================================================

import os

from ids_evaluation import bag, cli


def test_evaluate_main_on_exports(tmp_path, make_export):
    paths = []
    for name, kwargs in (('nominal_s1', {}),
                         ('step_a4_s1', {'events': [(1, 'step', (3,), 0.02, 30.0, 40.0)],
                                         'alarm_spans': [(30.05, 40.0)]})):
        path = str(tmp_path / f'{name}.npz')
        bag.save_export(path, make_export(duration=60.0, run=name, **kwargs))
        paths.append(path)
    out = str(tmp_path / 'out')
    assert cli.evaluate_main(paths + ['--out', out]) == 0
    text = open(os.path.join(out, 'report.md')).read()
    assert 'step_a4_s1' in text and 'nominal_s1' in text
    names = set(os.listdir(out))
    assert {'step_a4_s1_statistics.png', 'step_a4_s1_attack.png',
            'nominal_s1_friction.png'} <= names


def test_compare_main_residual_only(tmp_path, make_export):
    train = str(tmp_path / 'nominal_s1.npz')
    bag.save_export(train, make_export(duration=150.0, run='nominal_s1', seed=1))
    test = str(tmp_path / 'step_a4_s1.npz')
    bag.save_export(test, make_export(duration=150.0, run='step_a4_s1', seed=2,
                                      events=[(1, 'step', (3,), 0.02, 130.0, 140.0)],
                                      alarm_spans=[(130.05, 140.0)],
                                      disturbance_shift=[(3, 130.0, 140.0, 8.0)]))
    out = str(tmp_path / 'cmp')
    assert cli.compare_main([train, test, '--train', 'nominal_s1', '--out', out,
                             '--skip-replay', '--skip-momentum', '--no-plots']) == 0
    text = open(os.path.join(out, 'report.md')).read()
    assert '| static |' in text and '| ewma |' in text
    assert 'Baseline limits' in text
    assert cli.compare_main([train, test, '--train', train, '--out', out, '--skip-replay',
                             '--skip-momentum', '--no-plots']) == 0
    assert 'training: nominal_s1' in open(os.path.join(out, 'report.md')).read()


def test_split_legacy_main(tmp_path):
    import numpy as np
    from conftest import START
    from ids_evaluation import layout
    t = START + np.arange(0.0, 5.0, 0.01)
    flat = {'nominal_s1__start': np.array(START),
            'nominal_s1__v2_t': t, 'nominal_s1__v2': np.zeros((len(t), 41)),
            'nominal_s1__thesis_t': t, 'nominal_s1__thesis': np.zeros((len(t), 41)),
            'nominal_s1__reference': np.zeros((len(t), layout.REFERENCE_COLUMNS)),
            'nominal_s1__measurement': np.zeros((len(t), layout.MEASUREMENT_COLUMNS)),
            'nominal_s1__attack': np.zeros((0, 2))}
    path = str(tmp_path / 'legacy.npz')
    np.savez(path, **flat)
    out = str(tmp_path / 'split')
    assert cli.legacy_main([path, '--out', out, '--campaign', 'none.yaml']) == 0
    assert os.path.isfile(os.path.join(out, 'nominal_s1.npz'))
