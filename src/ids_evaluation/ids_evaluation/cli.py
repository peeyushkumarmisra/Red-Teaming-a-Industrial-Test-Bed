import argparse
import math
import os
import sys

import numpy as np

from ids_evaluation import bag, layout, pipeline, plots, replay, report


def _campaign_default():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('ids_bringup'), 'config',
                            'campaign.yaml')
    except Exception:
        return None


def _urdf(path):
    if path:
        with open(path) as f:
            return f.read()
    import xacro
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('iiwa_sim')
    return xacro.process_file(os.path.join(share, 'urdf', 'iiwa_sim.urdf.xacro')).toxml()


def _inputs(parser):
    parser.add_argument('paths', nargs='*', help='run folders, campaign folders or export.npz')
    parser.add_argument('--legacy', help='multi-run export from plot_runs (v1.2)')
    parser.add_argument('--campaign', default=None, help='campaign.yaml for legacy scenarios')
    parser.add_argument('--calibration-dir', default=pipeline.DEFAULT_CALIBRATION_DIR)
    parser.add_argument('--limits', type=float, nargs=4, default=None,
                        metavar=('V2', 'V2_SLOW', 'THESIS', 'THESIS_SLOW'),
                        help='limits for exports that do not carry them')
    parser.add_argument('--include-saturated', action='store_true',
                        help='keep runs whose summary reports torque saturation')


def _load(args):
    exports = [pipeline.load_run(p, args.calibration_dir) for p in pipeline.expand(args.paths)]
    if args.legacy:
        exports += pipeline.load_legacy(args.legacy, args.campaign or _campaign_default())
    if not exports:
        raise SystemExit('no runs given')
    if args.limits:
        for e in exports:
            known = np.asarray(e['limits'], dtype=float)
            e['limits'] = np.where(np.isnan(known), args.limits, known)
    exports, dropped = pipeline.keep_runs(exports, args.include_saturated)
    if not exports:
        raise SystemExit('every run reports torque saturation')
    return exports, dropped


def _dropped_note(dropped):
    return ([f'excluded for torque saturation: {", ".join(dropped)}'] if dropped else [])


def _figures(out_dir, export, series, zoom=True):
    name = pipeline.run_name(export)
    start = pipeline.start_of(export)
    events = pipeline.metrics.events_from_truth(export['attack'], start)
    panels = [plots.Panel(f'{s.name}', s.t + start, s.status, s.limit, s.slow_limit)
              for s in series if math.isfinite(s.limit)]
    if not panels:
        return []
    files = []
    path = os.path.join(out_dir, f'{name}_statistics.png')
    plots.statistics_figure(panels, events, start, f'{name}: statistic / limit', path)
    files.append(path)
    if zoom and events:
        ev = events[0]
        end = ev.end if math.isfinite(ev.end) else ev.start + 10.0
        joint = ev.joints[0] if len(ev.joints) == 1 else None
        path = os.path.join(out_dir, f'{name}_attack.png')
        plots.statistics_figure(panels, events, start, f'{name}: attack window', path,
                                joint=joint, xlim=(ev.start - 5.0, end + 10.0), binw=None)
        files.append(path)
    v2 = [s for s in series if s.name == 'v2']
    if v2 and not events:
        path = os.path.join(out_dir, f'{name}_friction.png')
        truth = export['friction_truth']
        plots.friction_figure(v2[0].t + start, v2[0].status, start,
                              truth[:, 0] if len(truth) else None,
                              truth[:, 1:8] if len(truth) else None,
                              f'{name}: friction estimate vs truth', path)
        files.append(path)
    return files


def export_main(argv=None):
    p = argparse.ArgumentParser(description='Export run bags to export.npz')
    p.add_argument('paths', nargs='+')
    p.add_argument('--calibration-dir', default=pipeline.DEFAULT_CALIBRATION_DIR)
    p.add_argument('--force', action='store_true')
    args = p.parse_args(argv)
    for path in pipeline.expand(args.paths):
        export = pipeline.load_run(path, args.calibration_dir, force=args.force)
        print(f'{path}: {len(export["measurement"])} measurements, '
              f'{len(export["v2"])} v2 windows, {len(export["thesis"])} thesis windows')
    return 0


def evaluate_main(argv=None):
    p = argparse.ArgumentParser(description='Score the live IDS instances on recorded runs')
    _inputs(p)
    p.add_argument('--out', required=True)
    p.add_argument('--grace', type=float, default=1.0)
    p.add_argument('--tail', type=float, default=30.0)
    p.add_argument('--no-plots', action='store_true')
    args = p.parse_args(argv)
    results, figures = [], []
    os.makedirs(args.out, exist_ok=True)
    exports, dropped = _load(args)
    for export in exports:
        series = pipeline.live_series(export)
        _, res = pipeline.evaluate(export, series, args.grace, args.tail)
        results += [(pipeline.run_name(export), r) for r in res]
        if not args.no_plots:
            figures += _figures(args.out, export, series)
    path = report.write_report(args.out, 'IDS evaluation', results,
                               {k: pipeline.SOURCES[k] for k in layout.INSTANCES},
                               notes=[f'grace {args.grace} s, tail {args.tail} s, detection '
                                      f'starts {pipeline.WARMUP} s after arming',
                                      *_dropped_note(dropped)],
                               figures=figures)
    print(path)
    return 0


def compare_main(argv=None):
    p = argparse.ArgumentParser(description='Compare the IDS with baseline detectors')
    _inputs(p)
    p.add_argument('--train', required=True, help='nominal run used for training')
    p.add_argument('--out', required=True)
    p.add_argument('--urdf', default=None)
    p.add_argument('--duration', type=float, default=120.0)
    p.add_argument('--skip-replay', action='store_true')
    p.add_argument('--skip-momentum', action='store_true')
    p.add_argument('--replay-binary', default=None)
    p.add_argument('--no-plots', action='store_true')
    args = p.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    tests, dropped = _load(args)
    key = os.path.basename(os.path.normpath(args.train))
    train = next((e for e in tests if pipeline.run_name(e) == key), None)
    if train is None and os.path.exists(args.train):
        train = pipeline.load_run(args.train, args.calibration_dir)
    if train is None:
        raise SystemExit(f'training run {args.train} not found')
    train_name = pipeline.run_name(train)
    train_end = pipeline.training_end(args.duration)
    series = {pipeline.run_name(e): pipeline.live_series(e) for e in tests}
    residual, residual_limits = pipeline.residual_baselines(train, tests, train_end)
    for run, items in residual.items():
        series[run] += items
    notes = [f'training: {train_name}, 0-{train_end:g} s after the first reference; '
             f'that part of {train_name} is excluded from scoring',
             'baseline limits: lowest limit with no debounced alarm on the training part',
             'residual baselines use the v2 disturbance estimate recorded live',
             *_dropped_note(dropped)]
    extra = []
    urdf = None
    if not (args.skip_replay and args.skip_momentum):
        urdf = _urdf(args.urdf)
    if not args.skip_momentum:
        mom, _ = pipeline.momentum_baseline(urdf, train, tests, train_end)
        for run, s in mom.items():
            series[run].append(s)
    if not args.skip_replay:
        rep, cals = pipeline.replay_baselines(urdf, train, tests,
                                              os.path.join(args.out, 'replay'), args.duration,
                                              binary=args.replay_binary)
        for run, items in rep.items():
            series[run] += items
        rows = []
        for e in tests:
            run = pipeline.run_name(e)
            live = {s.name: s for s in series[run]}
            for a, b in (('v2', 'v2_replay'), ('thesis', 'thesis_replay')):
                if a in live and b in live:
                    f = pipeline.replay_fidelity(live[a], live[b])
                    if f:
                        rows.append({'run': run, 'pair': f'{a} vs {b}', 'matched': f['matched'],
                                     'within 1e-3 N*m': f['agree_fraction'],
                                     'worst relative rms': max(f['relative_rms']),
                                     'lowest correlation': np.nanmin(f['correlation'])})
        if rows:
            extra.append(('Replay fidelity (disturbance estimate)', report.table(
                rows, list(rows[0]))))
        extra.append(('Replay calibrations', report.table(
            [{'detector': k, 'limit': v[0], 'slow limit': v[1]} for k, v in cals.items()],
            ('detector', 'limit', 'slow limit'))))
    extra.append(('Baseline limits', report.table(
        [{'detector': k, 'limit': v} for k, v in residual_limits.items()],
        ('detector', 'limit'))))
    results, figures = [], []
    for e in tests:
        run = pipeline.run_name(e)
        after = train_end if run == train_name else None
        _, res = pipeline.evaluate(e, series[run], after=after)
        results += [(run, r) for r in res]
        if not args.no_plots:
            figures += _figures(args.out, e, [s for s in series[run]
                                              if s.name not in ('v2_replay',)], zoom=True)
    names = sorted({r.instance for _, r in results}, key=list(pipeline.SOURCES).index)
    path = report.write_report(args.out, 'IDS versus baselines', results,
                               {k: pipeline.SOURCES[k] for k in names}, notes=notes,
                               figures=figures, extra=extra)
    print(path)
    return 0


def replay_main(argv=None):
    p = argparse.ArgumentParser(description='Replay one run through ids_replay')
    p.add_argument('run')
    p.add_argument('--out', required=True, help='output .npz with t and status')
    p.add_argument('--urdf', default=None)
    p.add_argument('--params', default='ids.yaml')
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--calibration')
    group.add_argument('--calibrate')
    p.add_argument('--duration', type=float, default=120.0)
    p.add_argument('--set', action='append', default=[], dest='overrides')
    p.add_argument('--calibration-dir', default=pipeline.DEFAULT_CALIBRATION_DIR)
    args = p.parse_args(argv)
    export = pipeline.load_run(args.run, args.calibration_dir)
    params = args.params if os.path.isfile(args.params) else replay.package_params(args.params)
    windows, limit, log = replay.run_replay(export, _urdf(args.urdf), params,
                                            calibration=args.calibration,
                                            calibrate=args.calibrate, duration=args.duration,
                                            overrides=args.overrides)
    t, status = replay.replay_status(windows, limit)
    np.savez_compressed(args.out, t=t, status=status, start=export['start'])
    sys.stderr.write(log)
    print(f'{args.out}: {len(t)} windows, limit {limit:g}')
    return 0


def legacy_main(argv=None):
    p = argparse.ArgumentParser(description='Split a v1.2 multi-run export into export files')
    p.add_argument('export')
    p.add_argument('--out', required=True)
    p.add_argument('--campaign', default=None)
    args = p.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    for e in pipeline.load_legacy(args.export, args.campaign or _campaign_default()):
        path = os.path.join(args.out, f'{pipeline.run_name(e)}.npz')
        bag.save_export(path, e)
        print(path)
    return 0